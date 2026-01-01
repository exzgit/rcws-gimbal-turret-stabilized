#include "stm32f4xx_hal.h"

/* ===== Global handle ===== */
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* PID for stabilization */
typedef struct {
  float kp;
  float ki;
  float kd;
  float prev_error;
  float integral;
} PID_t;

static float pid_update(PID_t *pid, float error, float dt)
{
  pid->integral += error * dt;
  float derivative = 0.0f;
  if (dt > 0.0f) derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;
  return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

/* ESC helpers for yaw (TIM1 CH1) and pitch (TIM1 CH2) */
void set_motor_yaw_us(uint16_t us)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
}

void set_motor_pitch_us(uint16_t us)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
}

/* ===== Function prototypes ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

void set_motor_speed_us(uint16_t us);

/* ===== MAIN ===== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* Start PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* ESC arming */
  set_motor_speed_us(1000);
  HAL_Delay(3000);

  /* Default neutral and setpoints */
  const uint16_t neutral_us = 1500;
  set_motor_yaw_us(neutral_us);
  set_motor_pitch_us(neutral_us);

  PID_t pid_yaw = { .kp = 6.0f, .ki = 0.5f, .kd = 0.2f, .prev_error = 0.0f, .integral = 0.0f };
  PID_t pid_pitch = { .kp = 6.0f, .ki = 0.5f, .kd = 0.2f, .prev_error = 0.0f, .integral = 0.0f };

  float setpoint_yaw = 0.0f;   // degrees
  float setpoint_pitch = 0.0f; // degrees
  float meas_yaw = 0.0f;
  float meas_pitch = 0.0f;

  /* Serial parsing buffer for IMU frames or single-char commands */
  char linebuf[64];
  uint8_t linepos = 0;
  uint8_t rx;

  uint32_t last_time = HAL_GetTick();

  while (1)
  {
    /* read available byte (non-blocking with timeout 100ms) */
    if (HAL_UART_Receive(&huart2, &rx, 1, 100) == HAL_OK)
    {
      /* simple textual protocol:
         - IMU frame: "I,<pitch>,<yaw>\n"  (angles in degrees, floats)
         - Command: single char 'L'/'R' adjust yaw setpoint by step
                    'U'/'D' adjust pitch setpoint
                    'S' set both to neutral(0)
      */
      if (rx == '\n' || rx == '\r')
      {
        linebuf[linepos] = '\0';
        if (linepos > 0)
        {
          if (linebuf[0] == 'I')
          {
            float p,y;
            if (sscanf(linebuf+2, "%f,%f", &p, &y) == 2)
            {
              meas_pitch = p;
              meas_yaw = y;
            }
          }
          else if (linepos == 1)
          {
            char cmd = linebuf[0];
            const float step = 5.0f; // degrees per press
            if (cmd == 'L' || cmd == 'l') setpoint_yaw -= step;
            else if (cmd == 'R' || cmd == 'r') setpoint_yaw += step;
            else if (cmd == 'U' || cmd == 'u') setpoint_pitch += step;
            else if (cmd == 'D' || cmd == 'd') setpoint_pitch -= step;
            else if (cmd == 'S' || cmd == 's') { setpoint_yaw = 0.0f; setpoint_pitch = 0.0f; }
          }
        }
        linepos = 0;
      }
      else
      {
        if (linepos < sizeof(linebuf)-1) linebuf[linepos++] = (char)rx;
      }
    }

    /* Run PID at fixed dt (approx) */
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) * 0.001f; // seconds
    if (dt >= 0.02f) /* 50Hz */
    {
      last_time = now;

      /* compute errors */
      float err_yaw = setpoint_yaw - meas_yaw;
      float err_pitch = setpoint_pitch - meas_pitch;

      float out_yaw = pid_update(&pid_yaw, err_yaw, dt);   // output in degrees->us mapping
      float out_pitch = pid_update(&pid_pitch, err_pitch, dt);

      /* Map PID output to microsecond offsets. Tune scale as needed. */
      const float scale_us_per_deg = 2.0f; // 2 us per degree (example)

      int32_t yaw_us = (int32_t)neutral_us + (int32_t)(out_yaw * scale_us_per_deg);
      int32_t pitch_us = (int32_t)neutral_us + (int32_t)(out_pitch * scale_us_per_deg);

      /* clamp to safe ESC range (1000..2000) */
      if (yaw_us < 1000) yaw_us = 1000; if (yaw_us > 2000) yaw_us = 2000;
      if (pitch_us < 1000) pitch_us = 1000; if (pitch_us > 2000) pitch_us = 2000;

      set_motor_yaw_us((uint16_t)yaw_us);
      set_motor_pitch_us((uint16_t)pitch_us);
    }
  }
}

/* ===== Implementation helpers ===== */
void set_motor_speed_us(uint16_t us)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    while (1) { }
  }
}
