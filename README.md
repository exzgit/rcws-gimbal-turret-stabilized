



```sh
arm-none-eabi-gcc   -T linker.ld   src/startup/stm32f411ceu.
s   src/main.c   -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard   -no
startfiles   -Wl,--gc-sections   -o output/firmware.elf
```


```

```sh
 arm-none-eabi-gcc   -T linker.ld   src/startup/stm32f411ceu.s   src/main.c   -mcpu=cortex-m4 -mthumb   -mfpu=fpv4-sp-d16 -mfloat-abi=hard   -nost
artfiles -nostdlib   -Wl,--gc-sections   -o output/firmware.elf
```


jika dibutuhkan:

