#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#ifndef S_IFCHR
#define S_IFCHR 0020000
#endif

int _write(int file, char *ptr, int len) { (void)file; (void)ptr; return len; }
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _close(int file) { (void)file; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }

/*
 * Provide _sbrk using heap boundaries defined in linker script.
 * Linker defines `_heap_start` and `_heap_end` in linker.ld.
 */
int _sbrk(int incr)
{
	extern char _heap_start; /* start set in linker.ld */
	extern char _heap_end;   /* end set in linker.ld */
	static char *heap = 0;
	char *prev;

	if (heap == 0) heap = &_heap_start;
	prev = heap;
	if ((char *)prev + incr > &_heap_end)
	{
		errno = ENOMEM;
		return -1;
	}
	heap += incr;
	return (int)prev;
}

void _exit(int status) { (void)status; while (1); }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
int _getpid() { return 1; }