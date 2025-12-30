#include <libopencm3/stm32/usart.h>
#include <sys/stat.h>
#include <stdint.h>
#include <errno.h>

int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    errno = EINVAL;
    return -1;
}

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

int _close(int file) {
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st) {
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

extern char end;  /* Defined by the linker script */
static char *heap_end;

int _sbrk(int incr) {
    char *prev_heap_end;

    if (heap_end == 0)
        heap_end = &end;
    prev_heap_end = heap_end;
    heap_end += incr;
    return (int)prev_heap_end;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}

void _exit(int status) {
    (void)status;
    while (1) {
        /* trap here if abort() is called */
    }
}