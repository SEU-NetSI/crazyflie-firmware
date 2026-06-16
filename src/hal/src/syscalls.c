/*
 * syscalls.c
 *
 * Implementation of newlib system calls for embedded systems
 */

#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>

// _sbrk - heap management
caddr_t _sbrk(int incr) {
    extern char _end; // Defined by the linker
    static char *heap_end = 0;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;

    // Simple heap limit check (adjust as needed)
    extern char _estack; // Stack top from linker script
    if (heap_end + incr > &_estack - 4096) { // Leave 4KB stack space
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

// _write - write to stdout/stderr (no-op for embedded)
int _write(int file, char *ptr, int len) {
    // No output in embedded system
    return len;
}

// _read - read from stdin (no-op for embedded)
int _read(int file, char *ptr, int len) {
    // No input in embedded system
    return 0;
}

// _close - close file (no-op for embedded)
int _close(int file) {
    return -1;
}

// _fstat - get file status (no-op for embedded)
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

// _isatty - check if file is a tty (no-op for embedded)
int _isatty(int file) {
    return 1;
}

// _lseek - seek in file (no-op for embedded)
int _lseek(int file, int ptr, int dir) {
    return 0;
}

// _exit - exit program (no-op for embedded)
void _exit(int status) {
    while(1);
}

// _kill - kill process (no-op for embedded)
int _kill(int pid, int sig) {
    return -1;
}

// _getpid - get process id (no-op for embedded)
int _getpid(void) {
    return 1;
}

// _fork - fork process (no-op for embedded)
int _fork(void) {
    return -1;
}

// _wait - wait for process (no-op for embedded)
int _wait(int *status) {
    return -1;
}