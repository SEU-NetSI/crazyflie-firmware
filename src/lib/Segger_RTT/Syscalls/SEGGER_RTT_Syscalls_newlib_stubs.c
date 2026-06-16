/**
 * Newlib syscalls stub implementation for ARM embedded systems.
 * Provides implementations of required system calls for newlib libc.
 * 
 * These stubs are required when using standard C library functions that depend
 * on low-level I/O operations (_sbrk, _read, _write, _close, etc.)
 * 
 * This implementation integrates with Crazyflie's console and debug infrastructure
 * to provide functional I/O support while maintaining robustness.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

/* External variables provided by the linker script */
extern char _end;       /* End of data section */

static char *heap_end = 0;

/* Forward declarations of Crazyflie console functions */
/* These are declared as weak symbols to allow compilation even if not linked */
extern int consolePutchar(int ch) __attribute__((weak));
extern void consoleFlush(void) __attribute__((weak));
extern int uart1Putchar(int ch) __attribute__((weak));

/**
 * Internal helper to safely output a character.
 * Tries multiple output methods in order of preference.
 */
static int safe_putchar(int ch)
{
    /* Try console first (preferred: goes to wireless/USB) */
    if (consolePutchar != NULL) {
        return consolePutchar(ch);
    }
    
    /* Fallback to UART1 if console not available */
    if (uart1Putchar != NULL) {
        return uart1Putchar(ch);
    }
    
    /* No output available, but don't fail */
    return 1;
}

/**
 * Increase program data space.
 * Newlib calls this to get more memory for malloc.
 * 
 * Implementation: Simple linear heap allocation from _end symbol
 * WARNING: No heap limit checking - ensure sufficient RAM available
 */
caddr_t _sbrk(int incr)
{
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;
    heap_end += incr;

    /* TODO: Add heap bounds checking against stack to prevent collisions */
    return (caddr_t) prev_heap_end;
}

/**
 * Close a file descriptor.
 * 
 * In embedded systems, we don't have real file descriptors.
 * This is a no-op for embedded operations.
 */
int _close(int fd)
{
    (void)fd;  /* Not used */
    return 0;  /* Success for standard streams */
}

/**
 * Check if a file descriptor refers to an interactive device.
 * 
 * Returns true (1) for stdout/stderr to indicate they are terminal devices.
 */
int _isatty(int fd)
{
    /* Treat stdout (1) and stderr (2) as terminal devices */
    return (fd == STDOUT_FILENO || fd == STDERR_FILENO);
}

/**
 * Get file status.
 * 
 * Provides minimal file status information.
 * All files are treated as character devices.
 */
int _fstat(int fd, struct stat *st)
{
    (void)fd;  /* Not used */
    if (st) {
        st->st_mode = S_IFCHR;  /* Character device */
        st->st_size = 0;
        st->st_blksize = 1;
    }
    return 0;
}

/**
 * Seek to a position in a file.
 * 
 * Not supported for character devices (console/UART).
 */
off_t _lseek(int fd, off_t offset, int whence)
{
    (void)fd;      /* Not used */
    (void)offset;  /* Not used */
    (void)whence;  /* Not used */
    errno = ESPIPE;  /* Illegal seek on pipe */
    return -1;
}

/**
 * Read from a file descriptor.
 * 
 * Currently not implemented - returns EOF (0 bytes).
 * Could be extended to read from UART or console buffer in future.
 */
int _read(int fd, char *ptr, int len)
{
    (void)fd;   /* Not used */
    (void)ptr;  /* Not used */
    (void)len;  /* Not used */
    errno = EBADF;  /* Bad file descriptor */
    return 0;   /* Return 0 bytes read (EOF) */
}

/**
 * Write to a file descriptor.
 * 
 * Redirects console output (stdout/stderr) to Crazyflie's console system.
 * Falls back to UART1 if console is not available.
 * 
 * @param fd File descriptor (typically 1 for stdout, 2 for stderr)
 * @param ptr Pointer to data buffer
 * @param len Number of bytes to write
 * @return Number of bytes written
 */
int _write(int fd, const char *ptr, int len)
{
    int written = 0;
    
    /* Only handle stdout (1) and stderr (2) */
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        /* Write each character */
        for (int i = 0; i < len; i++) {
            if (safe_putchar((unsigned char)ptr[i]) >= 0) {
                written++;
            } else {
                break;  /* Stop on error */
            }
        }
        
        /* Try to flush output buffer */
        if (consoleFlush != NULL) {
            consoleFlush();
        }
    } else {
        /* Invalid file descriptor */
        errno = EBADF;
        return -1;
    }
    
    return written;
}

/**
 * Exit the program.
 * 
 * In embedded systems, a true exit typically means:
 * 1. Disable interrupts
 * 2. Stop all FreeRTOS tasks
 * 3. Enter an infinite loop (waiting for watchdog reset)
 * 
 * This implementation uses a simple infinite loop.
 * For graceful shutdown, consider entering FreeRTOS kernel or bootloader.
 */
void _exit(int code)
{
    (void)code;  /* Not used */
    
    /* Disable all interrupts first for safety */
    __asm volatile ("cpsid i");
    
    /* Infinite loop - program will require hardware reset or watchdog trigger */
    while(1) {
        __asm volatile ("nop");  /* Prevent compiler optimization */
    }
}

/**
 * Kill a process or send a signal.
 * 
 * Not implemented for embedded systems - no process management.
 */
int _kill(int pid, int sig)
{
    (void)pid;  /* Not used */
    (void)sig;  /* Not used */
    errno = ENOSYS;  /* Not implemented */
    return -1;
}

/**
 * Get process ID.
 * 
 * Returns a dummy process ID. In embedded systems, there's typically only one "process".
 */
int _getpid(void)
{
    return 1;
}
