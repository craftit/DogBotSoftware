// Newlib stubs implementation

#include <sys/types.h>
#include <stdlib.h>

__attribute__((used))  pid_t _getpid(void)
{
        return 0;
}

__attribute__((used))  void _exit( int status )
{
        (void)status;
        while( 1 );
}


__attribute__((used))  int _kill( int pid, int sig )
{
        (void)pid; (void)sig;
        return -1;
}

int _open_r(void *reent, const char *file, int flags, int mode)
{
        (void)reent; (void)file; (void)flags; (void)mode;
        return -1;
}

void *__dso_handle = NULL;

void *_sbrk(intptr_t increment)
{
  (void) increment;
  return 0;
}
