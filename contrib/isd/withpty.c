#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(int argc, char **argv) {
  int pty = open("/dev/ptmx", O_RDWR);
  grantpt(pty);
  unlockpt(pty);

  close(0);
  close(1);
  close(2);
  dup2(pty, 0);
  dup2(pty, 1);
  dup2(pty, 2);
  close(pty); 
  execvp(argv[1], &argv[1]);
  return 1;
}

