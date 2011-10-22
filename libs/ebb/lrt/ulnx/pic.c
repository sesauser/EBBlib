#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/select.h>
#include <string.h>
#include <assert.h>

#include "../../../base/types.h"
#include "pic.h"

#define FIRST_VEC        16
#define NUM_MAPPABLE_VEC 15
#define NUM_RES_VEC      1
#define NUM_VEC          (NUM_MAPPABLE_VEC + NUM_RES_VEC) 


#ifndef FD_COPY
#define FD_COPY(src,dest) memcpy((dest),(src),sizeof(dest))
#endif


struct VecDesc {
  int fd;
  lrt_pic_handler h;
};

struct Pic {
  struct VecDesc vecs[NUM_VEC];
  fd_set fdset;
  int maxfd;
  uval free;
} pic;

uval 
lrt_pic_firstvec(void) 
{ 
  return FIRST_VEC; 
}

uval 
lrt_pic_numvec(void) 
{ 
  return NUM_MAPPABLE_VEC; 
}

sval
lrt_pic_init(void)
{
  int fd[FIRST_VEC];
  int i=0;

  bzero(&pic, sizeof(pic));
  bzero(&fd, sizeof(fd));

  fd[i] = open("/dev/null", O_RDONLY);

  while (fd[i] < (FIRST_VEC-1)) {
    i++;
    fd[i] = dup(fd[0]);
  };

  for (i=0; i<NUM_VEC; i++) {
    pic.vecs[i].fd=dup(fd[0]);
    if (pic.vecs[i].fd != (FIRST_VEC+i)) return -1;
  }
   
  for (i=0; i<FIRST_VEC; i++) if(fd[i]) close(fd[i]);

  FD_ZERO(&pic.fdset);
  
  return 1;
}

sval
lrt_pic_allocvec(uval *vec)
{
  uval rtn;
  uval i;

  for (i=0, rtn=pic.free; i<NUM_MAPPABLE_VEC; i++) {
    if (pic.vecs[rtn].h == NULL) {
      pic.vecs[rtn].h = (lrt_pic_handler)((uval)-1);
      pic.free++;
      if (pic.free >= NUM_MAPPABLE_VEC) pic.free=0;
      *vec = rtn;
      return 1;
    }
    rtn++;
    if (rtn >= NUM_MAPPABLE_VEC) rtn=0;
  }    
  return -1;
}

sval
lrt_pic_mapvec(lrt_pic_src s, uval vec, lrt_pic_handler h)
{
  int rc;
  int sfd = (int)s;

  if (vec >= NUM_MAPPABLE_VEC || pic.vecs[vec].h == 0) return -1;

  pic.vecs[vec].h = h;
  rc = dup2(sfd, pic.vecs[vec].fd);
  assert(rc == pic.vecs[vec].fd);
  FD_SET(rc, &pic.fdset);
  if (rc>pic.maxfd) pic.maxfd=rc;

  return 1;
}

sval 
lrt_pic_loop(void)
{
  fd_set rfds, efds;
  int v,i, rc;
  
  while (1) {
    FD_COPY(&pic.fdset, &rfds);
    FD_COPY(&pic.fdset, &efds);
    
    rc = pselect(pic.maxfd+1, &rfds, NULL, &efds, NULL, NULL);
    if (rc < 0) {
      fprintf(stderr, "Error: pselect failed (%d)\n", errno);
      perror("pselect");
      return -1;
    }

    // may later want to actually have a period event that has a programmable
    // period
    if (rc == 0) {
      fprintf(stderr, "What Select timed out\n");
      return -1;
    }
    if (rc > 0) {
      for (i = FIRST_VEC,v=0; i <= pic.maxfd; i++, v++) {
	if (FD_ISSET(i, &efds) || (FD_ISSET(i, &rfds))) {
	  if (pic.vecs[v].h) {
	    pic.vecs[v].h();
	  } else {
	    fprintf(stderr, "ERROR: %s: spurious interrupt on %d\n", __func__, i);
	  }
	  FD_SET(i, &efds); FD_SET(i, &rfds); 
	}
      }
    }
  }
  return -1;
}

