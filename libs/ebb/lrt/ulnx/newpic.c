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


// see http://www.win.tue.nl/~aeb/linux/lk/lk-12.html
// when implementing wakeupall and software based interrupts

#define FIRST_VEC        16
#define NUM_MAPPABLE_VEC 15
// reserve 2 : 1 for ipi and 1 for software generated
#define NUM_RES_VEC      2
#define NUM_VEC          (NUM_MAPPABLE_VEC + NUM_RES_VEC) 


#ifndef FD_COPY
#define FD_COPY(src,dest) memcpy((dest),(src),sizeof(dest))
#endif

#define MAX_PICS 128
#define FIRST_PIC_ID 0
#define LAST_PIC_ID (MAX_PICS-1)

typedef uval8 lrt_pic_id;
typedef uval64[MAX_PICS/64] lrt_pic_set;

struct VecDesc {
  lrt_pic_set picset;
  lrt_pic_handler h;
  int fd;
};

struct Pic {
  struct VecDesc vecs[NUM_VEC];
  fd_set fdset;
  struct timespec periodic;
  uval free;
  int maxfd;
  uval lock;
} pic;

static uval
lock(void)
{
  uval rc=0;
  
  while (!rc) {
    rc = __sync_bool_compare_and_swap(&(pic.lock), 0, 1);
  }
  return rc;
}

static void
unlock(void)
{
  __sync_bool_compare_and_swap(&(pic.lock), 1, 0);
}

static void
sighandler(int s)
{
  return;
}

static void
wakeupall(void)
{
  kill(0, SIGINT);
}

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
  sigset_t emptyset, blockset;
  struct sigaction sa;

  // confirm sanity of pic configuration 
  assert(MAX_PICS/64 * 64 == MAX_PICS);

  // initialize all pic state to zero
  bzero(&pic, sizeof(pic));

  // initialized working fd array 
  bzero(&fd, sizeof(fd));
  fd[i] = open("/dev/null", O_RDONLY);

  // get us to the FIRST_VEC fd by opening
  // fds until we get to FIRST_VEC
  while (fd[i] < (FIRST_VEC-1)) {
    i++;
    fd[i] = dup(fd[0]);
  };

  // reserve fds for our vectors
  for (i=0; i<NUM_VEC; i++) {
    pic.vecs[i].fd=dup(fd[0]);
    if (pic.vecs[i].fd != (FIRST_VEC+i)) return -1;
  }
   
  // close and free fd's that we allocated to get to
  // FIRST_VEC
  for (i=0; i<FIRST_VEC; i++) if(fd[i]) close(fd[i]);

  // explicity setup fdset so that we are not paying attention
  // to any vectors at start... vectors are added when they are mapped
  FD_ZERO(&pic.fdset);
  
  // setup default signal mask so that SIGINT is being ignored by
  // all pic threads when they start however ensure that a common 
  // handler is in place
  /* this code was based on http://lwn.net/Articles/176911/ */
  sigemptyset(&blockset);         /* Block SIGINT */
  sigaddset(&blockset, SIGINT);
  sigprocmask(SIG_BLOCK, &blockset, NULL);
  sa.sa_handler = sighandler;        /* Establish signal handler */
  sa.sa_flags = 0;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, NULL);

  return 1;
}

sval
lrt_pic_allocvec(uval *vec)
{
  uval rtn;
  uval i;
  sval rc=-1;

  lock();

  for (i=0, rtn=pic.free; i<NUM_MAPPABLE_VEC; i++) {
    if (pic.vecs[rtn].h == NULL) {
      pic.vecs[rtn].h = (lrt_pic_handler)((uval)-1);
      pic.free++;
      if (pic.free >= NUM_MAPPABLE_VEC) pic.free=0;
      *vec = rtn;
      rc=1;
      goto done;
    }
    rtn++;
    if (rtn >= NUM_MAPPABLE_VEC) rtn=0;
  }    

 done:
  unlock();
  return rc;

}

sval
lrt_pic_mapvec(lrt_pic_src s, uval vec, lrt_pic_handler h, lrt_pic_id id)
{
  int i;
  int sfd = (int)s;
  int rc=1;
  
  lock();

  if (pic.vecs[vec].h == 0) {
    rc=-1;
    goto done;
  }

  pic.vecs[vec].h = h;
  i = dup2(sfd, pic.vecs[vec].fd);
  assert(i == pic.vecs[vec].fd);
  FD_SET(i, &pic.fdset);
  if (i>pic.maxfd) pic.maxfd=i;
  lrt_pic_set_add(&pic.vecs[vec].set, id);
  wakeupall();

 done:
  unlock();
  return rc;
}

// conncurrent pic loop
// one loop for each local pic
// each pic will wake up on any interrupt occurring but only dispatches
// the handler if the interrupt includes the pic id of the loop in its pic set
sval 
lrt_pic_loop(lrt_pic_id myid)
{
  fd_set rfds, efds;
  int v,i, rc;
  sigset_t empty;

  lrt_pic_mask_clear(&mymask);
  lrt_pic_mask_set(myid);

  while (1) {
    FD_COPY(&pic.fdset, &rfds);
    FD_COPY(&pic.fdset, &efds);

    sigemptyset(&emptyset);
    rc = pselect(pic.maxfd+1, &rfds, NULL, &efds, NULL, &emptyset);

    if (rc < 0) {
      if (errno==EINTR) {
	// do nothing
      } else {
	fprintf(stderr, "Error: pselect failed (%d)\n", errno);
	perror("pselect");
	return -1;
      }
    }

    // may later want to actually have a period event that has a programmable
    // period
    if (rc == 0) {
      fprintf(stderr, "What Select timed out\n");
      return -1;
    }
    if (rc > 0) {
      for (i = FIRST_VEC,v=0; i <= pic.maxfd; i++, v++) {
	if (FD_ISSET(i, &efds) || (FD_ISSET(i, &rfds))
	    && lrt_pic_set_has(myid)) {
	  if (pic.vecs[v].h) {
	    pic.vecs[v].h(v);
	  } else {
	    fprintf(stderr, "ERROR: %s: spurious interrupt on %d\n", __func__, i);
	  }
	}
      }
    }
  }
  return -1;
}

