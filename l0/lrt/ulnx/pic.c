/*
 * Copyright (C) 2011 by Project SESA, Boston University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <config.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#include <inttypes.h>
#include <l0/lrt/ulnx/pic-unix.h>
#include <l0/lrt/ulnx/pic.h>

#ifdef __APPLE__
pthread_key_t lrt_pic_myid_pthreadkey;
#else
__thread lrt_pic_id lrt_pic_myid;
#endif

lrt_pic_id lrt_pic_firstid;
volatile lrt_pic_id lrt_pic_lastid;

/*
 * the global pic structure, maintains a 
 */
struct Pic {
  uintptr_t lock;		/* global lock, protects all changes pic */

  /* 
   * This is the global version of the vector, vectores can mapped
   * globally or locally in a single local pic. In either case, need to 
   * allocate the value of the vector globally.
   */
  lrt_pic_handler gvecs[NUM_VEC];
  uintptr_t free;

  volatile uintptr_t numlpics;	/* number local pics i.e., cores */
} pic;

/*
 * Array of pic structures with one per core. 
 */
//FIXME: Do we want to pad these to cacheline size
struct LPic {
  lrt_pic_handler lvecs[NUM_VEC];
  int enabled[NUM_VEC];		/* FIXME: make a bitvector */
  lrt_pic_id id;
  uintptr_t lcore;		/* logical core */
  volatile uintptr_t ipiStatus;
  volatile uintptr_t resetStatus;
} lpics[LRT_PIC_MAX_PICS];


static uintptr_t
lock(void)
{
  uintptr_t rc=0;
  
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

void
lrt_pic_enable(uintptr_t vec)
{
  lpics[lrt_pic_myid].enabled[vec] = 1;
}

void
lrt_pic_disable(uintptr_t vec)
{
  if (vec != RST_VEC) {
    lpics[lrt_pic_myid].enabled[vec] = 0;
  }
}

uintptr_t 
lrt_pic_getIPIvec(void)
{
  return IPI_VEC;
}


void
lrt_pic_enableipi(void)
{
  lrt_pic_enable(IPI_VEC);
}

void
lrt_pic_disableipi(void)
{
  lrt_pic_disable(IPI_VEC);
}


uintptr_t 
lrt_pic_firstvec(void) 
{ 
  return 0; 
}

uintptr_t 
lrt_pic_numvec(void) 
{ 
  return NUM_MAPPABLE_VEC; 
}
    
/*
 * allocate a new ID
 * this is always done on the core, so nothing returned
 */
static inline void
lrt_pic_allocate_core_id(void)
{
  int i;
  lrt_pic_id myid;
  // for first core, we are running at this point sequentially
  if (pic.numlpics == 0) { 
    pic.numlpics = 1;
    lrt_pic_firstid = lrt_pic_lastid = 0; /* id is 0 */
    myid = lrt_pic_firstid;
  } else {
    lrt_pic_id tmp;
    myid = __sync_fetch_and_add(&pic.numlpics, 1);
    assert(myid <= LRT_PIC_MAX_PICS);

    // automically check if myid > last_id, if so modify it
    tmp = lrt_pic_lastid;
    while (tmp < myid) {
      __sync_val_compare_and_swap(&lrt_pic_lastid, tmp, myid);
      tmp = lrt_pic_lastid;
    }
  }
#ifdef __APPLE__
  pthread_setspecific(lrt_pic_myid_pthreadkey, (void *)myid);
#else
  lrt_pic_myid = myid;
#endif

  // initialize my lcore to the underlying HW (e.g., threadid)
  lpics[lrt_pic_myid].lcore = lrt_pic_unix_getlcoreid();
  // FIXME DS: This format is from inttypes.h which isn't
  // freestanding =(
  fprintf(stderr, "***core %" PRIxPTR " started\n", lpics[lrt_pic_myid].lcore);

  // copy over the vector of interrupts
  lock();
  for (i=0; i<NUM_MAPPABLE_VEC; i++) {
    lpics[lrt_pic_myid].lvecs[i] = pic.gvecs[i];
  }
  // set initial reset vector
  lpics[lrt_pic_myid].lvecs[RST_VEC] = pic.gvecs[RST_VEC];
  unlock();

  assert(lpics[lrt_pic_myid].lcore != 0);
}

intptr_t
lrt_pic_add_core()
{
  uintptr_t core = lrt_pic_unix_addcore((void *(*)(void*))lrt_pic_loop, 0);
  // FIXME DS: This format is from inttypes.h which isn't
  // freestanding =(
  fprintf(stderr, "***core %" PRIxPTR " created\n", core);
  return core;
}

/*
 * routine called just on first core to initialize everything, then goes into 
 * loop
 */
intptr_t
lrt_pic_init(lrt_pic_handler h)
{
  // confirm sanity of pic configuration 
  assert(LRT_PIC_MAX_PICS/64 * 64 == LRT_PIC_MAX_PICS);

  // initialize all pic state to zero
  bzero(&pic, sizeof(pic));
  bzero(lpics, sizeof(lpics));

#ifdef __APPLE__
  // initialize key for pthreads
  pthread_key_create(&lrt_pic_myid_pthreadkey, NULL);
#endif

  if (lrt_pic_unix_init() != 0) { /* initialize logical HW for unix */
    //error, return -1;
    return -1;
  }

  // setup where the initial reset will be directed to
  lrt_pic_mapreset(h);

  // fall into loop
  lrt_pic_loop();

  assert(0);

  return 1;
}

intptr_t
lrt_pic_allocvec(uintptr_t *vec)
{
  uintptr_t rtn;
  uintptr_t i;
  intptr_t rc=-1;

  lock();

  for (i=0, rtn=pic.free; i<NUM_MAPPABLE_VEC; i++) {
    if (pic.gvecs[rtn] == NULL) {
      pic.gvecs[rtn] = (lrt_pic_handler)((uintptr_t)-1);
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

/*
 * this always maps it locally
 */
intptr_t 
lrt_pic_mapipi(lrt_pic_handler h)
{

  lpics[lrt_pic_myid].lvecs[IPI_VEC] = h;
  return 1;
}

intptr_t 
lrt_pic_mapreset(lrt_pic_handler h)
{

  pic.gvecs[RST_VEC] = h;		/* to start, need global vec set */
  lpics[lrt_pic_myid].lvecs[RST_VEC] = h;
  return 1;
}


intptr_t
lrt_pic_ipi(lrt_pic_id target)
{
  if (target>lrt_pic_lastid) return -1;
  // FIXME: probably need to make this at least volatile
  lpics[target].ipiStatus = 1;
  assert(lpics[target].lcore != 0);
  lrt_pic_unix_wakeup(lpics[target].lcore);
  return 1;
}

intptr_t
lrt_pic_reset(lrt_pic_id target)
{
  if (target>lrt_pic_lastid) return -1;
  lpics[target].resetStatus = 1;
  assert(lpics[target].lcore != 0);
  lrt_pic_unix_wakeup(lpics[target].lcore);
  return 1;
}

void
lrt_pic_ackipi(void)
{
  lpics[lrt_pic_myid].ipiStatus = 0;
}


intptr_t
lrt_pic_mapvec(lrt_pic_src s, uintptr_t vec, lrt_pic_handler h)
{
  int rc=1;
  lrt_pic_id i;
  
  lock();

  /*
   * must not be marked as free, if -1 it is allocated but not yet
   * assigned a handler
   */
  if (pic.gvecs[vec] == 0) {
    rc=-1;
    goto done;
  }

  pic.gvecs[vec] = h;
  for (i=lrt_pic_firstid; i<=lrt_pic_lastid; i++) {
    lpics[i].lvecs[vec] = h;
    
    // FIXME: will have to handle cores that are not up
    assert(lpics[i].lcore != 0);
    lrt_pic_unix_wakeup(lpics[i].lcore);
  }

  if ((rc = lrt_pic_unix_enable(s, vec)) != 0) {
    goto done;
  }

 done:
  unlock();
  return rc;
}

static void
bind_proc(uintptr_t p)
{
#if 0
  cpu_set_t mask;

  CPU_ZERO( &mask );
  CPU_SET(p, &mask);
  if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
    perror("ERROR: Could not set CPU Affinity");
  }
#else
  fprintf(stderr, "%s: NYI\n", __func__);
#endif
}

// conncurrent pic loop
// one loop for each local pic
// each pic will wake up on any interrupt occurring but only dispatches
// the handler if the interrupt includes the pic id of the loop in its pic set
intptr_t 
lrt_pic_loop()
{
  int v, numintr;
  struct LPic *lpic;

  // this will initialize lrt_pic_myid
  lrt_pic_allocate_core_id();

  lpic = lpics + lrt_pic_myid;

  bind_proc(lrt_pic_myid);

  lrt_pic_enableipi();

  // first operation that will happen is reset
  lrt_pic_enable(RST_VEC);  

  // send a reset to myself
  lrt_pic_reset(lrt_pic_myid);
  
  while (1) {
    lrt_pic_unix_ints intrSet;
    
    if ((numintr = lrt_pic_unix_blockforinterrupt(&intrSet)) <0 ) {
      return -1;
    }
    
    if (lpic->resetStatus) {
      lpic->resetStatus=0;
      lpic->lvecs[RST_VEC]();
    }
    
    if (lpic->ipiStatus) {
      if (lpic->enabled[IPI_VEC]) {
	lrt_pic_disableipi();
	assert(lpic->lvecs[IPI_VEC] != NULL);
	lpic->lvecs[IPI_VEC]();
      } else {
	fprintf(stderr, "FYI (%s:%s): interrupt when disabled\n", 
		__FILE__, __func__);
      }
    }
    
    // handle all interrupts in bit vector returned by HW
    for (v=0;v<NUM_VEC;v++) {
      if (lrt_pic_unix_ints_test(intrSet, v) && lpic->enabled[v]) {
	numintr--;
	if (lpic->lvecs[v]) {
	  lpic->lvecs[v]();
	} else {
	  fprintf(stderr, "ERROR: %s: spurious interrupt on %d\n", __func__, v);
	}
	if(numintr<=0) break;
      }
    }
  }
  return -1;
}

