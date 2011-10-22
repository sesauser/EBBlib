#include <stdio.h>
#include <sched.h>

/**************************************************************************/
/* This code seems to trigger a bug in the llvm compiler (the default     */ 
/* on OSXLion).  Use /Developer/usr/bin/gcc-4.2 (eg invoke gcc-4.2) and   */
/* things are ok probably related to:                                     */
/* http://llvm.org/bugs/show_bug.cgi?id=7878                              */
/**************************************************************************/

/* dummy pic implementations */
void lrt_pic_set_vector(int i, void *v) { printf("%s: %d: %p\n", __func__, i, v); }
void lrt_pic_loop(void) { sched_yield(); }

typedef int (*hdler)(void *);
hdler func[] = { NULL, NULL, NULL };
void *data[] = { NULL, NULL, NULL };

static void
halt(void)
{
  while (1) lrt_pic_loop();
}


#ifdef __APPLE__
__attribute__((__noinline__, optimize("-O0"))) void lrt_event_setvector(int i)
#else
__attribute__((__noinline__ , optimize("-O0"), __noclone__)) void lrt_event_setvector(int i)
#endif
{
  static const int array[] = { 
    &&V0 - &&V0, 
    &&V1 - &&V0,
    &&V2 - &&V0 
  };

  if (i>=0) {
    lrt_pic_set_vector(i, &&V0 + array[i]);
    return;
  }

 V0: 
    func[0](data[0]);
    halt();
 V1: 
    func[1](data[1]);
    halt();
 V2:
    func[1](data[2]);	
    halt();
}

int
main(int argc, char **argv)
{
  lrt_event_setvector(0);
  lrt_event_setvector(1);
  lrt_event_setvector(2);
  halt();
}


