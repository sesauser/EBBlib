#include "EBBEventMgrh.h"

/* 
 * this is the one and only implementation we have today of EventMgrPrim
 */ 

struct handlerInfo {
  EBBEventHandlerRef hand;
};

const int numEvents=256;
const int reservedEvents=256;

CObject(EBBEventMgrPrimImp){
  CObjInterface(EBBEventMgr);
  int allocated_eventNo;
  struct handlerInfo hander[numEvents];
};


static EBBRC
register(void *_self, int eventNo, EventHandlerRef handler)
{
  EBBEventMgrPrimImp self = _self;
  self->handler[eventNo] = handler;
  return 0;
}


static EBBRC 
trigger(void *_self, int location, int eventNo)
{
}


/*
 * We need to allocate the number across all nodes in the system
 * so we need to either partition the node space, or have a single implementation
 * we will partition, so we need to eventually change implementation to be a hash
 * function rather than an array. 
 */
static EBBRC 
allocEventNo(void *_self, int *eventNo)
{
}

static EBBRC init(void *_self)
{
  
}
