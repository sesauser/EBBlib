#include <string.h>
#include "../base/include.h"
#include "../base/types.h"
#include "../cobj/cobj.h"
#include "../base/lrtio.h"
#include "EBBTypes.h"
#include "CObjEBB.h"
#include "MsgMgr.h"
#include "EBBMgrPrim.h"
#include "CObjEBBUtils.h"
#include "CObjEBBRoot.h"
#include "CObjEBBRootShared.h"
#include "EBBEventMgrPrim.h"
#include "EBBEventMgrPrimImp.h"

/*
 * We define these in .c file, since EBBCALL may be different for
 * c++..
 */
#define EBBCALL(id, method, ...) COBJ_EBBCALL(id, method, ##__VA_ARGS__)

/* 
 * This is the one and only implementation we have today of EventMgrPrim
 Doing:
 o do the wrapper, simple unit test that invokes local object
 o look at how node number is assigned in one of the distributed
   implementations of object


 Comments:
 o EventNO will be allocated across all nodes in the system. We need
   to make this a sparse space, so we will eventually need to switch
   to a hash function for storing the per-event information.
 o There are no reserved events, i.e., we assume that ethernet may
   well come from a different NIC, generate a different interrupt on
   each node...
 o Discuss this, first implementation... right now, purely local.  The
   question is, the eventmgr purely local per node, or do we have
   global name space. If its global, we are basically doing the same
   thing for this as we are with EBBs, i.e. hash table, partitioned
   space go to a node to get mapping of eventno to ebbid...
 */ 

typedef struct  {
  EBBEventHandlerId handId;
} HandlerInfoStruc;

#define MAXEVENTS 256

CObject(EBBEventMgrPrimImp){
  CObjInterface(EBBEventMgrPrim) *ft;
  int allocatedEventNo;
  HandlerInfoStruc handerInfo[MAXEVENTS];
};


static EBBRC
EventMgrPrim_registerHandler(void *_self, int eventNo, EBBEventHandlerId handler)
{
  EBBEventMgrPrimImpRef self = _self;
  if ( (eventNo >= MAXEVENTS) || (eventNo<0) ){
    return EBBRC_BADPARAMETER;
  };
  
  if (self->handerInfo[eventNo].handId != NULL) {
    // for now, if its not null, assume error, should we ever be able
    // to change the handler for an event?
    return EBBRC_BADPARAMETER;
  };
    
  /* 
   * for now, just allocate in local array, not replicated to other nodes
   */
  self->handerInfo[eventNo].handId = handler;
  return 0;
}

/*
 * First implementation will just trigger event on local node, we are
 * not sharing space right now...
 */
static EBBRC 
EventMgrPrim_triggerHandler(void *_self, int eventNo)
{
  EBBRC rc;
  EBBEventMgrPrimImpRef self = _self;
  rc = EBBCALL(self->handerInfo[eventNo].handId, handleEvent);
  return rc;
}

/* 
 * Allocate event number; this is global across all nodes, we assume
 * any functionality that invokes an event can generate a unique
 * number for different machines.
 */
static EBBRC 
EventMgrPrim_allocEventNo(void *_self, int *eventNoPtr)
{
  EBBEventMgrPrimImpRef self = _self;
  int eventNo = 0;
  eventNo = self->allocatedEventNo;
  if ( (eventNo >= MAXEVENTS) || (eventNo<0) ){
    return EBBRC_OUTOFRESOURCES;
  };
  
  self->allocatedEventNo++;
  *eventNoPtr = eventNo;
  return EBBRC_OK;
}

static EBBRC 
EventMgrPrim_init(void *_self)
{
  EBBEventMgrPrimImpRef self = _self;
  self->allocatedEventNo = 0;
  bzero(&self->handerInfo[0], MAXEVENTS*sizeof(HandlerInfoStruc));
  return 0;
}

CObjInterface(EBBEventMgrPrim) EBBEventMgrPrimImp_ftable = {
  .registerHandler = EventMgrPrim_registerHandler, 
  .triggerHandler = EventMgrPrim_triggerHandler, 
  .allocEventNo = EventMgrPrim_allocEventNo, 
  .init = EventMgrPrim_init
};

static void
EBBEventMgrPrimSetFT(EBBEventMgrPrimImpRef o)
{
  o->ft = &EBBEventMgrPrimImp_ftable;
}

EBBEventMgrPrimId theEBBEventMgrPrimId;

EBBRC
EBBEventMgrPrimImpInit()
{
  EBBRC rc;
  static EBBEventMgrPrimImp theRep;
  static CObjEBBRootShared theRoot;
  EBBEventMgrPrimImpRef repRef = &theRep;
  CObjEBBRootSharedRef rootRef = &theRoot;

  // setup function tables
  CObjEBBRootSharedSetFT(rootRef);
  EBBEventMgrPrimSetFT(repRef);

  // setup my representative and root
  repRef->ft->init(repRef);
  // shared root knows about only one rep so we 
  // pass it along for it's init
  rootRef->ft->init(rootRef, &theRep);

  rc = EBBAllocLocalPrimId(&theEBBEventMgrPrimId);
  //  EBBRCAssert(rc);

  rc = CObjEBBBind(theEBBEventMgrPrimId, rootRef); 
  //  EBBRCAssert(rc);

  return EBBRC_OK;
};


static EBBRC 
EventHandler_handleEvent(void *_self)
{
  EBB_LRT_printf("Wheeeee Handler Invoked\n");
  return 0;
};

static EBBRC
EventHandler_init(void *_self)
{
  return 0;
};


static CObjInterface(EBBEventHandler) EBBEventHandler_ftable = {
  .handleEvent = EventHandler_handleEvent,
  .init = EventHandler_init
};


// change to dynamically allocate this
static EBBEventHandlerId
CreateTestHandler()
{
  EBBRC rc;
  static EBBEventHandler theRep;
  EBBEventHandlerRef repRef = &theRep;
  static CObjEBBRootShared theRoot;
  CObjEBBRootSharedRef rootRef = &theRoot;
  EBBEventHandlerId handlerId; 

  // setup function tables
  CObjEBBRootSharedSetFT(rootRef);
  repRef->ft = &EBBEventHandler_ftable;

  // setup my representative and root
  repRef->ft->init(repRef);
  // shared root knows about only one rep so we 
  // pass it along for it's init
  rootRef->ft->init(rootRef, &theRep);

  rc = EBBAllocLocalPrimId(&handlerId);
  //  EBBRCAssert(rc);

  rc = CObjEBBBind(handlerId, rootRef); 
  //  EBBRCAssert(rc);

  return handlerId;
}

/* move to a seperate file eventually, should not be imp specific */
void EBBEventMgrPrimImpTest(void) 
{
  EBBEventHandlerId handlerId;
  int eventNo;

  handlerId = CreateTestHandler();

  EBBCALL(theEBBEventMgrPrimId, allocEventNo, &eventNo);
  EBBCALL(theEBBEventMgrPrimId, registerHandler, eventNo, handlerId);
  EBBCALL(theEBBEventMgrPrimId, triggerHandler, eventNo);
}
