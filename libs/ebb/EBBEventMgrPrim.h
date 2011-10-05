#ifndef __EVENTMGR_H__
#define __EVENTMGR_H__

// All event handling ebbs must conform to these types
// Any existing ebbs that want to handle events must be
// frontended by an event handler EBB
CObjInterface(EBBEventHandler) {
  EBBRC (*handleEvent) (void *_self);
};

CObject(EBBEventHandler) {
  CObjInterface(EBBEventHandler);
};

/*
FIXME:
o put a typedef for location somewhere
*/

CObjInterface(EBBEventMgrPrim) {
  EBBRC (*register) (void *_self, int eventNo, EventHandlerRef handler);
  /* 
   * This is the first of a series of trigger operations, some will be for all 
   * nearby nods, some will be one of the nearby nodes, this one explicity defines
   * a node on an event to be triggered, the node is responsible for defining the core
   * within that node.
   */
  EBBRC (*trigger) (void *_self, int location, int eventNo);
  EBBRC (*allocEventNo) (void *_self, int *eventNo); /* this is global to the system */
  EBBRC (*init) (void *_self);
};

CObject(EBBEventMgrPrim) {
  CObjInterface(EBBEventMgr);
};

extern EBBEventMgrPrimRef *theEBBEventMgrPrimId;
#endif
