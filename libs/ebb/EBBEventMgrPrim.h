#ifndef __EVENTMGR_H__
#define __EVENTMGR_H__

/* 
 * All event handling ebbs must conform to these types
 * Any existing ebbs that want to handle events must be
 * frontended by an event handler EBB
 */
CObjInterface(EBBEventHandler) {
  EBBRC (*handleEvent) (void *_self);
  EBBRC (*init) (void *_self);
};

CObject(EBBEventHandler) {
  CObjInterface(EBBEventHandler) *ft;
};

typedef EBBEventHandlerRef *EBBEventHandlerId;

CObjInterface(EBBEventMgrPrim) {
  EBBRC (*registerHandler) (void *_self, int eventNo, EBBEventHandlerId handler);
  EBBRC (*triggerHandler) (void *_self, int eventNo);
  EBBRC (*allocEventNo) (void *_self, int *eventNoPtr); /* this is global to the system */
  EBBRC (*init) (void *_self);
};

CObject(EBBEventMgrPrim) {
  CObjInterface(EBBEventMgrPrim) *ft;
};

typedef EBBEventMgrPrimRef *EBBEventMgrPrimId;
// the ID of the one and only event manager
extern EBBEventMgrPrimId theEBBEventMgrPrimId;
#endif
