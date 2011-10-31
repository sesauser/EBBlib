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
  EBBRC (*registerHandler) (void *_self, uval eventNo,
			    EBBEventHandlerId handler, 
			    uval isrc);
  EBBRC (*allocEventNo) (void *_self, uval *eventNoPtr);
};

CObject(EBBEventMgrPrim) {
  CObjInterface(EBBEventMgrPrim) *ft;
};



typedef EBBEventMgrPrimRef *EBBEventMgrPrimId;
// the ID of the one and only event manager
extern EBBEventMgrPrimId theEBBEventMgrPrimId;

/*
 * You need to be able to get the event location of the 
 * node you are running on.  I am not making this a 
 * function on EventMgr, but rather a global function
 * so its clear that its the EL of the current core and not
 * the EL of the rep of the EventMgr
 */
typedef uval EvntLoc;
extern EvntLoc  EBBEventMgrPrim_GetMyEL();

#endif
