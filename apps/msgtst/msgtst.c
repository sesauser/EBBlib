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
#include <stdint.h>

#include <l0/lrt/types.h>
#include <l0/cobj/cobj.h>
#include <lrt/io.h>
#include <l0/lrt/pic.h>
#include <l0/lrt/trans.h>
#include <l0/types.h>
#include <l0/sys/trans.h>
#include <lrt/assert.h>
#include <l0/cobj/CObjEBB.h>
#include <l0/EBBMgrPrim.h>
#include <l0/cobj/CObjEBBUtils.h>
#include <l0/cobj/CObjEBBRoot.h>
#include <l0/cobj/CObjEBBRootMulti.h>
#include <l0/cobj/CObjEBBRootMultiImp.h>
#include <l0/EventMgrPrim.h>
#include <l0/EventMgrPrimImp.h>
#include <l0/lrt/pic.h>
#include <l0/MemMgr.h>
#include <l0/MemMgrPrim.h>
#include <l1/MsgMgr.h>
#include <l1/MsgMgrPrim.h>
#include <lrt/ulnx/boot_info.h>

// write a simple message handler
// make call to the message handler
// figure out number of cores, and send messages between them

typedef struct test_message_struct {
    int val1;
    int val2;
}test_message;

static EBBRC 
MsgHandlerTst_msg0(MsgHandlerRef _self)
{
  // ack that we are handing interrupt
  EBB_LRT_printf("%s: got message\n", __func__);
  return EBBRC_OK;
};
static EBBRC 
MsgHandlerTst_msg1(MsgHandlerRef _self, uintptr_t a1)
{
  // ack that we are handing interrupt
  EBB_LRT_printf("%s: got message \"%o\" at Event Location %o\n", __func__, (unsigned int)a1, (unsigned int)MyEL());
  return EBBRC_OK;
};
static EBBRC 
MsgHandlerTst_msg2(MsgHandlerRef _self, uintptr_t a1, uintptr_t a2)
{
  test_message * t;

  t=(test_message *)a2;

  // ack that we are handing interrupt
  EBB_LRT_printf("%s: got message \"%o\" at Event Location %o and test pointer struct has members %i and %i\n", __func__, (unsigned int)a1, (unsigned int)MyEL(), t->val1, t->val2);
  return EBBRC_OK;
};
static EBBRC 
MsgHandlerTst_msg3(MsgHandlerRef _self, uintptr_t a1, uintptr_t a2, 
		   uintptr_t a3)
{
  // ack that we are handing interrupt
  EBB_LRT_printf("%s: got message\n", __func__);
  return EBBRC_OK;
};


CObject(MsgHandlerTst) {
  CObjInterface(MsgHandler) *ft;
  CObjEBBRootMultiRef theRoot;	
};


static CObjInterface(MsgHandler) MsgHandlerTst_ftable = {
  .msg0 = MsgHandlerTst_msg0,
  .msg1 = MsgHandlerTst_msg1,
  .msg2 = MsgHandlerTst_msg2,
  .msg3 = MsgHandlerTst_msg3,
};

static EBBRep *
MsgHandlerTst_createRep(CObjEBBRootMultiRef _self) {
  MsgHandlerTstRef repRef;
  EBBPrimMalloc(sizeof(*repRef), &repRef, EBB_MEM_DEFAULT);
  repRef->ft = &MsgHandlerTst_ftable;
  repRef->theRoot = _self;
  return (EBBRep *)repRef;
}

static MsgHandlerId
InitMsgHandlerTst()
{
  CObjEBBRootMultiImpRef rootRef;
  MsgHandlerId id;
  static MsgHandlerId theMsgHandlerTstId=0;

  if (__sync_bool_compare_and_swap(&theMsgHandlerTstId, (MsgHandlerId)0,
				   (MsgHandlerId)-1)) {
    CObjEBBRootMultiImpCreate(&rootRef, MsgHandlerTst_createRep);
    id = (MsgHandlerId)EBBIdAlloc();
    EBBAssert(id != NULL);

    EBBIdBind((EBBId)id, CObjEBBMissFunc, (EBBMissArg) rootRef);
    theMsgHandlerTstId = id;
  } else {
    while (((volatile uintptr_t)theMsgHandlerTstId)==-1);
  }
  return theMsgHandlerTstId;
};


EBBRC ebbmain(void)
{
  MsgHandlerId id = InitMsgHandlerTst();
  EvntLoc i, el; 
  test_message * test;
  EBBRC rc;

  rc = EBBPrimMalloc(sizeof(*test), &test, EBB_MEM_DEFAULT);
  EBBRCAssert(rc);

  test->val1=101;
  test->val2=202;
  
  el=MyEL();

  for (i=0; i<get_boot_core_count();i++) {
	  if(i!=el) {	
// bogus call to test IPI to msgmgr
      EBB_LRT_printf("We are at event location %o about to send message to event location %o\n", (unsigned int)el, (unsigned int)i);
//      COBJ_EBBCALL(theMsgMgrId, msg0, i, id);
//      COBJ_EBBCALL(theMsgMgrId, msg1, i, id, el);
        COBJ_EBBCALL(theMsgMgrId, msg2, i, id, el, (uintptr_t)test);
//      COBJ_EBBCALL(theMsgMgrId, msg3, i, id, 1, 2, 3);

/*
	  	while(COBJ_EBBCALL(theMsgMgrId, msg0, i, id)==EBBRC_NOREP) { sleep(1); }
  		while(COBJ_EBBCALL(theMsgMgrId, msg1, i, id, 1)==EBBRC_NOREP) { sleep(1); }
  		while(COBJ_EBBCALL(theMsgMgrId, msg2, i, id, 1, 2)==EBBRC_NOREP) { sleep(1); }
  		while(COBJ_EBBCALL(theMsgMgrId, msg3, i, id, 1, 2, 3)==EBBRC_NOREP) { sleep(1); }
*/
	   }
  }
  return EBBRC_OK;
}
