/******************************************************************************
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
*****************************************************************************/
#include "../base/include.h"
#include "../base/types.h"
#include "../base/lrtio.h"
#include "../cobj/cobj.h"
#include "EBBTypes.h"
#include "CObjEBB.h"
#include "EBBMgrPrim.h"
#include "EBBMemMgr.h" 
#include "EBBMemMgrPrim.h"
#include "EBBEventMgrPrim.h"
#include "EBBEventMgrPrimImp.h"
#include "EBBCtr.h"
#include "EBBCtrPrim.h"
#include "EBBCtrPrimDistributed.h"
#include "clrBTB.h"
#include "EBB9PClient.h"
#include "EBB9PClientPrim.h"
#include "EBBFile.h"
#include "EBB9PFilePrim.h"
#include "P9FS.h"
#include "CmdMenu.h"
#include "CmdMenuPrim.h"
#include "P9FSPrim.h"
#include "EthTypeMgr.h"
#include "EthMgr.h"
#include "EthMgrPrim.h"
#include "EthEBBProto.h"
#include "EthEBBProtoPrim.h"
#include "EBBAssert.h"

#include <pthread.h>

#define EBBCALL(id, method, ...) COBJ_EBBCALL(id, method, ##__VA_ARGS__)

void 
EBBMgrPrimTest(void)
{
  EBBId id1, id2;
  EBBCtrPrimId c = NULLId;

  EBBRC rc;

  EBB_LRT_printf("0: EBBId_DREF(theEBBMgrPrimId)=%p: ",
		 EBBId_DREF(theEBBMgrPrimId));
  rc = EBBAllocLocalPrimId(&id1);
  EBB_LRT_printf("rc = %ld id1=%p\n", rc, id1);

  EBB_LRT_printf("1: EBBId_DREF(theEBBMgrPrimId)=%p: ", 
		 EBBId_DREF(theEBBMgrPrimId));
  rc = EBBAllocLocalPrimId(&id2);
  EBB_LRT_printf("rc = %ld id2=%p\n", rc, id2);

  //EBB_LRT_printf("ERROR: gtable miss on a local-only EBB\n");
  rc = EBBCALL(c, inc);
  EBB_LRT_printf("NULLId test: rc = %ld\n", rc);
  EBBRCAssert((rc!=EBBRC_OK));  
}


void
EBBMemMgrPrimTest(void)
{
  char *mem;
  EBBPrimMalloc(4, (void **)&mem, EBB_MEM_DEFAULT);
  EBB_LRT_printf("0: mem=%p\n", mem);
  EBBPrimFree(mem);
  EBBPrimMalloc(4, (void **)&mem, EBB_MEM_DEFAULT);
  EBB_LRT_printf("1: mem=%p\n", mem);
  EBBPrimFree(mem);
}

void 
EBBCtrTest(void)
{
  EBBCtrPrimId c;
  EBBCtrPrimDistributedId c2;
  EBBRC rc;
  uval v;

  EBBCtrPrimSharedCreate(&c);
  EBBCtrPrimDistributedCreate(&c2);

  EBB_LRT_printf("id=%p\n", c);
  rc = EC(c)->val(EB(c), &v);
  EBB_LRT_printf("rc=%ld, v=%ld\n", rc, v);

#if 1
  rc = EC(c)->inc(EB(c)); rc = EC(c)->val(EB(c), &v);  
  EBB_LRT_printf("rc=%ld, v=%ld\n", rc, v);

  rc = EC(c)->inc(EB(c)); rc = EC(c)->val(EB(c), &v);  
  EBB_LRT_printf("rc=%ld, v=%ld\n", rc, v);

  rc = EC(c)->inc(EB(c)); rc = EC(c)->val(EB(c), &v);  
  EBB_LRT_printf("rc=%ld, v=%ld\n", rc, v);

  rc = EC(c)->dec(EB(c)); rc = EC(c)->val(EB(c), &v);  
  EBB_LRT_printf("rc=%ld, v=%ld\n", rc, v);
#endif

  printf("id=%p\n", c2);
  rc = EC(c2)->val(EB(c2), &v);
  printf("rc=%ld, v=%ld\n", rc, v);

#if 1
  rc = EC(c2)->inc(EB(c2)); rc = EC(c2)->val(EB(c2), &v);  
  printf("rc=%ld, v=%ld\n", rc, v);

  rc = EC(c2)->inc(EB(c2)); rc = EC(c2)->val(EB(c2), &v);  
  printf("rc=%ld, v=%ld\n", rc, v);

  rc = EC(c2)->inc(EB(c2)); rc = EC(c2)->val(EB(c2), &v);  
  printf("rc=%ld, v=%ld\n", rc, v);

  rc = EC(c2)->dec(EB(c2)); rc = EC(c2)->val(EB(c2), &v);  
  printf("rc=%ld, v=%ld\n", rc, v);
#endif

#if 0
  sval i;
  EBBCtrPrimRef r = EB(c);
  EBBRC (*f) (void *_self) = r->ft->inc;
  EBBRC (**ftbl) (void *_self) = &f;
  // EBBRC (**ftbl) (void *_self) = r->ft;

  for (i=0; i<10000000; i++ ) {

#if 0
    clrBTB();
#endif

/*     for (j=0; j<2; j++) { */
#if 0
      rc = inc(r);
#endif
      
#if 0
      rc = f(r);
#endif
      
#if 0
      rc = ftbl[0](r);
#endif
      
#if 0
      rc = r->ft->inc(r);
#endif
      
#if 0
      rc = EC(c)->inc(EB(c)); 
#endif

      EBBRCAssert(rc);
/*     } */
  }

  rc = EC(c)->val(EB(c), &v);
  EBB_LRT_printf("i=%ld j=%ld rc=%ld, v=%ld\n", i, rc, v);
#endif
}

#if 0 				/* FIXME: disabled 9P */
void
EBB9PClientTest(char *address, char *path)
{
  EBB9PClientId p;
  EBBRC rc;
  IxpCFid  *fd;
  char buf[80];
  sval n;

  EBB_LRT_printf("EBB9PClientTest: BEGIN: address=%s path=%s\n", address, path);

  EBB9PClientPrimCreate(&p);

  rc = EBBCALL(p, mount, address);
  EBBRCAssert(rc);

  rc = EBBCALL(p, open, path, P9_OREAD, &fd);
  EBBRCAssert(rc);

  rc = EBBCALL(p, read, fd, buf, 80, &n); 
  EBBRCAssert(rc);  
  buf[n] = 0;
  EBB_LRT_printf("%s\n", buf);
  
  EBB_LRT_printf("EBB9PClientTest: END\n");
}

void 
P9FSTest(char *address)
{
  P9FSid fs;
  CmdMenuId menu;

  EBBRC rc;

  EBB_LRT_printf("P9FSTest: BEGIN: address=%s\n", address);

  CmdMenuPrimCreate(&menu);

  P9FSPrimCreate(&fs, menu);

  rc = EBBCALL(fs, serverloop, address);
  
  EBBRCAssert(rc);
}
#endif

void
EthTest(void)
{
  EthMgrId ethmgr;
#if 0 
  EthEBBProtoId proto;
#endif

  // turn this on when ready to start integrating
  EthMgrPrimCreate(&ethmgr);

#if 0
  EthEBBProtoPrimCreate(ethmgr, &proto);
#endif

}

#if 0
CObject(Boot) {
  CObjInterface(MsgHdlr) *ft;
  uval numEventLocations;
  uval numMemDomains;
};
  
typedef BootRef *BootId;

static EBBRC 
Boot_main(void *_self)
{
  __attributed__ ((unused)) BootRef self = (BootRef)_self

  // gather boot facts
  
  // now update the core EBBs with this facts

  // We are now ready to get going for real

  // the last thing we do is start up some external connection 
  // in this case we are hardcoding the start up of an ethernet device
  EthMgrInit();
  return EBBRC_OK;

}

CObjInterface(MsgHdlr) Boot_ftable = {
  .msg0 = boot_main;
};

static void
TriggerBootEvent(void)
{
  EBBRC rc;
  static Boot bootRep;
  static CObjEBBRootShared bootRoot;
  BootId bid;
  uval no;

  CObjEBBRootSsharedSetFT(bootRoot);
  bootRep.ft = &Boot_ftable;  
  bootRoot.ft->init(&bootRep);
  rc = EBBAllocLocalPrimId(&bid);
  EBBRCAssert(rc);
  rc = CObjEBBBind(bid, &bootRoot); 
  EBBRCAssert(rc);

  EBBCALL(theEBBEventMgrPrimId, allocEventNo, &no);
  EBBCALL(theEBBEventMgrPrimId, registerHandler, no, bid, mg);

}
#endif
