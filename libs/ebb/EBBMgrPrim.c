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
#include "../base/lrtio.h"
#include "../cobj/cobj.h"
#include "sys/trans.h" //FIXME: move EBBTransLSys out of this header
#include "EBBTypes.h"
#include "CObjEBB.h"
#include "CObjEBBRoot.h"
#include "EBBMgrPrim.h"
#include "CObjEBBUtils.h"
#include "EBBAssert.h"
#include "EBB9PClient.h"
#include "EBB9PClientPrim.h"
#include "EBBFile.h"
#include "EBB9PFilePrim.h"
#include "BootInfo.h"

#include __LRTINC(misc.h)

#define EBBCALL(id, method, ...) COBJ_EBBCALL(id, method, ##__VA_ARGS__)

// JA KLUDGE
#define MAXNODES 1024

#define STRLEN 160

#define panic() (*(uval *)0)

void *NULLId;

typedef struct EBBTransGSysStruct {
  EBBGTrans *gTable;
  uval pages;
} EBBTransGSys;

extern uval EBBNodeId;

CObjInterface(EBBMgrPrimRoot) 
{
  CObjImplements(CObjEBBRoot);
  void (*init)(void *_self);
};

CObject(EBBMgrPrimRoot) 
{
  CObjInterface(EBBMgrPrimRoot) *ft;
  EBBMgrPrim reps[EBB_TRANS_MAX_ELS];
  EBBTransLSys EBBMgrPrimLTrans[EBB_TRANS_MAX_ELS];
  EBBTransGSys gsys;  
  struct BootInfo binfo;
};

static EBBRC
AllocLocalId (void *_self, void **id) {
  EBBMgrPrimRef self = (EBBMgrPrimRef)_self;
  *id = (void *)EBBIdAllocLocal(self->lsys);
  return EBBRC_OK;
}
static EBBRC
AllocGlobalId (void *_self, void **id) {
  EBBMgrPrimRef self = (EBBMgrPrimRef)_self;
  if (!isGlobalSetup(self->lsys)) {
    if (EBBNodeId != 0) {
      SetupGlobal(self->lsys, EBBNodeId);
    } else {
      *id = NULL;
      return EBBRC_GENERIC_FAILURE;
    }
  }

  *id = (void *)EBBIdAllocGlobal(self->lsys);
  return EBBRC_OK;
}

static EBBRC
FreeId (void *_self, EBBId id) {
  EBBMgrPrimRef self = (EBBMgrPrimRef)_self;
  EBBIdFree(self->lsys, id);
  return EBBRC_OK;
}

static EBBRC
BindId (void *_self, EBBId id, EBBMissFunc mf, EBBMissArg arg) {
  //  EBBMgrPrimRef self = (EBBMgrPrimRef)_self;
  EBBIdBind(id, mf, arg);
  return EBBRC_OK;
}

static EBBRC
BindGlobalId (void *_self, EBBId id, EBBMissFunc mf, EBBMissArg arg,
	      EBBMissFunc globalMF) {
  EBBIdBindGlobal(id, mf, arg, globalMF);
  return EBBRC_OK;
}

static EBBRC
UnBindId (void *_self, EBBId id, EBBMissFunc *mf, EBBMissArg *arg) {
  //  EBBMgrPrimRef self = (EBBMgrPrimRef)_self;
  EBBIdUnBind(id, mf, arg);
  return EBBRC_OK;
}

static CObjInterface(EBBMgrPrim) EBBMgrPrim_ftable = {
  .AllocLocalId = AllocLocalId, 
  .AllocGlobalId = AllocGlobalId,
  .FreeId = FreeId, 
  .BindId = BindId, 
  .BindGlobalId = BindGlobalId,
  .UnBindId = UnBindId, 
};

EBBRC
globalMissHandler(uval arg0, uval arg1, uval arg2, uval arg3,
		  uval arg4, uval arg5, uval arg6, uval arg7,
		  uval *rcode) {
  EBBId id = (EBBId)arg0;
  EBB_LRT_printf("global miss received with id = %lX\n", (uval)id);
  EBBGTrans *gt = EBBIdToGTrans(id);
  *rcode = (uval)gt->globalMF;
  return EBBRC_OK;
}

static EBBRC EBBMgrPrimERRMF (void *_self, EBBLTrans *lt,
			      FuncNum fnum, EBBMissArg arg) {
  if (isLocalEBB(EBBLTransToGTrans(lt))) {
    EBB_LRT_printf("ERROR: gtable miss on a local-only EBB\n");
  } else if (getLTransNodeId(lt) == EBBNodeId) {
    EBB_LRT_printf("ERROR: gtable miss on a global EBB but we are the home node!\n");
  } 
  return EBBRC_GENERIC_FAILURE;
}

static uval
EBBMgrPrimRoot_handleMiss(void *_self, void *obj, EBBLTrans *lt,
				 FuncNum fnum)
{
  EBBMgrPrimRootRef self = _self;
  EBBMgrPrimRef rep;
  int numGTransPerEL;
 
  numGTransPerEL = self->gsys.pages * EBB_TRANS_PAGE_SIZE / 
    sizeof(EBBGTrans) / EBB_TRANS_MAX_ELS;

  rep = &(self->reps[EBBMyEL()]);

  rep->lsys = &(self->EBBMgrPrimLTrans[EBBMyEL()]);
  rep->lsys->localGTable = &(self->gsys.gTable[numGTransPerEL * EBBMyEL()]);
  rep->lsys->localLTable = EBBGTransToLTrans(rep->lsys->localGTable);
  rep->lsys->localFree = NULL;
  rep->lsys->localNumAllocated = 0;
  rep->lsys->localSize = numGTransPerEL;
  rep->lsys->globalGTable = NULL;
  rep->lsys->globalLTable = NULL;
  rep->lsys->globalFree = NULL;
  rep->lsys->globalNumAllocated = 0;
  rep->lsys->globalSize = 0;
  rep->myRoot = self;
  rep->ft = &EBBMgrPrim_ftable;
  
  *(void **)obj = rep;
  return EBBRC_OK;
}



static void
EBBMgrPrimRoot_init(void *_self)
{
  EBBMgrPrimRootRef self = _self;

  EBB_Trans_Mem_Init();
  self->gsys.pages = EBB_TRANS_NUM_PAGES;
  EBB_Trans_Mem_Alloc_Pages(self->gsys.pages, (uval8 **)&self->gsys.gTable);
  theERRMF = EBBMgrPrimERRMF;
  initGTable(self->gsys.gTable, self->gsys.pages);
  initAllLTables(EBBGTransToId(self->gsys.gTable), self->gsys.pages);
  bzero(&(self->binfo), sizeof(self->binfo));
}



static CObjInterface(EBBMgrPrimRoot) EBBMgrPrimRoot_ftable = {
  { .handleMiss = EBBMgrPrimRoot_handleMiss },
  .init = EBBMgrPrimRoot_init,
};
				     
//FIXME: have to statically allocate these because there is
//       no memory manager

#if 0
static EBBRC EBBMgrPrimMF (void *_self, EBBLTrans *lt,
		    FuncNum fnum, EBBMissArg arg) {
  EBBTransGSys gsys = *(EBBTransGSys *)arg;
  EBBMgrPrimRef ref = &EBBMgrPrimLObjs[EBBMyEL()];
  EBBMgrPrimRepInit(lt, ref, gsys);
  EBBCacheObj(lt, ref);
  *(EBBMgrPrimRef *)_self = ref;
  return EBBRC_OK;
}
#endif

// declarations of externals
EBBMgrPrimId theEBBMgrPrimId;

void EBBMgrPrimInit() {
  static EBBMgrPrimRoot theEBBMgrPrimRoot = { .ft = &EBBMgrPrimRoot_ftable };
  EBBRC rc;
  EBBId id;

  theEBBMgrPrimRoot.ft->init(&theEBBMgrPrimRoot);

  // manually binding the EBBMgrPrim in
  theEBBMgrPrimId = (EBBMgrPrimId)
    EBBGTransToId(theEBBMgrPrimRoot.gsys.gTable);

  EBBIdBind((EBBId) theEBBMgrPrimId, 
	    CObjEBBMissFunc,                
	    (EBBMissArg)&theEBBMgrPrimRoot);

  // do an alloc to account for manual binding 
  rc = EBBAllocLocalPrimId(&id);

  EBBRCAssert(rc);
  EBBAssert(id == (EBBId)theEBBMgrPrimId);

  // do an alloc to account for manual binding 
  rc = EBBAllocLocalPrimId(&NULLId);

  EBBRCAssert(rc);

}

// FIXME: MISC external declartions
EBBMissFunc theERRMF;
struct EBB_Trans_Mem EBB_Trans_Mem;
