#include "../base/include.h"
#include "../base/types.h"
#include "../base/lrtio.h"
#include "../cobj/cobj.h"
#include "sys/trans.h" //FIXME: move EBBTransLSys out of this header
#include "CObjEBB.h"
#include "EBBTypes.h"
#include "MsgMgr.h"
#include "EBBMgrPrim.h"
#include "EBBMemMgr.h"
#include "EBBMemMgrPrim.h"
#include "CObjEBBUtils.h"
#include "CObjEBBRoot.h"
#include "CObjEBBRootShared.h"

#include "EthTypeMgr.h"
#include "EthMgr.h"
#include "EthEBBProto.h"
#include "EBBAssert.h"

#define VERBOSE_PR(...) ( EBB_LRT_printf(__VA_ARGS__) )

#define EBBCALL(id, method, ...) COBJ_EBBCALL(id, method, ##__VA_ARGS__)

CObject(EthEBBProtoTypeMgr) {
  CObjInterface(EthTypeMgr) *ft;
  CObjectDefine(EthEBBProtoPrim) *ebbproto;
  uval rcnt;
} typeMgr;

CObject(EthEBBProtoPrim) {
  CObjInterface(EthEBBProto) *ft;
  EthTypeMgrId tmid;
  CObjectDefine(EthEBBProtoTypeMgr) typeMgr;
  uval scnt;
};

static EBBRC
EthEBBProtoPrim_numReceived(void *_self, uval *num)
{
  EthEBBProtoPrimRef self = _self;

  *num = self->typeMgr.rcnt;
  return EBBRC_OK;
}

static EBBRC
EthEBBProtoPrim_handlePacket(void *_self)
{
  EthEBBProtoPrimRef self = ((EthEBBProtoTypeMgr *)_self)->ebbproto;

  self->typeMgr.rcnt++;
  VERBOSE_PR("EthEbbProtoPrim_handlePacket: called rcnt=%ld scnt=%ld\n", 
	     self->typeMgr.rcnt, self->scnt);
  return EBBRC_OK;
}

CObjInterface(EthEBBProto) EthEBBProtoPrim_ftable = {
  .numReceived = EthEBBProtoPrim_numReceived,
  {
    .handlePacket = EthEBBProtoPrim_handlePacket
  }
};

static inline void 
EthEBBProtoPrimSetFT(EthEBBProtoPrimRef o) 
{ 
  o->ft = &EthEBBProtoPrim_ftable; 
  o->typeMgr.ft = &(EthEBBProtoPrim_ftable.EthTypeMgr_if);
}

EBBRC
EthEBBProtoPrimCreate(EthMgrId ethmgrid, EthEBBProtoId *id) 
{
  EBBRC rc;

  EthEBBProtoPrimRef repRef;
  CObjEBBRootSharedRef rootRef;
  

  EBBPrimMalloc(sizeof(*repRef), &repRef, EBB_MEM_DEFAULT);
  EBBPrimMalloc(sizeof(*rootRef), &rootRef, EBB_MEM_DEFAULT);
  
  CObjEBBRootSharedSetFT(rootRef);
  EthEBBProtoPrimSetFT(repRef);
  
  repRef->typeMgr.rcnt = 0;
  repRef->typeMgr.ebbproto = repRef;  
  repRef->scnt = 0;
  repRef->tmid = NULLId;

  rootRef->ft->init(rootRef, repRef);
  
  rc = EBBAllocLocalPrimId(id);
  EBBRCAssert(rc);

  rc = CObjEBBBind(*id, rootRef); 
  EBBRCAssert(rc);

  EBBPrimMalloc(sizeof(*rootRef), &rootRef, EBB_MEM_DEFAULT);
  CObjEBBRootSharedSetFT(rootRef);
  
  rootRef->ft->init(rootRef, &(repRef->typeMgr));

  rc = EBBAllocLocalPrimId(&(repRef->tmid));
  EBBRCAssert(rc);

  rc = CObjEBBBind(repRef->tmid, rootRef); 
  EBBRCAssert(rc);

  return EBBRC_OK;
}
