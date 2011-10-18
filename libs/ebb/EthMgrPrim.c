#include "../base/include.h"
#include "../base/types.h"
#include "../base/lrtio.h"
#include "../cobj/cobj.h"
#include "sys/trans.h" //FIXME: move EBBTransLSys out of this header
#include "CObjEBB.h"
#include "EBBTypes.h"
#include "EBBAssert.h"
#include "MsgMgr.h"
#include "EBBMgrPrim.h"
#include "EBBMemMgr.h"
#include "EBBMemMgrPrim.h"
#include "CObjEBBUtils.h"
#include "CObjEBBRoot.h"
#include "CObjEBBRootShared.h"

#include "EthTypeMgr.h"
#include "EthMgr.h"
#include "EthMgrPrim.h"

#include __LRTINC(misc.h)
#include __LRTINC(pic.h)
#include __LRTINC(ethlib.h)

#define VERBOSE_PR(...) ( EBB_LRT_printf(__VA_ARGS__) )

#define EBBCALL(id, method, ...) COBJ_EBBCALL(id, method, ##__VA_ARGS__)

#define NUMETHTYPES (1<<(sizeof(uval16) * 8))

CObject(EthMgrPrim) {
  CObjInterface(EthMgr) *ft;
  EthTypeMgrId typeMgrs[NUMETHTYPES];
};


static EBBRC
EthMgrPrim_init(void *_self)
{
  return EBBRC_GENERIC_FAILURE;
}

static EBBRC
EthMgrPrim_bind(void *_self, uval16 type, EthTypeMgrId id)
{
  return EBBRC_GENERIC_FAILURE;
}

CObjInterface(EthMgr) EthMgrPrim_ftable = {
  .init = EthMgrPrim_init,
  .bind = EthMgrPrim_bind
};

static inline void 
EthMgrPrimSetFT(EthMgrPrimRef o) 
{ 
  o->ft = &EthMgrPrim_ftable; 
}

static void
pic_handler(uval val) 
{
  //  fprintf(stderr, "%s: %ld\n", __func__, val);
  ethlib_nic_readpkt();
}

EBBRC
EthMgrPrimCreate(EthMgrId *id) 
{
  EBBRC rc;
  EthMgrPrimRef repRef;
  CObjEBBRootSharedRef rootRef;
  lrt_pic_src nicisrc;
  uval vec;

  EBBPrimMalloc(sizeof(*repRef), &repRef, EBB_MEM_DEFAULT);
  EBBPrimMalloc(sizeof(*rootRef), &rootRef, EBB_MEM_DEFAULT);
  
  CObjEBBRootSharedSetFT(rootRef);
  EthMgrPrimSetFT(repRef);

  bzero(repRef->typeMgrs, sizeof(repRef->typeMgrs));

  rootRef->ft->init(rootRef, repRef);
  
  rc = EBBAllocLocalPrimId(id);
  EBBRCAssert(rc);

  rc = CObjEBBBind(*id, rootRef); 
  EBBRCAssert(rc);

  // this needs to be done somewhere else
  rc = lrt_pic_init();
  EBBRCAssert(rc);

  rc = lrt_pic_allocvec(&vec);
  EBBRCAssert(rc);

  rc = ethlib_nic_init("eth1", &nicisrc);
  EBBRCAssert(rc);

  rc = lrt_pic_mapvec(nicisrc, vec, pic_handler);
  EBBRCAssert(rc);

  lrt_pic_loop();
  
  return EBBRC_OK;
}
