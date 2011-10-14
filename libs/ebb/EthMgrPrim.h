#ifndef __ETH_MGR_PRIM_H__
#define __ETH_MGR_PRIM_H__

#define ETHERNET_ADDR_LEN 6

struct EthernetHeader {
  uval8  dest[ETHERNET_ADDR_LEN];
  uval8  src[ETHERNET_ADDR_LEN];
  uval16 type;
};

CObjInterface(EthMgr) {
  EBBRC (*init)  (void *_self);
  EBBRC (*bind)  (void *_self, uval16 type, EthTypeMgrId id);
};

CObject(EthMgr) {
  CObjInterface(EthMgr) *ft;
};

typedef EthMgrPrimRef *EthMgrPrimId;

#endif  // __ETH_MGR_PRIM_H__
