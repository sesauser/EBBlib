#ifndef __ETH_EBB_PROTO_H__
#define __ETH_EBB_PROTO_H__

CObjInterface(EthEBBProto) {
  EBBRC (*numReceived)(void *_self, uval *num);
  CObjImplements(EthTypeMgr);
};

CObject(EthEBBProto) {
  CObjInterface(EthEBBProto) *ft;
};

typedef EthEBBProtoRef * EthEBBProtoId;

#endif  // __ETH_EBB_PROTO_H__
