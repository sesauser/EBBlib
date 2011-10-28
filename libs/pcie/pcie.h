/* Copyright 2011 Karlsruhe Institute of Technology. All rights reserved. */

/* Redistribution and use in source and binary forms, with or without modification, are */
/* permitted provided that the following conditions are met: */

/*    1. Redistributions of source code must retain the above copyright notice, this list of */
/*       conditions and the following disclaimer. */

/*    2. Redistributions in binary form must reproduce the above copyright notice, this list */
/*       of conditions and the following disclaimer in the documentation and/or other materials */
/*       provided with the distribution. */

/* THIS SOFTWARE IS PROVIDED BY BOSTON UNIVERSITY ``AS IS'' AND ANY EXPRESS OR IMPLIED */
/* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND */
/* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BOSTON UNIVERSITY OR */
/* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR */
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR */
/* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF */
/* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/* The views and conclusions contained in the software and documentation are those of the */
/* authors and should not be interpreted as representing official policies, either expressed */
/* or implied, of Karlsruhe Institute of Technology */

#ifndef _PCIE_H_
#define _PCIE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
struct pci_header_global
{
	unsigned short vendorId;
	unsigned short deviceId;
	unsigned short Command;
	unsigned short Status;
	// read only
	unsigned char revisionId;
	unsigned char ProgIF;
	unsigned char Subclass;
	unsigned char ClassCode;

	unsigned char CacheLineSize;
	unsigned char Latency Timer;
	unsigned char HeaderType;
	unsigned char BIST;
};
struct pci_header0
{
	pci_header_global global;
};*/

#define  PCI_EXP_TYPE_ENDPOINT	0x0	/* Express Endpoint */
#define  PCI_EXP_TYPE_LEG_END	0x1	/* Legacy Endpoint */
#define  PCI_EXP_TYPE_ROOT_PORT 0x4	/* Root Port */
#define  PCI_EXP_TYPE_UPSTREAM	0x5	/* Upstream Port */
#define  PCI_EXP_TYPE_DOWNSTREAM 0x6	/* Downstream Port */
#define  PCI_EXP_TYPE_PCI_BRIDGE 0x7	/* PCI/PCI-X Bridge */
#define  PCI_EXP_TYPE_RC_END	0x9	/* Root Complex Integrated Endpoint */
#define  PCI_EXP_TYPE_RC_EC	0x10	/* Root Complex Event Collector */

struct pcie_device
{
	struct pcie_device_internal* _internal;
	unsigned short vendorId;
	unsigned short deviceId;
	unsigned char pcieCap;
	unsigned char msixCap;
	unsigned char type;
	unsigned char defaultIrq;
	unsigned int bar[6];
	unsigned int barSize[6];
	unsigned char barType[6];
	unsigned short msixCount;
	unsigned int* msixTable;
	unsigned long long* msixPBA;
	unsigned long (*read_config)(struct pcie_device* self, unsigned short reg);
	void (*write_config)(struct pcie_device* self, unsigned short reg, unsigned long value);
	unsigned short (*read_config_word)(struct pcie_device* self, unsigned short reg);
	void (*write_config_word)(struct pcie_device* self, unsigned short reg, unsigned short value);
};

typedef int (*pcie_device_callback)(struct pcie_device*, void*);

extern void pcie_init(void);
// callback will be called for every (TODO not allocated) device
extern unsigned int pcie_find_device(pcie_device_callback callback, void* callbackData);
// search for specific
//extern unsigned int pcie_find_device_ById(unsigned short vendorId, unsigned short deviceId, pcie_device_callback callback);

#ifdef __cplusplus
}
#endif

#endif
