#include "infiniband_internal.h"
#include "../pcie/pcie.h"
#include <l4io.h>
#include "verbs.h"
//drivers
#include "mlx4.h"

// user defines
#define INFI_LOG printf
#define INFI_PREFIX "INFI: "
#define INFI_MAX_DEVICES 1

static struct Device g_Device[INFI_MAX_DEVICES];
static unsigned int g_DeviceCount = 0;

static struct ibv_device ibv_devices[INFI_MAX_DEVICES];
static struct ibv_device* ibv_devicesPtr[INFI_MAX_DEVICES];

static void* memcpy(void* dst, const void* src, unsigned int len)
{
	unsigned char *d = (unsigned char *) dst;
	unsigned char *s = (unsigned char *) src;

	while (len-- > 0)
		*d++ = *s++;

	return dst;
}

static int infiniband_pcie_device_callback(struct pcie_device* device, void* data)
{
	switch (device->vendorId)
	{
	case 0x15B3:		// Mellanox
		switch (device->deviceId)
		{
		case 0x6340: // MT25408 "Hermon" SDR
		case 0x634a: // MT25408 "Hermon" DDR
		case 0x6354: // MT25408 "Hermon" QDR
		case 0x6732: // MT25408 "Hermon" DDR PCIe gen2
		case 0x673c: // MT25408 "Hermon" QDR PCIe gen2
		case 0x6368: // MT25408 "Hermon" EN 10GigE
		case 0x6750: // MT25408 "Hermon" EN 10GigE PCIe gen2
		case 0x6372: // MT25458 ConnectX EN 10GBASE-T 10GigE
		case 0x675a: // MT25458 ConnectX EN 10GBASE-T+Gen2 10GigE
		case 0x6764: // MT26468 ConnectX EN 10GigE PCIe gen2
		case 0x6746: // MT26438 ConnectX EN 40GigE PCIe gen2 5GT/s
		case 0x676e: // MT26478 ConnectX2 40GigE PCIe gen2
		case 0x1002: // MT25400 Family [ConnectX-2 Virtual Function]
		case 0x1003: // MT27500 Family [ConnectX-3]
		case 0x1004: // MT27500 Family [ConnectX-3 Virtual Function]
		case 0x1005: // MT27510 Family
		case 0x1006: // MT27511 Family
		case 0x1007: // MT27520 Family
		case 0x1008: // MT27521 Family
		case 0x1009: // MT27530 Family
		case 0x100a: // MT27531 Family
		case 0x100b: // MT27540 Family
		case 0x100c: // MT27541 Family
		case 0x100d: // MT27550 Family
		case 0x100e: // MT27551 Family
		case 0x100f: // MT27560 Family
		case 0x1010: // MT27561 Family
			if (g_DeviceCount<INFI_MAX_DEVICES)
			{
				struct Device* dev = (struct Device*)(&g_Device[g_DeviceCount]);
				memcpy(&(dev->pcie), device, sizeof(struct pcie_device));
				if (infiniband_mlx4_init(dev, &ibv_devices[g_DeviceCount])==0)
				{
					g_DeviceCount++;
					return 1;
				}
			}
		};
		return 0;
	};
	return 0;
}

static struct ibv_context* alloc_context(struct ibv_device *device, int cmd_fd)
{	// default empty implementation
	return 0;
}

static void free_context(struct ibv_context *context)
{
}

void infiniband_init(void)
{
	static int __initialized = 0;
	if (__initialized>0)
		return;
	__initialized = 1;
	pcie_init();
	int i;
	for (i=0; i<INFI_MAX_DEVICES; ++i)
	{
		ibv_devices[i].ops.alloc_context = alloc_context;
		ibv_devices[i].ops.free_context = free_context;
		ibv_devices[i].transport_type = IBV_TRANSPORT_IB;
		ibv_devices[i].node_type = IBV_NODE_UNKNOWN;
		ibv_devicesPtr[i] = &ibv_devices[i];
	}
	int num = pcie_find_device(&infiniband_pcie_device_callback, 0);
	if (num==0)
	{
		INFI_LOG(INFI_PREFIX "no devices found\n");
		return;
	}
	INFI_LOG(INFI_PREFIX "found %u devices\n", num);
	for (i=0; i<num; ++i)
	{
		ibv_devices[i].node_type = IBV_NODE_CA;
	}
}



struct ibv_device **ibv_get_device_list(int *num_devices)
{
	infiniband_init();
	if (num_devices!=0)
		*num_devices = g_DeviceCount;
	return ibv_devicesPtr;
}

void ibv_free_device_list(struct ibv_device **list)
{
}

struct ibv_context *ibv_open_device(struct ibv_device *device)
{
	return 0;
}

int ibv_close_device(struct ibv_context* context)
{
	return 0;
}
