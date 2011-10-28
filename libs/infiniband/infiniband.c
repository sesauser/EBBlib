#include "infiniband_internal.h"
#include "../pcie/pcie.h"
#include <l4io.h>
#include <l4/thread.h>
#include <l4/kip.h>
#include <l4/ipc.h>
#include <l4/schedule.h>
#include "verbs.h"
//drivers
#include "mlx4.h"

// user defines
#define INFI_LOG printf
#define INFI_PREFIX "INFI: "
#define INFI_MAX_DEVICES 1

static struct Device g_Device[INFI_MAX_DEVICES];
static unsigned int g_DeviceCount = 0;

static struct ibv_device* ibv_devicesPtr[INFI_MAX_DEVICES];

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
				struct ibv_device* idev = infiniband_mlx4_init(dev);
				if (idev!=0)
				{
					ibv_devicesPtr[g_DeviceCount] = idev;
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

L4_ThreadId_t ib_threadId = { .raw = 0 };

void ib_thread()
{
	INFI_LOG(INFI_PREFIX "Thread started\n");
	pcie_init();
	int num = pcie_find_device(&infiniband_pcie_device_callback, 0);
	if (num>0)
	{
		INFI_LOG(INFI_PREFIX "found %u devices\n", num);
	}  else
	{
		INFI_LOG(INFI_PREFIX "no devices found\n");
	}
	INFI_LOG(INFI_PREFIX "IB: waiting for events\n");
	// should start to wait for msix events
	L4_ThreadId_t tid;
	L4_MsgTag_t tag;
	L4_Msg_t msg;
	while (1)
	{
		tag = L4_Wait_Timeout(L4_TimePeriod(100*1000), &tid);
		while (L4_IpcSucceeded(tag))
		{
			L4_MsgStore(tag, &msg);
			switch (L4_Label(tag))
			{
			case 1:
				{
					L4_MsgClear(&msg);
					L4_MsgAppendWord(&msg, g_DeviceCount);
					break;
				}
			default:
				INFI_LOG(INFI_PREFIX "unkown msg (%d)\n", L4_Label(tag));
				break;
			};
			L4_MsgLoad(&msg);
			L4_ThreadId_t nextid = L4_nilthread;
			tag = L4_ReplyWait_Timeout(tid, L4_TimePeriod(100*1000), &nextid);
			tid = nextid;
		}
	}
	L4_Sleep(L4_Never);
}

struct ibv_device **ibv_get_device_list(int *num_devices)
{
	L4_Msg_t msg;
	L4_MsgClear(&msg);
	L4_Set_MsgLabel(&msg, 1);
	L4_MsgLoad(&msg);
	L4_MsgTag_t tag = L4_Call(ib_threadId);
	if ((!L4_IpcSucceeded(tag)) || (L4_UntypedWords(tag) < 1))
	{
		if (!L4_IpcSucceeded(tag))
			INFI_LOG(INFI_PREFIX "ibv_get_device_list: IPC failed\n");
		else
			INFI_LOG(INFI_PREFIX "ibv_get_device_list: invalid answer\n");
		if (num_devices!=0)
			num_devices = 0;
		return 0;
	}
	int count = L4_MsgWord(&msg, 0);
	if (num_devices!=0)
		*num_devices = count;
	return ibv_devicesPtr;	// alloc pointer
}

void ibv_free_device_list(struct ibv_device **list)
{
	// dealloc pointer
}

#define MAX_CONTEXT 1
static struct ibv_context g_Context[MAX_CONTEXT];
static int g_ContextUsed[MAX_CONTEXT] = { 0 };

struct ibv_context *ibv_open_device(struct ibv_device *device)
{
	if (device==0 || device->ops.alloc_context==0)
	{
		INFI_LOG(INFI_PREFIX "ibv_open_device: invalid device\n");
		return 0;
	}
	return device->ops.alloc_context(device, 0);
}

int ibv_close_device(struct ibv_context* context)
{
	if (context==0 || context->device->ops.free_context==0)
	{
		INFI_LOG(INFI_PREFIX "ibv_close_device: invalid context\n");
		return 0;
	}
	context->device->ops.free_context(context);
	return 1;
}
