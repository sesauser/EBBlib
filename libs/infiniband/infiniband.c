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

static struct ibv_device ibv_devices[INFI_MAX_DEVICES];
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

static L4_ThreadId_t ib_threadId;
static char ib_stack[8*1024] __attribute__((aligned(64)));
static int ib_ready = 0;

static void ib_thread()
{
	INFI_LOG(INFI_PREFIX "Thread started\n");
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
	if (num>0)
	{
		INFI_LOG(INFI_PREFIX "found %u devices\n", num);
		for (i=0; i<num; ++i)
		{
			ibv_devices[i].node_type = IBV_NODE_CA;
		}
	}  else
	{
		INFI_LOG(INFI_PREFIX "no devices found\n");
	}
	INFI_LOG(INFI_PREFIX "IB: waiting for events\n");
	ib_ready = 1;
	// should start to wait for msix events
//	L4_Sleep(L4_Never);
}

static void infiniband_init(void)
{
	static int __initialized = 0;
	if (__initialized>0)
		return;
	__initialized = 1;
ib_thread();
return;
	// setup thread
	// Get kernel interface page.
	L4_KernelInterfacePage_t* kip = (L4_KernelInterfacePage_t *)L4_KernelInterface(0,0,0);

	//L4_ThreadId sigma0Id = L4_GlobalId(L4_ThreadIdUserBase(kip), 1);
	// Calculate API defined thread IDs.
	ib_threadId = L4_GlobalId(L4_ThreadIdUserBase(kip) + 8, 1);
	L4_Word_t ip = (L4_Word_t)&ib_thread;
	L4_Word_t sp = (L4_Word_t)(&ib_stack[sizeof(ib_stack) - 1]);
	L4_Word_t utcbsize = L4_UtcbSize(kip);
	/* do the ThreadControl call */
	if (!L4_ThreadControl(ib_threadId,	// new thread id
	                      L4_Myself(),	// address space
	                      L4_Myself(),	// scheduler
	                      L4_Myself(),	// pager
	                      ((void*)(((L4_Word_t)L4_MyLocalId().raw + utcbsize * (8)) & ~(utcbsize - 1)))))
	{
		INFI_LOG(INFI_PREFIX "couldn't start IB thread\n");
		return;
	}

	/* set thread on our code */
	L4_Start_SpIp(ib_threadId, sp, ip);
	INFI_LOG(INFI_PREFIX "waiting for IB thread\n");
{	// pager loop
	L4_ThreadId_t tid;
	L4_MsgTag_t tag;
	L4_Msg_t msg;
 
	while(ib_ready==0)
	{
		tag = L4_Wait_Timeout(L4_TimePeriod(100*1000), &tid);
		
		while(ib_ready==0)
		{
			L4_MsgStore(tag, &msg);
		
#if 0
			{
			printf ("Root-Pager got msg from %p (%p, %p, %p)\n",
				(void *) tid.raw, (void *) tag.raw,
				(void *) L4_MsgWord (&msg, 0), (void *) L4_MsgWord (&msg, 1));
			}
#endif

			if (L4_UntypedWords (tag) != 2 || L4_TypedWords (tag) != 0 ||
				!L4_IpcSucceeded (tag))
			{
				printf("malformed pagefault IPC from %p (tag=%p)\n",
					   (void *) tid.raw, (void *) tag.raw);
				printf("malformed pagefault in root\n");
				break;
			}
	
			L4_Word_t faddr = L4_MsgWord (&msg, 0);
			/* L4_Word_t fip   = L4_Get (&msg, 1); */
		
			/* This is really ugly, we just touch this address to bring 
			   the page into our address space */
			volatile char* dummy = (char*)faddr;
			*dummy;
		
			/* Send mapitem, note that this is a nop between threads in the 
			   the same address space */
			L4_MsgClear(&msg);
			L4_Fpage_t p = L4_FpageLog2(faddr & ~0xFFF, 12);
			L4_Set_Rights(&p, L4_FullyAccessible);
			L4_MsgAppendMapItem(&msg, L4_MapItem(p, faddr));
			L4_MsgLoad(&msg);
		
			L4_ThreadId_t nextid = L4_nilthread;
			tag = L4_ReplyWait_Timeout(tid, L4_TimePeriod(100*1000), &nextid);
			tid = nextid;
		}
	}
}
printf ("main pager loop ended\n");
	// sleep at least 10sec because the driver initialization would need that time
//	L4_Sleep(L4_TimePeriod(10000*1000));
//	while (ib_ready==0)
//		L4_ThreadSwitch(ib_threadId);
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
