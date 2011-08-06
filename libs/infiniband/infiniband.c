#include "infiniband.h"
#include "pcie.h"
#include <l4io.h>

// user defines
#define INFI_LOG printf
#define INFI_PREFIX "INFI: "

static int infiniband_pcie_device_callback(struct pcie_device* device)
{
	switch (device->vendorId)
	{
	case 0x15B3:		// Mellanox
		switch (device->deviceId)
		{
		case 0x673C:
			INFI_LOG("Mellanox ConnectX VPI PCIe 2.0 5GT/s - IB QDR / 10GigE\n");
			return 1;
		};
		return 0;
	};
	return 0;
}

void infiniband_init(void)
{
	int num = pcie_find_device(&infiniband_pcie_device_callback);
	if (num==0)
	{
		INFI_LOG(INFI_PREFIX "no devices found\n");
		return;
	}
	INFI_LOG(INFI_PREFIX "found %u devices\n", num);
}
