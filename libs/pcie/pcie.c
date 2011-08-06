// based on Linux source code

#include "pcie.h"
#include "acpi.h"
#include "../acpi/platform.h"
#include <l4io.h>

#define bool int
#define false 0
#define true 1

// user defines
#define PCIE_LOG printf
#define PCIE_PREFIX "PCIE: "
#define PCIE_MAX_HC 4

// MCFG - PCI Memory Mapped Configuration table and sub-table
// Version 1
// Conforms to "PCI Firmware Specification", Revision 3.0, June 20, 2005
#pragma pack(1)
struct acpi_table_mcfg {
	struct acpi_table_header header;	// Common ACPI table header
	u8 reserved[8];
};

// Subtable
struct acpi_mcfg_allocation {
	u64 address;			// Base address, processor-relative
	u16 pci_segment;		// PCI segment group number
	u8 start_bus_number;	// Starting PCI Bus number
	u8 end_bus_number;	// Final PCI Bus number
	u32 reserved;
};

#pragma pack()

struct HostController
{
	u64 address;
	int segment;
	int start_bus;
	int end_bus;
};

static struct HostController g_HostController[PCIE_MAX_HC];
static unsigned int g_HostControllerCount = 0;

static void pcie_init_from_pci()
{
	//pci_init();
	// todo implement
}

static UINT32 pcie_read_data(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg)
{
	if (reg>=(1<<12))
		return -1;	// default invalid value
	UINT32* address = (UINT32*)(((UINT8*)hc->address) + (bus << 20) + ((device & 0x1F) << 15) + ((function & 0x7) << 12) + (reg & 0xFFC));
	return *address;
}

static int find_anything(struct pcie_device* dev)
{
	switch (dev->vendorId)
	{
	case 0x1000:
		PCIE_LOG("LSI Logic: ");
		switch(dev->deviceId)
		{
		case 0x0030: PCIE_LOG("PCI-X to Ultra320 SCSI Controller\n");return 1;
		}
		break;
	case 0x15AD:
		PCIE_LOG("VMware Inc.: ");
		switch(dev->deviceId)
		{
		case 0x0405: PCIE_LOG("NVIDIA\n");return 1;
		case 0x0770: PCIE_LOG("Standard Enhanced PCI to USB Host Controller\n");return 1;
		case 0x0790: PCIE_LOG("PCI bridge\n");return 1;
		case 0x07A0: PCIE_LOG("PCI Express Root Port\n");return 1;
		}
		break;
	case 0x15B3:
		PCIE_LOG("Mellanox Technology: ");
		switch(dev->deviceId)
		{
		case 0x673C: PCIE_LOG("MT26428 [ConnectX VPI PCIe 2.0 5GT/s - IB QDR / 10GigE]\n");return 1;
		}
		break;
	case 0x8086:
		PCIE_LOG("Intel Corporation: ");
		switch(dev->deviceId)
		{
		case 0x100F: PCIE_LOG("Gigabit Ethernet Controller (copper)\n");return 1;
		case 0x7110: PCIE_LOG("PIIX4/4E/4M ISA Bridge\n");return 1;
		case 0x7112: PCIE_LOG("PIIX4/4E/4M USB Interface\n");return 1;
		case 0x7191: PCIE_LOG("440BX/ZX AGPset PCI-to-PCI bridge\n");return 1;
		case 0x7192: PCIE_LOG("440BX/ZX chipset Host-to-PCI Bridge\n");return 1;
		}
		break;
	};
	printf("Unknown Device\n");
	return 1;
}

static bool acpi_mcfg_check_entry(struct acpi_table_mcfg *mcfg, struct acpi_mcfg_allocation *cfg);

void pcie_init()
{
	static bool _initialized = false;
	if (_initialized)
	{
		return;
	}
	_initialized = true;
	int start = 0;
	// lookup acpi table
	ACPI_Table* table;
	while ((table = acpi_find_table("MCFG", &start))!=0)
	{
		struct acpi_table_mcfg* mcfg = (struct acpi_table_mcfg*)table;

		/* how many config structures do we have */
		int entries = (table->length - sizeof(struct acpi_table_mcfg)) / sizeof(struct acpi_mcfg_allocation);
		if (entries == 0) {
			PCIE_LOG(PCIE_PREFIX "MMCONFIG has no entries\n");
			continue;
		}
		struct acpi_mcfg_allocation* cfg_table = (struct acpi_mcfg_allocation *) &mcfg[1];
		int i;
		for (i = 0; i < entries; i++)
		{
			struct acpi_mcfg_allocation* cfg = &cfg_table[i];
			if (!acpi_mcfg_check_entry(mcfg, cfg))
			{
				continue;
			}
			if (g_HostControllerCount>=PCIE_MAX_HC)
			{
				PCIE_LOG(PCIE_PREFIX "no memory for HC entries\n");
				break;
			}
			struct HostController* hc = &g_HostController[g_HostControllerCount++];
			hc->address = cfg->address;
			hc->segment = cfg->pci_segment;
			hc->start_bus = cfg->start_bus_number;
			hc->end_bus = cfg->end_bus_number;
			PCIE_LOG(PCIE_PREFIX "Device at %X %u %u %u: ", hc->address, hc->segment, hc->start_bus, hc->end_bus);
			// map data
			platform_map(hc->address, 1 << 28);
// TODO only allocate needed space
//			platform_map(hc->address + (hc->start_bus << 20), (hc->end_bus - hc->start_bus + 1) << 20);
		}
	}
	if (start==0)
	{	// can't find table, so try alternative
		pcie_init_from_pci();
		return;
	}
	PCIE_LOG(PCIE_PREFIX "found %u Host Controller(s)\n", g_HostControllerCount);
	// search for devices
	int count = pcie_find_device(&find_anything);
	PCIE_LOG(PCIE_PREFIX "initialized (%u Device(s))\n", count);
}

static bool acpi_mcfg_check_entry(struct acpi_table_mcfg *mcfg,
					struct acpi_mcfg_allocation *cfg)
{
	if (cfg->address < 0xFFFFFFFF)
		return true;

	// check for SGI (see Linux code)
	if (mcfg->header.oem_id[0]!='S' ||
		mcfg->header.oem_id[1]!='G' ||
		mcfg->header.oem_id[2]!='I')
		return true;

	if (mcfg->header.revision >= 1)
	{
		// TODO check year? (see Linux code)
		//int year;
		//if (dmi_get_date(DMI_BIOS_DATE, &year, NULL, NULL) &&
		//    year >= 2010)
			return true;
	}

	PCIE_LOG(PCIE_PREFIX "MCFG region for %04x [bus %02x-%02x] at %#llx "
	       "is above 4GB, ignored\n", cfg->pci_segment,
	       cfg->start_bus_number, cfg->end_bus_number, cfg->address);
	return false;
}

unsigned int pcie_find_device(pcie_device_callback callback)
{
	unsigned int result = 0;
	int hcnr;
	for (hcnr=0; hcnr<g_HostControllerCount; ++hcnr)
	{
		struct HostController* hc = &g_HostController[hcnr];
		UINT8 bus;
		for (bus=hc->start_bus; bus<hc->end_bus; ++bus)
		{
			UINT8 device;
			for (device=0; device<32; ++device)
			{
				UINT32 id = pcie_read_data(hc, bus, device, 0, 0);
				if (id==-1)
					continue;
				struct pcie_device dev;
				dev.vendorId = id & 0xFFFF;
				dev.deviceId = (id>> 16) & 0xFFFF;
				PCIE_LOG(PCIE_PREFIX "Device(%u:%03u:%02u): %04X %04X\n", hcnr, bus, device, dev.vendorId, dev.deviceId);
				if ((*callback)(&dev))
				{
					result++;
				}
			}
		}
	}
	return result;
}

unsigned int pcie_find_device_ById(unsigned short vendorId, unsigned short deviceId, pcie_device_callback* callback)
{
	unsigned int result = 0;
	return result;
}
