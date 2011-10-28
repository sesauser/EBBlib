#include "pcie.h"
#include "../acpi/acpi.h"
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

struct pcie_device_internal
{
	struct HostController* hc;
	UINT8 bus;
	UINT8 device;
	UINT8 function;
};

static struct HostController g_HostController[PCIE_MAX_HC];
static unsigned int g_HostControllerCount = 0;
static int g_PCI_fallback = 0;

static inline u32 sysIn32 (u16 port) {
  u32 ret;
  __asm__ volatile ("inl %w1,%0":"=a"(ret) : "Nd"(port));
  return ret;
}

static inline u16 sysIn16 (u16 port) {
  u16 ret;
  __asm__ volatile ("inw %w1,%w0":"=a"(ret) : "Nd"(port));
  return ret;
}

static inline u8 sysIn8 (u16 port) {
  u16 ret;
  __asm__ volatile ("inb %w1,%b0":"=a"(ret) : "Nd"(port));
  return ret;
}

static inline void sysOut32 (u16 port, u32 val) {
  __asm__ volatile ("outl %0,%w1"::"a"(val), "Nd" (port));
}

static inline void sysOut16 (u16 port, u16 val) {
  __asm__ volatile ("outw %w0,%w1"::"a"(val), "Nd" (port));
}

static inline void sysOut8 (u16 port, u8 val) {
  __asm__ volatile ("outb %b0,%w1"::"a"(val), "Nd" (port));
}

static u32 pciConfigRead32 (u8 bus, u8 slot, u16 func, u16 offset) {
  sysOut32(0xCF8, 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) |
	   offset);
  return sysIn32(0xCFC);
}

static u16 pciConfigRead16 (u8 bus, u8 slot, u16 func, u16 offset) {
  sysOut32(0xCF8, 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) |
	   offset);
  return sysIn16(0xCFC + (offset & 2));
}

static u8 pciConfigRead8 (u8 bus, u8 slot, u16 func, u16 offset) {
  sysOut32(0xCF8, 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) |
	   offset);
  return sysIn8(0xCFC + (offset & 3));
}

static void pciConfigWrite32 (u8 bus, u8 slot, u16 func, u16 offset,
			     u32 val) {
  sysOut32(0xCF8, 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) |
	   offset);
  sysOut32(0xCFC, val);
}

static void pciConfigWrite16 (u8 bus, u8 slot, u16 func, u16 offset,
			     u16 val) {
  sysOut32(0xCF8, 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) |
	   offset);
  sysOut32(0xCFC + (offset & 2), val);
}

static void pciConfigWrite8 (u8 bus, u8 slot, u16 func, u16 offset,
			     u8 val) {
  sysOut32(0xCF8, 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) |
	   offset);
  sysOut32(0xCFC + (offset & 3), val);
}

static UINT32 pci_read_config(struct pcie_device_internal* device, UINT16 reg)
{
	if (device==0 || reg>0xFF)
		return -1;
	return pciConfigRead32(device->bus, device->device, device->function, reg);
}

static void pci_write_config(struct pcie_device_internal* device, UINT16 reg, UINT32 value)
{
	if (device==0 || reg>0xFF)
		return;
	return pciConfigWrite32(device->bus, device->device, device->function, reg, value);
}

static UINT16 pci_read_config_word(struct pcie_device_internal* device, UINT16 reg)
{
	if (device==0 || reg>0xFF)
		return -1;
	return pciConfigRead16(device->bus, device->device, device->function, reg);
}

static void pci_write_config_word(struct pcie_device_internal* device, UINT16 reg, UINT16 value)
{
	if (device==0 || reg>0xFF)
		return;
	return pciConfigWrite16(device->bus, device->device, device->function, reg, value);
}

static int enumerateDevices(int bus, pcie_device_callback callback, void* callbackData) {
	int result = 0;
  u16 vendor;
  u16 deviceId;
  u8 header;
  u8 dev_class;
  u8 subclass;
  u8  progif;
  int i;
 
  for (i = 0; i < 32; i++) {
    vendor = pciConfigRead16(bus,i,0,0);
    if (vendor == 0xFFFF)
      continue;
    deviceId = pciConfigRead16(bus,i,0,2);
    header = pciConfigRead8(bus,i,0,14);
/*    dev_class = pciConfigRead8(bus,i,0,11);
    subclass = pciConfigRead8(bus,i,0,10);
    progif = pciConfigRead8(bus,i,0,9);
    printf("Bus #%d, Device #%d, Vendor: 0x%x, Device: 0x%x, Header: 0x%x\n",
	   bus, i, vendor, deviceId, header);
    printf("  Class code: 0x%x, Subclass: 0x%x, Prog IF: 0x%x\n",
	   dev_class, subclass, progif);*/
    if(header & 1) {
      result+= enumerateDevices(pciConfigRead8(bus,i,0,25), callback, callbackData);
    }
    if ((header & 0x7F) == 0)
    {
	unsigned short device = i;	// just variable rename
	struct pcie_device dev;			// allocate instead
	struct pcie_device_internal internal;	// allocate instead
	dev._internal = &internal;
	dev.vendorId = vendor;
	dev.deviceId = deviceId;
	dev.defaultIrq = pciConfigRead8(bus,i,0,0x3C);
	dev.read_config = &pci_read_config;
	dev.write_config = &pci_write_config;
	dev.read_config_word = &pci_read_config_word;
	dev.write_config_word = &pci_write_config_word;
	unsigned char reg = 0;	// pcie capability register
	dev.type = (reg & 0xF0) >> 4;
	int bar;
	// save original value, set to -1, read back, restore original value
	for (bar=0; bar<6; ++bar)
	{
		dev.bar[bar] = pciConfigRead32(bus, device, 0, 0x10 + bar*4);
		dev.barType[bar] = dev.bar[bar] & 1;
		if (dev.barType[bar] == 0)
			dev.barType[bar] = dev.bar[bar] & 0xF;
		pciConfigWrite32(bus, device, 0, 0x10 + bar*4, 0xFFFFFFFF);
		dev.barSize[bar] = pciConfigRead32(bus, device, 0, 0x10 + bar*4);
		pciConfigWrite32(bus, device, 0, 0x10 + bar*4, dev.bar[bar]);
	}
	// calculate size
	for (bar=0; bar<6; ++bar)
	{
		if (dev.bar[bar] & 1 == 1)
		{
			dev.bar[bar] = dev.bar[bar] & 0xFFFFFFFC;
			dev.barSize[bar] = (~(dev.barSize[bar] & 0xFFFFFFFC))+1;
			continue;	// io space
		}
		// memory space
		unsigned char type = (dev.bar[bar] >> 1) & 0x3;
		// mask of info bits
		dev.bar[bar] = dev.bar[bar] & 0xFFFFFFF0;
		dev.barSize[bar] = (~(dev.barSize[bar] & 0xFFFFFFF0))+1;
		if (type == 0x02)	// 64-bit
		{
			bar++;
			if (dev.barSize[bar]==0xFFFFFFFF)
				dev.barSize[bar] = 0;
			else
				dev.barSize[bar]  = (~dev.barSize[bar]) +1;
		}
	}
	dev.pcieCap = 0;
	{	// search pcie cap
		unsigned short status = pciConfigRead16(bus, device, 0, 6);
		if ((status & (1 << 4)) > 0)
		{
			unsigned char pos = pciConfigRead8(bus, device, 0, 0x34);	// capabilities pointer
			bool isE = false;
			while (pos>=0x40)
			{
				pos &= ~3;
				unsigned char id = pciConfigRead8(bus, device, 0, pos);
				if (id == 0xff)
					break;
				if (id == 0x10)	// pci express cap
				{
					isE = true;
					break;
				}
				pos = pciConfigRead8(bus, device, 0, pos +1);
				if (pos < 0x40)
					break;
			}
			if (isE)
			{
				dev.pcieCap = pos;
			}
		}
	}
	internal.hc =0;
	internal.bus = bus;
	internal.device = device;
	internal.function = 0;
	if ((*callback)(&dev, callbackData))
	{
		result++;
	}
    }
  }
  return result;
}

static void pcie_init_from_pci()
{
PCIE_LOG(PCIE_PREFIX "Fallback to PCI\n");
	g_PCI_fallback = 1;
	//pci_init();
	// todo implement
	//int number;
	//enumerateDevices(0, find_anything, number);
}

static inline UINT8* pcie_config_address(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function)
{
	return ((UINT8*)hc->address) + (bus << 20) + ((device & 0x1F) << 15) + ((function & 0x7) << 12);
}

static UINT8 pcie_config_read_byte(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg)
{
	if (reg>=(1<<12))
		return -1;	// default invalid value
	UINT8* address = (UINT8*)(pcie_config_address(hc, bus, device, function) + (reg & 0xFFF));
	return *address;
}

static UINT16 pcie_config_read_word(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg)
{
	if (reg>=(1<<12))
		return -1;	// default invalid value
	UINT16* address = (UINT16*)(pcie_config_address(hc, bus, device, function) + (reg & 0xFFE));
	return *address;
}

static UINT32 pcie_config_read_dword(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg)
{
	if (reg>=(1<<12))
		return -1;	// default invalid value
	UINT32* address = (UINT32*)(pcie_config_address(hc, bus, device, function) + (reg & 0xFFC));
	return *address;
}

static void pcie_config_write_byte(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg, UINT8 value)
{
	if (reg>=(1<<12))
		return;
	UINT8* address = (UINT8*)(pcie_config_address(hc, bus, device, function) + (reg & 0xFFF));
	*address = value;
}

static void pcie_config_write_word(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg, UINT16 value)
{
	if (reg>=(1<<12))
		return;
	UINT16* address = (UINT16*)(pcie_config_address(hc, bus, device, function) + (reg & 0xFFE));
	*address = value;
}

static void pcie_config_write_dword(struct HostController* hc, UINT8 bus, UINT8 device, UINT8 function, UINT16 reg, UINT32 value)
{
	if (reg>=(1<<12))
		return;
	UINT32* address = (UINT32*)(pcie_config_address(hc, bus, device, function) + (reg & 0xFFC));
	*address = value;
}

static UINT32 read_config(struct pcie_device_internal* device, UINT16 reg)
{
	if (device==0)
		return -1;
	return pcie_config_read_dword(device->hc,device->bus, device->device, device->function, reg);
}

static void write_config(struct pcie_device_internal* device, UINT16 reg, UINT32 value)
{
	if (device==0)
		return;
	return pcie_config_write_dword(device->hc,device->bus, device->device, device->function, reg, value);
}

static UINT16 read_config_word(struct pcie_device_internal* device, UINT16 reg)
{
	if (device==0)
		return -1;
	return pcie_config_read_word(device->hc,device->bus, device->device, device->function, reg);
}

static void write_config_word(struct pcie_device_internal* device, UINT16 reg, UINT16 value)
{
	if (device==0)
		return;
	return pcie_config_write_word(device->hc,device->bus, device->device, device->function, reg, value);
}

static int find_anything(struct pcie_device* dev, void* data)
{
	if (dev->msixTable)
		PCIE_LOG("MSI-X ");
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
	printf("Unknown Device (%04X, %04X)\n", dev->vendorId, dev->deviceId);
	(*((UINT32*)data))++;
	return 0;
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
			//PCIE_LOG(PCIE_PREFIX "Device at %X %u %u %u: \n", hc->address, hc->segment, hc->start_bus, hc->end_bus);
			// map data
			platform_map(hc->address, 1 << 28);
// TODO only allocate needed space
//			platform_map(hc->address + (hc->start_bus << 20), (hc->end_bus - hc->start_bus + 1) << 20);
		}
	}
	if (start==0 || g_HostControllerCount==0)
	{	// can't find table, so try alternative
		pcie_init_from_pci();
		return;
	}
	PCIE_LOG(PCIE_PREFIX "found %u Host Controller(s)\n", g_HostControllerCount);
	// search for devices
	int count_unknown = 0;
	int count = pcie_find_device(&find_anything, &count_unknown);
	PCIE_LOG(PCIE_PREFIX "found (%u Device(s))\n", count+count_unknown);
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

unsigned int pcie_find_device(pcie_device_callback callback, void* callbackData)
{
	if (g_PCI_fallback>0)
	{
		return enumerateDevices(0, callback, callbackData);
	}
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
				UINT32 id = pcie_config_read_dword(hc, bus, device, 0, 0);
				if (id==-1)
					continue;
				unsigned char pcieCap = 0;	// pcie cap position
				unsigned char msixCap = 0;	// msix cap position
				{	// check for PCIE capability
					UINT32 extended;
					extended = pcie_config_read_dword(hc, bus, device, 0, 0x0C);
					if (((extended >> 16) & 0x7F)!=0)
						continue;	// not header type 0
					extended = pcie_config_read_dword(hc, bus, device, 0, 4);
					if ((extended & (1 << 20)) == 0)
						continue;	// no capability list available
					unsigned char pos = pcie_config_read_byte(hc, bus, device, 0, 0x34);	// capabilities pointer
					while (pos>=0x40)
					{
						pos &= ~3;
						unsigned char id = pcie_config_read_byte(hc, bus, device, 0, pos);
						if (id == 0xff)
							break;
						switch (id)
						{
						case 0x10:		// pci express cap
							{
								pcieCap = pos;
								break;
							}
						case 0x11:		// msix express cap
							{
								msixCap = pos;
								break;
							}
						}
						pos = pcie_config_read_byte(hc, bus, device, 0, pos +1);
						if (pos < 0x40)
							break;
					}
					if (!pcieCap)
						continue;
				}
//				PCIE_LOG(PCIE_PREFIX "Device(%u:%03u:%02u): %04X %04X cap: %04X\n", hcnr, bus, device, id & 0xFFFF, (id >> 16) & 0xFFFF, reg);
				struct pcie_device dev;			// allocate instead
				struct pcie_device_internal internal;	// allocate instead
				dev._internal = &internal;
				dev.vendorId = id & 0xFFFF;
				dev.deviceId = (id>> 16) & 0xFFFF;
				dev.pcieCap = pcieCap;
				dev.msixCap = msixCap;
				dev.defaultIrq = pcie_config_read_byte(hc, bus, device, 0, 0x3C);
				dev.read_config = &read_config;
				dev.write_config = &write_config;
				dev.read_config_word = &read_config_word;
				dev.write_config_word = &write_config_word;
				unsigned short reg = pcie_config_read_word(hc, bus, device, 0, pcieCap+2);
				dev.type = (reg & 0xF0) >> 4;
				int bar;
				// save original value, set to -1, read back, restore original value
				for (bar=0; bar<6; ++bar)
				{
					dev.bar[bar] = pcie_config_read_dword(hc, bus, device, 0, 0x10 + bar*4);
					dev.barType[bar] = dev.bar[bar] & 1;
					if (dev.barType[bar] == 0)
						dev.barType[bar] = dev.bar[bar] & 0xF;
					pcie_config_write_dword(hc, bus, device, 0, 0x10 + bar*4, 0xFFFFFFFF);
					dev.barSize[bar] = pcie_config_read_dword(hc, bus, device, 0, 0x10 + bar*4);
					pcie_config_write_dword(hc, bus, device, 0, 0x10 + bar*4, dev.bar[bar]);
				}
				// calculate size
				for (bar=0; bar<6; ++bar)
				{
					if (dev.bar[bar] & 1 == 1)
					{
						dev.bar[bar] = dev.bar[bar] & 0xFFFFFFFC;
						dev.barSize[bar] = (~(dev.barSize[bar] & 0xFFFFFFFC))+1;
						continue;	// io space
					}
					// memory space
					unsigned char type = (dev.bar[bar] >> 1) & 0x3;
					// mask of info bits
					dev.bar[bar] = dev.bar[bar] & 0xFFFFFFF0;
					dev.barSize[bar] = (~(dev.barSize[bar] & 0xFFFFFFF0))+1;
					if (type == 0x02)	// 64-bit
					{
						bar++;
						if (dev.barSize[bar]==0xFFFFFFFF)
							dev.barSize[bar] = 0;
						else
							dev.barSize[bar]  = (~dev.barSize[bar]) +1;
					}
				}
				dev.msixCount = 0;
				dev.msixTable = 0;
				dev.msixPBA = 0;
				if (msixCap)
				{	// msix
					unsigned short control = pcie_config_read_word(hc, bus, device, 0, msixCap+2);
					pcie_config_write_word(hc, bus, device, 0, msixCap+2, control & (~ (1 << 15)));
					dev.msixCount = ((control & 0x7FF)+1);
					unsigned int table_offset = pcie_config_read_dword(hc, bus, device, 0, msixCap+4);
					unsigned char bir = (unsigned char)(table_offset & (7));
					unsigned int pba_offset = pcie_config_read_dword(hc, bus, device, 0, msixCap+8);
					unsigned char pba_bir = (unsigned char)(pba_offset & (7));
					if (bir<6 && pba_bir<6)
					{
						table_offset &= ~(7);
						pba_offset &= ~(7);
						unsigned char* bar = (unsigned char*)dev.bar[bir];
						if (dev.barType[bir]==4)
							bar+= dev.bar[bir+1] << 32;
						dev.msixTable = (unsigned int*)(bar+table_offset);
						if (pba_bir!=bir)
						{
							bar = (unsigned char*)dev.bar[pba_bir];
							if (dev.barType[pba_bir]==4)
								bar+= dev.bar[pba_bir+1] << 32;
						}
						dev.msixPBA = (unsigned long long*)(bar+pba_offset);
					}
				}
				internal.hc = hc;
				internal.bus = bus;
				internal.device = device;
				internal.function = 0;
				if ((*callback)(&dev, callbackData))
				{
					PCIE_LOG(PCIE_PREFIX "%p bar: %p %p %p %p %p %p\n%p\n", pcie_config_address(hc, bus, device, 0), dev.bar[0], dev.bar[1], dev.bar[2], dev.bar[3], dev.bar[4], dev.bar[5], dev.msixTable);
					result++;
				}
//				UINT8 function;
//				for (function=1; function<8; ++function)
//				{
//					UINT32 id = pcie_config_read_dword(hc, bus, device, function, 0);
//					if (id==-1)
//						break;
//					PCIE_LOG(PCIE_PREFIX "\tFunction %u: %04X %04X\n", function, id & 0xFFFF, (id >> 16) & 0xFFFF);
//				}
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
