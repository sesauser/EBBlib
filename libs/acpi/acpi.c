// based on Linux source code

// TODO implementation should instead copy acpi data to own buffers
// shouldn't give the hardware pointers to user

#include "acpi.h"
#include <l4io.h>
#include <l4/sigma0.h>
#include "platform.h"

#define bool int
#define false 0
#define true 1


// user defines
#define ACPI_LOG printf
#define ACPI_PREFIX "ACPI: "
// logs the ACPI tables
//#define ACPI_SHOW_RSDP
#define ACPI_MAX_TABLES 128

// platform defines
#define ACPI_RSDP_SCAN_STEP             16
#define ACPI_RSDP_CHECKSUM_LENGTH       20
#define ACPI_RSDP_XCHECKSUM_LENGTH      36
#define ACPI_SIG_RSDP           "RSD PTR "  /* Root System Description Pointer */
#define ACPI_EBDA_PTR_LOCATION          0x0000040E	/* Physical Address */
#define ACPI_EBDA_PTR_LENGTH            2
#define ACPI_EBDA_WINDOW_SIZE           1024
#define ACPI_HI_RSDP_WINDOW_BASE 0x000E0000
#define ACPI_HI_RSDP_WINDOW_SIZE 0x00020000
#pragma pack(1)
// RSDP Table
typedef struct acpi_table_rsdp
{
	char                    Signature[8];               /* ACPI signature, contains "RSD PTR " */
	UINT8                   Checksum;                   /* ACPI 1.0 checksum */
	char                    OemId[ACPI_OEM_ID_SIZE];    /* OEM identification */
	UINT8                   Revision;                   /* Must be (0) for ACPI 1.0 or (2) for ACPI 2.0+ */
	UINT32                  RsdtPhysicalAddress;        /* 32-bit physical address of the RSDT */
	UINT32                  Length;                     /* Table length in bytes, including header (ACPI 2.0+) */
	UINT64                  XsdtPhysicalAddress;        /* 64-bit physical address of the XSDT (ACPI 2.0+) */
	UINT8                   ExtendedChecksum;           /* Checksum of entire table (ACPI 2.0+) */
	UINT8                   Reserved[3];                /* Reserved, must be zero */

} ACPI_TABLE_RSDP;
#pragma pack()

struct acpi_table_desc
{
	struct acpi_table_header* pointer;
	u32 length;
	union acpi_name signature;
	u8 flags;
};

static struct acpi_table_desc g_Tables[ACPI_MAX_TABLES];
static unsigned int g_TablesCount = 0;

// Calculates circular checksum of memory region.
// Buffer: Pointer to memory region to be checked
// Length: Length of this memory region
// returns Checksum (UINT8)
UINT8 AcpiTbChecksum (UINT8* Buffer, UINT32 Length)
{
	UINT8 Sum = 0;
	UINT8 *End = Buffer + Length;
	while (Buffer < End)
	{
		Sum = (UINT8) (Sum + *(Buffer++));
	}
	return Sum;
}

// Validate the RSDP (ptr)
// Rsdp: Pointer to unvalidated RSDP
// returns Success
static bool AcpiTbValidateRsdp(ACPI_TABLE_RSDP* Rsdp)
{
	// The signature and checksum must both be correct
	//Note: Sometimes there exists more than one RSDP in memory; the valid
	// RSDP has a valid checksum, all others have an invalid checksum.
	if (((char*)Rsdp)[0]!='R' ||
		((char*)Rsdp)[1]!='S' ||
		((char*)Rsdp)[2]!='D' ||
		((char*)Rsdp)[3]!=' ' ||
		((char*)Rsdp)[4]!='P' ||
		((char*)Rsdp)[5]!='T' ||
		((char*)Rsdp)[6]!='R' ||
		((char*)Rsdp)[7]!=' ')
	{
		// Nope, BAD Signature
		return false;
	}
	// Check the standard checksum
	if (AcpiTbChecksum ((UINT8 *) Rsdp, ACPI_RSDP_CHECKSUM_LENGTH) != 0)
	{
		ACPI_LOG(ACPI_PREFIX "invalid checksum\n");
		return false;
	}
	// Check extended checksum if table version >= 2
	if ((Rsdp->Revision >= 2) &&
		(AcpiTbChecksum ((UINT8 *) Rsdp, ACPI_RSDP_XCHECKSUM_LENGTH) != 0))
	{
		ACPI_LOG(ACPI_PREFIX "invalid extended checksum\n");
		return false;
	}
	return true;
}

static UINT8* AcpiTbScanMemoryForRsdp(
	UINT8* StartAddress, UINT32 Length)
{
	UINT8                   *MemRover;
	UINT8                   *EndAddress;
	StartAddress = platform_map(StartAddress, Length);
	if (!StartAddress)
		return 0;
	EndAddress = StartAddress + Length;
	// Search from given start address for the requested length
	for (MemRover = StartAddress; MemRover < EndAddress;
		MemRover += ACPI_RSDP_SCAN_STEP)
	{
		// The RSDP signature and checksum must both be correct

		bool Status = AcpiTbValidateRsdp ((ACPI_TABLE_RSDP*) MemRover);
		if (Status)
		{
			// Sig and checksum valid, we have found a real RSDP
			return MemRover;
		}
		// No sig match or bad checksum, keep searching
	}
	platform_unmap(StartAddress);
	// Searched entire block, no RSDP was found
	return 0;
}

// DESCRIPTION: Search lower 1Mbyte of memory for the root system descriptor
//              pointer structure.  If it is found, set *RSDP to point to it.
// NOTE1:       The RSDP must be either in the first 1K of the Extended
//              BIOS Data Area or between E0000 and FFFFF (From ACPI Spec.)
//              Only a 32-bit physical address is necessary.
void* AcpiFindRootPointer()
{
	// 1a) Get the location of the Extended BIOS Data Area (EBDA)
	UINT16* TablePtr = (UINT16*)platform_map(ACPI_EBDA_PTR_LOCATION, ACPI_EBDA_PTR_LENGTH);
	if (!TablePtr)
	{
		ACPI_LOG(ACPI_PREFIX "BDA couldn't be mapped\n");
	}
	UINT32 PhysicalAddress = 0;
	if (TablePtr!=0)
	{
		PhysicalAddress = (*TablePtr) << 4;
		platform_unmap(ACPI_EBDA_PTR_LOCATION);
	}
	// EBDA present?
	if (PhysicalAddress > 0x400)
	{
		// 1b) Search EBDA paragraphs (EBDA is required to be 
		//     minimum of 1K length)
		UINT8* MemRover = AcpiTbScanMemoryForRsdp ((UINT8*)PhysicalAddress, ACPI_EBDA_WINDOW_SIZE);
		if (MemRover)
		{
			return (void*)MemRover;
		}
	}
	// 2) Search upper memory: 16-byte boundaries in E0000h-FFFFFh
	UINT8* MemRover = AcpiTbScanMemoryForRsdp((UINT8*)ACPI_HI_RSDP_WINDOW_BASE, ACPI_HI_RSDP_WINDOW_SIZE);
	if (MemRover)
	{
		return (void*)MemRover;
	}
	// A valid RSDP was not found
	return 0;
}


static struct acpi_table_header *g_RootPointer = 0;
static UINT32 g_TableEntrySize = 0;
static UINT32 g_TableEntryCount = 0;

static bool acpi_init_tables(void* rsdpPtr)
{
	struct acpi_table_rsdp* rsdp = (struct acpi_table_rsdp*)platform_map(rsdpPtr, sizeof(struct acpi_table_rsdp));
	if (rsdp==0)
	{
		ACPI_LOG(ACPI_PREFIX "rsdp failed\n");
		return false;
	}
#ifdef ACPI_SHOW_RSDP
	ACPI_LOG(ACPI_PREFIX "oem, rev., rsdt: %s (%d) %x\n", rsdp->OemId, rsdp->Revision, rsdp->RsdtPhysicalAddress);
	if (rsdp->Revision>=2)
		ACPI_LOG(ACPI_PREFIX "length, xsdt: %d, %x\n", rsdp->Length, rsdp->XsdtPhysicalAddress);
#endif
	UINT64 address;
	// Differentiate between RSDT and XSDT root tables
	if (rsdp->Revision > 1 && rsdp->XsdtPhysicalAddress)
	{
		// Root table is an XSDT (64-bit physical addresses). We must use the
		// XSDT if the revision is > 1 and the XSDT pointer is present, as per
		// the ACPI specification.
		address = rsdp->XsdtPhysicalAddress;
		g_TableEntrySize = sizeof(UINT64);
		if (address==0)//acpi_tb_check_xsdt(address) == AE_NULL_ENTRY)
		{
			// XSDT has NULL entry, RSDT is used
			address = rsdp->RsdtPhysicalAddress;
			g_TableEntrySize = sizeof(UINT32);
			ACPI_LOG(ACPI_PREFIX "BIOS XSDT has NULL entry, using RSDT\n");
		}
	} else
	{	// Root table is an RSDT (32-bit physical addresses)
		address = rsdp->RsdtPhysicalAddress;
		g_TableEntrySize = sizeof(UINT32);
	}
	struct acpi_table_header* header = (struct acpi_table_header*)platform_map(address, PAGE_SIZE);
	if (header==0)
	{
		ACPI_LOG(ACPI_PREFIX "root table failed\n");
		return false;
	}
	if (header->length < sizeof(struct acpi_table_header))
	{
		ACPI_LOG(ACPI_PREFIX "root header too small\n");
		return false;
	}
	if (header->length>PAGE_SIZE)
	{	// load rest of the table
		platform_map((void*)(((L4_Word_t)address) + PAGE_SIZE), header->length-PAGE_SIZE);
	}
	// verify checksum
	if (AcpiTbChecksum(header, header->length)!=0)
	{
		ACPI_LOG(ACPI_PREFIX "checksum failed\n");
		return false;
	}
	g_RootPointer = header;
	// Calculate the number of tables described in the root table
	g_TableEntryCount = (u32)((header->length - sizeof(struct acpi_table_header)) /
			    g_TableEntrySize);
	bool hasNullEntry = false;
	UINT8* table_entry = ((UINT8*)header)+ sizeof(struct acpi_table_header);
	int i;
	for (i = 0; i < g_TableEntryCount; i++)
	{
		// Get the table physical address (32-bit for RSDT, 64-bit for XSDT)
		if (g_TableEntrySize==sizeof(UINT64))
		{
			address = (*((UINT64*)table_entry));
		} else {
			address = (*((UINT32*)table_entry));
		}
		if (address==0)
		{
			hasNullEntry = true;
			ACPI_LOG(ACPI_PREFIX "null pointer entry\n");
			continue;
		}
		struct acpi_table_header* header = (struct acpi_table_header*)platform_map(address, PAGE_SIZE);
		if (header==0)
		{
			ACPI_LOG(ACPI_PREFIX "table %x%x failed\n", (UINT32)(address>>32), (UINT32)address);
			continue;
		}
		if (header->length < sizeof(struct acpi_table_header))
		{
			ACPI_LOG(ACPI_PREFIX "table header too small\n");
			continue;
		}
		if (header->length>PAGE_SIZE)
		{	// load rest of the table
			platform_map((void*)(((L4_Word_t)address) + PAGE_SIZE), header->length-PAGE_SIZE);
		}
		if (g_TablesCount<ACPI_MAX_TABLES)
		{
			struct acpi_table_desc* desc = &g_Tables[g_TablesCount];
			g_TablesCount++;
			desc->pointer = header;
			desc->length = header->length;
			desc->signature = header->signature;
			desc->flags = 0;
		}
		//ACPI_LOG ("found %c%c%c%c %X (%u)\n", header->signature.ascii[0], header->signature.ascii[1], header->signature.ascii[2], header->signature.ascii[3], header, header->length);
		// TODO special case for FADT -> DSDT and FACS
		table_entry += g_TableEntrySize;
	}
	if (hasNullEntry)
	{
		g_TableEntryCount = 0;
		return false;
	}
	if (g_TablesCount==0)
	{
		ACPI_LOG(ACPI_PREFIX "no headers found\n");
		return false;
	}
	return true;
}

static bool __initialize()
{
	static bool _initialized = false;
	static bool _initializedSuccess = false;
	if (_initialized)
	{
		return _initializedSuccess;
	}
	// search RSDP (Root System Description Pointer)
	void* rsdp = AcpiFindRootPointer();
	_initializedSuccess = rsdp != 0 && acpi_init_tables(rsdp);
	_initialized = true;
	return _initializedSuccess;
}

void acpi_init()
{
	static bool _initialized = false;
	if (_initialized)
	{
		return;
	}
	_initialized = true;
	if (!__initialize())
	{
		return;
	}
	ACPI_LOG(ACPI_PREFIX "ACPI initialized\n");
}

ACPI_Table* acpi_find_table(const char* table, int* start)
{
	if (!__initialize())
		return 0;
	int j = 0;
	if (start!=0)
		j = *start;
	for (; j<g_TablesCount; j++)
	{
		struct acpi_table_desc* desc = (struct acpi_table_desc*)&g_Tables[j];
		if (*((UINT32*)table)==desc->signature.integer)
		{
			if (start!=0)
				*start = j+1;
			return desc->pointer;
		}
	}
	if (start==0 || (*start)==0)
		ACPI_LOG(ACPI_PREFIX "didn't find table %s\n", table);
	return 0;
}
