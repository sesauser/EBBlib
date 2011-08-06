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

#ifndef _ACPI_H_
#define _ACPI_H_

#include <l4/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef unsigned long long UINT64;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

#define ACPI_NAME_SIZE 4
#define ACPI_OEM_TABLE_ID_SIZE 8
#define ACPI_OEM_ID_SIZE 6

// All tables and structures must be byte-packed to match the ACPI
// specification, since the tables are provided by the system BIOS
#pragma pack(1)

// for easy signature compare
union acpi_name
{
	u32 integer;
	char ascii[4];
};

// Master ACPI Table Header. This common header is used by all ACPI tables
// except the RSDP and FACS.
typedef struct acpi_table_header ACPI_Table;
struct acpi_table_header
{
	union acpi_name signature;	/* ASCII table signature */
	u32 length;		/* Length of table in bytes, including this header */
	u8 revision;		/* ACPI Specification minor version # */
	u8 checksum;		/* To make sum of entire table == 0 */
	char oem_id[ACPI_OEM_ID_SIZE];	/* ASCII OEM identification */
	char oem_table_id[ACPI_OEM_TABLE_ID_SIZE];	/* ASCII OEM table identification */
	u32 oem_revision;	/* OEM revision number */
	union acpi_name asl_compiler_id;	/* ASCII ASL compiler vendor ID */
	u32 asl_compiler_revision;	/* ASL compiler version */
};

// Reset to default packing
#pragma pack()

// initialize ACPI
void acpi_init(void);
// get ACPI table by name (\param table)
// use start=0 to start, afterwards can continue with the changed value
ACPI_Table* acpi_find_table(const char* table, int* start);

#ifdef __cplusplus
}
#endif

#endif
