#ifndef _MSIX_H
#define _MSIX_H

#include "pcie.h"

struct msix_entry
{
	unsigned short vector;
	unsigned short entry;
};

extern int enable_msix(struct pcie_device* self, struct msix_entry* entries, int entryCount);

#endif
