#include "platform.h"
#include <l4io.h>
#include <l4/sigma0.h>

void* platform_map(void* address, L4_Word_t size)
{
	L4_Word_t offset = ((L4_Word_t)address) & ((PAGE_SIZE)-1);
	address = ((UINT8*)address)-offset;
	void* result = 0;
	while (true)
	{
		L4_Fpage_t fp = L4_Fpage(address, PAGE_SIZE), rcv = fp, res;
		L4_Set_Rights(&fp, 4);	// set read right
		res = L4_Sigma0_GetPage_RcvWindow(L4_nilthread, fp, rcv);
		if (L4_IsNilFpage(res))
		{
			ACPI_LOG(ACPI_PREFIX "mapping failed %x (remaining %d)\n", address, size);
			break;
		}
		if (result==0)
			result = L4_Address(res)+offset;
		if (size<PAGE_SIZE)
			break;
		address+= PAGE_SIZE;
		size-= PAGE_SIZE;
	}
	return result;
}

void platform_unmap(void* address)
{
	// TODO unmap
}
