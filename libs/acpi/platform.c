#include "platform.h"
#include <l4io.h>
#include <l4/sigma0.h>
#define LOG printf
#define PREFIX "L4_map: "

void* platform_map(void* address, L4_Word_t size)
{
	if (size==0)
	{
		LOG(PREFIX "mapping failed %X size==0\n", address);
		return 0;
	}
	if (size < PAGE_SIZE)
	{
		size = PAGE_SIZE;
	}
	L4_Word_t offset = ((L4_Word_t)address) & ((PAGE_SIZE)-1);
	address = ((unsigned char*)address)-offset;
	L4_Fpage_t fp = L4_Fpage(address, size), rcv = fp;
	L4_Set_Rights(&fp, L4_ReadWriteOnly);
	L4_Fpage_t res = L4_Sigma0_GetPage_RcvWindow(L4_nilthread, fp, rcv);
	if (L4_IsNilFpage(res))
	{
		LOG(PREFIX "mapping failed %X (size %d)\n", address, size);
		return 0;
	}
	return L4_Address(res)+offset;
}

void* platform_mapAny(L4_Word_t size)
{
	if (size==0)
	{
		LOG(PREFIX "mapping failed any size==0\n");
		return 0;
	}
	L4_Word_t requestSize = size;
	if (requestSize < PAGE_SIZE)
	{
		requestSize = PAGE_SIZE;
	}
	L4_Fpage_t rcv = L4_CompleteAddressSpace;
	L4_Fpage_t fp = L4_Fpage(~0UL, requestSize);
	L4_Set_Rights(&fp, L4_ReadWriteOnly);
	L4_Fpage_t res = L4_Sigma0_GetPage_RcvWindow(L4_nilthread, fp, rcv);
	if (L4_IsNilFpage(res))
	{
		LOG(PREFIX "mapping failed any (size %d)\n", size);
		return 0;
	}
/*	requestSize = L4_Size(res);
	if (requestSize!=size)
	{
		LOG(PREFIX "requested %d KiB but got %d KiB\n", size>>10, requestSize>>10);
	}*/
	return L4_Address(res);
}

void platform_unmap(void* address)
{
	// TODO unmap
}
