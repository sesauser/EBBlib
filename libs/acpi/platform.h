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

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

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

// map hardware address 1:1
void* platform_map(void* address, L4_Word_t size);
void* platform_mapAny(L4_Word_t size);
void platform_unmap(void* address);

static inline unsigned short bs16(unsigned short value)
{
	return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

static inline unsigned int bs32(unsigned int value)
{
	return (bs16(value & 0xFFFF) << 16) | bs16((value >> 16) & 0xFFFF);
}

static inline unsigned long long bs64(unsigned long long value)
{
	return (bs32(value & 0xFFFFFFFF) << 32) | bs32((value >> 32) & 0xFFFFFFFF);
}

#ifdef __x86_64__
#define mb() 	asm volatile("mfence":::"memory")
#define rmb()	asm volatile("lfence":::"memory")
#define wmb()	asm volatile("sfence" ::: "memory")
#else
#warning implement barriers for this platform
#endif

#define mmiowb()	asm volatile("" ::: "memory")

#ifdef __cplusplus
}
#endif

#endif
