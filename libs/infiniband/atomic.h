#ifndef ATOMIC_H
#define ATOMIC_H

#include "../acpi/platform.h"
#include <l4/ipc.h>

typedef struct 
{
	int value;
} atomic_t;

static inline int atomic_read(const atomic_t *a)
{
	return (*(volatile int *)&(a->value));
}

static inline void atomic_set(atomic_t *a, int value)
{
	a->value = value;
}
static inline void atomic_inc(atomic_t *a)
{
	asm volatile("lock; incl %0" : "+m" (a->value));
}
static inline int atomic_dec_and_test(atomic_t *a)
{
	unsigned char c=0;
	asm volatile("lock; decl %0; sete %1"
		     : "+m" (a->value), "=qm" (c)
		     : : "memory");
	return c != 0;
}
#endif
