#ifndef COMPLETION_H
#define COMPLETION_H

#include "../acpi/platform.h"
#include <l4/ipc.h>

struct completion
{
	int value;
};
/*
//
extern void init_completion(struct completion* comp);
//
extern int wait_for_completion_timeout(struct completion* comp, unsigned int timeout);
//
extern void complete(struct completion* comp);
*/

static inline void init_completion(struct completion* comp)
{
	comp->value = 0;
	wmb();
}
// timeout in ms
static inline void wait_for_completion(struct completion* comp)
{
	while (1)
	{
		if (comp->value>0)
			return;
		L4_Sleep(L4_TimePeriod(100*1000));
	}
}

static inline int wait_for_completion_timeout(struct completion* comp, unsigned int timeout)
{
	while (timeout>0)
	{
		if (comp->value>0)
			return 1;
		unsigned int wait = timeout;
		if (wait>100)
			wait = 100;
		L4_Sleep(L4_TimePeriod(wait*1000));
		timeout-= wait;
	}
	if (comp->value>0)
		return 1;
	return 0;
}
static inline void complete(struct completion* comp)
{
	wmb();
	comp->value = 1;
}

#endif
