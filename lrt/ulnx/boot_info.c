/*
 * Copyright (C) 2011 by Project SESA, Boston University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* These functions are so tiny they should be static inlined */

#include <config.h>
#include <lrt/ulnx/boot_info.h>
#include <lrt/assert.h>

intptr_t boot_cores=1;
intptr_t available_cores=1;

void set_boot_core_count(intptr_t cores)
{
	EBBAssert(cores>0);
	
	boot_cores = cores;
	available_cores = cores;
}

intptr_t get_boot_core_count(void)
{
	return boot_cores;
}

/* the following functions are probably the wrong thing to do, as it feels like this information should be tracked by an EBB */

void increment_core_count(void)
{
	available_cores++; // FIXME should probably be atomic
}

intptr_t get_available_cores(void)
{
	return available_cores;
}
