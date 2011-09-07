/*
 * Copyright (c) 2006, 2007 Cisco Systems, Inc.  All rights reserved.
 * Copyright (c) 2007, 2008 Mellanox Technologies. All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
*/
#include "mlx4.h"
#include "log.h"
#include <l4/schedule.h>	// for L4_Yield
#include <l4/ipc.h>

static inline void sleep(unsigned int msec)
{
	L4_Time_t t = L4_TimePeriod(msec*1000);
	// TODO check L4_Never
	L4_Sleep(t);
}

int mlx4_reset(struct mlx4_device *dev)
{
#define MLX4_RESET_BASE		0xf0000
#define MLX4_RESET_SIZE		  0x400
#define MLX4_SEM_OFFSET		  0x3fc
#define MLX4_RESET_OFFSET	   0x10
#define MLX4_RESET_VALUE	bs32(1)

#define MLX4_SEM_TIMEOUT_JIFFIES	(10 * HZ)
#define MLX4_RESET_TIMEOUT_JIFFIES	(2 * HZ)
	/*
	 * Reset the chip.  This is somewhat ugly because we have to
	 * save off the PCI header before reset and then restore it
	 * after the chip reboots.  We skip config space offsets 22
	 * and 23 since those have a special meaning.
	 */

	UINT32* hca_header = dev->cmd.pool;	// misuse cmd pool
//	pcie_cap = pci_find_capability(dev->pdev, PCI_CAP_ID_EXP);

	int i;
	for (i = 0; i < 64; ++i) {
		if (i == 22 || i == 23)
			continue;
		hca_header[i] = dev->pcie.read_config(&dev->pcie, i*4);
	}
	unsigned char* reset = ((unsigned char*)dev->dcs) + MLX4_RESET_BASE;
	// grab HW semaphore to lock out flash updates
	L4_Clock_t end = L4_SystemClock();
	end = L4_ClockAddUsec(end, 10*1000*1000);
	u32 sem;
	do
	{
		sem = *(UINT32*)(reset + MLX4_SEM_OFFSET);
		if (!sem)
			break;
		sleep(1);
	} while (L4_SystemClock().raw<end.raw);

	if (sem) {
		DRIVER_LOG(DRIVER_PREFIX "Failed to obtain HW semaphore, aborting\n");
		return 1;
	}
	// actually hit reset
	*(UINT32*)(reset + MLX4_RESET_OFFSET) = MLX4_RESET_VALUE;
	// Docs say to wait one second before accessing dev
	sleep(1000);

	unsigned short id;
	end = L4_SystemClock();
	end = L4_ClockAddUsec(end, 2*1000*1000);
	do
	{
		id = dev->pcie.read_config_word(&dev->pcie, 0);
		if (id != 0xFFFF)
			break;
		sleep(1);
	} while (L4_SystemClock().raw<end.raw);

	if (id == 0xFFFF) {
		DRIVER_LOG(DRIVER_PREFIX "PCI dev did not come back after reset, "
			  "aborting.\n");
		return 1;
	}

	// Now restore the PCI headers
	if (dev->pcie.pcieCap)
	{
		dev->pcie.write_config_word(&dev->pcie, dev->pcie.pcieCap + 8, hca_header[(dev->pcie.pcieCap + 8) / 4]);
		dev->pcie.write_config_word(&dev->pcie, dev->pcie.pcieCap + 16, hca_header[(dev->pcie.pcieCap + 16) / 4]);
	}
	// without?
	for (i = 0; i < 16; ++i) {
		if (i == 1)
			continue;
		dev->pcie.write_config(&dev->pcie, i*4, hca_header[i]);
	}
	// restore pci command
	dev->pcie.write_config(&dev->pcie, 4, hca_header[1]);
	return 0;
}
