/*
 * Copyright (c) 2006, 2007 Cisco Systems, Inc.  All rights reserved.
 * Copyright (c) 2005 Mellanox Technologies. All rights reserved.
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
#include <linux/io-mapping.h>

#include <asm/page.h>
*/
#include "mlx4.h"
#include "icm.h"
#include "log.h"

enum {
	MLX4_NUM_RESERVED_UARS = 8
};

int mlx4_pd_alloc(struct mlx4_device *dev, u32 *pdn)
{
	*pdn = mlx4_bitmap_alloc(&dev->pd_bitmap);
	if (*pdn == -1)
		return -1;

	return 0;
}

void mlx4_pd_free(struct mlx4_device *dev, u32 pdn)
{
	mlx4_bitmap_free(&dev->pd_bitmap, pdn);
}

int mlx4_init_pd_table(struct mlx4_device *dev)
{
	return mlx4_bitmap_init(&dev->pd_bitmap, dev->caps.num_pds,
				(1 << 24) - 1, dev->caps.reserved_pds, 0);
}

void mlx4_cleanup_pd_table(struct mlx4_device *dev)
{
	mlx4_bitmap_cleanup(&dev->pd_bitmap);
}


int mlx4_uar_alloc(struct mlx4_device *dev, struct mlx4_uar *uar)
{
	uar->index = mlx4_bitmap_alloc(&dev->uar_table.bitmap);
	if (uar->index == -1)
		return -1;

	uar->pfn = (((u64)dev->uar)  >> PAGE_SHIFT) + uar->index;
	uar->map = uar->pfn << PAGE_SHIFT;
	return 0;
}

void mlx4_uar_free(struct mlx4_device *dev, struct mlx4_uar *uar)
{
	mlx4_bitmap_free(&dev->uar_table.bitmap, uar->index);
}

int mlx4_bf_alloc(struct mlx4_device *dev, struct mlx4_bf *bf)
{
	struct mlx4_uar *uar;
	int err = 0;
	int idx;

//	if (!priv->bf_mapping)
		return -1;

	//mutex_lock(&priv->bf_mutex);
	if (!list_empty(&dev->bf_list))
		uar = list_entry(dev->bf_list.next, struct mlx4_uar, bf_list);
	else {
		if (mlx4_bitmap_avail(&dev->uar_table.bitmap) < MLX4_NUM_RESERVED_UARS) {
			err = -1;
			goto out;
		}
		uar = 0;//kmalloc(sizeof *uar, GFP_KERNEL);
		if (!uar) {
			err = -1;
			goto out;
		}
		err = mlx4_uar_alloc(dev, uar);
		if (err)
			goto free_kmalloc;

		uar->map = 0;//ioremap(uar->pfn << PAGE_SHIFT, PAGE_SIZE);
		if (!uar->map) {
			err = -1;
			goto free_uar;
		}

		uar->bf_map = 0;//io_mapping_map_wc(priv->bf_mapping, uar->index << PAGE_SHIFT);
		if (!uar->bf_map) {
			err = -1;
			goto unamp_uar;
		}
		uar->free_bf_bmap = 0;
		list_add(&uar->bf_list, &dev->bf_list);
	}

	bf->uar = uar;
	idx = ffz(uar->free_bf_bmap);
	uar->free_bf_bmap |= 1 << idx;
	bf->uar = uar;
	bf->offset = 0;
	bf->buf_size = dev->caps.bf_reg_size / 2;
	bf->reg = uar->bf_map + idx * dev->caps.bf_reg_size;
	if (uar->free_bf_bmap == (1 << dev->caps.bf_regs_per_page) - 1)
		list_del_init(&uar->bf_list);

	goto out;

unamp_uar:
	bf->uar = NULL;
	//iounmap(uar->map);

free_uar:
	mlx4_uar_free(dev, uar);

free_kmalloc:
	//kfree(uar);

out:
	//mutex_unlock(&priv->bf_mutex);
	return err;
}

void mlx4_bf_free(struct mlx4_device *dev, struct mlx4_bf *bf)
{
	int idx;

	if (!bf->uar || !bf->uar->bf_map)
		return;

//	mutex_lock(&priv->bf_mutex);
	idx = (bf->reg - bf->uar->bf_map) / dev->caps.bf_reg_size;
	bf->uar->free_bf_bmap &= ~(1 << idx);
	if (!bf->uar->free_bf_bmap) {
		if (!list_empty(&bf->uar->bf_list))
			list_del(&bf->uar->bf_list);

//		io_mapping_unmap(bf->uar->bf_map);
//		iounmap(bf->uar->map);
		mlx4_uar_free(dev, bf->uar);
//		kfree(bf->uar);
	} else if (list_empty(&bf->uar->bf_list))
		list_add(&bf->uar->bf_list, &dev->bf_list);

//	mutex_unlock(&priv->bf_mutex);
}

int mlx4_init_uar_table(struct mlx4_device *dev)
{
	if (dev->caps.num_uars <= 128) {
		DRIVER_LOG(DRIVER_PREFIX "Only %d UAR pages (need more than 128)\n",
			 dev->caps.num_uars);
		DRIVER_LOG(DRIVER_PREFIX "Increase firmware log2_uar_bar_megabytes?\n");
		return -1;
	}
DRIVER_LOG(DRIVER_PREFIX "mlx4_init_uar_table: %d, %d\n", dev->caps.num_uars, max(128, dev->caps.reserved_uars));
	return mlx4_bitmap_init(&dev->uar_table.bitmap,
				dev->caps.num_uars, dev->caps.num_uars - 1,
				max(128, dev->caps.reserved_uars), 0);
}

void mlx4_cleanup_uar_table(struct mlx4_device *dev)
{
	mlx4_bitmap_cleanup(&dev->uar_table.bitmap);
}
