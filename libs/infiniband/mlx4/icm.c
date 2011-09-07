/*
 * Copyright (c) 2005, 2006, 2007, 2008 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2006, 2007 Cisco Systems, Inc.  All rights reserved.
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
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
*/
#include "cmd.h"

#include "mlx4.h"
#include "icm.h"
#include "fw.h"

#define __GFP_HIGHMEM 0
#define __GFP_NOWARN 0
#define GFP_KERNEL 0
#define GFP_HIGHUSER 0
/*
 * We allocate in as big chunks as we can, up to a maximum of 256 KB
 * per chunk.
 */
enum {
	MLX4_ICM_ALLOC_SIZE	= 1 << 18,
	MLX4_TABLE_CHUNK_SIZE	= 1 << 18
};

static void mlx4_free_icm_pages(struct mlx4_device *dev, struct mlx4_icm_chunk *chunk)
{
	int i;

/*	if (chunk->nsg > 0)
		pci_unmap_sg(dev->pdev, chunk->mem, chunk->npages,
			     PCI_DMA_BIDIRECTIONAL);

	for (i = 0; i < chunk->npages; ++i)
		__free_pages(sg_page(&chunk->mem[i]),
			     get_order(chunk->mem[i].length));*/
}

static void mlx4_free_icm_coherent(struct mlx4_device *dev, struct mlx4_icm_chunk *chunk)
{
	int i;
/*
	for (i = 0; i < chunk->npages; ++i)
		dma_free_coherent(&dev->pdev->dev, chunk->mem[i].length,
				  lowmem_page_address(sg_page(&chunk->mem[i])),
				  sg_dma_address(&chunk->mem[i]));*/
}

void mlx4_free_icm(struct mlx4_device *dev, struct mlx4_icm *icm, int coherent)
{
	struct mlx4_icm_chunk *chunk, *tmp;

	if (!icm)
		return;

	list_for_each_entry_safe(chunk, tmp, &icm->chunk_list, list) {
		if (coherent)
			mlx4_free_icm_coherent(dev, chunk);
		else
			mlx4_free_icm_pages(dev, chunk);

		//kfree(chunk);
	}

	//kfree(icm);
}

static int mlx4_alloc_icm_pages(struct scatterlist *mem, int order, int gfp_mask)
{
	struct page *page;

	page = alloc_pages(gfp_mask, order);
	if (!page)
		return -1;

//	sg_set_page(mem, page, PAGE_SIZE << order, 0);
	return 0;
}

static int mlx4_alloc_icm_coherent(struct device *dev, struct scatterlist *mem,
				    int order, int gfp_mask)
{
	void *buf = 0;//dma_alloc_coherent(dev, PAGE_SIZE << order,
			//	       &sg_dma_address(mem), gfp_mask);
	if (!buf)
		return -1;

//	sg_set_buf(mem, buf, PAGE_SIZE << order);
//	BUG_ON(mem->offset);
//	sg_dma_len(mem) = PAGE_SIZE << order;
	return 0;
}

struct mlx4_icm *mlx4_alloc_icm(struct mlx4_device *dev, int npages,
				int gfp_mask, int coherent)
{
	struct mlx4_icm *icm;
	struct mlx4_icm_chunk *chunk = NULL;
	int cur_order;
	int ret;
#define tmpAlloc(size) { tmp+=size; tmpSize-= size; if (size<0) { DRIVER_LOG(DRIVER_PREFIX "not enough space"); return 0;} }
	/* We use sg_set_buf for coherent allocs, which assumes low memory */
//	BUG_ON(coherent && (gfp_mask & __GFP_HIGHMEM));
	u32 tmpSize = PAGE_SIZE;
	unsigned char* tmp = platform_mapAny(tmpSize);
	icm = tmp;//kmalloc(sizeof *icm, gfp_mask & ~(__GFP_HIGHMEM | __GFP_NOWARN));
	tmpAlloc(sizeof(*icm));
	if (!icm)
		return NULL;

	icm->refcount = 0;
	INIT_LIST_HEAD(&icm->chunk_list);

	cur_order = 18;//get_order(MLX4_ICM_ALLOC_SIZE);

	while (npages > 0) {
		if (!chunk) {
			chunk = tmp;//kmalloc(sizeof *chunk,
					//gfp_mask & ~(__GFP_HIGHMEM | __GFP_NOWARN));
			tmpAlloc(sizeof(*chunk));
			if (!chunk)
				goto fail;

			//sg_init_table(chunk->mem, MLX4_ICM_CHUNK_LEN);
			chunk->npages = 0;
			chunk->nsg    = 0;
			list_add_tail(&chunk->list, &icm->chunk_list);
		}

		while (1 << cur_order > npages)
			--cur_order;

		chunk->mem[chunk->npages] = platform_mapAny(PAGE_SIZE << cur_order);
		chunk->memSize[chunk->npages] = PAGE_SIZE << cur_order;
		ret = chunk->mem[chunk->npages] == 0;
//		if (coherent)
//			ret = 0;//mlx4_alloc_icm_coherent(&dev->pdev->dev,
				//		      &chunk->mem[chunk->npages],
				//		      cur_order, gfp_mask);
//		else
//			ret = 0;//mlx4_alloc_icm_pages(&chunk->mem[chunk->npages],
				//		   cur_order, gfp_mask);

		if (ret) {
			if (--cur_order < 0)
			{
				DRIVER_LOG(DRIVER_PREFIX "couldn't alloc all needed pages\n");
				goto fail;
			}
			else
				continue;
		}

		++chunk->npages;

//		if (coherent)
			++chunk->nsg;
/*		else if (chunk->npages == MLX4_ICM_CHUNK_LEN) {
			chunk->nsg = 1;//pci_map_sg(dev->pdev, chunk->mem,
					//	chunk->npages,
					//	PCI_DMA_BIDIRECTIONAL);

			if (chunk->nsg <= 0)
				goto fail;
		}*/

		if (chunk->npages == MLX4_ICM_CHUNK_LEN)
		{
			chunk = NULL;
		}

		npages -= 1 << cur_order;
	}

	if (!coherent && chunk) {
		//chunk->nsg = 1;//pci_map_sg(dev->pdev, chunk->mem,
					//chunk->npages,
					//PCI_DMA_BIDIRECTIONAL);

		if (chunk->nsg <= 0)
			goto fail;
	}

	return icm;

fail:
	mlx4_free_icm(dev, icm, coherent);
	return NULL;
}

static int mlx4_MAP_ICM(struct mlx4_device *dev, struct mlx4_icm *icm, u64 virt)
{
	return mlx4_map_cmd(dev, MLX4_CMD_MAP_ICM, icm, virt);
}

static int mlx4_UNMAP_ICM(struct mlx4_device *dev, u64 virt, u32 page_count)
{
	return mlx4_cmd(dev, virt, page_count, 0, MLX4_CMD_UNMAP_ICM,
			MLX4_CMD_TIME_CLASS_B);
}

int mlx4_MAP_ICM_AUX(struct mlx4_device *dev, struct mlx4_icm *icm)
{
	return mlx4_map_cmd(dev, MLX4_CMD_MAP_ICM_AUX, icm, -1);
}

int mlx4_UNMAP_ICM_AUX(struct mlx4_device *dev)
{
	return mlx4_cmd(dev, 0, 0, 0, MLX4_CMD_UNMAP_ICM_AUX, MLX4_CMD_TIME_CLASS_B);
}

int mlx4_table_get(struct mlx4_device *dev, struct mlx4_icm_table *table, int obj)
{
	int i = (obj & (table->num_obj - 1)) / (MLX4_TABLE_CHUNK_SIZE / table->obj_size);
	int ret = 0;

//	mutex_lock(&table->mutex);

	if (table->icm[i]) {
		++table->icm[i]->refcount;
		goto out;
	}

	table->icm[i] = mlx4_alloc_icm(dev, MLX4_TABLE_CHUNK_SIZE >> PAGE_SHIFT,
				       (table->lowmem ? GFP_KERNEL : GFP_HIGHUSER) |
				       __GFP_NOWARN, table->coherent);
	if (!table->icm[i]) {
		ret = -1;
		DRIVER_LOG(DRIVER_PREFIX "couldn't alloc icm\n");
		goto out;
	}

	if (mlx4_MAP_ICM(dev, table->icm[i], table->virt + (u64) i * MLX4_TABLE_CHUNK_SIZE))
	{
		mlx4_free_icm(dev, table->icm[i], table->coherent);
		table->icm[i] = NULL;
		DRIVER_LOG(DRIVER_PREFIX "couldn't map icm\n");
		ret = -1;
		goto out;
	}

	++table->icm[i]->refcount;

out:
//	mutex_unlock(&table->mutex);
	return ret;
}

void mlx4_table_put(struct mlx4_device *dev, struct mlx4_icm_table *table, int obj)
{
	int i;

	i = (obj & (table->num_obj - 1)) / (MLX4_TABLE_CHUNK_SIZE / table->obj_size);

//	mutex_lock(&table->mutex);

	if (--table->icm[i]->refcount == 0) {
		mlx4_UNMAP_ICM(dev, table->virt + i * MLX4_TABLE_CHUNK_SIZE,
			       MLX4_TABLE_CHUNK_SIZE / MLX4_ICM_PAGE_SIZE);
		mlx4_free_icm(dev, table->icm[i], table->coherent);
		table->icm[i] = NULL;
	}

//	mutex_unlock(&table->mutex);
}

void *mlx4_table_find(struct mlx4_icm_table *table, int obj, u64 *dma_handle)
{
	int idx, offset, dma_offset, i;
	struct mlx4_icm_chunk *chunk;
	struct mlx4_icm *icm;
	struct page *page = NULL;

	if (!table->lowmem)
	{
		DRIVER_LOG(DRIVER_PREFIX "lowmem is null\n");
		return NULL;
	}

//	mutex_lock(&table->mutex);

	idx = (obj & (table->num_obj - 1)) * table->obj_size;
	icm = table->icm[idx / MLX4_TABLE_CHUNK_SIZE];
	dma_offset = offset = idx % MLX4_TABLE_CHUNK_SIZE;

	if (!icm)
	{
		DRIVER_LOG(DRIVER_PREFIX "no icm\n");
		return NULL;
	}

	list_for_each_entry(chunk, &icm->chunk_list, list) {
		for (i = 0; i < chunk->npages; ++i) {
			if (dma_handle && dma_offset >= 0) {
				if (chunk->memSize[i] > dma_offset)
					*dma_handle = (unsigned char*)chunk->mem[i] +
						dma_offset;
				dma_offset -= chunk->memSize[i];
			}
			/*
			 * DMA mapping can merge pages but not split them,
			 * so if we found the page, dma_handle has already
			 * been assigned to.
			 */
			if (chunk->memSize[i] > offset)
			{
				return chunk->mem[i] + offset;
			}
			offset -= chunk->memSize[i];
		}
	}
	DRIVER_LOG(DRIVER_PREFIX "didn't found obj\n");
//	mutex_unlock(&table->mutex);
	return NULL;
}

int mlx4_table_get_range(struct mlx4_device *dev, struct mlx4_icm_table *table,
			 int start, int end)
{
	int inc = MLX4_TABLE_CHUNK_SIZE / table->obj_size;
	int i, err;

	for (i = start; i <= end; i += inc) {
		err = mlx4_table_get(dev, table, i);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "couldn't get table index %d\n", i);
			goto fail;
		}
	}

	return 0;

fail:
	while (i > start) {
		i -= inc;
		mlx4_table_put(dev, table, i);
	}

	return err;
}

void mlx4_table_put_range(struct mlx4_device *dev, struct mlx4_icm_table *table,
			  int start, int end)
{
	int i;

	for (i = start; i <= end; i += MLX4_TABLE_CHUNK_SIZE / table->obj_size)
		mlx4_table_put(dev, table, i);
}

int mlx4_init_icm_table(struct mlx4_device *dev, struct mlx4_icm_table *table,
			u64 virt, int obj_size,	int nobj, int reserved,
			int use_lowmem, int use_coherent)
{
	int obj_per_chunk;
	int num_icm;
	unsigned chunk_size;
	int i;

	obj_per_chunk = MLX4_TABLE_CHUNK_SIZE / obj_size;
	num_icm = (nobj + obj_per_chunk - 1) / obj_per_chunk;

	table->icm      = platform_mapAny(num_icm* sizeof(void*));//kcalloc(num_icm, sizeof *table->icm, GFP_KERNEL);
	if (!table->icm)
		return -1;
	table->virt     = virt;
	table->num_icm  = num_icm;
	table->num_obj  = nobj;
	table->obj_size = obj_size;
	table->lowmem   = use_lowmem;
	table->coherent = use_coherent;
//	mutex_init(&table->mutex);

	for (i = 0; i * MLX4_TABLE_CHUNK_SIZE < reserved * obj_size; ++i) {
		chunk_size = MLX4_TABLE_CHUNK_SIZE;
		if ((i + 1) * MLX4_TABLE_CHUNK_SIZE > nobj * obj_size)
		{
			chunk_size = nobj * obj_size - i * MLX4_TABLE_CHUNK_SIZE;
			chunk_size = (((chunk_size) + (0xFFF)) & ~(0xFFF));
		}

		table->icm[i] = mlx4_alloc_icm(dev, chunk_size >> PAGE_SHIFT,
					       (use_lowmem ? GFP_KERNEL : GFP_HIGHUSER) |
					       __GFP_NOWARN, use_coherent);
		if (!table->icm[i])
			goto err;
		if (mlx4_MAP_ICM(dev, table->icm[i], virt + i * MLX4_TABLE_CHUNK_SIZE)) {
			mlx4_free_icm(dev, table->icm[i], use_coherent);
			table->icm[i] = NULL;
			goto err;
		}

		/*
		 * Add a reference to this ICM chunk so that it never
		 * gets freed (since it contains reserved firmware objects).
		 */
		++table->icm[i]->refcount;
	}

	return 0;

err:
	for (i = 0; i < num_icm; ++i)
		if (table->icm[i]) {
			mlx4_UNMAP_ICM(dev, virt + i * MLX4_TABLE_CHUNK_SIZE,
				       MLX4_TABLE_CHUNK_SIZE / MLX4_ICM_PAGE_SIZE);
			mlx4_free_icm(dev, table->icm[i], use_coherent);
		}

	return -1;
}

void mlx4_cleanup_icm_table(struct mlx4_device *dev, struct mlx4_icm_table *table)
{
	int i;

	for (i = 0; i < table->num_icm; ++i)
		if (table->icm[i]) {
			mlx4_UNMAP_ICM(dev, table->virt + i * MLX4_TABLE_CHUNK_SIZE,
				       MLX4_TABLE_CHUNK_SIZE / MLX4_ICM_PAGE_SIZE);
			mlx4_free_icm(dev, table->icm[i], table->coherent);
		}

//	kfree(table->icm);
}
