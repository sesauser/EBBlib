/*
 * Copyright (c) 2004 Topspin Communications.  All rights reserved.
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

#include "cmd.h"
#include "mlx4.h"
#include "icm.h"
#include "../bitmap.h"	// bitop defs

/*
 * Must be packed because mtt_seg is 64 bits but only aligned to 32 bits.
 */
struct mlx4_mpt_entry {
	u32 flags;
	u32 qpn;
	u32 key;
	u32 pd_flags;
	u64 start;
	u64 length;
	u32 lkey;
	u32 win_cnt;
	u8	reserved1[3];
	u8	mtt_rep;
	u64 mtt_seg;
	u32 mtt_sz;
	u32 entity_size;
	u32 first_byte_offset;
} __attribute__ ((__packed__));

#define MLX4_MPT_FLAG_SW_OWNS	    (0xfUL << 28)
#define MLX4_MPT_FLAG_FREE	    (0x3UL << 28)
#define MLX4_MPT_FLAG_MIO	    (1 << 17)
#define MLX4_MPT_FLAG_BIND_ENABLE   (1 << 15)
#define MLX4_MPT_FLAG_PHYSICAL	    (1 <<  9)
#define MLX4_MPT_FLAG_REGION	    (1 <<  8)

#define MLX4_MPT_PD_FLAG_FAST_REG   (1 << 27)
#define MLX4_MPT_PD_FLAG_RAE	    (1 << 28)
#define MLX4_MPT_PD_FLAG_EN_INV	    (3 << 24)

#define MLX4_MPT_STATUS_SW		0xF0
#define MLX4_MPT_STATUS_HW		0x00

static u32 mlx4_buddy_alloc(struct mlx4_buddy *buddy, int order)
{
	int o;
	int m;
	u32 seg;

	//spin_lock(&buddy->lock);

	for (o = order; o <= buddy->max_order; ++o)
		if (buddy->num_free[o]) {
			m = 1 << (buddy->max_order - o);
			seg = find_first_bit(buddy->bits[o], m);
			if (seg < m)
				goto found;
		}
	//spin_unlock(&buddy->lock);
	return -1;

 found:
	clear_bit(seg, buddy->bits[o]);
	--buddy->num_free[o];

	while (o > order) {
		--o;
		seg <<= 1;
		set_bit(seg ^ 1, buddy->bits[o]);
		++buddy->num_free[o];
	}

	//spin_unlock(&buddy->lock);

	seg <<= order;

	return seg;
}

static void mlx4_buddy_free(struct mlx4_buddy *buddy, u32 seg, int order)
{
	seg >>= order;

	//spin_lock(&buddy->lock);

	while (test_bit(seg ^ 1, buddy->bits[order])) {
		clear_bit(seg ^ 1, buddy->bits[order]);
		--buddy->num_free[order];
		seg >>= 1;
		++order;
	}

	set_bit(seg, buddy->bits[order]);
	++buddy->num_free[order];

	//spin_unlock(&buddy->lock);
}

static int mlx4_buddy_init(struct mlx4_buddy *buddy, int max_order)
{
	int i, s;

	buddy->max_order = max_order;
	//spin_lock_init(&buddy->lock);

	int tmpSize1 = (buddy->max_order + 1) * sizeof (unsigned long *);
	int tmpSize2 = (buddy->max_order + 1) * sizeof (unsigned int);
	unsigned char* mem = platform_mapAny(tmpSize1+tmpSize2);
	buddy->bits = mem;//kzalloc((buddy->max_order + 1) * sizeof (long *),
			//      GFP_KERNEL);
	buddy->num_free = mem+tmpSize1;//kzalloc((buddy->max_order + 1) * sizeof (int *),
				//  GFP_KERNEL);
	if (!buddy->bits || !buddy->num_free)
		goto err_out;

	for (i = 0; i <= buddy->max_order; ++i) {
		s = BITS_TO_LONGS(1 << (buddy->max_order - i));
		buddy->bits[i] = platform_mapAny(s * sizeof (long));//kmalloc(s * sizeof (long), GFP_KERNEL);
		if (!buddy->bits[i])
			goto err_out_free;
		bitmap_zero(buddy->bits[i], 1 << (buddy->max_order - i));
	}

	set_bit(0, buddy->bits[buddy->max_order]);
	buddy->num_free[buddy->max_order] = 1;

	return 0;

err_out_free:
	for (i = 0; i <= buddy->max_order; ++i)
		//kfree(buddy->bits[i]);

err_out:
	//kfree(buddy->bits);
	//kfree(buddy->num_free);

	return -1;
}

static void mlx4_buddy_cleanup(struct mlx4_buddy *buddy)
{
	int i;

//	for (i = 0; i <= buddy->max_order; ++i)
		//kfree(buddy->bits[i]);

	//kfree(buddy->bits);
	//kfree(buddy->num_free);
}

static u32 mlx4_alloc_mtt_range(struct mlx4_device *dev, int order)
{
	struct mlx4_mr_table *mr_table = &dev->mr_table;
	u32 seg;

	seg = mlx4_buddy_alloc(&mr_table->mtt_buddy, order);
	if (seg == -1)
	{
		DRIVER_LOG(DRIVER_PREFIX "mlx4_buddy_alloc() no seg\n");
		return -1;
	}

	if (mlx4_table_get_range(dev, &mr_table->mtt_table, seg,
				 seg + (1 << order) - 1))
	{
		mlx4_buddy_free(&mr_table->mtt_buddy, seg, order);
		DRIVER_LOG(DRIVER_PREFIX "mlx4_table_get_range() failed\n");
		return -1;
	}

	return seg;
}

int mlx4_mtt_init(struct mlx4_device *dev, int npages, int page_shift,
		  struct mlx4_mtt *mtt)
{
	int i;

	if (!npages) {
		mtt->order      = -1;
		mtt->page_shift = MLX4_ICM_PAGE_SHIFT;
		return 0;
	} else
		mtt->page_shift = page_shift;

	for (mtt->order = 0, i = dev->caps.mtts_per_seg; i < npages; i <<= 1)
		++mtt->order;

	mtt->first_seg = mlx4_alloc_mtt_range(dev, mtt->order);
	if (mtt->first_seg == -1)
		return -1;

	return 0;
}

void mlx4_mtt_cleanup(struct mlx4_device *dev, struct mlx4_mtt *mtt)
{
	struct mlx4_mr_table *mr_table = &dev->mr_table;

	if (mtt->order < 0)
		return;

	mlx4_buddy_free(&mr_table->mtt_buddy, mtt->first_seg, mtt->order);
	mlx4_table_put_range(dev, &mr_table->mtt_table, mtt->first_seg,
			     mtt->first_seg + (1 << mtt->order) - 1);
}

u64 mlx4_mtt_addr(struct mlx4_device *dev, struct mlx4_mtt *mtt)
{
	return (u64) mtt->first_seg * dev->caps.mtt_entry_sz;
}

static u32 hw_index_to_key(u32 ind)
{
	return (ind >> 24) | (ind << 8);
}

static u32 key_to_hw_index(u32 key)
{
	return (key << 24) | (key >> 8);
}

static int mlx4_SW2HW_MPT(struct mlx4_device *dev, struct mlx4_cmd_mailbox *mailbox,
			  int mpt_index)
{
	return mlx4_cmd(dev, mailbox->dma, mpt_index, 0, MLX4_CMD_SW2HW_MPT,
			MLX4_CMD_TIME_CLASS_B);
}

static int mlx4_HW2SW_MPT(struct mlx4_device *dev, struct mlx4_cmd_mailbox *mailbox,
			  int mpt_index)
{
	return mlx4_cmd_box(dev, 0, mailbox ? mailbox->dma : 0, mpt_index,
			    !mailbox, MLX4_CMD_HW2SW_MPT, MLX4_CMD_TIME_CLASS_B);
}

int mlx4_mr_alloc(struct mlx4_device *dev, u32 pd, u64 iova, u64 size, u32 access,
		  int npages, int page_shift, struct mlx4_mr *mr)
{
	u32 index;
	int err;

	index = mlx4_bitmap_alloc(&dev->mr_table.mpt_bitmap);
	if (index == -1)
		return -1;

	mr->iova       = iova;
	mr->size       = size;
	mr->pd	       = pd;
	mr->access     = access;
	mr->enabled    = 0;
	mr->key	       = hw_index_to_key(index);

	err = mlx4_mtt_init(dev, npages, page_shift, &mr->mtt);
	if (err)
		mlx4_bitmap_free(&dev->mr_table.mpt_bitmap, index);

	return err;
}

void mlx4_mr_free(struct mlx4_device *dev, struct mlx4_mr *mr)
{
	int err;

	if (mr->enabled) {
		err = mlx4_HW2SW_MPT(dev, NULL,
				     key_to_hw_index(mr->key) &
				     (dev->caps.num_mpts - 1));
		if (err)
			DRIVER_LOG(DRIVER_PREFIX "HW2SW_MPT failed (%d)\n", err);
	}

	mlx4_mtt_cleanup(dev, &mr->mtt);
	mlx4_bitmap_free(&dev->mr_table.mpt_bitmap, key_to_hw_index(mr->key));
}

int mlx4_mr_enable(struct mlx4_device *dev, struct mlx4_mr *mr)
{
	struct mlx4_mr_table *mr_table = &dev->mr_table;
	struct mlx4_cmd_mailbox *mailbox;
	struct mlx4_mpt_entry *mpt_entry;
	int err;

	err = mlx4_table_get(dev, &mr_table->dmpt_table, key_to_hw_index(mr->key));
	if (err)
		return err;

	mailbox = mlx4_alloc_cmd_mailbox(dev);
	if (mailbox==0) {
		err = 1;
		goto err_table;
	}
	mpt_entry = mailbox->buf;

	memset(mpt_entry, 0, sizeof *mpt_entry);

	mpt_entry->flags = bs32(MLX4_MPT_FLAG_MIO	 |
				       MLX4_MPT_FLAG_REGION	 |
				       mr->access);

	mpt_entry->key	       = bs32(key_to_hw_index(mr->key));
	mpt_entry->pd_flags    = bs32(mr->pd | MLX4_MPT_PD_FLAG_EN_INV);
	mpt_entry->start       = bs64(mr->iova);
	mpt_entry->length      = bs64(mr->size);
	mpt_entry->entity_size = bs32(mr->mtt.page_shift);

	if (mr->mtt.order < 0) {
		mpt_entry->flags |= bs32(MLX4_MPT_FLAG_PHYSICAL);
		mpt_entry->mtt_seg = 0;
	} else {
		mpt_entry->mtt_seg = bs64(mlx4_mtt_addr(dev, &mr->mtt));
	}

	if (mr->mtt.order >= 0 && mr->mtt.page_shift == 0) {
		/* fast register MR in free state */
		mpt_entry->flags    |= bs32(MLX4_MPT_FLAG_FREE);
		mpt_entry->pd_flags |= bs32(MLX4_MPT_PD_FLAG_FAST_REG |
						   MLX4_MPT_PD_FLAG_RAE);
		mpt_entry->mtt_sz    = bs32((1 << mr->mtt.order) *
						   dev->caps.mtts_per_seg);
	} else {
		mpt_entry->flags    |= bs32(MLX4_MPT_FLAG_SW_OWNS);
	}

	err = mlx4_SW2HW_MPT(dev, mailbox,
			     key_to_hw_index(mr->key) & (dev->caps.num_mpts - 1));
	if (err) {
		DRIVER_LOG(DRIVER_PREFIX "SW2HW_MPT failed (%d)\n", err);
		goto err_cmd;
	}

	mr->enabled = 1;

	mlx4_free_cmd_mailbox(dev, mailbox);

	return 0;

err_cmd:
	mlx4_free_cmd_mailbox(dev, mailbox);

err_table:
	mlx4_table_put(dev, &mr_table->dmpt_table, key_to_hw_index(mr->key));
	return err;
}

static int mlx4_write_mtt_chunk(struct mlx4_device *dev, struct mlx4_mtt *mtt,
				int start_index, int npages, u64 *page_list)
{
	u64 *mtts;
	u64 dma_handle;
	int i;
	int s = start_index * sizeof (u64);

	/* All MTTs must fit in the same page */
	if (start_index / (PAGE_SIZE / sizeof (u64)) !=
	    (start_index + npages - 1) / (PAGE_SIZE / sizeof (u64)))
		return -1;

	if (start_index & (dev->caps.mtts_per_seg - 1))
		return -1;

	mtts = mlx4_table_find(&dev->mr_table.mtt_table, mtt->first_seg +
				s / dev->caps.mtt_entry_sz, &dma_handle);
	if (!mtts)
		return -1;

//	dma_sync_single_for_cpu(&dev->pdev->dev, dma_handle,
//				npages * sizeof (u64), DMA_TO_DEVICE);

	for (i = 0; i < npages; ++i)
		mtts[i] = bs64(page_list[i] | MLX4_MTT_FLAG_PRESENT);

//	dma_sync_single_for_device(&dev->pdev->dev, dma_handle,
//				   npages * sizeof (u64), DMA_TO_DEVICE);

	return 0;
}

int mlx4_write_mtt(struct mlx4_device *dev, struct mlx4_mtt *mtt,
		   int start_index, int npages, u64 *page_list)
{
	int chunk;
	int err;

	if (mtt->order < 0)
		return -1;

	while (npages > 0) {
		chunk = min_t(int, PAGE_SIZE / sizeof(u64), npages);
		err = mlx4_write_mtt_chunk(dev, mtt, start_index, chunk, page_list);
		if (err)
			return err;

		npages      -= chunk;
		start_index += chunk;
		page_list   += chunk;
	}

	return 0;
}

int mlx4_buf_write_mtt(struct mlx4_device *dev, struct mlx4_mtt *mtt,
		       struct mlx4_buf *buf)
{
	u64 *page_list;
	int err;
	int i;

	page_list = platform_mapAny(buf->npages * sizeof(*page_list));//kmalloc(buf->npages * sizeof *page_list, GFP_KERNEL);
	if (!page_list)
		return -1;

	for (i = 0; i < buf->npages; ++i)
		if (buf->nbufs == 1)
			page_list[i] = buf->direct.map + (i << buf->page_shift);
		else
			page_list[i] = buf->page_list[i].map;

	err = mlx4_write_mtt(dev, mtt, 0, buf->npages, page_list);

	platform_unmap(page_list);
	return err;
}

int mlx4_init_mr_table(struct mlx4_device *dev)
{
	struct mlx4_mr_table *mr_table = &dev->mr_table;
	int err;

	err = mlx4_bitmap_init(&mr_table->mpt_bitmap, dev->caps.num_mpts,
			       ~0, dev->caps.reserved_mrws, 0);
	if (err)
	{
		DRIVER_LOG(DRIVER_PREFIX "init_mr_table() bitmap init failed\n");
		return err;
	}

	err = mlx4_buddy_init(&mr_table->mtt_buddy,
			      ilog2(dev->caps.num_mtt_segs));
	if (err)
	{
		DRIVER_LOG(DRIVER_PREFIX "init_mr_table() mlx4_buddy_init failed\n");
		goto err_buddy;
	}

	if (dev->caps.reserved_mtts) {
		if (mlx4_alloc_mtt_range(dev, fls(dev->caps.reserved_mtts - 1)) == -1) {
			DRIVER_LOG(DRIVER_PREFIX "MTT table of order %d is too small.\n",
				  mr_table->mtt_buddy.max_order);
			err = 1;
			goto err_reserve_mtts;
		}
	}

	return 0;

err_reserve_mtts:
	mlx4_buddy_cleanup(&mr_table->mtt_buddy);

err_buddy:
	mlx4_bitmap_cleanup(&mr_table->mpt_bitmap);

	return err;
}

void mlx4_cleanup_mr_table(struct mlx4_device *dev)
{
	struct mlx4_mr_table *mr_table = &dev->mr_table;

	mlx4_buddy_cleanup(&mr_table->mtt_buddy);
	mlx4_bitmap_cleanup(&mr_table->mpt_bitmap);
}

static inline int mlx4_check_fmr(struct mlx4_fmr *fmr, u64 *page_list,
				  int npages, u64 iova)
{
	int i, page_mask;

	if (npages > fmr->max_pages)
		return -1;

	page_mask = (1 << fmr->page_shift) - 1;

	/* We are getting page lists, so va must be page aligned. */
	if (iova & page_mask)
		return -1;

	/* Trust the user not to pass misaligned data in page_list */
	if (0)
		for (i = 0; i < npages; ++i) {
			if (page_list[i] & ~page_mask)
				return -1;
		}

	if (fmr->maps >= fmr->max_maps)
		return -1;

	return 0;
}

int mlx4_map_phys_fmr(struct mlx4_device *dev, struct mlx4_fmr *fmr, u64 *page_list,
		      int npages, u64 iova, u32 *lkey, u32 *rkey)
{
	u32 key;
	int i, err;

	err = mlx4_check_fmr(fmr, page_list, npages, iova);
	if (err)
		return err;

	++fmr->maps;

	key = key_to_hw_index(fmr->mr.key);
	key += dev->caps.num_mpts;
	*lkey = *rkey = fmr->mr.key = hw_index_to_key(key);

	*(u8 *) fmr->mpt = MLX4_MPT_STATUS_SW;

	/* Make sure MPT status is visible before writing MTT entries */
	wmb();

//	dma_sync_single_for_cpu(&dev->pdev->dev, fmr->dma_handle,
//				npages * sizeof(u64), DMA_TO_DEVICE);

	for (i = 0; i < npages; ++i)
		fmr->mtts[i] = bs64(page_list[i] | MLX4_MTT_FLAG_PRESENT);

//	dma_sync_single_for_device(&dev->pdev->dev, fmr->dma_handle,
//				   npages * sizeof(u64), DMA_TO_DEVICE);

	fmr->mpt->key    = bs32(key);
	fmr->mpt->lkey   = bs32(key);
	fmr->mpt->length = bs64(npages * (1ull << fmr->page_shift));
	fmr->mpt->start  = bs64(iova);

	/* Make MTT entries are visible before setting MPT status */
	wmb();

	*(u8 *) fmr->mpt = MLX4_MPT_STATUS_HW;

	/* Make sure MPT status is visible before consumer can use FMR */
	wmb();

	return 0;
}

int mlx4_fmr_alloc(struct mlx4_device *dev, u32 pd, u32 access, int max_pages,
		   int max_maps, u8 page_shift, struct mlx4_fmr *fmr)
{
	u64 mtt_seg;
	int err = -1;

	if (page_shift < (ffs(dev->caps.page_size_cap) - 1) || page_shift >= 32)
		return -1;

	/* All MTTs must fit in the same page */
	if (max_pages * sizeof *fmr->mtts > PAGE_SIZE)
		return -1;

	fmr->page_shift = page_shift;
	fmr->max_pages  = max_pages;
	fmr->max_maps   = max_maps;
	fmr->maps = 0;

	err = mlx4_mr_alloc(dev, pd, 0, 0, access, max_pages,
			    page_shift, &fmr->mr);
	if (err)
		return err;

	mtt_seg = fmr->mr.mtt.first_seg * dev->caps.mtt_entry_sz;

	fmr->mtts = mlx4_table_find(&dev->mr_table.mtt_table,
				    fmr->mr.mtt.first_seg,
				    &fmr->dma_handle);
	if (!fmr->mtts) {
		err = -1;
		goto err_free;
	}

	return 0;

err_free:
	mlx4_mr_free(dev, &fmr->mr);
	return err;
}

int mlx4_fmr_enable(struct mlx4_device *dev, struct mlx4_fmr *fmr)
{
	int err;

	err = mlx4_mr_enable(dev, &fmr->mr);
	if (err)
		return err;

	fmr->mpt = mlx4_table_find(&dev->mr_table.dmpt_table,
				    key_to_hw_index(fmr->mr.key), NULL);
	if (!fmr->mpt)
		return -1;

	return 0;
}

void mlx4_fmr_unmap(struct mlx4_device *dev, struct mlx4_fmr *fmr,
		    u32 *lkey, u32 *rkey)
{
	if (!fmr->maps)
		return;

	fmr->maps = 0;

	*(u8 *) fmr->mpt = MLX4_MPT_STATUS_SW;
}

int mlx4_fmr_free(struct mlx4_device *dev, struct mlx4_fmr *fmr)
{
	if (fmr->maps)
		return -1;

	fmr->mr.enabled = 0;
	mlx4_mr_free(dev, &fmr->mr);

	return 0;
}

int mlx4_SYNC_TPT(struct mlx4_device *dev)
{
	return mlx4_cmd(dev, 0, 0, 0, MLX4_CMD_SYNC_TPT, 1000);
}
