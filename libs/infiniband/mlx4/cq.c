/*
 * Copyright (c) 2004, 2005 Topspin Communications.  All rights reserved.
 * Copyright (c) 2005 Sun Microsystems, Inc. All rights reserved.
 * Copyright (c) 2005, 2006, 2007 Cisco Systems, Inc. All rights reserved.
 * Copyright (c) 2005, 2006, 2007, 2008 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2004 Voltaire, Inc. All rights reserved.
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
#include <linux/hardirq.h>
#include <linux/gfp.h>
*/
#include "cmd.h"
#include "cq.h"

#include "mlx4.h"
#include "icm.h"
#include "log.h"

struct mlx4_cq_context {
	u32			flags;
	u16			reserved1[3];
	u16			page_offset;
	u32			logsize_usrpage;
	u16			cq_period;
	u16			cq_max_count;
	u8			reserved2[3];
	u8			comp_eqn;
	u8			log_page_size;
	u8			reserved3[2];
	u8			mtt_base_addr_h;
	u32			mtt_base_addr_l;
	u32			last_notified_index;
	u32			solicit_producer_index;
	u32			consumer_index;
	u32			producer_index;
	u32			reserved4[2];
	u64			db_rec_addr;
};

#define MLX4_CQ_STATUS_OK		( 0 << 28)
#define MLX4_CQ_STATUS_OVERFLOW		( 9 << 28)
#define MLX4_CQ_STATUS_WRITE_FAIL	(10 << 28)
#define MLX4_CQ_FLAG_CC			( 1 << 18)
#define MLX4_CQ_FLAG_OI			( 1 << 17)
#define MLX4_CQ_STATE_ARMED		( 9 <<  8)
#define MLX4_CQ_STATE_ARMED_SOL		( 6 <<  8)
#define MLX4_EQ_STATE_FIRED		(10 <<  8)

void mlx4_cq_completion(struct mlx4_device *dev, u32 cqn)
{
	struct mlx4_cq *cq;

	int idx = cqn & (dev->caps.num_cqs - 1);
	cq = 0;//radix_tree_lookup(&mlx4_priv(dev)->cq_table.tree,
		//	       cqn & (dev->caps.num_cqs - 1));
	if (!cq) {
		DRIVER_LOG(DRIVER_PREFIX "Completion event for bogus CQ %08x\n", cqn);
		return;
	}

	++cq->arm_sn;

	cq->comp(cq);
}

void mlx4_cq_event(struct mlx4_device *dev, u32 cqn, int event_type)
{
	struct mlx4_cq_table *cq_table = &dev->cq_table;
	struct mlx4_cq *cq;

	//spin_lock(&cq_table->lock);

	cq = 0;//radix_tree_lookup(&cq_table->tree, cqn & (dev->caps.num_cqs - 1));
//	if (cq)
//		atomic_inc(&cq->refcount);

//	spin_unlock(&cq_table->lock);

	if (!cq) {
		DRIVER_LOG(DRIVER_PREFIX "Async event for bogus CQ %08x\n", cqn);
		return;
	}

	cq->event(cq, event_type);

//	if (atomic_dec_and_test(&cq->refcount))
//		complete(&cq->free);
}

static int mlx4_SW2HW_CQ(struct mlx4_device *dev, struct mlx4_cmd_mailbox *mailbox,
			 int cq_num)
{
	return mlx4_cmd(dev, mailbox->dma, cq_num, 0, MLX4_CMD_SW2HW_CQ,
			MLX4_CMD_TIME_CLASS_A);
}

static int mlx4_MODIFY_CQ(struct mlx4_device *dev, struct mlx4_cmd_mailbox *mailbox,
			 int cq_num, u32 opmod)
{
	return mlx4_cmd(dev, mailbox->dma, cq_num, opmod, MLX4_CMD_MODIFY_CQ,
			MLX4_CMD_TIME_CLASS_A);
}

static int mlx4_HW2SW_CQ(struct mlx4_device *dev, struct mlx4_cmd_mailbox *mailbox,
			 int cq_num)
{
	return mlx4_cmd_box(dev, 0, mailbox ? mailbox->dma : 0, cq_num,
			    mailbox ? 0 : 1, MLX4_CMD_HW2SW_CQ,
			    MLX4_CMD_TIME_CLASS_A);
}

int mlx4_cq_modify(struct mlx4_device *dev, struct mlx4_cq *cq,
		   u16 count, u16 period)
{
	struct mlx4_cmd_mailbox *mailbox;
	struct mlx4_cq_context *cq_context;
	int err;

	mailbox = mlx4_alloc_cmd_mailbox(dev);
	if (mailbox==0)
		return 1;

	cq_context = mailbox->buf;
	memset(cq_context, 0, sizeof *cq_context);

	cq_context->cq_max_count = bs16(count);
	cq_context->cq_period    = bs16(period);

	err = mlx4_MODIFY_CQ(dev, mailbox, cq->cqn, 1);

	mlx4_free_cmd_mailbox(dev, mailbox);
	return err;
}

int mlx4_cq_resize(struct mlx4_device *dev, struct mlx4_cq *cq,
		   int entries, struct mlx4_mtt *mtt)
{
	struct mlx4_cmd_mailbox *mailbox;
	struct mlx4_cq_context *cq_context;
	u64 mtt_addr;
	int err;

	mailbox = mlx4_alloc_cmd_mailbox(dev);
	if (mailbox==0)
		return 1;

	cq_context = mailbox->buf;
	memset(cq_context, 0, sizeof *cq_context);

	cq_context->logsize_usrpage = bs32(ilog2(entries) << 24);
	cq_context->log_page_size   = mtt->page_shift - 12;
	mtt_addr = 0;//mlx4_mtt_addr(dev, mtt);
	cq_context->mtt_base_addr_h = mtt_addr >> 32;
	cq_context->mtt_base_addr_l = bs32(mtt_addr & 0xffffffff);

	err = mlx4_MODIFY_CQ(dev, mailbox, cq->cqn, 0);

	mlx4_free_cmd_mailbox(dev, mailbox);
	return err;
}

int mlx4_cq_alloc(struct mlx4_device *dev, int nent, struct mlx4_mtt *mtt,
		  struct mlx4_uar *uar, u64 db_rec, struct mlx4_cq *cq,
		  unsigned vector, int collapsed)
{
	struct mlx4_cq_table *cq_table = &dev->cq_table;
	struct mlx4_cmd_mailbox *mailbox;
	struct mlx4_cq_context *cq_context;
	u64 mtt_addr;
	int err;

	if (vector > dev->caps.num_comp_vectors + dev->caps.comp_pool)
		return -1;

	cq->vector = vector;

	cq->cqn = mlx4_bitmap_alloc(&cq_table->bitmap);
	if (cq->cqn == -1)
		return -1;

	err = mlx4_table_get(dev, &cq_table->table, cq->cqn);
	if (err)
		goto err_out;

	err = mlx4_table_get(dev, &cq_table->cmpt_table, cq->cqn);
	if (err)
		goto err_put;
// TODO use radix tree for faster lookup on event, see mlx4_cq_completion, mlx4_cq_event
//	spin_lock_irq(&cq_table->lock);
//	err = 1;//radix_tree_insert(&cq_table->tree, cq->cqn, cq);
//	spin_unlock_irq(&cq_table->lock);
	if (err)
		goto err_cmpt_put;

	mailbox = mlx4_alloc_cmd_mailbox(dev);
	if (mailbox==0) {
		err = 1;
		goto err_radix;
	}

	cq_context = mailbox->buf;
	memset(cq_context, 0, sizeof *cq_context);

	cq_context->flags	    = bs32(!!collapsed << 18);
	cq_context->logsize_usrpage = bs32((ilog2(nent) << 24) | uar->index);
	cq_context->comp_eqn	    = dev->eq_table.eq[vector].eqn;
	cq_context->log_page_size   = mtt->page_shift - MLX4_ICM_PAGE_SHIFT;

	mtt_addr = mlx4_mtt_addr(dev, mtt);
	cq_context->mtt_base_addr_h = mtt_addr >> 32;
	cq_context->mtt_base_addr_l = bs32(mtt_addr & 0xffffffff);
	cq_context->db_rec_addr     = bs64(db_rec);

	err = mlx4_SW2HW_CQ(dev, mailbox, cq->cqn);
	mlx4_free_cmd_mailbox(dev, mailbox);
	if (err)
		goto err_radix;

	cq->cons_index = 0;
	cq->arm_sn     = 1;
	cq->uar        = uar;
//	atomic_set(&cq->refcount, 1);
//	init_completion(&cq->free);

	return 0;

err_radix:
//	spin_lock_irq(&cq_table->lock);
//	radix_tree_delete(&cq_table->tree, cq->cqn);
//	spin_unlock_irq(&cq_table->lock);

err_cmpt_put:
	mlx4_table_put(dev, &cq_table->cmpt_table, cq->cqn);

err_put:
	mlx4_table_put(dev, &cq_table->table, cq->cqn);

err_out:
	mlx4_bitmap_free(&cq_table->bitmap, cq->cqn);

	return err;
}

void mlx4_cq_free(struct mlx4_device *dev, struct mlx4_cq *cq)
{
	struct mlx4_cq_table *cq_table = &dev->cq_table;
	int err;

	err = mlx4_HW2SW_CQ(dev, NULL, cq->cqn);
	if (err)
		DRIVER_LOG(DRIVER_PREFIX "HW2SW_CQ failed (%d) for CQN %06x\n", err, cq->cqn);

//	synchronize_irq(priv->eq_table.eq[cq->vector].irq);

//	spin_lock_irq(&cq_table->lock);
//	radix_tree_delete(&cq_table->tree, cq->cqn);
//	spin_unlock_irq(&cq_table->lock);

//	if (atomic_dec_and_test(&cq->refcount))
//		complete(&cq->free);
//	wait_for_completion(&cq->free);

	mlx4_table_put(dev, &cq_table->table, cq->cqn);
	mlx4_bitmap_free(&cq_table->bitmap, cq->cqn);
}

int mlx4_init_cq_table(struct mlx4_device *dev)
{
	struct mlx4_cq_table *cq_table = &dev->cq_table;
	int err;

//	spin_lock_init(&cq_table->lock);
//	INIT_RADIX_TREE(&cq_table->tree, GFP_ATOMIC);

	err = mlx4_bitmap_init(&cq_table->bitmap, dev->caps.num_cqs,
			       dev->caps.num_cqs - 1, dev->caps.reserved_cqs, 0);
	if (err)
		return err;

	return 0;
}

void mlx4_cleanup_cq_table(struct mlx4_device *dev)
{
	/* Nothing to do to clean up radix_tree */
	mlx4_bitmap_cleanup(&dev->cq_table.bitmap);
}
