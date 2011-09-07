/*
 * Copyright (c) 2004, 2005 Topspin Communications.  All rights reserved.
 * Copyright (c) 2005, 2006, 2007, 2008 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2005, 2006, 2007 Cisco Systems, Inc.  All rights reserved.
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
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/errno.h>
*/
#include "cmd.h"
#include "../completion.h"
//#include <asm/io.h>

#include "mlx4.h"
#include "log.h"

#include <l4/schedule.h>	// for L4_Yield

enum {
	/* command completed successfully: */
	CMD_STAT_OK		= 0x00,
	/* Internal error (such as a bus error) occurred while processing command: */
	CMD_STAT_INTERNAL_ERR	= 0x01,
	/* Operation/command not supported or opcode modifier not supported: */
	CMD_STAT_BAD_OP		= 0x02,
	/* Parameter not supported or parameter out of range: */
	CMD_STAT_BAD_PARAM	= 0x03,
	/* System not enabled or bad system state: */
	CMD_STAT_BAD_SYS_STATE	= 0x04,
	/* Attempt to access reserved or unallocaterd resource: */
	CMD_STAT_BAD_RESOURCE	= 0x05,
	/* Requested resource is currently executing a command, or is otherwise busy: */
	CMD_STAT_RESOURCE_BUSY	= 0x06,
	/* Required capability exceeds device limits: */
	CMD_STAT_EXCEED_LIM	= 0x08,
	/* Resource is not in the appropriate state or ownership: */
	CMD_STAT_BAD_RES_STATE	= 0x09,
	/* Index out of range: */
	CMD_STAT_BAD_INDEX	= 0x0a,
	/* FW image corrupted: */
	CMD_STAT_BAD_NVMEM	= 0x0b,
	/* Error in ICM mapping (e.g. not enough auxiliary ICM pages to execute command): */
	CMD_STAT_ICM_ERROR	= 0x0c,
	/* Attempt to modify a QP/EE which is not in the presumed state: */
	CMD_STAT_BAD_QP_STATE   = 0x10,
	/* Bad segment parameters (Address/Size): */
	CMD_STAT_BAD_SEG_PARAM	= 0x20,
	/* Memory Region has Memory Windows bound to: */
	CMD_STAT_REG_BOUND	= 0x21,
	/* HCA local attached memory not present: */
	CMD_STAT_LAM_NOT_PRE	= 0x22,
	/* Bad management packet (silently discarded): */
	CMD_STAT_BAD_PKT	= 0x30,
	/* More outstanding CQEs in CQ than new CQ size: */
	CMD_STAT_BAD_SIZE	= 0x40,
	/* Multi Function device support required: */
	CMD_STAT_MULTI_FUNC_REQ	= 0x50,
};

enum {
	HCR_IN_PARAM_OFFSET	= 0x00,
	HCR_IN_MODIFIER_OFFSET	= 0x08,
	HCR_OUT_PARAM_OFFSET	= 0x0c,
	HCR_TOKEN_OFFSET	= 0x14,
	HCR_STATUS_OFFSET	= 0x18,

	HCR_OPMOD_SHIFT		= 12,
	HCR_T_BIT		= 21,
	HCR_E_BIT		= 22,
	HCR_GO_BIT		= 23
};

enum {
	GO_BIT_TIMEOUT_MSECS	= 10000
};

struct mlx4_cmd_context {
	struct completion	done;
	int			result;
	int			next;
	u64			out_param;
	u16			token;
};

static int mlx4_status_to_errno(u8 status)
{
	switch (status)
	{
	case 0: return 0;
	case CMD_STAT_INTERNAL_ERR: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_INTERNAL_ERR\n"); break;
	case CMD_STAT_BAD_OP: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_OP\n"); break;
	case CMD_STAT_BAD_PARAM: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_PARAM\n"); break;
	case CMD_STAT_BAD_SYS_STATE: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_SYS_STATE\n"); break;
	case CMD_STAT_BAD_RESOURCE: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_RESOURCE\n"); break;
	case CMD_STAT_RESOURCE_BUSY: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_RESOURCE_BUSY\n"); break;
	case CMD_STAT_EXCEED_LIM: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_EXCEED_LIM\n"); break;
	case CMD_STAT_BAD_RES_STATE: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_RES_STATE\n"); break;
	case CMD_STAT_BAD_INDEX: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_INDEX\n"); break;
	case CMD_STAT_BAD_NVMEM: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_NVMEM\n"); break;
	case CMD_STAT_ICM_ERROR: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_ICM_ERROR\n"); break;
	case CMD_STAT_BAD_QP_STATE: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_QP_STATE\n"); break;
	case CMD_STAT_BAD_SEG_PARAM: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_SEG_PARAM\n"); break;
	case CMD_STAT_REG_BOUND: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_REG_BOUND\n"); break;
	case CMD_STAT_LAM_NOT_PRE: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_LAM_NOT_PRE\n"); break;
	case CMD_STAT_BAD_PKT: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_PKT\n"); break;
	case CMD_STAT_BAD_SIZE: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_BAD_SIZE\n"); break;
	case CMD_STAT_MULTI_FUNC_REQ: DRIVER_LOG(DRIVER_PREFIX "CMD_STAT_MULTI_FUNC_REQ\n"); break;
	default: DRIVER_LOG(DRIVER_PREFIX "unknown status\n"); break;
	};
	return 1;
}

static int cmd_pending(struct mlx4_device *dev)
{
	u32 status = *(UINT32*)(dev->cmd.hcr + HCR_STATUS_OFFSET);
	return (status & bs32(1 << HCR_GO_BIT)) ||
		(dev->cmd.toggle == !!(status & bs32(1 << HCR_T_BIT)));
}
static int mlx4_cmd_post(struct mlx4_device *dev, u64 in_param, u64 out_param,
			 u32 in_modifier, u8 op_modifier, u16 op, u16 token,
			 int event)
{
	struct mlx4_cmd *cmd = &dev->cmd;
	u32 *hcr = cmd->hcr;
	int ret = 1;//-EAGAIN;
//	mutex_lock(&cmd->hcr_mutex);

	L4_Clock_t end = L4_SystemClock();
	if (event)
		end = L4_ClockAddUsec(end, 10000*1000);
	while (cmd_pending(dev))
	{
		if (L4_SystemClock().raw>=end.raw)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_cmd_post timeout\n");
			return 1;
		}
		L4_Yield();
	}

	*(hcr + 0) = bs32(in_param >> 32);
	*(hcr + 1) = bs32(in_param & 0xFFFFFFFF);
	*(hcr + 2) = bs32(in_modifier);
	*(hcr + 3) = bs32(out_param >> 32);
	*(hcr + 4) = bs32(out_param & 0xFFFFFFFF);
	*(hcr + 5) = bs32(token << 16);

	// __raw_writel may not order writes.
	wmb();

	*(hcr + 6) = bs32((1 << HCR_GO_BIT)		|
					       (cmd->toggle << HCR_T_BIT)	|
					       (event ? (1 << HCR_E_BIT) : 0)	|
					       (op_modifier << HCR_OPMOD_SHIFT) |
					       op);
	// Make sure that our HCR writes don't get mixed in with
	// writes from another CPU starting a FW command.
	mmiowb();

	cmd->toggle = cmd->toggle ^ 1;

	ret = 0;

out:
//	mutex_unlock(&cmd->hcr_mutex);
	return ret;
}

static int mlx4_cmd_poll(struct mlx4_device *dev, u64 in_param, u64 *out_param,
			 int out_is_imm, u32 in_modifier, u8 op_modifier,
			 u16 op, unsigned long timeout)
{
	unsigned char* hcr = dev->cmd.hcr;
//	down(&priv->cmd.poll_sem);

	int err = mlx4_cmd_post(dev, in_param, out_param ? *out_param : 0,
			    in_modifier, op_modifier, op, 0xFFFF, 0);
	if (err)
	{
		DRIVER_LOG(DRIVER_PREFIX "mlx4_cmd_post failed\n");
		goto out;
	}
	
	L4_Clock_t end = L4_ClockAddUsec(L4_SystemClock(), timeout*1000);
	while (cmd_pending(dev) && L4_SystemClock().raw<end.raw)
	{
		L4_Yield();
	}

	if (cmd_pending(dev)) {
		DRIVER_LOG(DRIVER_PREFIX "timeout\n");
		end = L4_ClockAddUsec(L4_SystemClock(), timeout*1000);
		while (cmd_pending(dev) && L4_SystemClock().raw<end.raw)
		{
			L4_Yield();
		}
		if (cmd_pending(dev)) {
		err = 1;
		DRIVER_LOG(DRIVER_PREFIX "2nd timeout\n");
		goto out;
		}
	}

	if (out_is_imm)
		*out_param =
			(u64) bs32(*(UINT32*)(hcr + HCR_OUT_PARAM_OFFSET)) << 32 |
			(u64) bs32(*(UINT32*)(hcr + HCR_OUT_PARAM_OFFSET + 4));

	err = mlx4_status_to_errno(bs32(*(UINT32*)(hcr + HCR_STATUS_OFFSET)) >> 24);
out:
//	up(&priv->cmd.poll_sem);
	return err;
}
void mlx4_cmd_event(struct mlx4_device *dev, u16 token, u8 status, u64 out_param)
{
	struct mlx4_cmd_context *context = &dev->cmd.context[token & dev->cmd.token_mask];

	/* previously timed out command completing at long last */
	if (token != context->token)
		return;

	context->result = mlx4_status_to_errno(status);
	context->out_param = out_param;

//	complete(&context->done);
}

static int mlx4_cmd_wait(struct mlx4_device *dev, u64 in_param, u64 *out_param,
			 int out_is_imm, u32 in_modifier, u8 op_modifier,
			 u16 op, unsigned long timeout)
{
	struct mlx4_cmd *cmd = &dev->cmd;
	struct mlx4_cmd_context *context;
	int err = 0;

//	down(&cmd->event_sem);

//	spin_lock(&cmd->context_lock);
//	BUG_ON(cmd->free_head < 0);
	context = &cmd->context[cmd->free_head];
	context->token += cmd->token_mask + 1;
	cmd->free_head = context->next;
//	spin_unlock(&cmd->context_lock);

	init_completion(&context->done);

	mlx4_cmd_post(dev, in_param, out_param ? *out_param : 0,
		      in_modifier, op_modifier, op, context->token, 1);

	if (!wait_for_completion_timeout(&context->done, (timeout))) {
		DRIVER_LOG(DRIVER_PREFIX "wait_for_completion_timeout()\n");
		err = -1;
		goto out;
	}

	err = context->result;
	if (err)
		goto out;

	if (out_is_imm)
		*out_param = context->out_param;

out:
//	spin_lock(&cmd->context_lock);
	context->next = cmd->free_head;
	cmd->free_head = context - cmd->context;
//	spin_unlock(&cmd->context_lock);

//	up(&cmd->event_sem);
	return err;
}

int __mlx4_cmd(struct mlx4_device *dev, u64 in_param, u64 *out_param,
	       int out_is_imm, u32 in_modifier, u8 op_modifier,
	       u16 op, unsigned long timeout)
{
	if (dev->cmd.use_events)
		return mlx4_cmd_wait(dev, in_param, out_param, out_is_imm,
				     in_modifier, op_modifier, op, timeout);
	else
		return mlx4_cmd_poll(dev, in_param, out_param, out_is_imm,
				     in_modifier, op_modifier, op, timeout);
}

int mlx4_cmd_init(struct mlx4_device *dev)
{
	//mutex_init(&priv->cmd.hcr_mutex);
	//sema_init(&priv->cmd.poll_sem, 1);
	dev->cmd.use_events = 0;
	dev->cmd.toggle     = 1;
	dev->cmd.context = 0;

	dev->cmd.hcr = dev->dcs+MLX4_HCR_BASE;//ioremap(pci_resource_start(dev->pdev, 0) + MLX4_HCR_BASE,
				//MLX4_HCR_SIZE);
	if (!dev->cmd.hcr) {
		DRIVER_LOG(DRIVER_PREFIX "Couldn't map command register.");
		return -1;
	}

	//dev->cmd.pool = 0;//pci_pool_create("mlx4_cmd", dev->pdev,
				//	 MLX4_MAILBOX_SIZE,
				//	 MLX4_MAILBOX_SIZE, 0);
	if (!dev->cmd.pool)
	{
//		iounmap(priv->cmd.hcr);
		return -1;
	}

	return 0;
}

void mlx4_cmd_cleanup(struct mlx4_device *dev)
{

//	pci_pool_destroy(priv->cmd.pool);
//	iounmap(priv->cmd.hcr);
}

/*
 * Switch to using events to issue FW commands (can only be called
 * after event queue for command events has been initialized).
 */
int mlx4_cmd_use_events(struct mlx4_device *dev)
{
	int i;

	if (dev->cmd.context==0)
		dev->cmd.context = platform_mapAny(dev->cmd.max_cmds * sizeof (struct mlx4_cmd_context));//kmalloc(priv->cmd.max_cmds *
				//   sizeof (struct mlx4_cmd_context),
				//   GFP_KERNEL);
	if (!dev->cmd.context)
		return -1;

	for (i = 0; i < dev->cmd.max_cmds; ++i) {
		dev->cmd.context[i].token = i;
		dev->cmd.context[i].next  = i + 1;
	}

	dev->cmd.context[dev->cmd.max_cmds - 1].next = -1;
	dev->cmd.free_head = 0;

	//sema_init(&priv->cmd.event_sem, priv->cmd.max_cmds);
	//spin_lock_init(&priv->cmd.context_lock);

	for (dev->cmd.token_mask = 1;
	     dev->cmd.token_mask < dev->cmd.max_cmds;
	     dev->cmd.token_mask <<= 1)
		; /* nothing */
	--dev->cmd.token_mask;

	dev->cmd.use_events = 1;

	//down(&priv->cmd.poll_sem);

	return 0;
}

/*
 * Switch back to polling (used when shutting down the device)
 */
void mlx4_cmd_use_polling(struct mlx4_device *dev)
{
	int i;

	dev->cmd.use_events = 0;

//	for (i = 0; i < priv->cmd.max_cmds; ++i)
//		down(&priv->cmd.event_sem);

	//kfree(priv->cmd.context);

	//up(&priv->cmd.poll_sem);
}

static struct mlx4_cmd_mailbox g_mailbox;
static int g_mailboxUsed = 0;

struct mlx4_cmd_mailbox *mlx4_alloc_cmd_mailbox(struct mlx4_device *dev)
{	// TODO
	if (g_mailboxUsed)
	{
		DRIVER_LOG(DRIVER_PREFIX "mailbox already used\n");
		return 0;
	}
	g_mailboxUsed = 1;
	g_mailbox.buf = g_mailbox.dma = dev->cmd.pool;
	return &g_mailbox;
	
	struct mlx4_cmd_mailbox *mailbox;

	mailbox = 0;//kmalloc(sizeof *mailbox, GFP_KERNEL);
	if (!mailbox)
		return 0;

	mailbox->buf = 0;//pci_pool_alloc(mlx4_priv(dev)->cmd.pool, GFP_KERNEL,
				//      &mailbox->dma);
	if (!mailbox->buf) {
		//kfree(mailbox);
		return 0;
	}

	return mailbox;
}

void mlx4_free_cmd_mailbox(struct mlx4_device *dev, struct mlx4_cmd_mailbox *mailbox)
{
	if (!mailbox)
		return;
	if (g_mailboxUsed==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "mailbox was not used! check accesses\n");
	}
	g_mailboxUsed = 0;
//	pci_pool_free(mlx4_priv(dev)->cmd.pool, mailbox->buf, mailbox->dma);
//	kfree(mailbox);
}
