#include "mlx4.h"
#include "../pcie/pcie.h"
#include <l4io.h>
#include <l4/types.h>
#include <l4/ipc.h>
#include <l4/schedule.h>	// for L4_Yield
#include "../acpi/platform.h"

// user defines
#define DRIVER_LOG printf
#define DRIVER_PREFIX "MLX4: "
#define DRIVER_MAX_DEVICES 1
#define DRIVER_MAX_CONTEXT 2

#define MLX4_MAX_PORTS 2
#define MLX4_NUM_QP_REGION 4

static inline void sleep(unsigned int msec)
{
	L4_Time_t t = L4_TimePeriod(msec*1000);
	// TODO check L4_Never
	L4_Sleep(t);
}
enum {
	/* initialization and general commands */
	MLX4_CMD_SYS_EN		 = 0x1,
	MLX4_CMD_SYS_DIS	 = 0x2,
	MLX4_CMD_MAP_FA		 = 0xfff,
	MLX4_CMD_UNMAP_FA	 = 0xffe,
	MLX4_CMD_RUN_FW		 = 0xff6,
	MLX4_CMD_MOD_STAT_CFG	 = 0x34,
	MLX4_CMD_QUERY_DEV_CAP	 = 0x3,
	MLX4_CMD_QUERY_FW	 = 0x4,
	MLX4_CMD_ENABLE_LAM	 = 0xff8,
	MLX4_CMD_DISABLE_LAM	 = 0xff7,
	MLX4_CMD_QUERY_DDR	 = 0x5,
	MLX4_CMD_QUERY_ADAPTER	 = 0x6,
	MLX4_CMD_INIT_HCA	 = 0x7,
	MLX4_CMD_CLOSE_HCA	 = 0x8,
	MLX4_CMD_INIT_PORT	 = 0x9,
	MLX4_CMD_CLOSE_PORT	 = 0xa,
	MLX4_CMD_QUERY_HCA	 = 0xb,
	MLX4_CMD_QUERY_PORT	 = 0x43,
	MLX4_CMD_SENSE_PORT	 = 0x4d,
	MLX4_CMD_HW_HEALTH_CHECK = 0x50,
	MLX4_CMD_SET_PORT	 = 0xc,
	MLX4_CMD_SET_NODE	 = 0x5a,
	MLX4_CMD_ACCESS_DDR	 = 0x2e,
	MLX4_CMD_MAP_ICM	 = 0xffa,
	MLX4_CMD_UNMAP_ICM	 = 0xff9,
	MLX4_CMD_MAP_ICM_AUX	 = 0xffc,
	MLX4_CMD_UNMAP_ICM_AUX	 = 0xffb,
	MLX4_CMD_SET_ICM_SIZE	 = 0xffd,

	/* TPT commands */
	MLX4_CMD_SW2HW_MPT	 = 0xd,
	MLX4_CMD_QUERY_MPT	 = 0xe,
	MLX4_CMD_HW2SW_MPT	 = 0xf,
	MLX4_CMD_READ_MTT	 = 0x10,
	MLX4_CMD_WRITE_MTT	 = 0x11,
	MLX4_CMD_SYNC_TPT	 = 0x2f,

	/* EQ commands */
	MLX4_CMD_MAP_EQ		 = 0x12,
	MLX4_CMD_SW2HW_EQ	 = 0x13,
	MLX4_CMD_HW2SW_EQ	 = 0x14,
	MLX4_CMD_QUERY_EQ	 = 0x15,

	/* CQ commands */
	MLX4_CMD_SW2HW_CQ	 = 0x16,
	MLX4_CMD_HW2SW_CQ	 = 0x17,
	MLX4_CMD_QUERY_CQ	 = 0x18,
	MLX4_CMD_MODIFY_CQ	 = 0x2c,

	/* SRQ commands */
	MLX4_CMD_SW2HW_SRQ	 = 0x35,
	MLX4_CMD_HW2SW_SRQ	 = 0x36,
	MLX4_CMD_QUERY_SRQ	 = 0x37,
	MLX4_CMD_ARM_SRQ	 = 0x40,

	/* QP/EE commands */
	MLX4_CMD_RST2INIT_QP	 = 0x19,
	MLX4_CMD_INIT2RTR_QP	 = 0x1a,
	MLX4_CMD_RTR2RTS_QP	 = 0x1b,
	MLX4_CMD_RTS2RTS_QP	 = 0x1c,
	MLX4_CMD_SQERR2RTS_QP	 = 0x1d,
	MLX4_CMD_2ERR_QP	 = 0x1e,
	MLX4_CMD_RTS2SQD_QP	 = 0x1f,
	MLX4_CMD_SQD2SQD_QP	 = 0x38,
	MLX4_CMD_SQD2RTS_QP	 = 0x20,
	MLX4_CMD_2RST_QP	 = 0x21,
	MLX4_CMD_QUERY_QP	 = 0x22,
	MLX4_CMD_INIT2INIT_QP	 = 0x2d,
	MLX4_CMD_SUSPEND_QP	 = 0x32,
	MLX4_CMD_UNSUSPEND_QP	 = 0x33,
	/* special QP and management commands */
	MLX4_CMD_CONF_SPECIAL_QP = 0x23,
	MLX4_CMD_MAD_IFC	 = 0x24,

	/* multicast commands */
	MLX4_CMD_READ_MCG	 = 0x25,
	MLX4_CMD_WRITE_MCG	 = 0x26,
	MLX4_CMD_MGID_HASH	 = 0x27,

	/* miscellaneous commands */
	MLX4_CMD_DIAG_RPRT	 = 0x30,
	MLX4_CMD_NOP		 = 0x31,

	/* debug commands */
	MLX4_CMD_QUERY_DEBUG_MSG = 0x2a,
	MLX4_CMD_SET_DEBUG_MSG	 = 0x2b,
};

enum {
	MLX4_CMD_TIME_CLASS_A	= 10000,
	MLX4_CMD_TIME_CLASS_B	= 10000,
	MLX4_CMD_TIME_CLASS_C	= 10000,
};
enum {
	/* set port opcode modifiers */
	MLX4_SET_PORT_GENERAL   = 0x0,
	MLX4_SET_PORT_RQP_CALC  = 0x1,
	MLX4_SET_PORT_MAC_TABLE = 0x2,
	MLX4_SET_PORT_VLAN_TABLE = 0x3,
	MLX4_SET_PORT_PRIO_MAP  = 0x4,
	MLX4_SET_PORT_GID_TABLE = 0x5,
};

struct mlx4_cmd_context {
//	struct completion	done;
	int			result;
	int			next;
	u64			out_param;
	u16			token;
};

struct mlx4_cmd {
	unsigned char* pool;
	unsigned char* hcr;
	//struct mutex		hcr_mutex;
	//struct semaphore	poll_sem;
	//struct semaphore	event_sem;
	int			max_cmds;
	//spinlock_t		context_lock;
	int			free_head;
	struct mlx4_cmd_context *context;
	UINT16			token_mask;
	UINT8			use_events;
	UINT8			toggle;
};

enum mlx4_port_type {
	MLX4_PORT_TYPE_IB	= 1,
	MLX4_PORT_TYPE_ETH	= 2,
	MLX4_PORT_TYPE_AUTO	= 3
};

struct mlx4_caps {
	UINT64			fw_ver;
	int			num_ports;
	int			vl_cap[MLX4_MAX_PORTS + 1];
	int			ib_mtu_cap[MLX4_MAX_PORTS + 1];
	UINT32			ib_port_def_cap[MLX4_MAX_PORTS + 1];
	UINT64			def_mac[MLX4_MAX_PORTS + 1];
	int			eth_mtu_cap[MLX4_MAX_PORTS + 1];
	int			gid_table_len[MLX4_MAX_PORTS + 1];
	int			pkey_table_len[MLX4_MAX_PORTS + 1];
	int			trans_type[MLX4_MAX_PORTS + 1];
	int			vendor_oui[MLX4_MAX_PORTS + 1];
	int			wavelength[MLX4_MAX_PORTS + 1];
	UINT64			trans_code[MLX4_MAX_PORTS + 1];
	int			local_ca_ack_delay;
	int			num_uars;
	int			bf_reg_size;
	int			bf_regs_per_page;
	int			max_sq_sg;
	int			max_rq_sg;
	int			num_qps;
	int			max_wqes;
	int			max_sq_desc_sz;
	int			max_rq_desc_sz;
	int			max_qp_init_rdma;
	int			max_qp_dest_rdma;
	int			sqp_start;
	int			num_srqs;
	int			max_srq_wqes;
	int			max_srq_sge;
	int			reserved_srqs;
	int			num_cqs;
	int			max_cqes;
	int			reserved_cqs;
	int			num_eqs;
	int			reserved_eqs;
	int			num_comp_vectors;
	int			comp_pool;
	int			num_mpts;
	int			num_mtt_segs;
	int			mtts_per_seg;
	int			fmr_reserved_mtts;
	int			reserved_mtts;
	int			reserved_mrws;
	int			reserved_uars;
	int			num_mgms;
	int			num_amgms;
	int			reserved_mcgs;
	int			num_qp_per_mgm;
	int			num_pds;
	int			reserved_pds;
	int			mtt_entry_sz;
	UINT32			max_msg_sz;
	UINT32			page_size_cap;
	UINT32			flags;
	UINT32			bmme_flags;
	UINT32			reserved_lkey;
	UINT16			stat_rate_support;
	int			udp_rss;
	int			loopback_support;
	int			vep_uc_steering;
	int			vep_mc_steering;
	int			wol;
	UINT8			port_width_cap[MLX4_MAX_PORTS + 1];
	int			max_gso_sz;
	int                     reserved_qps_cnt[MLX4_NUM_QP_REGION];
	int			reserved_qps;
	int                     reserved_qps_base[MLX4_NUM_QP_REGION];
	int                     log_num_macs;
	int                     log_num_vlans;
	int                     log_num_prios;
	enum mlx4_port_type	port_type[MLX4_MAX_PORTS + 1];
	UINT8			supported_type[MLX4_MAX_PORTS + 1];
	UINT32			port_mask;
	enum mlx4_port_type	possible_type[MLX4_MAX_PORTS + 1];
};

struct mlx4_device
{
	struct Device* device;
	unsigned char* dcs;
	unsigned char* uar;
	struct mlx4_cmd cmd;
	struct mlx4_caps caps;
};

static struct mlx4_device g_Device[DRIVER_MAX_DEVICES];
static unsigned int g_DeviceCount = 0;

static struct ibv_context mlx_context[DRIVER_MAX_CONTEXT];

static struct ibv_pd* alloc_pd(struct ibv_context *context)
{
	return 0;
}

static int dealloc_pd(struct ibv_pd *pd)
{
	return 0;
}

static struct ibv_context* alloc_context(struct ibv_device *device, int cmd_fd)
{
	struct ibv_context* ctx = &mlx_context[0];
	ctx->device = device;
/*
	int			(*query_device)(struct ibv_context *context,
					      struct ibv_device_attr *device_attr);
	int			(*query_port)(struct ibv_context *context, uint8_t port_num,
					      struct ibv_port_attr *port_attr);*/
	ctx->ops.alloc_pd = alloc_pd;
	ctx->ops.dealloc_pd =dealloc_pd;
/*	struct ibv_mr *		(*reg_mr)(struct ibv_pd *pd, void *addr, size_t length,
					  int access);
	struct ibv_mr *		(*rereg_mr)(struct ibv_mr *mr,
					    int flags,
					    struct ibv_pd *pd, void *addr,
					    size_t length,
					    int access);
	int			(*dereg_mr)(struct ibv_mr *mr);
	struct ibv_mw *		(*alloc_mw)(struct ibv_pd *pd, enum ibv_mw_type type);
	int			(*bind_mw)(struct ibv_qp *qp, struct ibv_mw *mw,
					   struct ibv_mw_bind *mw_bind);
	int			(*dealloc_mw)(struct ibv_mw *mw);
	struct ibv_cq *		(*create_cq)(struct ibv_context *context, int cqe,
					     struct ibv_comp_channel *channel,
					     int comp_vector);
	int			(*poll_cq)(struct ibv_cq *cq, int num_entries, struct ibv_wc *wc);
	int			(*req_notify_cq)(struct ibv_cq *cq, int solicited_only);
	void			(*cq_event)(struct ibv_cq *cq);
	int			(*resize_cq)(struct ibv_cq *cq, int cqe);
	int			(*destroy_cq)(struct ibv_cq *cq);
	struct ibv_srq *	(*create_srq)(struct ibv_pd *pd,
					      struct ibv_srq_init_attr *srq_init_attr);
	int			(*modify_srq)(struct ibv_srq *srq,
					      struct ibv_srq_attr *srq_attr,
					      int srq_attr_mask);
	int			(*query_srq)(struct ibv_srq *srq,
					     struct ibv_srq_attr *srq_attr);
	int			(*destroy_srq)(struct ibv_srq *srq);
	int			(*post_srq_recv)(struct ibv_srq *srq,
						 struct ibv_recv_wr *recv_wr,
						 struct ibv_recv_wr **bad_recv_wr);
	struct ibv_qp *		(*create_qp)(struct ibv_pd *pd, struct ibv_qp_init_attr *attr);
	int			(*query_qp)(struct ibv_qp *qp, struct ibv_qp_attr *attr,
					    int attr_mask,
					    struct ibv_qp_init_attr *init_attr);
	int			(*modify_qp)(struct ibv_qp *qp, struct ibv_qp_attr *attr,
					     int attr_mask);
	int			(*destroy_qp)(struct ibv_qp *qp);
	int			(*post_send)(struct ibv_qp *qp, struct ibv_send_wr *wr,
					     struct ibv_send_wr **bad_wr);
	int			(*post_recv)(struct ibv_qp *qp, struct ibv_recv_wr *wr,
					     struct ibv_recv_wr **bad_wr);
	struct ibv_ah *		(*create_ah)(struct ibv_pd *pd, struct ibv_ah_attr *attr);
	int			(*destroy_ah)(struct ibv_ah *ah);
	int			(*attach_mcast)(struct ibv_qp *qp, const union ibv_gid *gid,
						uint16_t lid);
	int			(*detach_mcast)(struct ibv_qp *qp, const union ibv_gid *gid,
						uint16_t lid);
	void			(*async_event)(struct ibv_async_event *event);
*/
	ctx->cmd_fd = cmd_fd;
	ctx->async_fd = 0;
	ctx->num_comp_vectors = 0;
	ctx->abi_compat = 0;
	return ctx;
}

static void free_context(struct ibv_context *context)
{
}



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
	while (cmd_pending(dev) && L4_SystemClock().raw<end.raw)
		L4_Yield();

/*	end = jiffies;
	if (event)
		end += msecs_to_jiffies(GO_BIT_TIMEOUT_MSECS);

	while (cmd_pending(dev)) {
		if (time_after_eq(jiffies, end))
			goto out;
		cond_resched();
	}
*/
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

#define CMD_POLL_TOKEN 0xffff
static int mlx4_cmd_poll(struct mlx4_device *dev, u64 in_param, u64 *out_param,
			 int out_is_imm, u32 in_modifier, u8 op_modifier,
			 u16 op, unsigned long timeout)
{
	unsigned char* hcr = dev->cmd.hcr;
DRIVER_LOG(DRIVER_PREFIX "mlx4_cmd_poll (%X, %04X)\n", in_param, op);
//	down(&priv->cmd.poll_sem);

	int err = mlx4_cmd_post(dev, in_param, out_param ? *out_param : 0,
			    in_modifier, op_modifier, op, CMD_POLL_TOKEN, 0);
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
		err = 1;//-ETIMEDOUT;
		DRIVER_LOG(DRIVER_PREFIX "timeout\n");
		goto out;
	}

	if (out_is_imm)
		*out_param =
			(u64) bs32(*(UINT32*)(hcr + HCR_OUT_PARAM_OFFSET)) << 32 |
			(u64) bs32(*(UINT32*)(hcr + HCR_OUT_PARAM_OFFSET + 4));

	if ((bs32(*(UINT32*)(hcr + HCR_STATUS_OFFSET)) >> 24) ==0)
		err = 0;
	else
	{
		DRIVER_LOG(DRIVER_PREFIX "status error\n");
		err = 1;
	}

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

	context->result = status;//mlx4_status_to_errno(status);
	context->out_param = out_param;

//	complete(&context->done);
}

static int mlx4_cmd_wait(struct mlx4_device *dev, u64 in_param, u64 *out_param,
			 int out_is_imm, u32 in_modifier, u8 op_modifier,
			 u16 op, unsigned long timeout)
{
//	struct mlx4_cmd *cmd = &mlx4_priv(dev)->cmd;
//	struct mlx4_cmd_context *context;
	int err = 0;
/* TODO
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

	if (!wait_for_completion_timeout(&context->done, msecs_to_jiffies(timeout))) {
		err = -EBUSY;
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
*/
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

/* Invoke a command with no output parameter */
static inline int mlx4_cmd(struct mlx4_device *dev, u64 in_param, u32 in_modifier,
			   u8 op_modifier, u16 op, unsigned long timeout)
{
	return __mlx4_cmd(dev, in_param, 0, 0, in_modifier,
			  op_modifier, op, timeout);
}

/* Invoke a command with an output mailbox */
static inline int mlx4_cmd_box(struct mlx4_device *dev, u64 in_param, u64 out_param,
			       u32 in_modifier, u8 op_modifier, u16 op,
			       unsigned long timeout)
{
	return __mlx4_cmd(dev, in_param, &out_param, 0, in_modifier,
			  op_modifier, op, timeout);
}

int infiniband_mlx4_init(struct Device* device, struct ibv_device* ibv_dev)
{
	if (g_DeviceCount>=DRIVER_MAX_DEVICES)
	{
		DRIVER_LOG(DRIVER_PREFIX "Cannot allocate enough space for more Devices\n");
		return 1;
	}
/*	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot enable PCI device, "
			"aborting.\n");
		return err;
	}
*/
	// Check for BARs.  We expect 0: 1MB
// hack for qemu, just use memory pages
//	if ((device->pcie.barType[2] & 6)!=4)
	if ((device->pcie.barType[0] & 1)!=0
		|| device->pcie.barSize[0] != 1 << 20)
	{
		DRIVER_LOG(DRIVER_PREFIX "Missing DCS, aborting. %X:%X %X %X:%X\n", device->pcie.bar[1], device->pcie.bar[0], device->pcie.barType[0], device->pcie.barSize[1], device->pcie.barSize[0]);
		return 1;
	}
//	if ((device->pcie.barType[2] & 6)!=4)
	if ((device->pcie.barType[0] & 1)!=0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Missing UAR, aborting. %X:%X %X %X:%X\n", device->pcie.bar[3], device->pcie.bar[2], device->pcie.barType[2], device->pcie.barSize[3], device->pcie.barSize[2]);
		return 1;
	}
	struct mlx4_device* mlx4 = &g_Device[g_DeviceCount];
	mlx4->dcs= platform_map((void*)((L4_Word_t)device->pcie.bar[0] + ((L4_Word_t)device->pcie.bar[1] << 32)), 1 << 20);
	mlx4->uar = platform_map((void*)((L4_Word_t)device->pcie.bar[2] + ((L4_Word_t)device->pcie.bar[3] << 32)), (L4_Word_t)device->pcie.barSize[2] + ((L4_Word_t)device->pcie.barSize[3] << 32));
	if (mlx4->dcs==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Couldn't map DCS.\n");
		return 1;
	}
	if (mlx4->uar==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Couldn't map UAR.\n");
		return 1;
	}
	ibv_dev->ops.alloc_context = alloc_context;
	ibv_dev->ops.free_context = free_context;
	// init cmd
	mlx4->cmd.pool = platform_mapAny(4096);
	if (mlx4->cmd.pool==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Couldn't allocate cmd pool.\n");
		return 1;
	}
	mlx4->cmd.hcr = ((unsigned char*)mlx4->dcs)+0x80680;
	mlx4->cmd.use_events = 0;
	mlx4->cmd.toggle = 1;
	{	// reset hardware to prevent undefined state from boot rom
		DRIVER_LOG(DRIVER_PREFIX "reseting device...\n");
#define MLX4_RESET_BASE		0xf0000
#define MLX4_RESET_SIZE		  0x400
#define MLX4_SEM_OFFSET		  0x3fc
#define MLX4_RESET_OFFSET	   0x10
#define MLX4_RESET_VALUE	bs32(1)

		/*
		 * Reset the chip.  This is somewhat ugly because we have to
		 * save off the PCI header before reset and then restore it
		 * after the chip reboots.  We skip config space offsets 22
		 * and 23 since those have a special meaning.
		 */

		UINT32* hca_header = mlx4->cmd.pool;	// misuse cmd pool
//		pcie_cap = pci_find_capability(dev->pdev, PCI_CAP_ID_EXP);

		int i;
		for (i = 0; i < 64; ++i) {
			if (i == 22 || i == 23)
				continue;
			hca_header[i] = device->pcie.read_config(&device->pcie, i*4);
		}
//memory barrier
mmiowb();
		unsigned char* reset = ((unsigned char*)mlx4->dcs) + MLX4_RESET_BASE;
		// grab HW semaphore to lock out flash updates
		L4_Clock_t end = L4_SystemClock();
		end = L4_ClockAddUsec(end, 10*1000*1000);
		int sem;
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
mmiowb();
		// actually hit reset
		*(UINT32*)(reset + MLX4_RESET_OFFSET) = MLX4_RESET_VALUE;

		// Docs say to wait one second before accessing device
		sleep(1000);
mmiowb();
		unsigned short id;
		end = L4_SystemClock();
		end = L4_ClockAddUsec(end, 2*1000*1000);
		do
		{
			id = device->pcie.read_config_word(&device->pcie, 0);
			if (id != 0xFFFF)
				break;
			sleep(1);
		} while (L4_SystemClock().raw<end.raw);

		if (id == 0xFFFF) {
			DRIVER_LOG(DRIVER_PREFIX "PCI device did not come back after reset, "
				  "aborting.\n");
			return 1;
		}

		// Now restore the PCI headers
		if (device->pcie.pcieCap)
		{
			device->pcie.write_config_word(&device->pcie, device->pcie.pcieCap + 8, hca_header[(device->pcie.pcieCap + 8) / 4]);
			device->pcie.write_config_word(&device->pcie, device->pcie.pcieCap + 16, hca_header[(device->pcie.pcieCap + 16) / 4]);
		}
		// without?
		for (i = 0; i < 16; ++i) {
			if (i == 1)
				continue;
			device->pcie.write_config(&device->pcie, i*4, hca_header[i]);
		}
		// restore pci command
		device->pcie.write_config(&device->pcie, 4, hca_header[1]);
		DRIVER_LOG(DRIVER_PREFIX "reset device\n");
	}
	int err;
	{	// init HCA
		{	// query FW
#define QUERY_FW_OUT_SIZE             0x100
#define QUERY_FW_VER_OFFSET            0x00
#define QUERY_FW_CMD_IF_REV_OFFSET     0x0a
#define QUERY_FW_MAX_CMD_OFFSET        0x0f
#define QUERY_FW_ERR_START_OFFSET      0x30
#define QUERY_FW_ERR_SIZE_OFFSET       0x38
#define QUERY_FW_ERR_BAR_OFFSET        0x3c

#define QUERY_FW_SIZE_OFFSET           0x00
#define QUERY_FW_CLR_INT_BASE_OFFSET   0x20
#define QUERY_FW_CLR_INT_BAR_OFFSET    0x28

			unsigned char* outbox = (unsigned char*)mlx4->cmd.pool;
			
			err = mlx4_cmd_box(mlx4, 0, outbox, 0, 0, MLX4_CMD_QUERY_FW,
					    MLX4_CMD_TIME_CLASS_A);
			if (err)
			{
				DRIVER_LOG(DRIVER_PREFIX "Couldn't send cmd MLX4_CMD_QUERY_FW.\n");
				return 1;
			}

			UINT64 fw_ver = bs64(*(UINT64*)(outbox+QUERY_FW_VER_OFFSET));
			// FW subminor version is at more significant bits than minor
			//version, so swap here.
			mlx4->caps.fw_ver = (fw_ver & 0xffff00000000) |
				((fw_ver & 0xffff0000) >> 16) |
				((fw_ver & 0x0000ffff) << 16);
			DRIVER_LOG(DRIVER_PREFIX "(Installed FW version is %d.%d.%03d)\n",
				 (int) (mlx4->caps.fw_ver >> 32),
				 (int) (mlx4->caps.fw_ver >> 16) & 0xffff,
				 (int) mlx4->caps.fw_ver & 0xffff);

			UINT16 cmd_if_rev = bs16(*(UINT64*)(outbox+QUERY_FW_CMD_IF_REV_OFFSET));
			if (cmd_if_rev < 2 || cmd_if_rev > 3)
			{
				DRIVER_LOG(DRIVER_PREFIX "Installed FW has unsupported "
					 "command interface revision %d.\n",
					 cmd_if_rev);
				DRIVER_LOG(DRIVER_PREFIX "This driver version supports only revisions 2 to 3.\n");
				return 1;
			}

			if (cmd_if_rev < 3)
			{
				DRIVER_LOG(DRIVER_PREFIX "using old port cmds\n");
//				mlx4->flags |= MLX4_FLAG_OLD_PORT_CMDS;
			}
			unsigned char lg = *(outbox+QUERY_FW_MAX_CMD_OFFSET);
			mlx4->cmd.max_cmds = 1 << lg;

			DRIVER_LOG(DRIVER_PREFIX "Command interface rev %d, max commands %d\n",
				 cmd_if_rev, mlx4->cmd.max_cmds);

/*			MLX4_GET(fw->catas_offset, outbox, QUERY_FW_ERR_START_OFFSET);
			MLX4_GET(fw->catas_size,   outbox, QUERY_FW_ERR_SIZE_OFFSET);
			MLX4_GET(fw->catas_bar,    outbox, QUERY_FW_ERR_BAR_OFFSET);
			fw->catas_bar = (fw->catas_bar >> 6) * 2;

			DRIVER_LOG(DRIVER_PREFIX "Catastrophic error buffer at 0x%llx, size 0x%x, BAR %d\n",
				 (unsigned long long) fw->catas_offset, fw->catas_size, fw->catas_bar);

			MLX4_GET(fw->fw_pages,     outbox, QUERY_FW_SIZE_OFFSET);
			MLX4_GET(fw->clr_int_base, outbox, QUERY_FW_CLR_INT_BASE_OFFSET);
			MLX4_GET(fw->clr_int_bar,  outbox, QUERY_FW_CLR_INT_BAR_OFFSET);
			fw->clr_int_bar = (fw->clr_int_bar >> 6) * 2;

			mlx4_dbg(dev, "FW size %d KB\n", fw->fw_pages >> 2);

			 // Round up number of system pages needed in case
			// MLX4_ICM_PAGE_SIZE < PAGE_SIZE.
			fw->fw_pages =
				ALIGN(fw->fw_pages, PAGE_SIZE / MLX4_ICM_PAGE_SIZE) >>
				(PAGE_SHIFT - MLX4_ICM_PAGE_SHIFT);

			mlx4_dbg(dev, "Clear int @ %llx, BAR %d\n",
				 (unsigned long long) fw->clr_int_base, fw->clr_int_bar);
*/
		}
		// blue flame

		{	// init HCA
#define INIT_HCA_IN_SIZE		 0x200
#define INIT_HCA_VERSION_OFFSET		 0x000
#define	 INIT_HCA_VERSION		 2
#define INIT_HCA_CACHELINE_SZ_OFFSET	 0x0e
#define INIT_HCA_FLAGS_OFFSET		 0x014
#define INIT_HCA_QPC_OFFSET		 0x020
#define	 INIT_HCA_QPC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x10)
#define	 INIT_HCA_LOG_QP_OFFSET		 (INIT_HCA_QPC_OFFSET + 0x17)
#define	 INIT_HCA_SRQC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x28)
#define	 INIT_HCA_LOG_SRQ_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x2f)
#define	 INIT_HCA_CQC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x30)
#define	 INIT_HCA_LOG_CQ_OFFSET		 (INIT_HCA_QPC_OFFSET + 0x37)
#define	 INIT_HCA_ALTC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x40)
#define	 INIT_HCA_AUXC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x50)
#define	 INIT_HCA_EQC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x60)
#define	 INIT_HCA_LOG_EQ_OFFSET		 (INIT_HCA_QPC_OFFSET + 0x67)
#define	 INIT_HCA_RDMARC_BASE_OFFSET	 (INIT_HCA_QPC_OFFSET + 0x70)
#define	 INIT_HCA_LOG_RD_OFFSET		 (INIT_HCA_QPC_OFFSET + 0x77)
#define INIT_HCA_MCAST_OFFSET		 0x0c0
#define	 INIT_HCA_MC_BASE_OFFSET	 (INIT_HCA_MCAST_OFFSET + 0x00)
#define	 INIT_HCA_LOG_MC_ENTRY_SZ_OFFSET (INIT_HCA_MCAST_OFFSET + 0x12)
#define	 INIT_HCA_LOG_MC_HASH_SZ_OFFSET	 (INIT_HCA_MCAST_OFFSET + 0x16)
#define  INIT_HCA_UC_STEERING_OFFSET	 (INIT_HCA_MCAST_OFFSET + 0x18)
#define	 INIT_HCA_LOG_MC_TABLE_SZ_OFFSET (INIT_HCA_MCAST_OFFSET + 0x1b)
#define INIT_HCA_TPT_OFFSET		 0x0f0
#define	 INIT_HCA_DMPT_BASE_OFFSET	 (INIT_HCA_TPT_OFFSET + 0x00)
#define	 INIT_HCA_LOG_MPT_SZ_OFFSET	 (INIT_HCA_TPT_OFFSET + 0x0b)
#define	 INIT_HCA_MTT_BASE_OFFSET	 (INIT_HCA_TPT_OFFSET + 0x10)
#define	 INIT_HCA_CMPT_BASE_OFFSET	 (INIT_HCA_TPT_OFFSET + 0x18)
#define INIT_HCA_UAR_OFFSET		 0x120
#define	 INIT_HCA_LOG_UAR_SZ_OFFSET	 (INIT_HCA_UAR_OFFSET + 0x0a)
#define  INIT_HCA_UAR_PAGE_SZ_OFFSET     (INIT_HCA_UAR_OFFSET + 0x0b)
/*
			UINT32* inbox = (UINT32*)mlx4->cmd.pool;
			memset(inbox, 0, INIT_HCA_IN_SIZE);

			*(((UINT8 *) inbox)+ INIT_HCA_VERSION_OFFSET) = INIT_HCA_VERSION;

			*(((UINT8 *) inbox)+ INIT_HCA_CACHELINE_SZ_OFFSET) =
	//			(ilog2(cache_line_size()) - 4) << 5;	// TODO

			*(inbox + INIT_HCA_FLAGS_OFFSET / 4) &= ~(1 << 1);
			// Check port for UD address vector:
			*(inbox + INIT_HCA_FLAGS_OFFSET / 4) |= 1;

			// QPC/EEC/CQC/EQC/RDMARC attributes

			MLX4_PUT(inbox, param->qpc_base,      INIT_HCA_QPC_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_num_qps,   INIT_HCA_LOG_QP_OFFSET);
			MLX4_PUT(inbox, param->srqc_base,     INIT_HCA_SRQC_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_num_srqs,  INIT_HCA_LOG_SRQ_OFFSET);
			MLX4_PUT(inbox, param->cqc_base,      INIT_HCA_CQC_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_num_cqs,   INIT_HCA_LOG_CQ_OFFSET);
			MLX4_PUT(inbox, param->altc_base,     INIT_HCA_ALTC_BASE_OFFSET);
			MLX4_PUT(inbox, param->auxc_base,     INIT_HCA_AUXC_BASE_OFFSET);
			MLX4_PUT(inbox, param->eqc_base,      INIT_HCA_EQC_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_num_eqs,   INIT_HCA_LOG_EQ_OFFSET);
			MLX4_PUT(inbox, param->rdmarc_base,   INIT_HCA_RDMARC_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_rd_per_qp, INIT_HCA_LOG_RD_OFFSET);

			// multicast attributes

			MLX4_PUT(inbox, param->mc_base,		INIT_HCA_MC_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_mc_entry_sz, INIT_HCA_LOG_MC_ENTRY_SZ_OFFSET);
			MLX4_PUT(inbox, param->log_mc_hash_sz,  INIT_HCA_LOG_MC_HASH_SZ_OFFSET);
			if (dev->caps.vep_mc_steering)
				MLX4_PUT(inbox, (UINT8) (1 << 3),	INIT_HCA_UC_STEERING_OFFSET);
			MLX4_PUT(inbox, param->log_mc_table_sz, INIT_HCA_LOG_MC_TABLE_SZ_OFFSET);

			// TPT attributes

			MLX4_PUT(inbox, param->dmpt_base,  INIT_HCA_DMPT_BASE_OFFSET);
			MLX4_PUT(inbox, param->log_mpt_sz, INIT_HCA_LOG_MPT_SZ_OFFSET);
			MLX4_PUT(inbox, param->mtt_base,   INIT_HCA_MTT_BASE_OFFSET);
			MLX4_PUT(inbox, param->cmpt_base,  INIT_HCA_CMPT_BASE_OFFSET);

			// UAR attributes

			MLX4_PUT(inbox, (UINT8) (0), INIT_HCA_UAR_PAGE_SZ_OFFSET);
			MLX4_PUT(inbox, param->log_uar_sz,      INIT_HCA_LOG_UAR_SZ_OFFSET);

			err = mlx4_cmd(dev, mailbox->dma, 0, 0, MLX4_CMD_INIT_HCA, 10000);
			if (err!=0)
			{
				return 1;
			}*/
		}
		{	// query adapter
		}
	}
	// read board id
	
	// allocate eq table
	
	// enable msix
	
	// enable steering
	
	{	// setup hca
		// init tables
		// enable cmd events
		// test event handling (send NOP)
	}
	// init ports
	
	// sensing
DRIVER_LOG(DRIVER_PREFIX "not fully implemented\n");
return 1;
	DRIVER_LOG(DRIVER_PREFIX "success\n");
	return 0;
}
