/*
 * Copyright (c) 2004, 2005 Topspin Communications.  All rights reserved.
 * Copyright (c) 2005 Sun Microsystems, Inc. All rights reserved.
 * Copyright (c) 2005, 2006, 2007 Cisco Systems.  All rights reserved.
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

#ifndef MLX4_H
#define MLX4_H

#include "../../acpi/platform.h"
/*#include <linux/mutex.h>
#include <linux/radix-tree.h>
#include <linux/timer.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
*/
#include "../list.h"
#include "../atomic.h"
#include "../bitmap.h"
#include "../completion.h"
#include "../../pcie/pcie.h"
//#include "doorbell.h"
#include "log.h"
#include <verbs.h>

#define DRV_NAME	"mlx4_core"
#define DRV_VERSION	"0.01"
#define DRV_RELDATE	"May 1, 2007"

#define MAX_MSIX_P_PORT		17
#define MAX_MSIX		64
#define MSIX_LEGACY_SZ		4
#define MIN_MSIX_P_PORT		5

enum {
	MLX4_FLAG_MSI_X		= 1 << 0,
	MLX4_FLAG_OLD_PORT_CMDS	= 1 << 1,
};

enum {
	MLX4_MAX_PORTS		= 2
};

enum {
	MLX4_BOARD_ID_LEN = 64
};

enum {
	MLX4_DEV_CAP_FLAG_RC		= 1 <<  0,
	MLX4_DEV_CAP_FLAG_UC		= 1 <<  1,
	MLX4_DEV_CAP_FLAG_UD		= 1 <<  2,
	MLX4_DEV_CAP_FLAG_SRQ		= 1 <<  6,
	MLX4_DEV_CAP_FLAG_IPOIB_CSUM	= 1 <<  7,
	MLX4_DEV_CAP_FLAG_BAD_PKEY_CNTR	= 1 <<  8,
	MLX4_DEV_CAP_FLAG_BAD_QKEY_CNTR	= 1 <<  9,
	MLX4_DEV_CAP_FLAG_DPDP		= 1 << 12,
	MLX4_DEV_CAP_FLAG_BLH		= 1 << 15,
	MLX4_DEV_CAP_FLAG_MEM_WINDOW	= 1 << 16,
	MLX4_DEV_CAP_FLAG_APM		= 1 << 17,
	MLX4_DEV_CAP_FLAG_ATOMIC	= 1 << 18,
	MLX4_DEV_CAP_FLAG_RAW_MCAST	= 1 << 19,
	MLX4_DEV_CAP_FLAG_UD_AV_PORT	= 1 << 20,
	MLX4_DEV_CAP_FLAG_UD_MCAST	= 1 << 21,
	MLX4_DEV_CAP_FLAG_IBOE		= 1 << 30
};

enum {
	MLX4_BMME_FLAG_LOCAL_INV	= 1 <<  6,
	MLX4_BMME_FLAG_REMOTE_INV	= 1 <<  7,
	MLX4_BMME_FLAG_TYPE_2_WIN	= 1 <<  9,
	MLX4_BMME_FLAG_RESERVED_LKEY	= 1 << 10,
	MLX4_BMME_FLAG_FAST_REG_WR	= 1 << 11,
};

enum mlx4_event {
	MLX4_EVENT_TYPE_COMP		   = 0x00,
	MLX4_EVENT_TYPE_PATH_MIG	   = 0x01,
	MLX4_EVENT_TYPE_COMM_EST	   = 0x02,
	MLX4_EVENT_TYPE_SQ_DRAINED	   = 0x03,
	MLX4_EVENT_TYPE_SRQ_QP_LAST_WQE	   = 0x13,
	MLX4_EVENT_TYPE_SRQ_LIMIT	   = 0x14,
	MLX4_EVENT_TYPE_CQ_ERROR	   = 0x04,
	MLX4_EVENT_TYPE_WQ_CATAS_ERROR	   = 0x05,
	MLX4_EVENT_TYPE_EEC_CATAS_ERROR	   = 0x06,
	MLX4_EVENT_TYPE_PATH_MIG_FAILED	   = 0x07,
	MLX4_EVENT_TYPE_WQ_INVAL_REQ_ERROR = 0x10,
	MLX4_EVENT_TYPE_WQ_ACCESS_ERROR	   = 0x11,
	MLX4_EVENT_TYPE_SRQ_CATAS_ERROR	   = 0x12,
	MLX4_EVENT_TYPE_LOCAL_CATAS_ERROR  = 0x08,
	MLX4_EVENT_TYPE_PORT_CHANGE	   = 0x09,
	MLX4_EVENT_TYPE_EQ_OVERFLOW	   = 0x0f,
	MLX4_EVENT_TYPE_ECC_DETECT	   = 0x0e,
	MLX4_EVENT_TYPE_CMD		   = 0x0a
};

enum {
	MLX4_PORT_CHANGE_SUBTYPE_DOWN	= 1,
	MLX4_PORT_CHANGE_SUBTYPE_ACTIVE	= 4
};

enum {
	MLX4_PERM_LOCAL_READ	= 1 << 10,
	MLX4_PERM_LOCAL_WRITE	= 1 << 11,
	MLX4_PERM_REMOTE_READ	= 1 << 12,
	MLX4_PERM_REMOTE_WRITE	= 1 << 13,
	MLX4_PERM_ATOMIC	= 1 << 14
};

enum {
	MLX4_OPCODE_NOP			= 0x00,
	MLX4_OPCODE_SEND_INVAL		= 0x01,
	MLX4_OPCODE_RDMA_WRITE		= 0x08,
	MLX4_OPCODE_RDMA_WRITE_IMM	= 0x09,
	MLX4_OPCODE_SEND		= 0x0a,
	MLX4_OPCODE_SEND_IMM		= 0x0b,
	MLX4_OPCODE_LSO			= 0x0e,
	MLX4_OPCODE_RDMA_READ		= 0x10,
	MLX4_OPCODE_ATOMIC_CS		= 0x11,
	MLX4_OPCODE_ATOMIC_FA		= 0x12,
	MLX4_OPCODE_MASKED_ATOMIC_CS	= 0x14,
	MLX4_OPCODE_MASKED_ATOMIC_FA	= 0x15,
	MLX4_OPCODE_BIND_MW		= 0x18,
	MLX4_OPCODE_FMR			= 0x19,
	MLX4_OPCODE_LOCAL_INVAL		= 0x1b,
	MLX4_OPCODE_CONFIG_CMD		= 0x1f,

	MLX4_RECV_OPCODE_RDMA_WRITE_IMM	= 0x00,
	MLX4_RECV_OPCODE_SEND		= 0x01,
	MLX4_RECV_OPCODE_SEND_IMM	= 0x02,
	MLX4_RECV_OPCODE_SEND_INVAL	= 0x03,

	MLX4_CQE_OPCODE_ERROR		= 0x1e,
	MLX4_CQE_OPCODE_RESIZE		= 0x16,
};

enum {
	MLX4_STAT_RATE_OFFSET	= 5
};

enum mlx4_protocol {
	MLX4_PROT_IB_IPV6 = 0,
	MLX4_PROT_ETH,
	MLX4_PROT_IB_IPV4,
	MLX4_PROT_FCOE
};

enum mlx4_dev_event {
	MLX4_DEV_EVENT_CATASTROPHIC_ERROR,
	MLX4_DEV_EVENT_PORT_UP,
	MLX4_DEV_EVENT_PORT_DOWN,
	MLX4_DEV_EVENT_PORT_REINIT,
};
// TODO
struct mlx4_device;

struct mlx4_interface {
	void *			(*add)	 (struct mlx4_device *dev);
	void			(*remove)(struct mlx4_device *dev, void *context);
	void			(*event) (struct mlx4_device *dev, void *context,
					  enum mlx4_dev_event event, int port);
	void *			(*get_dev)(struct mlx4_device *dev, void *context, u8 port);
	struct list_head	list;
	enum mlx4_protocol	protocol;
};

int mlx4_register_interface(struct mlx4_interface *intf);
void mlx4_unregister_interface(struct mlx4_interface *intf);

void *mlx4_get_protocol_dev(struct mlx4_device *dev, enum mlx4_protocol proto, int port);
//----

enum {
	MLX4_MTT_FLAG_PRESENT		= 1
};

enum mlx4_qp_region {
	MLX4_QP_REGION_FW = 0,
	MLX4_QP_REGION_ETH_ADDR,
	MLX4_QP_REGION_FC_ADDR,
	MLX4_QP_REGION_FC_EXCH,
	MLX4_NUM_QP_REGION
};

enum mlx4_port_type {
	MLX4_PORT_TYPE_IB	= 1,
	MLX4_PORT_TYPE_ETH	= 2,
	MLX4_PORT_TYPE_AUTO	= 3
};

enum mlx4_special_vlan_idx {
	MLX4_NO_VLAN_IDX        = 0,
	MLX4_VLAN_MISS_IDX,
	MLX4_VLAN_REGULAR
};

enum mlx4_steer_type {
	MLX4_MC_STEER = 0,
	MLX4_UC_STEER,
	MLX4_NUM_STEERS
};

enum {
	MLX4_NUM_FEXCH          = 64 * 1024,
};

enum {
	MLX4_MAX_FAST_REG_PAGES = 511,
};

static inline u64 mlx4_fw_ver(u64 major, u64 minor, u64 subminor)
{
	return (major << 32) | (minor << 16) | subminor;
}

struct mlx4_caps {
	u64			fw_ver;
	int			num_ports;
	int			vl_cap[MLX4_MAX_PORTS + 1];
	int			ib_mtu_cap[MLX4_MAX_PORTS + 1];
	u32			ib_port_def_cap[MLX4_MAX_PORTS + 1];	// big endian
	u64			def_mac[MLX4_MAX_PORTS + 1];
	int			eth_mtu_cap[MLX4_MAX_PORTS + 1];
	int			gid_table_len[MLX4_MAX_PORTS + 1];
	int			pkey_table_len[MLX4_MAX_PORTS + 1];
	int			trans_type[MLX4_MAX_PORTS + 1];
	int			vendor_oui[MLX4_MAX_PORTS + 1];
	int			wavelength[MLX4_MAX_PORTS + 1];
	u64			trans_code[MLX4_MAX_PORTS + 1];
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
	u32			max_msg_sz;
	u32			page_size_cap;
	u32			flags;
	u32			bmme_flags;
	u32			reserved_lkey;
	u16			stat_rate_support;
	int			udp_rss;
	int			loopback_support;
	int			vep_uc_steering;
	int			vep_mc_steering;
	int			wol;
	u8			port_width_cap[MLX4_MAX_PORTS + 1];
	int			max_gso_sz;
	int                     reserved_qps_cnt[MLX4_NUM_QP_REGION];
	int			reserved_qps;
	int                     reserved_qps_base[MLX4_NUM_QP_REGION];
	int                     log_num_macs;
	int                     log_num_vlans;
	int                     log_num_prios;
	enum mlx4_port_type	port_type[MLX4_MAX_PORTS + 1];
	u8			supported_type[MLX4_MAX_PORTS + 1];
	u32			port_mask;
	enum mlx4_port_type	possible_type[MLX4_MAX_PORTS + 1];
};

struct mlx4_buf_list {
	void		       *buf;
	void*		map;	// same as buf because we use hw addr and no iommu
};

struct mlx4_buf {
	struct mlx4_buf_list	direct;
	struct mlx4_buf_list   *page_list;
	int			nbufs;
	int			npages;
	int			page_shift;
};

struct mlx4_mtt {
	u32			first_seg;
	int			order;
	int			page_shift;
};

enum {
	MLX4_DB_PER_PAGE = PAGE_SIZE / 4
};

struct mlx4_db_pgdir {
	struct list_head	list;
	DECLARE_BITMAP(order0, MLX4_DB_PER_PAGE);
	DECLARE_BITMAP(order1, MLX4_DB_PER_PAGE / 2);
	unsigned long	       *bits[2];
	u32		       *db_page;
	void*		db_dma;
};

struct mlx4_ib_user_db_page;

struct mlx4_db {
	u32			*db;
	union {
		struct mlx4_db_pgdir		*pgdir;
		struct mlx4_ib_user_db_page	*user_page;
	}			u;
	void*		dma;
	int			index;
	int			order;
};

struct mlx4_hwq_resources {
	struct mlx4_db		db;
	struct mlx4_mtt		mtt;
	struct mlx4_buf		buf;
};

struct mlx4_mr {
	struct mlx4_mtt		mtt;
	u64			iova;
	u64			size;
	u32			key;
	u32			pd;
	u32			access;
	int			enabled;
};

struct mlx4_fmr {
	struct mlx4_mr		mr;
	struct mlx4_mpt_entry  *mpt;
	u64		       *mtts;
	void*		dma_handle;
	int			max_pages;
	int			max_maps;
	int			maps;
	u8			page_shift;
};

struct mlx4_uar {
	unsigned long		pfn;
	int			index;
	struct list_head	bf_list;
	unsigned		free_bf_bmap;
	void* map;
	void* bf_map;
};

struct mlx4_bf {
	unsigned long		offset;
	int			buf_size;
	struct mlx4_uar	       *uar;
	void* reg;
};

struct mlx4_cq {
	void (*comp)		(struct mlx4_cq *);
	void (*event)		(struct mlx4_cq *, enum mlx4_event);

	struct mlx4_uar	       *uar;

	u32			cons_index;

	u32		       *set_ci_db;
	u32		       *arm_db;
	int			arm_sn;

	int			cqn;
	unsigned		vector;

	atomic_t		refcount;
	struct completion	free;
};

struct mlx4_qp {
	void (*event)		(struct mlx4_qp *, enum mlx4_event);

	int			qpn;

	atomic_t		refcount;
	struct completion	free;
};

struct mlx4_srq {
	void (*event)		(struct mlx4_srq *, enum mlx4_event);

	int			srqn;
	int			max;
	int			max_gs;
	int			wqe_shift;

	atomic_t		refcount;
	struct completion	free;
};

struct mlx4_av {
	u32			port_pd;
	u8			reserved1;
	u8			g_slid;
	u16			dlid;
	u8			reserved2;
	u8			gid_index;
	u8			stat_rate;
	u8			hop_limit;
	u32			sl_tclass_flowlabel;
	u8			dgid[16];
};

struct mlx4_eth_av {
	u32		port_pd;
	u8		reserved1;
	u8		smac_idx;
	u16		reserved2;
	u8		reserved3;
	u8		gid_index;
	u8		stat_rate;
	u8		hop_limit;
	u32		sl_tclass_flowlabel;
	u8		dgid[16];
	u32		reserved4[2];
	u16		vlan;
	u8		mac[6];
};

union mlx4_ext_av {
	struct mlx4_av		ib;
	struct mlx4_eth_av	eth;
};

struct mlx4_init_port_param {
	int			set_guid0;
	int			set_node_guid;
	int			set_si_guid;
	u16			mtu;
	int			port_width_cap;
	u16			vl_cap;
	u16			max_gid;
	u16			max_pkey;
	u64			guid0;
	u64			node_guid;
	u64			si_guid;
};

enum {
	MLX4_HCR_BASE		= 0x80680,
	MLX4_HCR_SIZE		= 0x0001c,
	MLX4_CLR_INT_SIZE	= 0x00008
};

enum {
	MLX4_MGM_ENTRY_SIZE	=  0x100,
	MLX4_QP_PER_MGM		= 4 * (MLX4_MGM_ENTRY_SIZE / 16 - 2),
	MLX4_MTT_ENTRY_PER_SEG	= 8
};

enum {
	MLX4_NUM_PDS		= 1 << 15
};

enum {
	MLX4_CMPT_TYPE_QP	= 0,
	MLX4_CMPT_TYPE_SRQ	= 1,
	MLX4_CMPT_TYPE_CQ	= 2,
	MLX4_CMPT_TYPE_EQ	= 3,
	MLX4_CMPT_NUM_TYPE
};

enum {
	MLX4_CMPT_SHIFT		= 24,
	MLX4_NUM_CMPTS		= MLX4_CMPT_NUM_TYPE << MLX4_CMPT_SHIFT
};

struct mlx4_bitmap {
	u32			last;
	u32			top;
	u32			max;
	u32                     reserved_top;
	u32			mask;
	u32			avail;
//	spinlock_t		lock;
	unsigned long	       *table;
};

struct mlx4_buddy {
	unsigned long	      **bits;
	unsigned int	       *num_free;
	int			max_order;
//	spinlock_t		lock;
};

struct mlx4_icm;

struct mlx4_icm_table {
	u64			virt;
	int			num_icm;
	int			num_obj;
	int			obj_size;
	int			lowmem;
	int			coherent;
//	struct mutex		mutex;
	struct mlx4_icm	      **icm;
};

struct mlx4_eq {
	struct mlx4_device	       *dev;
	void	       *doorbell;
	int			eqn;
	u32			cons_index;
	u16			irq;
	u16			have_irq;
	int			nent;
	struct mlx4_buf_list   *page_list;
	struct mlx4_mtt		mtt;
};

struct mlx4_profile {
	int			num_qp;
	int			rdmarc_per_qp;
	int			num_srq;
	int			num_cq;
	int			num_mcg;
	int			num_mpt;
	int			num_mtt;
};

struct mlx4_fw {
	u64			clr_int_base;
	u64			catas_offset;
	struct mlx4_icm	       *fw_icm;
	struct mlx4_icm	       *aux_icm;
	u32			catas_size;
	u16			fw_pages;
	u8			clr_int_bar;
	u8			catas_bar;
};

#define MGM_QPN_MASK       0x00FFFFFF
#define MGM_BLCK_LB_BIT    30

struct mlx4_promisc_qp {
	struct list_head list;
	u32 qpn;
};

struct mlx4_steer_index {
	struct list_head list;
	unsigned int index;
	struct list_head duplicates;
};

struct mlx4_mgm {
	u32			next_gid_index;
	u32			members_count;
	u32			reserved[2];
	u8			gid[16];
	u32			qp[MLX4_QP_PER_MGM];
};

struct mlx4_cmd {
//	struct pci_pool	       *pool;
	unsigned char* pool;
	unsigned char* hcr;
//	struct mutex		hcr_mutex;
//	struct semaphore	poll_sem;
//	struct semaphore	event_sem;
	int			max_cmds;
//	spinlock_t		context_lock;
	int			free_head;
	struct mlx4_cmd_context *context;
	u16			token_mask;
	u8			use_events;
	u8			toggle;
};

struct mlx4_uar_table {
	struct mlx4_bitmap	bitmap;
};

struct mlx4_mr_table {
	struct mlx4_bitmap	mpt_bitmap;
	struct mlx4_buddy	mtt_buddy;
	u64			mtt_base;
	u64			mpt_base;
	struct mlx4_icm_table	mtt_table;
	struct mlx4_icm_table	dmpt_table;
};

struct mlx4_cq_table {
	struct mlx4_bitmap	bitmap;
//	spinlock_t		lock;
//	struct radix_tree_root	tree;
	struct mlx4_icm_table	table;
	struct mlx4_icm_table	cmpt_table;
};

struct mlx4_eq_table {
	struct mlx4_bitmap	bitmap;
//	char		       *irq_names;
	void 	       *clr_int;
	void 	      **uar_map;
	u32			clr_mask;
	struct mlx4_eq	       *eq;
	struct mlx4_icm_table	table;
	struct mlx4_icm_table	cmpt_table;
	int			have_irq;
	u8			inta_pin;
};

struct mlx4_srq_table {
	struct mlx4_bitmap	bitmap;
//	spinlock_t		lock;
//	struct radix_tree_root	tree;
	struct mlx4_icm_table	table;
	struct mlx4_icm_table	cmpt_table;
};

struct mlx4_qp_table {
	struct mlx4_bitmap	bitmap;
	u32			rdmarc_base;
	int			rdmarc_shift;
//	spinlock_t		lock;
	struct mlx4_icm_table	qp_table;
	struct mlx4_icm_table	auxc_table;
	struct mlx4_icm_table	altc_table;
	struct mlx4_icm_table	rdmarc_table;
	struct mlx4_icm_table	cmpt_table;
};

struct mlx4_mcg_table {
//	struct mutex		mutex;
	struct mlx4_bitmap	bitmap;
	struct mlx4_icm_table	table;
};

struct mlx4_catas_err {
	u32        *map;
//	struct timer_list	timer;
	struct list_head	list;
};

#define MLX4_MAX_MAC_NUM	128
#define MLX4_MAC_TABLE_SIZE	(MLX4_MAX_MAC_NUM << 3)

struct mlx4_mac_table {
	u64			entries[MLX4_MAX_MAC_NUM];
	int			refs[MLX4_MAX_MAC_NUM];
//	struct mutex		mutex;
	int			total;
	int			max;
};

#define MLX4_MAX_VLAN_NUM	128
#define MLX4_VLAN_TABLE_SIZE	(MLX4_MAX_VLAN_NUM << 2)

struct mlx4_vlan_table {
	u32			entries[MLX4_MAX_VLAN_NUM];
	int			refs[MLX4_MAX_VLAN_NUM];
//	struct mutex		mutex;
	int			total;
	int			max;
};

struct mlx4_mac_entry {
	u64 mac;
};

struct mlx4_port_info {
	struct mlx4_device	       *dev;
	int			port;
	char			dev_name[16];
//	struct device_attribute port_attr;
	enum mlx4_port_type	tmp_type;
	struct mlx4_mac_table	mac_table;
//	struct radix_tree_root	mac_tree;
	struct mlx4_vlan_table	vlan_table;
	int			base_qpn;
};

struct mlx4_sense {
	struct mlx4_device		*dev;
	u8			do_sense_port[MLX4_MAX_PORTS + 1];
	u8			sense_allowed[MLX4_MAX_PORTS + 1];
//	struct delayed_work	sense_poll;
};

struct mlx4_msix_ctl {
	u64		pool_bm;
//	spinlock_t	pool_lock;
};

struct mlx4_steer {
	struct list_head promisc_qps[MLX4_NUM_STEERS];
	struct list_head steer_entries[MLX4_NUM_STEERS];
	struct list_head high_prios;
};

struct mlx4_device {
	struct ibv_device ibv_device;
	struct pcie_device pcie;
	unsigned long		flags;
	struct mlx4_caps	caps;
//	struct radix_tree_root	qp_table_tree;
	u8			rev_id;
	char			board_id[MLX4_BOARD_ID_LEN];
// private
	struct list_head	dev_list;
	struct list_head	ctx_list;
//	spinlock_t		ctx_lock;

	struct list_head        pgdir_list;
//	struct mutex            pgdir_mutex;

	struct mlx4_fw		fw;
	struct mlx4_cmd		cmd;
	
	unsigned char* dcs;
	unsigned char* uar;

	struct mlx4_bitmap	pd_bitmap;
	struct mlx4_uar_table	uar_table;
	struct mlx4_mr_table	mr_table;
	struct mlx4_cq_table	cq_table;
	struct mlx4_eq_table	eq_table;
	struct mlx4_srq_table	srq_table;
	struct mlx4_qp_table	qp_table;
	struct mlx4_mcg_table	mcg_table;

	struct mlx4_catas_err	catas_err;

	void 	       *clr_base;

	struct mlx4_uar		driver_uar;
	void 	       *kar;
	struct mlx4_port_info	port[MLX4_MAX_PORTS + 1];
	struct mlx4_sense       sense;
//	struct mutex		port_mutex;
	struct mlx4_msix_ctl	msix_ctl;
	struct mlx4_steer	*steer;
	struct list_head	bf_list;
//	struct mutex		bf_mutex;
//	struct io_mapping	*bf_mapping;
	unsigned char* bf_mapping;
};

#define MLX4_SENSE_RANGE	(HZ * 3)

int mlx4_init(struct mlx4_device* device);

extern struct workqueue_struct *mlx4_wq;

#define mlx4_foreach_port(port, dev, type)				\
	for ((port) = 1; (port) <= (dev)->caps.num_ports; (port)++)	\
		if (((type) == MLX4_PORT_TYPE_IB ? (dev)->caps.port_mask : \
		     ~(dev)->caps.port_mask) & 1 << ((port) - 1))

#define mlx4_foreach_ib_transport_port(port, dev)			\
	for ((port) = 1; (port) <= (dev)->caps.num_ports; (port)++)	\
		if (((dev)->caps.port_mask & 1 << ((port) - 1)) ||	\
		    ((dev)->caps.flags & MLX4_DEV_CAP_FLAG_IBOE))


int mlx4_buf_alloc(struct mlx4_device *dev, int size, int max_direct,
		   struct mlx4_buf *buf);
void mlx4_buf_free(struct mlx4_device *dev, int size, struct mlx4_buf *buf);
static inline void *mlx4_buf_offset(struct mlx4_buf *buf, int offset)
{
	if (buf->nbufs == 1)
		return buf->direct.buf + offset;
	else
		return buf->page_list[offset >> PAGE_SHIFT].buf +
			(offset & (PAGE_SIZE - 1));
}

int mlx4_pd_alloc(struct mlx4_device *dev, u32 *pdn);
void mlx4_pd_free(struct mlx4_device *dev, u32 pdn);

int mlx4_uar_alloc(struct mlx4_device *dev, struct mlx4_uar *uar);
void mlx4_uar_free(struct mlx4_device *dev, struct mlx4_uar *uar);
int mlx4_bf_alloc(struct mlx4_device *dev, struct mlx4_bf *bf);
void mlx4_bf_free(struct mlx4_device *dev, struct mlx4_bf *bf);

int mlx4_mtt_init(struct mlx4_device *dev, int npages, int page_shift,
		  struct mlx4_mtt *mtt);
void mlx4_mtt_cleanup(struct mlx4_device *dev, struct mlx4_mtt *mtt);
u64 mlx4_mtt_addr(struct mlx4_device *dev, struct mlx4_mtt *mtt);

int mlx4_mr_alloc(struct mlx4_device *dev, u32 pd, u64 iova, u64 size, u32 access,
		  int npages, int page_shift, struct mlx4_mr *mr);
void mlx4_mr_free(struct mlx4_device *dev, struct mlx4_mr *mr);
int mlx4_mr_enable(struct mlx4_device *dev, struct mlx4_mr *mr);
int mlx4_write_mtt(struct mlx4_device *dev, struct mlx4_mtt *mtt,
		   int start_index, int npages, u64 *page_list);
int mlx4_buf_write_mtt(struct mlx4_device *dev, struct mlx4_mtt *mtt,
		       struct mlx4_buf *buf);

int mlx4_db_alloc(struct mlx4_device *dev, struct mlx4_db *db, int order);
void mlx4_db_free(struct mlx4_device *dev, struct mlx4_db *db);

int mlx4_alloc_hwq_res(struct mlx4_device *dev, struct mlx4_hwq_resources *wqres,
		       int size, int max_direct);
void mlx4_free_hwq_res(struct mlx4_device *mdev, struct mlx4_hwq_resources *wqres,
		       int size);

int mlx4_cq_alloc(struct mlx4_device *dev, int nent, struct mlx4_mtt *mtt,
		  struct mlx4_uar *uar, u64 db_rec, struct mlx4_cq *cq,
		  unsigned vector, int collapsed);
void mlx4_cq_free(struct mlx4_device *dev, struct mlx4_cq *cq);

int mlx4_qp_reserve_range(struct mlx4_device *dev, int cnt, int align, int *base);
void mlx4_qp_release_range(struct mlx4_device *dev, int base_qpn, int cnt);

int mlx4_qp_alloc(struct mlx4_device *dev, int qpn, struct mlx4_qp *qp);
void mlx4_qp_free(struct mlx4_device *dev, struct mlx4_qp *qp);

int mlx4_srq_alloc(struct mlx4_device *dev, u32 pdn, struct mlx4_mtt *mtt,
		   u64 db_rec, struct mlx4_srq *srq);
void mlx4_srq_free(struct mlx4_device *dev, struct mlx4_srq *srq);
int mlx4_srq_arm(struct mlx4_device *dev, struct mlx4_srq *srq, int limit_watermark);
int mlx4_srq_query(struct mlx4_device *dev, struct mlx4_srq *srq, int *limit_watermark);

int mlx4_INIT_PORT(struct mlx4_device *dev, int port);
int mlx4_CLOSE_PORT(struct mlx4_device *dev, int port);

int mlx4_multicast_attach(struct mlx4_device *dev, struct mlx4_qp *qp, u8 gid[16],
			  int block_mcast_loopback, enum mlx4_protocol protocol);
int mlx4_multicast_detach(struct mlx4_device *dev, struct mlx4_qp *qp, u8 gid[16],
			  enum mlx4_protocol protocol);
int mlx4_multicast_promisc_add(struct mlx4_device *dev, u32 qpn, u8 port);
int mlx4_multicast_promisc_remove(struct mlx4_device *dev, u32 qpn, u8 port);
int mlx4_unicast_promisc_add(struct mlx4_device *dev, u32 qpn, u8 port);
int mlx4_unicast_promisc_remove(struct mlx4_device *dev, u32 qpn, u8 port);
int mlx4_SET_MCAST_FLTR(struct mlx4_device *dev, u8 port, u64 mac, u64 clear, u8 mode);

int mlx4_register_mac(struct mlx4_device *dev, u8 port, u64 mac, int *qpn, u8 wrap);
void mlx4_unregister_mac(struct mlx4_device *dev, u8 port, int qpn);
int mlx4_replace_mac(struct mlx4_device *dev, u8 port, int qpn, u64 new_mac, u8 wrap);

int mlx4_find_cached_vlan(struct mlx4_device *dev, u8 port, u16 vid, int *idx);
int mlx4_register_vlan(struct mlx4_device *dev, u8 port, u16 vlan, int *index);
void mlx4_unregister_vlan(struct mlx4_device *dev, u8 port, int index);

int mlx4_map_phys_fmr(struct mlx4_device *dev, struct mlx4_fmr *fmr, u64 *page_list,
		      int npages, u64 iova, u32 *lkey, u32 *rkey);
int mlx4_fmr_alloc(struct mlx4_device *dev, u32 pd, u32 access, int max_pages,
		   int max_maps, u8 page_shift, struct mlx4_fmr *fmr);
int mlx4_fmr_enable(struct mlx4_device *dev, struct mlx4_fmr *fmr);
void mlx4_fmr_unmap(struct mlx4_device *dev, struct mlx4_fmr *fmr,
		    u32 *lkey, u32 *rkey);
int mlx4_fmr_free(struct mlx4_device *dev, struct mlx4_fmr *fmr);
int mlx4_SYNC_TPT(struct mlx4_device *dev);
int mlx4_test_interrupts(struct mlx4_device *dev);
int mlx4_assign_eq(struct mlx4_device *dev, char* name , int* vector);
void mlx4_release_eq(struct mlx4_device *dev, int vec);

int mlx4_wol_read(struct mlx4_device *dev, u64 *config, int port);
int mlx4_wol_write(struct mlx4_device *dev, u64 config, int port);

u32 mlx4_bitmap_alloc(struct mlx4_bitmap *bitmap);
void mlx4_bitmap_free(struct mlx4_bitmap *bitmap, u32 obj);
u32 mlx4_bitmap_alloc_range(struct mlx4_bitmap *bitmap, int cnt, int align);
void mlx4_bitmap_free_range(struct mlx4_bitmap *bitmap, u32 obj, int cnt);
u32 mlx4_bitmap_avail(struct mlx4_bitmap *bitmap);
int mlx4_bitmap_init(struct mlx4_bitmap *bitmap, u32 num, u32 mask,
		     u32 reserved_bot, u32 resetrved_top);
void mlx4_bitmap_cleanup(struct mlx4_bitmap *bitmap);

int mlx4_reset(struct mlx4_device *dev);

int mlx4_alloc_eq_table(struct mlx4_device *dev);
void mlx4_free_eq_table(struct mlx4_device *dev);

int mlx4_init_pd_table(struct mlx4_device *dev);
int mlx4_init_uar_table(struct mlx4_device *dev);
int mlx4_init_mr_table(struct mlx4_device *dev);
int mlx4_init_eq_table(struct mlx4_device *dev);
int mlx4_init_cq_table(struct mlx4_device *dev);
int mlx4_init_qp_table(struct mlx4_device *dev);
int mlx4_init_srq_table(struct mlx4_device *dev);
int mlx4_init_mcg_table(struct mlx4_device *dev);

void mlx4_cleanup_pd_table(struct mlx4_device *dev);
void mlx4_cleanup_uar_table(struct mlx4_device *dev);
void mlx4_cleanup_mr_table(struct mlx4_device *dev);
void mlx4_cleanup_eq_table(struct mlx4_device *dev);
void mlx4_cleanup_cq_table(struct mlx4_device *dev);
void mlx4_cleanup_qp_table(struct mlx4_device *dev);
void mlx4_cleanup_srq_table(struct mlx4_device *dev);
void mlx4_cleanup_mcg_table(struct mlx4_device *dev);

void mlx4_start_catas_poll(struct mlx4_device *dev);
void mlx4_stop_catas_poll(struct mlx4_device *dev);
void mlx4_catas_init(void);
int mlx4_restart_one(struct pci_dev *pdev);
int mlx4_register_device(struct mlx4_device *dev);
void mlx4_unregister_device(struct mlx4_device *dev);
void mlx4_dispatch_event(struct mlx4_device *dev, enum mlx4_dev_event type, int port);

struct mlx4_dev_cap;
struct mlx4_init_hca_param;

u64 mlx4_make_profile(struct mlx4_device *dev,
		      struct mlx4_profile *request,
		      struct mlx4_dev_cap *dev_cap,
		      struct mlx4_init_hca_param *init_hca);

int mlx4_cmd_init(struct mlx4_device *dev);
void mlx4_cmd_cleanup(struct mlx4_device *dev);
void mlx4_cmd_event(struct mlx4_device *dev, u16 token, u8 status, u64 out_param);
int mlx4_cmd_use_events(struct mlx4_device *dev);
void mlx4_cmd_use_polling(struct mlx4_device *dev);

void mlx4_cq_completion(struct mlx4_device *dev, u32 cqn);
void mlx4_cq_event(struct mlx4_device *dev, u32 cqn, int event_type);

void mlx4_qp_event(struct mlx4_device *dev, u32 qpn, int event_type);

void mlx4_srq_event(struct mlx4_device *dev, u32 srqn, int event_type);

void mlx4_handle_catas_err(struct mlx4_device *dev);

int mlx4_SENSE_PORT(struct mlx4_device *dev, int port,
		    enum mlx4_port_type *type);
void mlx4_do_sense_ports(struct mlx4_device *dev,
			 enum mlx4_port_type *stype,
			 enum mlx4_port_type *defaults);
void mlx4_start_sense(struct mlx4_device *dev);
void mlx4_stop_sense(struct mlx4_device *dev);
void mlx4_sense_init(struct mlx4_device *dev);
/*int mlx4_check_port_params(struct mlx4_device *dev,
			   enum mlx4_port_type *port_type);
int mlx4_change_port_types(struct mlx4_device *dev,
			   enum mlx4_port_type *port_types);
*/
void mlx4_init_mac_table(struct mlx4_device *dev, struct mlx4_mac_table *table);
void mlx4_init_vlan_table(struct mlx4_device *dev, struct mlx4_vlan_table *table);

int mlx4_SET_PORT(struct mlx4_device *dev, u8 port);
int mlx4_get_port_ib_caps(struct mlx4_device *dev, u8 port, u32 *caps);

int mlx4_qp_detach_common(struct mlx4_device *dev, struct mlx4_qp *qp, u8 gid[16],
			  enum mlx4_protocol prot, enum mlx4_steer_type steer);
int mlx4_qp_attach_common(struct mlx4_device *dev, struct mlx4_qp *qp, u8 gid[16],
			  int block_mcast_loopback, enum mlx4_protocol prot,
			  enum mlx4_steer_type steer);
#endif /* MLX4_H */
