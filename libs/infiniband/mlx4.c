#include "mlx4.h"
#include "mlx4/mlx4.h"
#include "mlx4/cq.h"
#include "mlx4/cmd.h"
#include "../pcie/pcie.h"
#include <l4io.h>
#include <l4/types.h>
#include <l4/kdebug.h>
#include "../acpi/platform.h"

// user defines
#define DRIVER_LOG printf
#define DRIVER_PREFIX "MLX4: "
#define DRIVER_MAX_DEVICES 1
#define DRIVER_MAX_CONTEXT 2

enum {
	MLX4_MAX_WQE_SIZE = 1008
};

static int align(int v1, int v2)
{
	int t = v1/v2;
	if (t*v2==v1)
		return v1;
	return t*v2+v2;
}

static struct mlx4_device g_Device[DRIVER_MAX_DEVICES];
static unsigned int g_DeviceCount = 0;

struct mlx4_ib_pd {
	struct ibv_pd		ibpd;
	u32			pdn;
};

struct mlx4_ib_mr {
	struct ibv_mr		ibmr;
	struct mlx4_mr		mmr;
	struct ib_umem	       *umem;
};

struct mlx4_ib_cq_buf {
	struct mlx4_buf		buf;
	struct mlx4_mtt		mtt;
};

struct mlx4_ib_cq_resize {
	struct mlx4_ib_cq_buf	buf;
	int			cqe;
};

struct mlx4_ib_cq {
	struct ibv_cq		ibcq;
	struct mlx4_cq		mcq;
	struct mlx4_ib_cq_buf	buf;
	struct mlx4_ib_cq_resize *resize_buf;
	struct mlx4_db		db;
	//spinlock_t		lock;
	//struct mutex		resize_mutex;
	struct ib_umem	       *umem;
	struct ib_umem	       *resize_umem;
};

struct mlx4_ib_wq {
	u64		       *wrid;
	//spinlock_t		lock;
	int			wqe_cnt;
	int			max_post;
	int			max_gs;
	int			offset;
	int			wqe_shift;
	unsigned		head;
	unsigned		tail;
};

struct mlx4_ib_qp {
	struct ibv_qp			ibv_qp;
	struct mlx4_qp		mqp;
	struct mlx4_buf		buf;

	struct mlx4_db		db;
	struct mlx4_ib_wq	rq;

	u32			doorbell_qpn;
	u32			sq_signal_bits;
	unsigned		sq_next_wqe;
	int			sq_max_wqes_per_wr;
	int			sq_spare_wqes;
	struct mlx4_ib_wq	sq;

	struct ib_umem	       *umem;
	struct mlx4_mtt		mtt;
	int			buf_size;
	//struct mutex		mutex;
	u32			flags;
	u8			port;
	u8			alt_port;
	u8			atomic_rd_en;
	u8			resp_depth;
	u8			sq_no_prefetch;
	u8			state;
	int			mlx_type;
};
struct mlx4_wqe_ctrl_seg {
	u32		owner_opcode;
	u16                vlan_tag;
	u8                 ins_vlan;
	u8			fence_size;
	/*
	 * High 24 bits are SRC remote buffer; low 8 bits are flags:
	 * [7]   SO (strong ordering)
	 * [5]   TCP/UDP checksum
	 * [4]   IP checksum
	 * [3:2] C (generate completion queue entry)
	 * [1]   SE (solicited event)
	 * [0]   FL (force loopback)
	 */
	u32		xrcrb_flags;
	/*
	 * imm is immediate data for send/RDMA write w/ immediate;
	 * also invalidation key for send with invalidate; input
	 * modifier for WQEs on CCQs.
	 */
	u32		imm;
};

struct mlx4_wqe_datagram_seg {
	u32		av[8];
	u32		dqpn;
	u32		qkey;
	u16		vlan;
	u8			mac[6];
};

struct mlx4_wqe_data_seg {
	u32		byte_count;
	u32		lkey;
	uint64_t		addr;
};

struct mlx4_wqe_inline_seg {
	u32		byte_count;
};

struct mlx4_wqe_srq_next_seg {
	u16		reserved1;
	u16		next_wqe_index;
	u32		reserved2[3];
};

struct mlx4_wqe_raddr_seg {
	uint64_t		raddr;
	u32		rkey;
	u32		reserved;
};

struct mlx4_wqe_atomic_seg {
	uint64_t		swap_add;
	uint64_t		compare;
};

struct mlx4_wqe_bind_seg {
	u32		flags1;
	u32		flags2;
	u32		new_rkey;
	u32		lkey;
	uint64_t		addr;
	uint64_t		length;
};

#define MLX4_QP_TABLE_SIZE 256
#define MLX4_XRC_SRQ_TABLE_SIZE 256
#define MLX4_NUM_DB_TYPE 2

struct mlx4_context {
	struct ibv_context		ibv_ctx;

	struct mlx4_uar uar;
	//pthread_spinlock_t		uar_lock;

	void			       *bf_page;
	int				bf_buf_size;
	int				bf_offset;
	//pthread_spinlock_t		bf_lock;

	struct {
		struct mlx4_qp	      **table;
		int			refcnt;
	}				qp_table[MLX4_QP_TABLE_SIZE];
	//pthread_mutex_t			qp_table_mutex;
	int				num_qps;
	int				qp_table_shift;
	int				qp_table_mask;
	int				max_qp_wr;
	int				max_sge;
	int				max_cqe;

	struct {
		struct mlx4_srq       **table;
		int			refcnt;
	}				xrc_srq_table[MLX4_XRC_SRQ_TABLE_SIZE];
	//pthread_mutex_t			xrc_srq_table_mutex;
	int				num_xrc_srqs;
	int				xrc_srq_table_shift;
	int				xrc_srq_table_mask;

//	struct mlx4_db_page	       *db_list[MLX4_NUM_DB_TYPE];
	//pthread_mutex_t			db_list_mutex;
};

static struct mlx4_context mlx_context[DRIVER_MAX_CONTEXT];


static struct ibv_pd* alloc_pd(struct ibv_context *context)
{
	static struct mlx4_ib_pd g_pd;
	g_pd.ibpd.context = context;
	if (mlx4_pd_alloc((struct mlx4_device*)context->device, &g_pd.pdn))
	{
		DRIVER_LOG(DRIVER_PREFIX "couldn't alloc pd\n");
		return 0;
	}
	g_pd.ibpd.handle = 1;
	return &g_pd.ibpd;
}

static int dealloc_pd(struct ibv_pd *pd)
{
	mlx4_pd_free((struct mlx4_device*)pd->context->device, ((struct mlx4_ib_pd*)pd)->pdn);
	return 0;
}

static u32 convert_access(int acc)
{
	return (acc & IBV_ACCESS_REMOTE_ATOMIC ? MLX4_PERM_ATOMIC       : 0) |
	       (acc & IBV_ACCESS_REMOTE_WRITE  ? MLX4_PERM_REMOTE_WRITE : 0) |
	       (acc & IBV_ACCESS_REMOTE_READ   ? MLX4_PERM_REMOTE_READ  : 0) |
	       (acc & IBV_ACCESS_LOCAL_WRITE   ? MLX4_PERM_LOCAL_WRITE  : 0) |
	       MLX4_PERM_LOCAL_READ;
}

static struct ibv_mr* reg_mr(struct ibv_pd *pd, void *addr, size_t length, int access)
{
	static struct mlx4_ib_mr g_mr;

	struct mlx4_ib_mr* mr = &g_mr;
	
	struct mlx4_device* dev = (struct mlx4_device*)pd->context->device;
	int n = (length+PAGE_SIZE-1)/PAGE_SIZE;
	int shift = PAGE_SHIFT;
	int err = mlx4_mr_alloc(dev, ((struct mlx4_ib_pd*)pd)->pdn, addr, length, convert_access(access), n, shift, &mr->mmr);

	if (err)
		goto err_umem;

//	err = mlx4_ib_umem_write_mtt(dev, &mr->mmr.mtt, mr->umem);
	if (err)
		goto err_mr;

	err = mlx4_mr_enable(dev, &mr->mmr);
	if (err)
		goto err_mr;

	mr->ibmr.rkey = mr->ibmr.lkey = mr->mmr.key;
	mr->ibmr.addr = addr;
	mr->ibmr.length = length;
	mr->ibmr.handle = 1;
	
	return &mr->ibmr;
err_umem:
err_mr:
return 0;
}

static int dereg_mr(struct ibv_mr *mr)
{
	return 0;
}

struct ibv_cq* create_cq(struct ibv_context *context, int cqe, struct ibv_comp_channel *channel, int comp_vector)
{
/*	static struct mlx4_ib_cq g_cq;
	struct mlx4_device* dev = (struct mlx4_device*)context->device;
	struct mlx4_ib_cq *cq = &g_cq;
	struct mlx4_uar *uar;
	int err;

	if (entries < 1 || entries > dev->caps.max_cqes)
		return 0;

	// cq = kmalloc(sizeof *cq, GFP_KERNEL);
	// if (!cq)
		// return ERR_PTR(-ENOMEM);

	entries      = roundup_pow_of_two(entries + 1);
	cq->ibcq.cqe = entries - 1;
	//mutex_init(&cq->resize_mutex);
	//spin_lock_init(&cq->lock);
	cq->resize_buf = NULL;
	cq->resize_umem = NULL;

	if (context) {
		struct mlx4_ib_create_cq ucmd;

		// if (ib_copy_from_udata(&ucmd, udata, sizeof ucmd)) {
			// err = -1;
			// goto err_cq;
		// }

//		err = mlx4_ib_get_cq_umem(dev, context, &cq->buf, &cq->umem,
//					  ucmd.buf_addr, entries);
		if (err)
			goto err_cq;

		err = mlx4_ib_db_map_user(to_mucontext(context), ucmd.db_addr,
					  &cq->db);
		if (err)
			goto err_mtt;

		uar = &to_mucontext(context)->uar;
	} else {
		err = mlx4_db_alloc(dev, &cq->db, 1);
		if (err)
			goto err_cq;

		cq->mcq.set_ci_db  = cq->db.db;
		cq->mcq.arm_db     = cq->db.db + 1;
		*cq->mcq.set_ci_db = 0;
		*cq->mcq.arm_db    = 0;

		err = mlx4_ib_alloc_cq_buf(dev, &cq->buf, entries);
		if (err)
			goto err_db;

		uar = &dev->priv_uar;
	}

	err = mlx4_cq_alloc(dev, entries, &cq->buf.mtt, uar,
			    cq->db.dma, &cq->mcq, vector, 0);
	if (err)
		goto err_dbmap;

	cq->mcq.comp  = mlx4_ib_cq_comp;
	cq->mcq.event = mlx4_ib_cq_event;

	if (context)
		if (ib_copy_to_udata(udata, &cq->mcq.cqn, sizeof (__u32))) {
			err = -1;
			goto err_dbmap;
		}

	return &cq->ibcq;

err_dbmap:
	if (context)
		mlx4_ib_db_unmap_user(to_mucontext(context), &cq->db);

err_mtt:
	mlx4_mtt_cleanup(dev, &cq->buf.mtt);

	if (context)
		ib_umem_release(cq->umem);
	else
		mlx4_ib_free_cq_buf(dev, &cq->buf, cq->ibcq.cqe);

err_db:
	if (!context)
		mlx4_db_free(dev, &cq->db);

err_cq:
	//kfree(cq);
*/
	return 0;
}

static int verify_sizes(struct ibv_qp_init_attr *attr, struct mlx4_context *context)
{
	int size;
	int nsegs;

	if (attr->cap.max_send_wr     > context->max_qp_wr ||
	    attr->cap.max_recv_wr     > context->max_qp_wr ||
	    attr->cap.max_send_sge    > context->max_sge   ||
	    attr->cap.max_recv_sge    > context->max_sge)
		return -1;

	if (attr->cap.max_inline_data) {
		nsegs = num_inline_segs(attr->cap.max_inline_data, attr->qp_type);
		size = MLX4_MAX_WQE_SIZE - nsegs * sizeof (struct mlx4_wqe_inline_seg);
		switch (attr->qp_type) {
		case IBV_QPT_UD:
			size -= (sizeof (struct mlx4_wqe_ctrl_seg) +
				 sizeof (struct mlx4_wqe_datagram_seg));
			break;

		case IBV_QPT_RC:
		case IBV_QPT_UC:
//		case IBV_QPT_XRC:
			size -= (sizeof (struct mlx4_wqe_ctrl_seg) +
				 sizeof (struct mlx4_wqe_raddr_seg));
			break;

		default:
			return 0;
		}

		if (attr->cap.max_inline_data > size)
			return -1;
	}

	return 0;
}

static void mlx4_calc_sq_wqe_size(struct ibv_qp_cap *cap, enum ibv_qp_type type, struct mlx4_ib_qp *qp)
{
	int size;
	int max_sq_sge;

	max_sq_sge	 = align(cap->max_inline_data +
				 num_inline_segs(cap->max_inline_data, type) *
				 sizeof (struct mlx4_wqe_inline_seg),
				 sizeof (struct mlx4_wqe_data_seg)) /
		sizeof (struct mlx4_wqe_data_seg);
	if (max_sq_sge < cap->max_send_sge)
		max_sq_sge = cap->max_send_sge;

	size = max_sq_sge * sizeof (struct mlx4_wqe_data_seg);
	switch (type) {
	case IBV_QPT_UD:
		size += sizeof (struct mlx4_wqe_datagram_seg);
		break;

	case IBV_QPT_UC:
		size += sizeof (struct mlx4_wqe_raddr_seg);
		break;

//	case IBV_QPT_XRC:
	case IBV_QPT_RC:
		size += sizeof (struct mlx4_wqe_raddr_seg);
		/*
		 * An atomic op will require an atomic segment, a
		 * remote address segment and one scatter entry.
		 */
		if (size < (sizeof (struct mlx4_wqe_atomic_seg) +
			    sizeof (struct mlx4_wqe_raddr_seg) +
			    sizeof (struct mlx4_wqe_data_seg)))
			size = (sizeof (struct mlx4_wqe_atomic_seg) +
				sizeof (struct mlx4_wqe_raddr_seg) +
				sizeof (struct mlx4_wqe_data_seg));
		break;

	default:
		break;
	}

	/* Make sure that we have enough space for a bind request */
	if (size < sizeof (struct mlx4_wqe_bind_seg))
		size = sizeof (struct mlx4_wqe_bind_seg);

	size += sizeof (struct mlx4_wqe_ctrl_seg);

	for (qp->sq.wqe_shift = 6; 1 << qp->sq.wqe_shift < size;
	     qp->sq.wqe_shift++)
		; /* nothing */
}

static int mlx4_alloc_qp_buf(struct ibv_pd *pd, struct ibv_qp_cap *cap, enum ibv_qp_type type, struct mlx4_ib_qp *qp)
{
	qp->rq.max_gs	 = cap->max_recv_sge;

	int sentSize = qp->sq.wqe_cnt * sizeof (u64);
	int receiveSize = qp->rq.wqe_cnt * sizeof (u64);
	void* all = platform_mapAny(align(sentSize, 1 << 12) + align(receiveSize, 1 << 12));
	qp->sq.wrid = all;
	if (!qp->sq.wrid)
		return -1;

	if (qp->rq.wqe_cnt) {
		qp->rq.wrid = ((char*)all)+align(sentSize, 1 << 12);
		if (!qp->rq.wrid) {
			//free(qp->sq.wrid);
			return -1;
		}
	}

	for (qp->rq.wqe_shift = 4;
	     1 << qp->rq.wqe_shift < qp->rq.max_gs * sizeof (struct mlx4_wqe_data_seg);
	     qp->rq.wqe_shift++)
		; // nothing

	qp->buf_size = (qp->rq.wqe_cnt << qp->rq.wqe_shift) + (qp->sq.wqe_cnt << qp->sq.wqe_shift);
	if (qp->rq.wqe_shift > qp->sq.wqe_shift) {
		qp->rq.offset = 0;
		qp->sq.offset = qp->rq.wqe_cnt << qp->rq.wqe_shift;
	} else {
		qp->rq.offset = qp->sq.wqe_cnt << qp->sq.wqe_shift;
		qp->sq.offset = 0;
	}
	int page_size = 1 << 12;//(struct mlx4_device*)(pd->context->device)->page_size;
	if (mlx4_alloc_buf(&qp->buf,
			    align(qp->buf_size, page_size),
			    page_size)) {
		//free(qp->sq.wrid);
		//free(qp->rq.wrid);
		return -1;
	}

	memset(qp->buf.direct.buf, 0, qp->buf_size);

	return 0;
}

static int mlx4_store_qp(struct mlx4_context *ctx, u32 qpn, struct mlx4_ib_qp *qp)
{
	int tind = (qpn & (ctx->num_qps - 1)) >> ctx->qp_table_shift;

	if (!ctx->qp_table[tind].refcnt) {
		ctx->qp_table[tind].table = calloc(ctx->qp_table_mask + 1,
						   sizeof (struct mlx4_qp *));
		if (!ctx->qp_table[tind].table)
			return -1;
	}

	++ctx->qp_table[tind].refcnt;
	ctx->qp_table[tind].table[qpn & ctx->qp_table_mask] = qp;
	return 0;
}

static int align_queue_size(int req)
{
	int nent;
	for (nent = 1; nent < req; nent <<= 1)
		; /* nothing */
	return nent;
}

struct ibv_qp* create_qp(struct ibv_pd *pd, struct ibv_qp_init_attr *attr)
{
/*	static struct mlx4_ib_qp g_qp;
	struct mlx4_ib_qp		 *qp = &g_qp;
	int			  ret;
	struct mlx4_context	 *context = (struct mlx4_context*)(pd->context);
	// Sanity check QP size before proceeding
	if (verify_sizes(attr, context))
		return NULL;

	mlx4_calc_sq_wqe_size(&attr->cap, attr->qp_type, qp);

	// We need to leave 2 KB + 1 WQE of headroom in the SQ to
	// allow HW to prefetch.
	qp->sq_spare_wqes = (2048 >> qp->sq.wqe_shift) + 1;
	qp->sq.wqe_cnt = align_queue_size(attr->cap.max_send_wr + qp->sq_spare_wqes);
	qp->rq.wqe_cnt = align_queue_size(attr->cap.max_recv_wr);

	if (attr->srq || attr->qp_type == IBV_QPT_XRC)
		attr->cap.max_recv_wr = qp->rq.wqe_cnt = 0;
	else {
		if (attr->cap.max_recv_sge < 1)
			attr->cap.max_recv_sge = 1;
		if (attr->cap.max_recv_wr < 1)
			attr->cap.max_recv_wr = 1;
	}

	if (mlx4_alloc_qp_buf(pd, &attr->cap, attr->qp_type, qp))
		goto err;

	qp->sq.head	 = 0;
	qp->sq.tail	 = 0;
	qp->rq.head	 = 0;
	qp->rq.tail	 = 0;

	// if (pthread_spin_init(&qp->sq.lock, PTHREAD_PROCESS_PRIVATE) ||
	    // pthread_spin_init(&qp->rq.lock, PTHREAD_PROCESS_PRIVATE))
		// goto err_free;

	if (!attr->srq && attr->qp_type != IBV_QPT_XRC) {
		qp->db = mlx4_alloc_db(to_mctx(pd->context), MLX4_DB_TYPE_RQ);
		if (!qp->db)
			goto err_free;

		*qp->db = 0;
	}

	cmd.buf_addr	    = (uintptr_t) qp->buf.buf;
	if (attr->srq || attr->qp_type == IBV_QPT_XRC)
		cmd.db_addr = 0;
	else
		cmd.db_addr = (uintptr_t) qp->db;
	cmd.log_sq_stride   = qp->sq.wqe_shift;
	for (cmd.log_sq_bb_count = 0;
	     qp->sq.wqe_cnt > 1 << cmd.log_sq_bb_count;
	     ++cmd.log_sq_bb_count)
		; // nothing
	cmd.sq_no_prefetch = 0;	// OK for ABI 2: just a reserved field
	memset(cmd.reserved, 0, sizeof cmd.reserved);

	//pthread_mutex_lock(&to_mctx(pd->context)->qp_table_mutex);

	
	
	if (attr->create_flags & ~(IBV_QP_CREATE_IPOIB_UD_LSO | IBV_QP_CREATE_BLOCK_MULTICAST_LOOPBACK))
		goto err_rq_db;

	if (attr->create_flags && (pd->uobject || attr->qp_type != IB_QPT_UD))
		goto err_rq_db;

	switch (attr->qp_type) {
	case IB_QPT_RC:
	case IB_QPT_UC:
	case IB_QPT_UD:
	{
	INIT_LIST_HEAD(&qp->gid_list);

	qp->state	 = IB_QPS_RESET;
	if (init_attr->sq_sig_type == IB_SIGNAL_ALL_WR)
		qp->sq_signal_bits = bs32(MLX4_WQE_CTRL_CQ_UPDATE);

	err = set_rq_size(dev, &init_attr->cap, !!pd->uobject, !!init_attr->srq, qp);
	if (err)
		goto err;

	if (pd->uobject) {
		struct mlx4_ib_create_qp ucmd;

		if (ib_copy_from_udata(&ucmd, udata, sizeof ucmd)) {
			err = -EFAULT;
			goto err;
		}

		qp->sq_no_prefetch = ucmd.sq_no_prefetch;

		err = set_user_sq_size(dev, qp, &ucmd);
		if (err)
			goto err;

		qp->umem = ib_umem_get(pd->uobject->context, ucmd.buf_addr,
				       qp->buf_size, 0, 0);
		if (IS_ERR(qp->umem)) {
			err = PTR_ERR(qp->umem);
			goto err;
		}

		err = mlx4_mtt_init(dev, ib_umem_page_count(qp->umem),
				    ilog2(qp->umem->page_size), &qp->mtt);
		if (err)
			goto err_buf;

		err = mlx4_ib_umem_write_mtt(dev, &qp->mtt, qp->umem);
		if (err)
			goto err_mtt;

		if (!init_attr->srq) {
			err = mlx4_ib_db_map_user(to_mucontext(pd->uobject->context),
						  ucmd.db_addr, &qp->db);
			if (err)
				goto err_mtt;
		}
	} else {
	// not implemented
		goto err;
	}

	err = mlx4_qp_reserve_range(dev, 1, 1, &qpn);
	if (err)
		goto err_wrid;

	err = mlx4_qp_alloc(dev, qpn, &qp->mqp);
	if (err)
		goto err_qpn;

	// Hardware wants QPN written in big-endian order (after
	// shifting) for send doorbell.  Precompute this value to save
	// a little bit when posting sends.
	qp->doorbell_qpn = bs32(qp->mqp.qpn << 8);
//TODO
//	qp->mqp.event = mlx4_ib_qp_event;


		qp->ibqp.qp_num = qp->mqp.qpn;

		break;
	}
	default:
		goto err_rq_db;
	}
	goto err_success;
err_qpn:
err_wrid:
err_mtt:
err_buf:
err_rq_db:
ret = 1;
err_success:
	// ret = ibv_cmd_create_qp(pd, &qp->ibv_qp, attr, &cmd.ibv_cmd, sizeof cmd,
				// &resp, sizeof resp);
	if (ret)
		goto err_rq_db;

	ret = mlx4_store_qp(to_mctx(pd->context), qp->ibv_qp.qp_num, qp);
	if (ret)
		goto err_destroy;
	//pthread_mutex_unlock(&to_mctx(pd->context)->qp_table_mutex);

	qp->rq.wqe_cnt = attr->cap.max_recv_wr;
	qp->rq.max_gs  = attr->cap.max_recv_sge;

	// adjust rq maxima to not exceed reported device maxima
	attr->cap.max_recv_wr = min(context->max_qp_wr, attr->cap.max_recv_wr);
	attr->cap.max_recv_sge = min(context->max_sge, attr->cap.max_recv_sge);

	qp->rq.max_post = attr->cap.max_recv_wr;
	mlx4_set_sq_sizes(qp, &attr->cap, attr->qp_type);
#define htonl bs32
	qp->doorbell_qpn    = htonl(qp->ibv_qp.qp_num << 8);
	if (attr->sq_sig_all)
		qp->sq_signal_bits = htonl(MLX4_WQE_CTRL_CQ_UPDATE);
	else
		qp->sq_signal_bits = 0;
#undef htonl
	return &qp->ibv_qp;

err_destroy:
	ibv_cmd_destroy_qp(&qp->ibv_qp);

err_rq_db:
	//pthread_mutex_unlock(&to_mctx(pd->context)->qp_table_mutex);
//	if (!attr->srq && attr->qp_type != IBV_QPT_XRC)
//		mlx4_free_db(to_mctx(pd->context), MLX4_DB_TYPE_RQ, qp->db);

err_free:
//	free(qp->sq.wrid);
//	if (qp->rq.wqe_cnt)
//		free(qp->rq.wrid);
	mlx4_free_buf(&qp->buf);

err:
	//free(qp);
*/
	return 0;
}

int query_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr, int attr_mask, struct ibv_qp_init_attr *init_attr)
{
return 0;
}


static int __mlx4_ib_modify_qp(struct ibv_qp *ibqp,
			       const struct ibv_qp_attr *attr, int attr_mask,
			       enum ibv_qp_state cur_state, enum ibv_qp_state new_state)
{
	struct mlx4_ib_dev *dev = (struct mlx4_ib_dev*)ibqp->context->device;
	struct mlx4_ib_qp *qp = (struct mlx4_ib_qp*)ibqp;
	struct mlx4_qp_context *context = (struct mlx4_qp_context*)ibqp->context;
//	enum mlx4_qp_optpar optpar = 0;
	int sqd_event;
	int err = -1;
/*
	context = kzalloc(sizeof *context, GFP_KERNEL);
	if (!context)
		return -ENOMEM;

	context->flags = bs32((to_mlx4_state(new_state) << 28) |
				     (to_mlx4_st(ibqp->qp_type) << 16));

	if (!(attr_mask & IB_QP_PATH_MIG_STATE))
		context->flags |= bs32(MLX4_QP_PM_MIGRATED << 11);
	else {
		optpar |= MLX4_QP_OPTPAR_PM_STATE;
		switch (attr->path_mig_state) {
		case IB_MIG_MIGRATED:
			context->flags |= bs32(MLX4_QP_PM_MIGRATED << 11);
			break;
		case IB_MIG_REARM:
			context->flags |= bs32(MLX4_QP_PM_REARM << 11);
			break;
		case IB_MIG_ARMED:
			context->flags |= bs32(MLX4_QP_PM_ARMED << 11);
			break;
		}
	}

	if (ibqp->qp_type == IB_QPT_GSI || ibqp->qp_type == IB_QPT_SMI)
		context->mtu_msgmax = (IB_MTU_4096 << 5) | 11;
	else if (ibqp->qp_type == IB_QPT_UD) {
		if (qp->flags & MLX4_IB_QP_LSO)
			context->mtu_msgmax = (IB_MTU_4096 << 5) |
					      ilog2(dev->caps.max_gso_sz);
		else
			context->mtu_msgmax = (IB_MTU_4096 << 5) | 12;
	} else if (attr_mask & IB_QP_PATH_MTU) {
		if (attr->path_mtu < IB_MTU_256 || attr->path_mtu > IB_MTU_4096) {
			printk(KERN_ERR "path MTU (%u) is invalid\n",
			       attr->path_mtu);
			goto out;
		}
		context->mtu_msgmax = (attr->path_mtu << 5) |
			ilog2(dev->caps.max_msg_sz);
	}

	if (qp->rq.wqe_cnt)
		context->rq_size_stride = ilog2(qp->rq.wqe_cnt) << 3;
	context->rq_size_stride |= qp->rq.wqe_shift - 4;

	if (qp->sq.wqe_cnt)
		context->sq_size_stride = ilog2(qp->sq.wqe_cnt) << 3;
	context->sq_size_stride |= qp->sq.wqe_shift - 4;

	if (cur_state == IB_QPS_RESET && new_state == IB_QPS_INIT)
		context->sq_size_stride |= !!qp->sq_no_prefetch << 7;

	if (qp->ibqp.uobject)
		context->usr_page = bs32(to_mucontext(ibqp->uobject->context)->uar.index);
	else
		context->usr_page = bs32(dev->priv_uar.index);

	if (attr_mask & IB_QP_DEST_QPN)
		context->remote_qpn = bs32(attr->dest_qp_num);

	if (attr_mask & IB_QP_PORT) {
		if (cur_state == IB_QPS_SQD && new_state == IB_QPS_SQD &&
		    !(attr_mask & IB_QP_AV)) {
			mlx4_set_sched(&context->pri_path, attr->port_num);
			optpar |= MLX4_QP_OPTPAR_SCHED_QUEUE;
		}
	}

	if (attr_mask & IB_QP_PKEY_INDEX) {
		context->pri_path.pkey_index = attr->pkey_index;
		optpar |= MLX4_QP_OPTPAR_PKEY_INDEX;
	}

	if (attr_mask & IB_QP_AV) {
		if (mlx4_set_path(dev, &attr->ah_attr, &context->pri_path,
				  attr_mask & IB_QP_PORT ? attr->port_num : qp->port))
			goto out;

		optpar |= (MLX4_QP_OPTPAR_PRIMARY_ADDR_PATH |
			   MLX4_QP_OPTPAR_SCHED_QUEUE);
	}

	if (attr_mask & IB_QP_TIMEOUT) {
		context->pri_path.ackto |= attr->timeout << 3;
		optpar |= MLX4_QP_OPTPAR_ACK_TIMEOUT;
	}

	if (attr_mask & IB_QP_ALT_PATH) {
		if (attr->alt_port_num == 0 ||
		    attr->alt_port_num > dev->caps.num_ports)
			goto out;

		if (attr->alt_pkey_index >=
		    dev->caps.pkey_table_len[attr->alt_port_num])
			goto out;

		if (mlx4_set_path(dev, &attr->alt_ah_attr, &context->alt_path,
				  attr->alt_port_num))
			goto out;

		context->alt_path.pkey_index = attr->alt_pkey_index;
		context->alt_path.ackto = attr->alt_timeout << 3;
		optpar |= MLX4_QP_OPTPAR_ALT_ADDR_PATH;
	}

	context->pd	    = bs32(to_mpd(ibqp->pd)->pdn);
	context->params1    = bs32(MLX4_IB_ACK_REQ_FREQ << 28);

	// Set "fast registration enabled" for all kernel QPs
	if (!qp->ibqp.uobject)
		context->params1 |= bs32(1 << 11);

	if (attr_mask & IB_QP_RNR_RETRY) {
		context->params1 |= bs32(attr->rnr_retry << 13);
		optpar |= MLX4_QP_OPTPAR_RNR_RETRY;
	}

	if (attr_mask & IB_QP_RETRY_CNT) {
		context->params1 |= bs32(attr->retry_cnt << 16);
		optpar |= MLX4_QP_OPTPAR_RETRY_COUNT;
	}

	if (attr_mask & IB_QP_MAX_QP_RD_ATOMIC) {
		if (attr->max_rd_atomic)
			context->params1 |=
				bs32(fls(attr->max_rd_atomic - 1) << 21);
		optpar |= MLX4_QP_OPTPAR_SRA_MAX;
	}

	if (attr_mask & IB_QP_SQ_PSN)
		context->next_send_psn = bs32(attr->sq_psn);

	context->cqn_send = bs32(to_mcq(ibqp->send_cq)->mcq.cqn);

	if (attr_mask & IB_QP_MAX_DEST_RD_ATOMIC) {
		if (attr->max_dest_rd_atomic)
			context->params2 |=
				bs32(fls(attr->max_dest_rd_atomic - 1) << 21);
		optpar |= MLX4_QP_OPTPAR_RRA_MAX;
	}

	if (attr_mask & (IB_QP_ACCESS_FLAGS | IB_QP_MAX_DEST_RD_ATOMIC)) {
		context->params2 |= to_mlx4_access_flags(qp, attr, attr_mask);
		optpar |= MLX4_QP_OPTPAR_RWE | MLX4_QP_OPTPAR_RRE | MLX4_QP_OPTPAR_RAE;
	}

	if (ibqp->srq)
		context->params2 |= bs32(MLX4_QP_BIT_RIC);

	if (attr_mask & IB_QP_MIN_RNR_TIMER) {
		context->rnr_nextrecvpsn |= bs32(attr->min_rnr_timer << 24);
		optpar |= MLX4_QP_OPTPAR_RNR_TIMEOUT;
	}
	if (attr_mask & IB_QP_RQ_PSN)
		context->rnr_nextrecvpsn |= bs32(attr->rq_psn);

	context->cqn_recv = bs32(to_mcq(ibqp->recv_cq)->mcq.cqn);

	if (attr_mask & IB_QP_QKEY) {
		context->qkey = bs32(attr->qkey);
		optpar |= MLX4_QP_OPTPAR_Q_KEY;
	}

	if (ibqp->srq)
		context->srqn = bs32(1 << 24 | to_msrq(ibqp->srq)->msrq.srqn);

	if (!ibqp->srq && cur_state == IB_QPS_RESET && new_state == IB_QPS_INIT)
		context->db_rec_addr = cpu_to_be64(qp->db.dma);

	if (cur_state == IB_QPS_INIT &&
	    new_state == IB_QPS_RTR  &&
	    (ibqp->qp_type == IB_QPT_GSI || ibqp->qp_type == IB_QPT_SMI ||
	     ibqp->qp_type == IB_QPT_UD)) {
		context->pri_path.sched_queue = (qp->port - 1) << 6;
		if (is_qp0(dev, qp))
			context->pri_path.sched_queue |= MLX4_IB_DEFAULT_QP0_SCHED_QUEUE;
		else
			context->pri_path.sched_queue |= MLX4_IB_DEFAULT_SCHED_QUEUE;
	}

	if (cur_state == IB_QPS_RTS && new_state == IB_QPS_SQD	&&
	    attr_mask & IB_QP_EN_SQD_ASYNC_NOTIFY && attr->en_sqd_async_notify)
		sqd_event = 1;
	else
		sqd_event = 0;

	if (!ibqp->uobject && cur_state == IB_QPS_RESET && new_state == IB_QPS_INIT)
		context->rlkey |= (1 << 4);

	// Before passing a kernel QP to the HW, make sure that the
	// ownership bits of the send queue are set and the SQ
	// headroom is stamped so that the hardware doesn't start
	// processing stale work requests.
	if (!ibqp->uobject && cur_state == IB_QPS_RESET && new_state == IB_QPS_INIT) {
		struct mlx4_wqe_ctrl_seg *ctrl;
		int i;

		for (i = 0; i < qp->sq.wqe_cnt; ++i) {
			ctrl = get_send_wqe(qp, i);
			ctrl->owner_opcode = bs32(1 << 31);
			if (qp->sq_max_wqes_per_wr == 1)
				ctrl->fence_size = 1 << (qp->sq.wqe_shift - 4);

			stamp_send_wqe(qp, i, 1 << qp->sq.wqe_shift);
		}
	}

	err = mlx4_qp_modify(dev, &qp->mtt, to_mlx4_state(cur_state),
			     to_mlx4_state(new_state), context, optpar,
			     sqd_event, &qp->mqp);
	if (err)
		goto out;

	qp->state = new_state;

	if (attr_mask & IB_QP_ACCESS_FLAGS)
		qp->atomic_rd_en = attr->qp_access_flags;
	if (attr_mask & IB_QP_MAX_DEST_RD_ATOMIC)
		qp->resp_depth = attr->max_dest_rd_atomic;
	if (attr_mask & IB_QP_PORT) {
		qp->port = attr->port_num;
		update_mcg_macs(dev, qp);
	}
	if (attr_mask & IB_QP_ALT_PATH)
		qp->alt_port = attr->alt_port_num;

	if (is_sqp(dev, qp))
		store_sqp_attrs(to_msqp(qp), attr, attr_mask);

	// If we moved QP0 to RTR, bring the IB link up; if we moved
	// QP0 to RESET or ERROR, bring the link back down.
	if (is_qp0(dev, qp)) {
		if (cur_state != IB_QPS_RTR && new_state == IB_QPS_RTR)
			if (mlx4_INIT_PORT(dev, qp->port))
				printk(KERN_WARNING "INIT_PORT failed for port %d\n",
				       qp->port);

		if (cur_state != IB_QPS_RESET && cur_state != IB_QPS_ERR &&
		    (new_state == IB_QPS_RESET || new_state == IB_QPS_ERR))
			mlx4_CLOSE_PORT(dev, qp->port);
	}

	// If we moved a kernel QP to RESET, clean up all old CQ
	// entries and reinitialize the QP.
	if (new_state == IB_QPS_RESET && !ibqp->uobject) {
		mlx4_ib_cq_clean(to_mcq(ibqp->recv_cq), qp->mqp.qpn,
				 ibqp->srq ? to_msrq(ibqp->srq): NULL);
		if (ibqp->send_cq != ibqp->recv_cq)
			mlx4_ib_cq_clean(to_mcq(ibqp->send_cq), qp->mqp.qpn, NULL);

		qp->rq.head = 0;
		qp->rq.tail = 0;
		qp->sq.head = 0;
		qp->sq.tail = 0;
		qp->sq_next_wqe = 0;
		if (!ibqp->srq)
			*qp->db.db  = 0;
	}

out:*/
	//kfree(context);
	return err;
}

int modify_qp(struct ibv_qp *ibqp, struct ibv_qp_attr *attr, int attr_mask)
{
/*	if (ibqp->state == IBV_QPS_RESET && attr_mask & IBV_QP_STATE && attr->qp_state == IBV_QPS_INIT)
	{
		DRIVER_LOG(DRIVER_PREFIX "modify_qp: have to init sq ownership\n");
		//mlx4_qp_init_sq_ownership((struct mlx4_ib_qp*)ibqp);
	}
	
	struct mlx4_ib_dev *dev = (struct mlx4_ib_dev*)ibqp->device;
	struct mlx4_ib_qp *qp = (struct mlx4_ib_qp*)ibqp;
	enum ibv_qp_state cur_state, new_state;
	
	int ret = -1;

	//mutex_lock(&qp->mutex);

	cur_state = attr_mask & IBV_QP_CUR_STATE ? attr->cur_qp_state : qp->state;
	new_state = attr_mask & IBV_QP_STATE ? attr->qp_state : cur_state;

	if (!ib_modify_qp_is_ok(cur_state, new_state, ibqp->qp_type, attr_mask))
		goto out;

	if ((attr_mask & IBV_QP_PORT) &&
	    (attr->port_num == 0 || attr->port_num > dev->caps.num_ports)) {
		goto out;
	}

	if (attr_mask & IBV_QP_PKEY_INDEX) {
		int p = attr_mask & IBv_QP_PORT ? attr->port_num : qp->port;
		if (attr->pkey_index >= dev->caps.pkey_table_len[p])
			goto out;
	}

	if (attr_mask & IBV_QP_MAX_QP_RD_ATOMIC &&
	    attr->max_rd_atomic > dev->caps.max_qp_init_rdma) {
		goto out;
	}

	if (attr_mask & IBV_QP_MAX_DEST_RD_ATOMIC &&
	    attr->max_dest_rd_atomic > dev->caps.max_qp_dest_rdma) {
		goto out;
	}

	if (cur_state == new_state && cur_state == IBV_QPS_RESET) {
		ret = 0;
		goto out;
	}

	ret = __mlx4_ib_modify_qp(ibqp, attr, attr_mask, cur_state, new_state);

out:
	//mutex_unlock(&qp->mutex);
	
	if (!ret && (attr_mask & IBV_QP_STATE) && attr->qp_state == IBV_QPS_RESET)
	{
		DRIVER_LOG(DRIVER_PREFIX "modify_qp: have to do cleanup..\n");
		// mlx4_cq_clean(to_mcq(qp->recv_cq), qp->qp_num,
			       // qp->srq ? to_msrq(qp->srq) : NULL);
		// if (qp->send_cq != qp->recv_cq)
			// mlx4_cq_clean(to_mcq(qp->send_cq), qp->qp_num, NULL);

		// mlx4_init_qp_indices(to_mqp(qp));
		// if (!qp->srq && qp->qp_type != IBV_QPT_XRC)
			// *to_mqp(qp)->db = 0;
	}
	return ret;*/
	return -1;
}

int destroy_qp(struct ibv_qp *qp)
{
return 0;
}

int post_send(struct ibv_qp *ibvqp, struct ibv_send_wr *wr, struct ibv_send_wr **bad_wr)
{
*bad_wr = wr;
return -1;
/*	struct mlx4_ib_qp *qp = (struct mlx4_ib_qp*)ibvqp;
	void *wqe;
	struct mlx4_wqe_ctrl_seg *ctrl;
	struct mlx4_wqe_data_seg *dseg;
	unsigned long flags;
	int nreq;
	int err = 0;
	unsigned ind;
	int stamp;
	int size;
	unsigned seglen;
	u32 dummy;
	u32 *lso_wqe;
	u32 lso_hdr_sz;
	u32 blh;
	int i;
	u16 vlan = bs16(0xffff);

	//spin_lock_irqsave(&qp->sq.lock, flags);

	ind = qp->sq_next_wqe;

	for (nreq = 0; wr; ++nreq, wr = wr->next) {
		lso_wqe = &dummy;
		blh = 0;

		if (mlx4_wq_overflow(&qp->sq, nreq, qp->ibqp.send_cq)) {
			err = -1;
			*bad_wr = wr;
			goto out;
		}

		if (unlikely(wr->num_sge > qp->sq.max_gs)) {
			err = -1;
			*bad_wr = wr;
			goto out;
		}

		ctrl = wqe = get_send_wqe(qp, ind & (qp->sq.wqe_cnt - 1));
		qp->sq.wrid[(qp->sq.head + nreq) & (qp->sq.wqe_cnt - 1)] = wr->wr_id;

		ctrl->srcrb_flags =
			(wr->send_flags & IB_SEND_SIGNALED ?
			 bs32(MLX4_WQE_CTRL_CQ_UPDATE) : 0) |
			(wr->send_flags & IB_SEND_SOLICITED ?
			 bs32(MLX4_WQE_CTRL_SOLICITED) : 0) |
			((wr->send_flags & IB_SEND_IP_CSUM) ?
			 bs32(MLX4_WQE_CTRL_IP_CSUM |
				     MLX4_WQE_CTRL_TCP_UDP_CSUM) : 0) |
			qp->sq_signal_bits;

		ctrl->imm = send_ieth(wr);

		wqe += sizeof *ctrl;
		size = sizeof *ctrl / 16;

		switch (ibvqp->qp_type) {
		case IB_QPT_RC:
		case IB_QPT_UC:
			switch (wr->opcode) {
			case IB_WR_ATOMIC_CMP_AND_SWP:
			case IB_WR_ATOMIC_FETCH_AND_ADD:
			case IB_WR_MASKED_ATOMIC_FETCH_AND_ADD:
				set_raddr_seg(wqe, wr->wr.atomic.remote_addr,
					      wr->wr.atomic.rkey);
				wqe  += sizeof (struct mlx4_wqe_raddr_seg);

				set_atomic_seg(wqe, wr);
				wqe  += sizeof (struct mlx4_wqe_atomic_seg);

				size += (sizeof (struct mlx4_wqe_raddr_seg) +
					 sizeof (struct mlx4_wqe_atomic_seg)) / 16;

				break;

			case IB_WR_MASKED_ATOMIC_CMP_AND_SWP:
				set_raddr_seg(wqe, wr->wr.atomic.remote_addr,
					      wr->wr.atomic.rkey);
				wqe  += sizeof (struct mlx4_wqe_raddr_seg);

				set_masked_atomic_seg(wqe, wr);
				wqe  += sizeof (struct mlx4_wqe_masked_atomic_seg);

				size += (sizeof (struct mlx4_wqe_raddr_seg) +
					 sizeof (struct mlx4_wqe_masked_atomic_seg)) / 16;

				break;

			case IB_WR_RDMA_READ:
			case IB_WR_RDMA_WRITE:
			case IB_WR_RDMA_WRITE_WITH_IMM:
				set_raddr_seg(wqe, wr->wr.rdma.remote_addr,
					      wr->wr.rdma.rkey);
				wqe  += sizeof (struct mlx4_wqe_raddr_seg);
				size += sizeof (struct mlx4_wqe_raddr_seg) / 16;
				break;

			case IB_WR_LOCAL_INV:
				ctrl->srcrb_flags |=
					bs32(MLX4_WQE_CTRL_STRONG_ORDER);
				set_local_inv_seg(wqe, wr->ex.invalidate_rkey);
				wqe  += sizeof (struct mlx4_wqe_local_inval_seg);
				size += sizeof (struct mlx4_wqe_local_inval_seg) / 16;
				break;

			case IB_WR_FAST_REG_MR:
				ctrl->srcrb_flags |=
					bs32(MLX4_WQE_CTRL_STRONG_ORDER);
				set_fmr_seg(wqe, wr);
				wqe  += sizeof (struct mlx4_wqe_fmr_seg);
				size += sizeof (struct mlx4_wqe_fmr_seg) / 16;
				break;

			default:
				// No extra segments required for sends
				break;
			}
			break;

		case IB_QPT_UD:
			set_datagram_seg(wqe, wr, &vlan);
			wqe  += sizeof (struct mlx4_wqe_datagram_seg);
			size += sizeof (struct mlx4_wqe_datagram_seg) / 16;

			if (wr->opcode == IB_WR_LSO) {
				err = build_lso_seg(wqe, wr, qp, &seglen, &lso_hdr_sz, &blh);
				if (unlikely(err)) {
					*bad_wr = wr;
					goto out;
				}
				lso_wqe = (__be32 *) wqe;
				wqe  += seglen;
				size += seglen / 16;
			}
			break;

		case IB_QPT_SMI:
		case IB_QPT_GSI:
			err = 1;
			// err = build_mlx_header(to_msqp(qp), wr, ctrl, &seglen);
			// if (unlikely(err)) {
				// *bad_wr = wr;
				// goto out;
			// }
			// wqe  += seglen;
			// size += seglen / 16;
			break;

		default:
			break;
		}

		// Write data segments in reverse order, so as to
		// overwrite cacheline stamp last within each
		// cacheline.  This avoids issues with WQE
		// prefetching.

		dseg = wqe;
		dseg += wr->num_sge - 1;
		size += wr->num_sge * (sizeof (struct mlx4_wqe_data_seg) / 16);

		// Add one more inline data segment for ICRC for MLX sends
		if (unlikely(qp->ibqp.qp_type == IB_QPT_SMI ||
			     qp->ibqp.qp_type == IB_QPT_GSI)) {
			set_mlx_icrc_seg(dseg + 1);
			size += sizeof (struct mlx4_wqe_data_seg) / 16;
		}

		for (i = wr->num_sge - 1; i >= 0; --i, --dseg)
			set_data_seg(dseg, wr->sg_list + i);

		// Possibly overwrite stamping in cacheline with LSO
		// segment only after making sure all data segments
		// are written.
		wmb();
		*lso_wqe = lso_hdr_sz;

		ctrl->fence_size = (wr->send_flags & IB_SEND_FENCE ?
				    MLX4_WQE_CTRL_FENCE : 0) | size;

		if (be16_to_cpu(vlan) < 0x1000) {
			ctrl->ins_vlan = 1 << 6;
			ctrl->vlan_tag = vlan;
		}

		// Make sure descriptor is fully written before
		// setting ownership bit (because HW can start
		// executing as soon as we do).
		wmb();

		if (wr->opcode < 0 || wr->opcode >= ARRAY_SIZE(mlx4_ib_opcode)) {
			err = -EINVAL;
			goto out;
		}

		ctrl->owner_opcode = mlx4_ib_opcode[wr->opcode] |
			(ind & qp->sq.wqe_cnt ? bs32(1 << 31) : 0) | blh;

		stamp = ind + qp->sq_spare_wqes;
		ind += DIV_ROUND_UP(size * 16, 1U << qp->sq.wqe_shift);

		// We can improve latency by not stamping the last
		// send queue WQE until after ringing the doorbell, so
		// only stamp here if there are still more WQEs to post.
		//
		// Same optimization applies to padding with NOP wqe
		// in case of WQE shrinking (used to prevent wrap-around
		// in the middle of WR).
		if (wr->next) {
			stamp_send_wqe(qp, stamp, size * 16);
			ind = pad_wraparound(qp, ind);
		}
	}

out:
	if (likely(nreq)) {
		qp->sq.head += nreq;

		// Make sure that descriptors are written before
		// doorbell record.
		wmb();

		writel(qp->doorbell_qpn,
		       ((struct mlx4_device*)ibvqp->device)->uar_map + MLX4_SEND_DOORBELL);

		// Make sure doorbells don't leak out of SQ spinlock
		// and reach the HCA out of order.
		mmiowb();

		stamp_send_wqe(qp, stamp, size * 16);

		ind = pad_wraparound(qp, ind);
		qp->sq_next_wqe = ind;
	}

	//spin_unlock_irqrestore(&qp->sq.lock, flags);

	return err;*/
}

int post_recv(struct ibv_qp *ibvqp, struct ibv_recv_wr *wr, struct ibv_recv_wr **bad_wr)
{
*bad_wr = wr;
return -1;
/*	struct mlx4_ib_qp *qp = (struct mlx4_ib_qp*)ibvqp;
	struct mlx4_wqe_data_seg *scat;
	unsigned long flags;
	int err = 0;
	int nreq;
	int ind;
	int i;

	//spin_lock_irqsave(&qp->rq.lock, flags);

	ind = qp->rq.head & (qp->rq.wqe_cnt - 1);

	for (nreq = 0; wr; ++nreq, wr = wr->next) {
		if (mlx4_wq_overflow(&qp->rq, nreq, qp->ibqp.recv_cq)) {
			err = -1;
			*bad_wr = wr;
			goto out;
		}

		if (wr->num_sge > qp->rq.max_gs) {
			err = -1;
			*bad_wr = wr;
			goto out;
		}

		scat = get_recv_wqe(qp, ind);

		for (i = 0; i < wr->num_sge; ++i)
			__set_data_seg(scat + i, wr->sg_list + i);

		if (i < qp->rq.max_gs) {
			scat[i].byte_count = 0;
			scat[i].lkey       = bs32(MLX4_INVALID_LKEY);
			scat[i].addr       = 0;
		}

		qp->rq.wrid[ind] = wr->wr_id;

		ind = (ind + 1) & (qp->rq.wqe_cnt - 1);
	}

out:
	if (nreq) {
		qp->rq.head += nreq;

		// Make sure that descriptors are written before
		// doorbell record.
		wmb();

		*qp->db.db = bs32(qp->rq.head & 0xffff);
	}

	//spin_unlock_irqrestore(&qp->rq.lock, flags);

	return err;*/
}


static struct ibv_context* alloc_context(struct ibv_device *device, int cmd_fd)
{
	struct mlx4_device* dev = (struct mlx4_device*)device;
	struct mlx4_context* mlx4_ctx = &mlx_context[0];
	struct ibv_context* ctx = &mlx4_ctx->ibv_ctx;
	
	if (mlx4_uar_alloc(dev, &mlx4_ctx->uar))
	{
		return 0;
	}
	ctx->device = device;
/*
	int			(*query_device)(struct ibv_context *context,
					      struct ibv_device_attr *device_attr);
	int			(*query_port)(struct ibv_context *context, u8 port_num,
					      struct ibv_port_attr *port_attr);*/
	ctx->ops.alloc_pd = alloc_pd;
	ctx->ops.dealloc_pd = dealloc_pd;
	ctx->ops.reg_mr = reg_mr;
/*	struct ibv_mr *		(*rereg_mr)(struct ibv_mr *mr,
					    int flags,
					    struct ibv_pd *pd, void *addr,
					    size_t length,
					    int access);*/
	ctx->ops.dereg_mr = dereg_mr;
/*	struct ibv_mw *		(*alloc_mw)(struct ibv_pd *pd, enum ibv_mw_type type);
	int			(*bind_mw)(struct ibv_qp *qp, struct ibv_mw *mw,
					   struct ibv_mw_bind *mw_bind);
	int			(*dealloc_mw)(struct ibv_mw *mw);*/
	ctx->ops.create_cq = create_cq;
/*	int			(*poll_cq)(struct ibv_cq *cq, int num_entries, struct ibv_wc *wc);
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
						 struct ibv_recv_wr **bad_recv_wr);*/
	ctx->ops.create_qp = create_qp;
	ctx->ops.query_qp = query_qp;
	ctx->ops.modify_qp = modify_qp;
	ctx->ops.destroy_qp = destroy_qp;
	ctx->ops.post_send = post_send;
	ctx->ops.post_recv = post_recv;
/*	struct ibv_ah *		(*create_ah)(struct ibv_pd *pd, struct ibv_ah_attr *attr);
	int			(*destroy_ah)(struct ibv_ah *ah);
	int			(*attach_mcast)(struct ibv_qp *qp, const union ibv_gid *gid,
						u16 lid);
	int			(*detach_mcast)(struct ibv_qp *qp, const union ibv_gid *gid,
						u16 lid);
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

struct ibv_device* infiniband_mlx4_init(struct Device* device)
{
	if (g_DeviceCount>=DRIVER_MAX_DEVICES)
	{
		DRIVER_LOG(DRIVER_PREFIX "Cannot allocate enough space for more Devices\n");
		return 0;
	}
	// enable device
	unsigned short cmd = device->pcie.read_config_word(&device->pcie, 4);
	mb();
	unsigned short newcmd = cmd | 0x06;	// enable bus master and memory access
	if (cmd!=newcmd)
	{
		device->pcie.write_config_word(&device->pcie, 4, newcmd);
		mb();
	}
	// Check for BARs.  We expect 0: 1MB
// hack for qemu, just use memory pages
//	if ((device->pcie.barType[2] & 6)!=4)
	if ((device->pcie.barType[0] & 1)!=0
		|| device->pcie.barSize[0] != 1 << 20)
	{
		DRIVER_LOG(DRIVER_PREFIX "Missing DCS, aborting. %X:%X %X %X:%X\n", device->pcie.bar[1], device->pcie.bar[0], device->pcie.barType[0], device->pcie.barSize[1], device->pcie.barSize[0]);
		return 0;
	}
//	if ((device->pcie.barType[2] & 6)!=4)
	if ((device->pcie.barType[2] & 1)!=0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Missing UAR, aborting. %X:%X %X %X:%X\n", device->pcie.bar[3], device->pcie.bar[2], device->pcie.barType[2], device->pcie.barSize[3], device->pcie.barSize[2]);
		return 0;
	}
	struct mlx4_device* mlx4 = &g_Device[g_DeviceCount];
	memcpy(&mlx4->pcie, &device->pcie, sizeof(struct pcie_device));
	mlx4->dcs= platform_map((void*)((L4_Word_t)device->pcie.bar[0] + ((L4_Word_t)device->pcie.bar[1] << 32)), 1 << 20);
	mlx4->uar = platform_map((void*)((L4_Word_t)device->pcie.bar[2] + ((L4_Word_t)device->pcie.bar[3] << 32)), (L4_Word_t)device->pcie.barSize[2] + ((L4_Word_t)device->pcie.barSize[3] << 32));
	if (mlx4->dcs==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Couldn't map DCS.\n");
		return 0;
	}
	if (mlx4->uar==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Couldn't map UAR.\n");
		return 0;
	}
	// init cmd
	mlx4->cmd.pool = platform_mapAny(4096);
	if (mlx4->cmd.pool==0)
	{
		DRIVER_LOG(DRIVER_PREFIX "Couldn't allocate cmd pool.\n");
		return 0;
	}
	if (mlx4_init(mlx4))
	{
		DRIVER_LOG(DRIVER_PREFIX "error\n");
		return 0;
	}
	g_DeviceCount++;
	mlx4->ibv_device.node_type = IBV_NODE_CA;
	mlx4->ibv_device.ops.alloc_context = alloc_context;
	mlx4->ibv_device.ops.free_context = free_context;
	mlx4->ibv_device.transport_type = IBV_TRANSPORT_IB;
	return &mlx4->ibv_device;
}



enum {
	MLX4_CQ_ENTRY_SIZE		= 0x20
};


int alloc_mtt_buffer(struct mlx4_device* dev, struct mlx4_mr* mr, void *addr, u64 page_count)
{
	u64* inparam = platform_mapAny(1 << 12);
	struct mlx4_mtt *mtt = &mr->mtt;
	int i = 0, n = 0, j;
	for (j = 0; j < page_count; ++j)
	{
		inparam[i++] = ((u64)addr)+(1<<12)*j;
		if (i == PAGE_SIZE / sizeof (u64))
		{
			if (mlx4_write_mtt(dev, mtt, n, i, inparam))
			{
				platform_unmap(inparam);
				DRIVER_LOG(DRIVER_PREFIX "mlx4_write_mtt failed (%X, %d)\n", n, i);
				return 1;
			}
			n += i;
			i = 0;
		}
	}
	if (i && mlx4_write_mtt(dev, mtt, n, i, inparam))	// the rest
	{
		platform_unmap(inparam);
		DRIVER_LOG(DRIVER_PREFIX "mlx4_write_mtt failed (%X, %d)\n", n, i);
		return 1;
	}
	platform_unmap(inparam);
	return mlx4_mr_enable(dev, mr);
}

static int mlx4_ib_alloc_cq_buf(struct mlx4_dev *dev, struct mlx4_ib_cq_buf *buf, int nent)
{
	int err;
	
	err = mlx4_buf_alloc(dev, nent * sizeof(struct mlx4_cqe), PAGE_SIZE * 2, &buf->buf);
	if (err)
	{
		return 1;
	}

	err = mlx4_mtt_init(dev, buf->buf.npages, buf->buf.page_shift, &buf->mtt);
	if (err)
	{
		return 1;
	}

	err = mlx4_buf_write_mtt(dev, &buf->mtt, &buf->buf);
	if (err)
	{
		return 1;
	}

	return 0;
}

struct ibv_qp *mlx4_create_qp(struct ibv_pd *pd, struct ibv_qp_init_attr *attr)
{
/*	int ret;
	struct mlx4_context	 *context = (struct mlx4_context*)(pd->context);

	// Sanity check QP size before proceeding
	if (verify_sizes(attr, context))
		return NULL;

	static struct mlx4_ib_qp g_qp;
	struct mlx4_ib_qp* qp = &g_qp;

	mlx4_calc_sq_wqe_size(&attr->cap, attr->qp_type, qp);

	// We need to leave 2 KB + 1 WQE of headroom in the SQ to
	// allow HW to prefetch.
	qp->sq_spare_wqes = (2048 >> qp->sq.wqe_shift) + 1;
	qp->sq.wqe_cnt = align_queue_size(attr->cap.max_send_wr + qp->sq_spare_wqes);
	qp->rq.wqe_cnt = align_queue_size(attr->cap.max_recv_wr);

	if (attr->srq || attr->qp_type == IBV_QPT_XRC)
		attr->cap.max_recv_wr = qp->rq.wqe_cnt = 0;
	else {
		if (attr->cap.max_recv_sge < 1)
			attr->cap.max_recv_sge = 1;
		if (attr->cap.max_recv_wr < 1)
			attr->cap.max_recv_wr = 1;
	}

	if (mlx4_alloc_qp_buf(pd, &attr->cap, attr->qp_type, qp))
		goto err;

	mlx4_init_qp_indices(qp);

	// if (pthread_spin_init(&qp->sq.lock, PTHREAD_PROCESS_PRIVATE) ||
	    // pthread_spin_init(&qp->rq.lock, PTHREAD_PROCESS_PRIVATE))
		// goto err_free;

	if (!attr->srq && attr->qp_type != IBV_QPT_XRC) {
		qp->db = mlx4_alloc_db(context, MLX4_DB_TYPE_RQ);
		if (!qp->db)
			goto err_free;

		*qp->db = 0;
	}

	cmd.buf_addr	    = (uintptr_t) qp->buf.buf;
	if (attr->srq || attr->qp_type == IBV_QPT_XRC)
		cmd.db_addr = 0;
	else
		cmd.db_addr = (uintptr_t) qp->db;
	cmd.log_sq_stride   = qp->sq.wqe_shift;
	for (cmd.log_sq_bb_count = 0;
	     qp->sq.wqe_cnt > 1 << cmd.log_sq_bb_count;
	     ++cmd.log_sq_bb_count)
		; // nothing
	cmd.sq_no_prefetch = 0;	// OK for ABI 2: just a reserved field
	memset(cmd.reserved, 0, sizeof cmd.reserved);

	pthread_mutex_lock(&to_mctx(pd->context)->qp_table_mutex);

	ret = ibv_cmd_create_qp(pd, &qp->ibv_qp, attr, &cmd.ibv_cmd, sizeof cmd,
				&resp, sizeof resp);
	if (ret)
		goto err_rq_db;

	ret = mlx4_store_qp(to_mctx(pd->context), qp->ibv_qp.qp_num, qp);
	if (ret)
		goto err_destroy;
	pthread_mutex_unlock(&to_mctx(pd->context)->qp_table_mutex);

	qp->rq.wqe_cnt = attr->cap.max_recv_wr;
	qp->rq.max_gs  = attr->cap.max_recv_sge;

	// adjust rq maxima to not exceed reported device maxima
	attr->cap.max_recv_wr = min(context->max_qp_wr, attr->cap.max_recv_wr);
	attr->cap.max_recv_sge = min(context->max_sge, attr->cap.max_recv_sge);

	qp->rq.max_post = attr->cap.max_recv_wr;
	mlx4_set_sq_sizes(qp, &attr->cap, attr->qp_type);

	qp->doorbell_qpn    = htonl(qp->ibv_qp.qp_num << 8);
	if (attr->sq_sig_all)
		qp->sq_signal_bits = htonl(MLX4_WQE_CTRL_CQ_UPDATE);
	else
		qp->sq_signal_bits = 0;

	return &qp->ibv_qp;

err_destroy:
	ibv_cmd_destroy_qp(&qp->ibv_qp);

err_rq_db:
	pthread_mutex_unlock(&to_mctx(pd->context)->qp_table_mutex);
	if (!attr->srq && attr->qp_type != IBV_QPT_XRC)
		mlx4_free_db(to_mctx(pd->context), MLX4_DB_TYPE_RQ, qp->db);

err_free:
	free(qp->sq.wrid);
	if (qp->rq.wqe_cnt)
		free(qp->rq.wrid);
	mlx4_free_buf(&qp->buf);

err:
	free(qp);
*/
	return 0;
}
/*
static void *get_recv_wqe(struct mlx4_ib_qp *qp, int n)
{
	return qp->buf.buf + qp->rq.offset + (n << qp->rq.wqe_shift);
}

static void *get_send_wqe(struct mlx4_ib_qp *qp, int n)
{
	return qp->buf.buf + qp->sq.offset + (n << qp->sq.wqe_shift);
}

static void stamp_send_wqe(struct mlx4_ib_qp *qp, int n)
{
	u32 *wqe = get_send_wqe(qp, n);
	int i;
	int ds = (((struct mlx4_wqe_ctrl_seg *)wqe)->fence_size & 0x3f) << 2;

	for (i = 16; i < ds; i += 16)
		wqe[i] = 0xffffffff;
}
*/
void mlx4_init_qp_indices(struct mlx4_ib_qp *qp)
{
	qp->sq.head	 = 0;
	qp->sq.tail	 = 0;
	qp->rq.head	 = 0;
	qp->rq.tail	 = 0;
}
/*
void mlx4_qp_init_sq_ownership(struct mlx4_ib_qp *qp)
{
	struct mlx4_wqe_ctrl_seg *ctrl;
	int i;

	for (i = 0; i < qp->sq.wqe_cnt; ++i) {
		ctrl = get_send_wqe(qp, i);
		ctrl->owner_opcode = bs32(1 << 31);
		ctrl->fence_size = 1 << (qp->sq.wqe_shift - 4);

		stamp_send_wqe(qp, i);
	}
}

struct mlx4_cqe {
	u32	my_qpn;
	u32	immed_rss_invalid;
	u32	g_mlpath_rqpn;
	u16	sl_vid;
	u16	rlid;
	u32	reserved2;
	u32	byte_cnt;
	u16	wqe_index;
	u16	checksum;
	u8		reserved3[3];
	u8		owner_sr_opcode;
};

static struct mlx4_cqe *get_cqe(struct mlx4_ib_cq *cq, int entry)
{
	return cq->buf.buf + entry * MLX4_CQ_ENTRY_SIZE;
}

static void update_cons_index(struct mlx4_ib_cq *cq)
{
	*cq->mcq.set_ci_db = bs32(cq->mcq.cons_index & 0xffffff);
}

void mlx4_cq_clean(struct mlx4_ib_cq *cq, u32 qpn, struct mlx4_srq *srq)
{
	struct mlx4_cqe *cqe, *dest;
	u32 prod_index;
	u8 owner_bit;
	int nfreed = 0;
	int is_xrc_srq = 0;

//	if (srq && srq->ibv_srq.xrc_cq)
//		is_xrc_srq = 1;

	// First we need to find the current producer index, so we
	// know where to start cleaning from.  It doesn't matter if HW
	// adds new entries after this loop -- the QP we're worried
	// about is already in RESET, so the new entries won't come
	// from our QP and therefore don't need to be checked.
	for (prod_index = cq->cons_index; get_sw_cqe(cq, prod_index); ++prod_index)
		if (prod_index == cq->cons_index + cq->ibv_cq.cqe)
			break;

	// Now sweep backwards through the CQ, removing CQ entries
	// that match our QP by copying older entries on top of them.
	while ((int) --prod_index - (int) cq->cons_index >= 0) {
		cqe = get_cqe(cq, prod_index & cq->ibv_cq.cqe);
		if (is_xrc_srq &&
		    (ntohl(cqe->g_mlpath_rqpn & 0xffffff) == srq->srqn) &&
		    !(cqe->owner_sr_opcode & MLX4_CQE_IS_SEND_MASK)) {
			mlx4_free_srq_wqe(srq, ntohs(cqe->wqe_index));
			++nfreed;
		} else if ((bs32(cqe->my_qpn) & 0xffffff) == qpn) {
//			if (srq && !(cqe->owner_sr_opcode & MLX4_CQE_IS_SEND_MASK))
//				mlx4_free_srq_wqe(srq, ntohs(cqe->wqe_index));
			++nfreed;
		} else if (nfreed) {
			dest = get_cqe(cq, (prod_index + nfreed) & cq->ibv_cq.cqe);
			owner_bit = dest->owner_sr_opcode & MLX4_CQE_OWNER_MASK;
			memcpy(dest, cqe, sizeof *cqe);
			dest->owner_sr_opcode = owner_bit | (dest->owner_sr_opcode & ~MLX4_CQE_OWNER_MASK);
		}
	}

	if (nfreed) {
		cq->cons_index += nfreed;
		// Make sure update of buffer contents is done before
		// updating consumer index.
		wmb();
		update_cons_index(cq);
	}
}
*/
void mlx4_test(struct ibv_pd* ibvpd, int client)
{
	DRIVER_LOG(DRIVER_PREFIX "test started in %s mode\n", client ? "client" : "server");
	int err;
	struct mlx4_ib_pd* pd = (struct mlx4_ib_pd*)ibvpd;
	struct mlx4_context* ctx = (struct mlx4_context*)ibvpd->context;
	struct mlx4_device* dev = (struct mlx4_device*)ibvpd->context->device;
	// reg MR
	struct mlx4_mr g_mr;
	struct mlx4_mr* mr = &g_mr;
	void* addr = platform_mapAny(1 << 23);	// 8MB buffer size
	if (addr == 0)
	{
		DRIVER_LOG(DRIVER_PREFIX "couldn't alloc memory\n");
	}
	err = mlx4_mr_alloc(dev, pd->pdn, addr, 1 << 23, MLX4_PERM_REMOTE_WRITE | MLX4_PERM_LOCAL_WRITE | MLX4_PERM_LOCAL_READ, 1 << 11, 1 << 12, mr);
	if (err)
	{
		DRIVER_LOG(DRIVER_PREFIX "mlx4_mr_alloc failed\n");
		return;
	}
	// map buffer to mtt
	if (alloc_mtt_buffer(dev, mr, addr, 1 << 11))
	{
		DRIVER_LOG(DRIVER_PREFIX "alloc_mtt_buffer failed\n");
		return;
	}
	DRIVER_LOG(DRIVER_PREFIX "memory region registered\n");
	struct mlx4_ib_cq g_cq;
	struct mlx4_ib_cq *cq = &g_cq;
	{	// create CQ
		// alloc doorbell
		err = mlx4_db_alloc(dev, &cq->db, 1);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_db_alloc failed\n");
			return;
		}
		cq->mcq.set_ci_db  = cq->db.db;
		cq->mcq.arm_db     = cq->db.db + 1;
		*cq->mcq.set_ci_db = 0;
		*cq->mcq.arm_db    = 0;

		// allocate buffer for events
		int nent = 512;
		err = mlx4_ib_alloc_cq_buf(dev, &cq->buf, 512);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_ib_alloc_cq_buf failed\n");
			return;
		}
		err = mlx4_cq_alloc(dev, 512, &cq->buf.mtt, &ctx->uar, cq->db.dma, &cq->mcq, 0, 0);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_cq_alloc failed\n");
			return;
		}
		// cq->mcq.comp  = mlx4_ib_cq_comp;
		// cq->mcq.event = mlx4_ib_cq_event;
		DRIVER_LOG(DRIVER_PREFIX "cq allocated\n");
	}
	struct mlx4_ib_qp g_qp;
	struct mlx4_ib_qp *qp = &g_qp;
	{	// create QP
		qp->state	 = IBV_QPS_RESET;
		qp->rq.wqe_cnt	 = 512;
		qp->rq.max_gs	 = 1;
		qp->rq.wqe_shift = ilog2(qp->rq.max_gs * sizeof (struct mlx4_wqe_data_seg));
		int s = max(1 * sizeof (struct mlx4_wqe_data_seg),
			400 + sizeof (struct mlx4_wqe_inline_seg)) +
			sizeof (struct mlx4_wqe_ctrl_seg) +
			sizeof (struct mlx4_wqe_atomic_seg) +
			sizeof (struct mlx4_wqe_raddr_seg);

		qp->sq.wqe_shift = ilog2(roundup_pow_of_two(s));
		for (;;) {
			qp->sq_max_wqes_per_wr = (s +(1U << qp->sq.wqe_shift)-1)/ (1U << qp->sq.wqe_shift);
			
			/*
			 * We need to leave 2 KB + 1 WR of headroom in the SQ to
			 * allow HW to prefetch.
			 */
			qp->sq_spare_wqes = (2048 >> qp->sq.wqe_shift) + qp->sq_max_wqes_per_wr;
			qp->sq.wqe_cnt = roundup_pow_of_two(1 *
								qp->sq_max_wqes_per_wr +
								qp->sq_spare_wqes);

			if (qp->sq.wqe_cnt <= dev->caps.max_wqes)
				break;

			if (qp->sq_max_wqes_per_wr <= 1)
			{
				DRIVER_LOG(DRIVER_PREFIX "qp->sq_max_wqes_per_wr <= 1 failed\n");
				return;
			}

			++qp->sq.wqe_shift;
		}
		qp->sq.max_gs = (min(dev->caps.max_sq_desc_sz,
					 (qp->sq_max_wqes_per_wr << qp->sq.wqe_shift)) -
				(sizeof (struct mlx4_wqe_ctrl_seg) +
				sizeof (struct mlx4_wqe_atomic_seg) +
				sizeof (struct mlx4_wqe_raddr_seg))) /
			sizeof (struct mlx4_wqe_data_seg);

		qp->buf_size = (qp->rq.wqe_cnt << qp->rq.wqe_shift) + (qp->sq.wqe_cnt << qp->sq.wqe_shift);
		if (qp->rq.wqe_shift > qp->sq.wqe_shift) {
			qp->rq.offset = 0;
			qp->sq.offset = qp->rq.wqe_cnt << qp->rq.wqe_shift;
		} else {
			qp->rq.offset = qp->sq.wqe_cnt << qp->sq.wqe_shift;
			qp->sq.offset = 0;
		}
		
		err = mlx4_db_alloc(dev, &qp->db, 0);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_db_alloc failed\n");
			return;
		}
		*qp->db.db = 0;

		mlx4_init_qp_indices(qp);
		
		if (mlx4_buf_alloc(dev, qp->buf_size, PAGE_SIZE * 2, &qp->buf))
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_buf_alloc failed\n");
			return;
		}

		err = mlx4_mtt_init(dev, qp->buf.npages, qp->buf.page_shift, &qp->mtt);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_mtt_init failed\n");
			return;
		}

		err = mlx4_buf_write_mtt(dev, &qp->mtt, &qp->buf);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_buf_write_mtt failed\n");
			return;
		}

		qp->sq.wrid  = platform_mapAny(qp->sq.wqe_cnt * sizeof (u64));
		qp->rq.wrid  = platform_mapAny(qp->rq.wqe_cnt * sizeof (u64));

		if (!qp->sq.wrid || !qp->rq.wrid)
		{
			DRIVER_LOG(DRIVER_PREFIX "alloc sq/rq.wrid failed\n");
			return;
		}
		int qpn;
		err = mlx4_qp_reserve_range(dev, 1, 1, &qpn);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_qp_reserve_range failed\n");
			return;
		}
		err = mlx4_qp_alloc(dev, qpn, &qp->mqp);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_qp_alloc failed\n");
			return;
		}

		/*
		 * Hardware wants QPN written in big-endian order (after
		 * shifting) for send doorbell.  Precompute this value to save
		 * a little bit when posting sends.
		 */
		qp->doorbell_qpn = bs32(qp->mqp.qpn << 8);

//		qp->mqp.event = mlx4_ib_qp_event;
//		qp->ibqp.qp_num = qp->mqp.qpn;
		DRIVER_LOG(DRIVER_PREFIX "qp allocated\n");
	}
	// following code doesn't work
/*	{	// setup QP
		mlx4_qp_init_sq_ownership(qp);
		struct mlx4_qp_context* context = platform_mapAny(sizeof(struct mlx4_qp_context));
		enum mlx4_qp_optpar optpar = 0;
		if (!context)
		{
			DRIVER_LOG(DRIVER_PREFIX "context alloc failed\n");
			return;
		}
		context->flags = bs32((MLX4_QP_STATE_INIT << 28) | (MLX4_QP_ST_RC << 16));
		context->flags |= bs32(MLX4_QP_PM_MIGRATED << 11);

		if (qp->rq.wqe_cnt)
			context->rq_size_stride = ilog2(qp->rq.wqe_cnt) << 3;
		context->rq_size_stride |= qp->rq.wqe_shift - 4;

		if (qp->sq.wqe_cnt)
			context->sq_size_stride = ilog2(qp->sq.wqe_cnt) << 3;
		context->sq_size_stride |= qp->sq.wqe_shift - 4;

		context->sq_size_stride |= !!qp->sq_no_prefetch << 7;
		context->usr_page = bs32(ctx->uar.index);
		context->pri_path.pkey_index = 0;
		optpar |= MLX4_QP_OPTPAR_PKEY_INDEX;
		context->pd	    = bs32(pd->pdn);
		context->params1    = bs32(MLX4_IB_ACK_REQ_FREQ << 28);
		context->cqn_send = bs32(cq->cqn);
		context->params2 |= bs32(MLX4_QP_BIT_RWE);
		optpar |= MLX4_QP_OPTPAR_RWE | MLX4_QP_OPTPAR_RRE | MLX4_QP_OPTPAR_RAE;
		context->cqn_recv = bs32(cq->cqn);
		context->db_rec_addr = bs64(qp->db.dma);

		err = mlx4_qp_modify(dev, &qp->mtt, MLX4_QP_STATE_RST, MLX4_QP_STATE_INIT, context, optpar, 0, &qp->mqp);
		if (err)
		{
			DRIVER_LOG(DRIVER_PREFIX "mlx4_qp_modify failed\n");
			return;
		}
		qp->state = IBV_QPS_INIT;
		qp->atomic_rd_en = IBV_ACCESS_REMOTE_WRITE | IBV_ACCESS_LOCAL_WRITE;
		qp->port = 1;
		update_mcg_macs(dev, qp);
		platform_unmap(context);
		mlx4_cq_clean(qp->recv_cq, qp->qp_num, 0);
		mlx4_init_qp_indices(qp);
		*qp->db.db = 0;
	}
	{	// open connection
		int dest_lid = client ? 3 : 1;
		int dest_qpn;
		int dest_psn;
		unsigned rkey;
		struct mlx4_qp_context* context = platform_mapAny(sizeof(struct mlx4_qp_context));
		
		platform_unmap(context);
	}*/
	DRIVER_LOG(DRIVER_PREFIX "setup qp failed\n");
	// send/receive data
}

