#include "mlx4.h"
#include "mlx4/mlx4.h"
#include "mlx4/cmd.h"
#include "../pcie/pcie.h"
#include <l4io.h>
#include <l4/types.h>
#include "../acpi/platform.h"

// user defines
#define DRIVER_LOG printf
#define DRIVER_PREFIX "MLX4: "
#define DRIVER_MAX_DEVICES 1
#define DRIVER_MAX_CONTEXT 2

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
	memcpy(&mlx4->pcie, &device->pcie, sizeof(struct pcie_device));
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
	if (mlx4_init(mlx4))
	{
		DRIVER_LOG(DRIVER_PREFIX "error\n");
		return 1;
	}
	g_DeviceCount++;
	DRIVER_LOG(DRIVER_PREFIX "success\n");
	return 0;
}
