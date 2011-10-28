/* Copyright 2011 Boston University. All rights reserved. */

/* Redistribution and use in source and binary forms, with or without modification, are */
/* permitted provided that the following conditions are met: */

/*    1. Redistributions of source code must retain the above copyright notice, this list of */
/*       conditions and the following disclaimer. */

/*    2. Redistributions in binary form must reproduce the above copyright notice, this list */
/*       of conditions and the following disclaimer in the documentation and/or other materials */
/*       provided with the distribution. */

/* THIS SOFTWARE IS PROVIDED BY BOSTON UNIVERSITY ``AS IS'' AND ANY EXPRESS OR IMPLIED */
/* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND */
/* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BOSTON UNIVERSITY OR */
/* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR */
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR */
/* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF */
/* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/* The views and conclusions contained in the software and documentation are those of the */
/* authors and should not be interpreted as representing official policies, either expressed */
/* or implied, of Boston University */


#include <l4io.h>
#include <l4/kdebug.h>
#include <l4/kip.h>
#include <l4/ipc.h>

#include <verbs.h>

static inline void* memset(void* dst, int src, unsigned int len)
{
	unsigned char *d = (unsigned char *) dst;
	while (len-- > 0)
		*d++ = src;
	return dst;
}

#define BUFFERSIZE  (1 << 23)

char buffer[BUFFERSIZE] __attribute__ ((aligned (1 << 12)));	// page aligned buffer

extern "C" void mlx4_test(struct ibv_pd* ibvpd, int client);

int main_method(void)
{
	printf("Infiniband Test application");
	int dev_num;
	struct ibv_device** dev_list = ibv_get_device_list(&dev_num);
	if (dev_list && dev_list[0])
	{
		struct ibv_context* context = ibv_open_device(dev_list[0]);
		ibv_free_device_list(dev_list);
		if (context!=0)
		{
			int choice = 0;
			do
			{
				printf("(C)lient or (S)erver:\n");
				choice = getc();
				if (choice=='C') choice = 'c';
				if (choice=='S') choice = 's';
			}
			while (choice!='c' && choice!='s');
			int size = 1024;
			// setup ib qp
			struct ibv_comp_channel *channel = 0;
			struct ibv_pd      *pd;
			struct ibv_mr      *mr;
			struct ibv_cq      *cq;
			struct ibv_qp      *qp;
			void               *buf;
			buf = &buffer;
			pd = ibv_alloc_pd(context);
			int tx_depth = 300;
			int rx_depth = 300;
			if (!pd)
			{
				printf("ibv_alloc_pd failed.\n");
				return 1;
			}
			mlx4_test(pd, choice=='c');
			return 0;
			mr = ibv_reg_mr(pd, buf, size * 2, IBV_ACCESS_REMOTE_WRITE | IBV_ACCESS_LOCAL_WRITE);
			if (!mr)
			{
				printf("ibv_reg_mr failed.\n");
				return 1;
			}
			// if use events
			//channel = ibv_create_comp_channel(ctx->context);
			cq = ibv_create_cq(context, rx_depth, 0, channel, 0);
			if (!cq)
			{
				printf("ibv_create_cq failed.\n");
				return 1;
			}
			{
				struct ibv_qp_init_attr attr;
				memset(&attr, 0, sizeof(struct ibv_qp_init_attr));
				attr.send_cq = cq;
				attr.recv_cq = cq; 
				attr.cap.max_send_wr  = tx_depth;
				/* Work around:  driver doesnt support
				 * recv_wr = 0 */
				attr.cap.max_recv_wr  = rx_depth;
				attr.cap.max_send_sge = 1;
				attr.cap.max_recv_sge = 1;
				attr.cap.max_inline_data = 400;//
				attr.qp_type = IBV_QPT_RC;
				/*attr.sq_sig_all = 0;*/
				qp = ibv_create_qp(pd, &attr);
				if (!qp)
				{
					printf("ibv_create_qp failed.\n");
					return 1;
				}
			}
			{
				struct ibv_qp_attr attr;
				attr.qp_state        = IBV_QPS_INIT;
				attr.pkey_index      = 0;
				attr.port_num        = 1;//
				attr.qp_access_flags = IBV_ACCESS_REMOTE_WRITE | IBV_ACCESS_LOCAL_WRITE;
				if (ibv_modify_qp(qp, &attr,
							 IBV_QP_STATE              |
							 IBV_QP_PKEY_INDEX         |
							 IBV_QP_PORT               |
							 IBV_QP_ACCESS_FLAGS)) {
				}
			}
		// connect
/*			{
				int dest_lid;
				int dest_qpn;
				int dest_psn;
				unsigned rkey;
				union ibv_gid       dest_dgid;

				struct ibv_qp_attr attr;
				memset(&attr, 0, sizeof attr);
				attr.qp_state 		= IBV_QPS_RTR;
				attr.path_mtu               = IBV_MTU_2048;
				attr.dest_qp_num 	= dest_qpn;
				attr.rq_psn 		= dest_psn;
				if (user_parm->connection_type == RC) {
					attr.max_dest_rd_atomic     = 1;
					attr.min_rnr_timer          = 12;
				}
				if (user_parm->gid_index < 0) {
					attr.ah_attr.is_global  = 0;
					attr.ah_attr.dlid       = dest_lid;
					attr.ah_attr.sl         = sl;
				} else {
					attr.ah_attr.is_global  = 1;
					attr.ah_attr.grh.dgid   = dest_dgid;
					attr.ah_attr.grh.hop_limit = 1;
					attr.ah_attr.sl         = 0;
				}
				attr.ah_attr.src_path_bits = 0;
				attr.ah_attr.port_num   = port;
				if (ibv_modify_qp(qp, &attr,
						  IBV_QP_STATE              |
						  IBV_QP_AV                 |
						  IBV_QP_PATH_MTU           |
						  IBV_QP_DEST_QPN           |
						  IBV_QP_RQ_PSN             |
						  IBV_QP_MIN_RNR_TIMER      |
						  IBV_QP_MAX_DEST_RD_ATOMIC)) {
					printf("Failed to modify RC QP to RTR\n");
					return 1;
				}
				attr.timeout            = user_parm->qp_timeout;
				attr.retry_cnt          = 7;
				attr.rnr_retry          = 7;
				attr.qp_state 	    = IBV_QPS_RTS;
				attr.sq_psn 	    = my_psn;
				attr.max_rd_atomic  = 1;
				attr.max_rd_atomic  = 1;
				if (ibv_modify_qp(qp, &attr,
						  IBV_QP_STATE              |
						  IBV_QP_SQ_PSN             |
						  IBV_QP_TIMEOUT            |
						  IBV_QP_RETRY_CNT          |
						  IBV_QP_RNR_RETRY          |
						  IBV_QP_MAX_QP_RD_ATOMIC)) {
					printf("Failed to modify RC QP to RTS\n");
					return 1;
				}
				// post recieve max msg size
				{
					int i;
					struct ibv_recv_wr      *bad_wr_recv;
					//recieve
					rwr.wr_id      = PINGPONG_RECV_WRID;
					rwr.sg_list    = &recv_list;
					rwr.num_sge    = 1;
					rwr.next       = 0;
					recv_list.addr = (uintptr_t) buf;
					recv_list.length = size;
					recv_list.lkey = mr->lkey;
					for (i = 0; i < rx_depth; ++i)
						if (ibv_post_recv(qp, &rwr, &bad_wr_recv)) {
							printf("Couldn't post recv: counter=%d\n", i);
							return 14;
						}
				}
				post_recv = rx_depth;
			}*/
			int size_max_pow = 10;
			// send
			int i;
			for (i = 1; i < size_max_pow ; ++i)
			{
				size = 1 << i;
/*				if (user_param.duplex) {
					if(run_iter_bi(ctx, &user_param, rem_dest, size))
						return 17;
				} else {
					if(run_iter_uni(ctx, &user_param, rem_dest, size))
						return 17;
				}
				if (user_param.servername) {
					print_report(user_param.iters, size, user_param.duplex, tposted, tcompleted, noPeak, no_cpu_freq_fail);
					// sync again for the sake of UC/UC
					rem_dest = pp_client_exch_dest(sockfd, &my_dest, &user_param);
				} else
					rem_dest = pp_server_exch_dest(sockfd, &my_dest, &user_param);*/
			}

		
			if (choice == 'c')
			{	// client mode
				printf("start sending data...\n");
			} else
			{	// server mode
				printf("listening for data\n");
			}
			ibv_close_device(context);
		} else
		{
			printf("Couldn't open context\n");
		}
	} else
	{
		printf("Failed to get IB devices list\n");
	}
	return 0;
}

int main_thread(void)
{
	main_method();
	for (;;)
		L4_KDB_Enter("main_thread_EOW");
	return 0;
}

static char ib_stack[8*1024] __attribute__((aligned(64)));
static char main_stack[4*1024] __attribute__((aligned(64)));

extern "C" void ib_thread();
extern "C" L4_ThreadId_t ib_threadId;

int main(void)
{
	// Get kernel interface page.
	L4_KernelInterfacePage_t* kip = (L4_KernelInterfacePage_t *)L4_KernelInterface(0,0,0);
	L4_Word_t utcbsize = L4_UtcbSize(kip);
	// setup threads
	ib_threadId = L4_GlobalId(L4_ThreadIdUserBase(kip) + 8, 1);
	L4_ThreadId_t main_threadId = L4_GlobalId(L4_ThreadIdUserBase(kip) + 9, 1);
	// do the ThreadControl call
	if (!L4_ThreadControl(ib_threadId,	// new thread id
	                      L4_Myself(),	// address space
	                      L4_Myself(),	// scheduler
	                      L4_Myself(),	// pager
	                      ((void*)(((L4_Word_t)L4_MyLocalId().raw + utcbsize * (8)) & ~(utcbsize - 1)))))
	{
		printf("couldn't start IB thread\n");
		return 1;
	}
	if (!L4_ThreadControl(main_threadId,	// new thread id
	                      L4_Myself(),	// address space
	                      L4_Myself(),	// scheduler
	                      L4_Myself(),	// pager
	                      ((void*)(((L4_Word_t)L4_MyLocalId().raw + utcbsize * (9)) & ~(utcbsize - 1)))))
	{
		printf("couldn't start main thread\n");
		return 1;
	}
	// start threads
	L4_Word_t ip = (L4_Word_t)&ib_thread;
	L4_Word_t sp = (L4_Word_t)(&ib_stack[sizeof(ib_stack) - 1]);
	L4_Start_SpIp(ib_threadId, sp, ip);
	ip = (L4_Word_t)&main_thread;
	sp = (L4_Word_t)(&main_stack[sizeof(main_stack) - 1]);
	L4_Start_SpIp(main_threadId, sp, ip);
{	// pager loop
	L4_ThreadId_t tid;
	L4_MsgTag_t tag;
	L4_Msg_t msg;
 
	while(1)
	{
		tag = L4_Wait_Timeout(L4_TimePeriod(100*1000), &tid);
		
		while(1)
		{
			L4_MsgStore(tag, &msg);
		
#if 0
			{
			printf ("Root-Pager got msg from %p (%p, %p, %p)\n",
				(void *) tid.raw, (void *) tag.raw,
				(void *) L4_MsgWord (&msg, 0), (void *) L4_MsgWord (&msg, 1));
			}
#endif
			if (L4_UntypedWords (tag) != 2 || L4_TypedWords (tag) != 0 ||
				!L4_IpcSucceeded (tag))
			{
				if (L4_IpcSucceeded (tag))
				{
					printf("malformed pagefault IPC from %p (tag=%p)\n",
						   (void *) tid.raw, (void *) tag.raw);
					printf("malformed pagefault in root\n");
				}
				break;
			}
			L4_Word_t faddr = L4_MsgWord (&msg, 0);
			/* L4_Word_t fip   = L4_Get (&msg, 1); */
		
			/* This is really ugly, we just touch this address to bring 
			   the page into our address space */
			volatile char* dummy = (char*)faddr;
			*dummy;
		
			/* Send mapitem, note that this is a nop between threads in the 
			   the same address space */
			L4_MsgClear(&msg);
			L4_Fpage_t p = L4_FpageLog2(faddr & ~0xFFF, 12);
			L4_Set_Rights(&p, L4_FullyAccessible);
			L4_MsgAppendMapItem(&msg, L4_MapItem(p, faddr));
			L4_MsgLoad(&msg);
		
			L4_ThreadId_t nextid = L4_nilthread;
			tag = L4_ReplyWait_Timeout(tid, L4_TimePeriod(100*1000), &nextid);
			tid = nextid;
		}
	}
}
	for (;;)
		L4_KDB_Enter("EOW");
	return 0;
}
