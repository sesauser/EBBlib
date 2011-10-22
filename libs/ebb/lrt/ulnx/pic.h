#ifndef __LRT_ULNX_PIC_H__
#define __LRT_ULNX_PIC_H__

typedef void (*lrt_pic_handler)(void);
typedef uval lrt_pic_src;

extern uval lrt_pic_firstvec(void);
extern uval lrt_pic_numvec(void);
extern sval lrt_pic_init(void);
extern sval lrt_pic_loop(void);
extern sval lrt_pic_allocvec(uval *vec);
extern sval lrt_pic_mapvec(lrt_pic_src src, uval vec, lrt_pic_handler h);

#endif
