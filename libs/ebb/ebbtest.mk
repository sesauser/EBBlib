IXPDIR=../libixp-0.5
CFLAGS:=-Wall -I$(IXPDIR)/include -D SESA_ARCH=$(SESA_ARCH) -D SESA_LRT=$(SESA_LRT)
ulnx_libs:=-lpthread -lpcap
LIBS:=$($(SESA_LRT)_libs) -lpcap -L$(IXPDIR)/lib -lixp
#CFLAGS := -O4 
CFLAGS += -g 
SRCS := EBBMgrPrim.c CObjEBBRootShared.c CObjEBB.c \
	EBBEventMgrPrimImp.c \
	EBBCtrPrimDistributed.c CObjEBBRootMulti.c \
	sys/defFT.c EBBMemMgrPrim.c EBBCtrPrim.c EBB9PClientPrim.c \
	EBB9PFilePrim.c P9FSPrim.c CmdMenuPrim.c ebbtest.c \
	EthMgrPrim.c EthEBBProtoPrim.c \
	EBBStart.c \
	sys/arch/$(SESA_ARCH)/defFT.S \
	lrt/$(SESA_LRT)/pic.c lrt/$(SESA_LRT)/ethlib.c \
	lrt/$(SESA_LRT)/lrt_start.c

OBJS := $(patsubst %.c, %.o, $(filter %.c, $(SRCS)))
OBJS += $(patsubst %.S, %.o, $(filter %.S, $(SRCS)))
DEPS := $(patsubst %.c, %.d, $(filter %.c, $(SRCS)))
DEPS += $(patsubst %.S, %.d, $(filter %.S, $(SRCS)))

all: ebbtest pictest

ebbtest: $(OBJS) ebbtest.mk $(IXPDIR)/lib/libixp.a
	gcc $(CFLAGS) $(OBJS) $(LIBS) -o $@

objctest: objctest.m clrBTB.o $(OBJS) sys/arch/amd64/defFT.o
	gcc-mp-4.4 -fgnu-runtime $(CFLAGS) objctest.m clrBTB.o $(OBJS) -lobjc sys/arch/amd64/defFT.o -o $@ 

pictest: lrt/$(SESA_LRT)/pic.c lrt/$(SESA_LRT)/pic.h
	gcc $(CFLAGS) -DPIC_TEST lrt/$(SESA_LRT)/pic.c -o pictest -lpthread

$(IXPDIR)/lib/libixp.a:
	make -C $(IXPDIR)

clrBTB.o: jmps.S clrBTB.S
	gcc -c clrBTB.S

clrBTB.S: jmps.S
	touch clrBTB.S

jmps.S:
	./mkjmps 1024 > jmps.S

-include $(DEPS)

%.o : %.c ebbtest.mk
	gcc $(CFLAGS) -MMD -MP -c $< -o $@

%.o: %.S ebbtest.mk
	gcc $(CFLAGS) -MMD -MP -c $< -o $@

EBBObj.o: EBBObj.H EBBObj.c
	g++ $(CFLAGS) -c EBBObj.c -o EBBObj.o

clean:
	-rm $(wildcard $(OBJS) $(DEPS) ebbtest pictest)
	make -C $(IXPDIR) clean