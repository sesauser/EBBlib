#ifndef __EBB_TRANS_H__
#define __EBB_TRANS_H__
/******************************************************************************
* Copyright (C) 2011 by Project SESA, Boston University
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*****************************************************************************/

#include "../../base/types.h"
#include "../EBBTypes.h"
#include "../EBBConst.h"

//FIXME: All Trans Mem is statically allocated
extern struct EBB_Trans_Mem {
  uval8 GMem [EBB_TRANS_PAGE_SIZE * EBB_TRANS_NUM_PAGES];
  uval8 LMem [EBB_TRANS_PAGE_SIZE * EBB_TRANS_NUM_PAGES *
	      EBB_TRANS_MAX_ELS];
  uval8 *free;  // pointer to next available range of GMem
} EBB_Trans_Mem;

static void EBB_Trans_Mem_Init(void) {
  EBB_Trans_Mem.free = EBB_Trans_Mem.GMem;
}  
// could also do initial mapping here if memory is
// is not actually static reservation

static void EBB_Trans_Mem_Alloc_Pages(uval num_pages, uval8 **pages) {
  if (&(EBB_Trans_Mem.free[EBB_TRANS_PAGE_SIZE * num_pages]) >
      &(EBB_Trans_Mem.GMem[EBB_TRANS_PAGE_SIZE * EBB_TRANS_NUM_PAGES])) {
    *pages = NULL;
  } else {
    *pages = EBB_Trans_Mem.free;
    EBB_Trans_Mem.free = EBB_Trans_Mem.free + (EBB_TRANS_PAGE_SIZE * num_pages);
  }
}

typedef EBBRC (*EBBFunc) (void *);
typedef EBBFunc *EBBFuncTable;

extern EBBFunc EBBDefFT[EBB_TRANS_MAX_FUNCS];
/* extern EBBFunc EBBNullFT[EBB_TRANS_MAX_FUNCS]; */

extern EBBMissFunc theERRMF;

struct EBBTransStruct {
  union {
    uval v1;
    EBBFuncTable *obj; //as a local entry
    EBBMissFunc mf; //as a global entry
  };
  union {
    uval v2;
    EBBFuncTable ftable; //as a local entry (by default)
    EBBMissArg arg; //as a global entry
  };
  //FIXME: used for a free list, probably should be separate
  union {
    uval v3;
    EBBGTrans *next; 
  };
  union {
    uval v4; //FIXME: padding - there was a bug without this
    EBBMissFunc globalMF;
  };
};

static inline EBBId EBBLTransToId(EBBLTrans *lt) {
  return (EBBId)((uval)lt - EBBMyLTransIndex() *
		 EBB_TRANS_PAGE_SIZE * EBB_TRANS_NUM_PAGES);
}

static inline EBBGTrans * EBBIdToGTrans(EBBId id) {
  return (EBBGTrans *)((uval)id - (uval)EBB_Trans_Mem.LMem +
		       (uval)EBB_Trans_Mem.GMem);
}

static inline EBBId EBBGTransToId(EBBGTrans *gt) {
  return (EBBId)((uval)gt - (uval)EBB_Trans_Mem.GMem +
		 (uval)EBB_Trans_Mem.LMem);
}

static inline EBBGTrans * EBBLTransToGTrans(EBBLTrans *lt) {
  return EBBIdToGTrans(EBBLTransToId(lt));
}

static inline EBBLTrans * EBBGTransToLTrans(EBBGTrans *gt) {
  return EBBIdToLTrans(EBBGTransToId(gt));
}

/* struct EBBTransLSysStruct { */
/*   EBBGTrans *gTable; */
/*   EBBLTrans *lTable; */
/*   EBBGTrans *free; */
/*   uval numAllocated; */
/*   uval size; //number of EBBGTrans in our portion of the gTable */
/* }; */

/* static inline EBBId EBBIdAlloc(EBBTransLSys *sys) { */
/*   EBBGTrans *ret = sys->free; */
/*   if(ret != NULL) { */
/*     ret->next = (EBBGTrans *)-1; */
/*     sys->free = sys->free->next; */
/*     return EBBGTransToId(ret); */
/*   } */
/*   int i; */
/*   for (i = 0; i < sys->size; i++) { */
/*     if((uval)sys->gTable[i].next != -1) { */
/*       sys->gTable[i].next = (EBBGTrans *)-1; */
/*       return EBBGTransToId(&sys->gTable[i]); */
/*     } */
/*   } */
/*   return NULL; */
/* } */
struct EBBTransLSysStruct {
  EBBGTrans *localGTable; //our portion of the gtable for allocating local ebbs
  EBBLTrans *localLTable;
  EBBGTrans *localFree;
  uval localNumAllocated;
  uval localSize;
  EBBGTrans *globalGTable; //our portion of the gtable for allocating global ebbs
  EBBLTrans *globalLTable;
  EBBGTrans *globalFree;
  uval globalNumAllocated;
  uval globalSize;
};

static inline uval isGlobalSetup(EBBTransLSys *sys) {
  return !!sys->globalGTable;
}

static inline void SetupGlobal(EBBTransLSys *sys, uval nodeId) {
  uval numGTransPerEL;
  numGTransPerEL = EBB_TRANS_NUM_PAGES * EBB_TRANS_PAGE_SIZE / 
    sizeof(EBBGTrans) / EBB_TRANS_MAX_ELS / EBB_TRANS_MAX_NODES;
  
  //FIXME: assuming gsys.pages = EBB_TRANS_NUM_PAGES
  //I can't possibly get this right
  sys->globalGTable = (EBBGTrans *)
    &EBB_Trans_Mem.GMem[nodeId * 
		       EBB_TRANS_PAGE_SIZE * 
		       EBB_TRANS_NUM_PAGES / EBB_TRANS_MAX_NODES +
		       numGTransPerEL * EBBMyEL() * sizeof(EBBGTrans)];
  sys->globalLTable = EBBGTransToLTrans(sys->globalGTable);
  sys->globalFree = NULL;
  sys->globalNumAllocated = 0;
  sys->globalSize = numGTransPerEL;
}
  
    
static inline EBBId EBBIdAllocGlobal(EBBTransLSys *sys) {
  EBBGTrans *ret = sys->globalFree;
  if(ret != NULL) {
    ret->next = (EBBGTrans *)-1;
    sys->globalFree = sys->globalFree->next;
    sys->globalNumAllocated++;
    return EBBGTransToId(ret);
  }
  int i;
  for (i = 0; i < sys->globalSize; i++) {
    if((uval)sys->globalGTable[i].next != -1) {
      sys->globalGTable[i].next = (EBBGTrans *)-1;
      sys->globalNumAllocated++;
      return EBBGTransToId(&sys->globalGTable[i]);
    }
  }
  return NULL;
}    

static inline EBBId EBBIdAllocLocal(EBBTransLSys *sys) {
  EBBGTrans *ret = sys->localFree;
  if(ret != NULL) {
    ret->next = (EBBGTrans *)-1;
    sys->localFree = sys->localFree->next;
    sys->localNumAllocated++;
    return EBBGTransToId(ret);
  }
  int i;
  for (i = 0; i < sys->localSize; i++) {
    if((uval)sys->localGTable[i].next != -1) {
      sys->localGTable[i].next = (EBBGTrans *)-1;
      sys->localNumAllocated++;
      return EBBGTransToId(&sys->localGTable[i]);
    }
  }
  return NULL;
}    

static inline uval getLTransNodeId(EBBLTrans *lt) {
  uval val = (((uval)lt) - ((uval)EBB_Trans_Mem.LMem)) /
    (EBB_TRANS_PAGE_SIZE * EBB_TRANS_NUM_PAGES / EBB_TRANS_MAX_NODES);
  return val;
}

//FIXME: assuming gsys.pages = EBB_TRANS_NUM_PAGES
//this is probably wrong with some +/- 1 issue
static inline uval isLocalEBB(EBBGTrans *gt) {
  return ((void *) gt < (void *)
	  &EBB_Trans_Mem.GMem[EBB_TRANS_PAGE_SIZE * EBB_TRANS_NUM_PAGES /
			      EBB_TRANS_MAX_NODES]);
}



static inline void EBBIdFree(EBBTransLSys *sys, EBBId id) {
  EBBGTrans *free = EBBIdToGTrans(id);
  if(isLocalEBB(free)) {
    free->next = sys->localFree;
    sys->localFree = free;
  } else {
    free->next = sys->globalFree;
    sys->globalFree = free;
  }
}

static inline void EBBSetLTrans(EBBLTrans *lt,
				EBBFuncTable ftable) {
  EBBCacheObj(lt, &lt->ftable);
  lt->ftable = ftable;
}

static inline void EBBSetAllLTrans(EBBId id, EBBFuncTable ftable) {
  int i;
  EBBLTrans *lt;
  for (i = 0; i < EBB_TRANS_MAX_ELS; i++) {
    lt = EBBIdToSpecificLTrans(id, i);
    EBBSetLTrans(lt, ftable);
  }
}

static inline void EBBIdBind(EBBId id, EBBMissFunc mf, 
			     EBBMissArg arg) {
  EBBGTrans *gt = EBBIdToGTrans(id);
  gt->mf = mf;
  gt->arg = arg;
}

static inline void EBBIdBindGlobal(EBBId id, EBBMissFunc mf,
				   EBBMissArg arg, EBBMissFunc globalMF) {
  EBBGTrans *gt = EBBIdToGTrans(id);
  gt->mf = mf;
  gt->arg = arg;
  gt->globalMF = globalMF;
}

static inline void EBBIdUnBind(EBBId id, EBBMissFunc *mf,
			       EBBMissArg *arg) {
  EBBGTrans *gt = EBBIdToGTrans(id);
  if (mf)
    *mf = gt->mf;
  if (arg)
    *arg = gt->arg;
  //FIXME: this is how we reset the local tables after an unbind
  EBBSetAllLTrans(id, EBBDefFT);
  EBBIdBind(id, theERRMF, 0);
}

//initialize the portion of ltable from lt
// to the specified number of pages
static void initLTable(EBBLTrans *lt, uval pages) {
  EBBLTrans *iter;
  for (iter = lt; 
       ((void *)iter) < (void *)((&((char *)lt)[EBB_TRANS_PAGE_SIZE * pages]));
       iter++) {
    EBBSetLTrans(iter, EBBDefFT);
  }
}

//init all ltables from lt to the specified number of pages
//id must be the first id allocated
static void initAllLTables(EBBId id, uval pages) {
  int i;
  for (i = 0; i < EBB_TRANS_MAX_ELS; i++) {
    initLTable(EBBIdToSpecificLTrans(id, i), pages);
  }
}

// FIXME: JA think there is a bug here.  We should be calcing NUM explicity
static void initGTable(EBBGTrans *gt, uval pages) {
  EBBGTrans *iter;
  for (iter = gt;
       iter < (EBBGTrans *)((uval)gt + pages * EBB_TRANS_PAGE_SIZE);
       iter++) {
    EBBIdBind(EBBGTransToId(iter), theERRMF, 0);
  }
}
  
#endif
