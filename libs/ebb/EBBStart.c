#include "../base/include.h"
#include "../base/types.h"
#include "../base/lrtio.h"
#include __LRTINC(pic.h)
#include __LRTINC(EBBAssert.h)


void
EBBStart(void)
{
  /* Three main EBB's are EBBMgrPrim, EBBEventMgrPrim EBBMemMgrPrim    */
  /* There creation and initialization are interdependent and requires */
  /* fancy footwork */

  // put code here to get preboot versions of core EBBs ready

  // then invoke a method of BootInfo object on first message
  // this object should gather boot information (sysfacts and boot args)
  // and then get full blown primitive core EBBS up (perhaps by a hot swap)

  EBB_LRT_printf("%s: ADD RESET OF INIT CODE HERE!\n", __func__);
  LRT_EBBAssert(0);

  // Righ now this should be the equivalent to:
  // 
  // EBBMgrPrimInit();
  // EBBMemMgrPrimInit();
  // EBBEventMgrPrimImpInit();

}
