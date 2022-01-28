
#ifndef stepper_h
#define stepper_h 

#include "GrblMeth.h"

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize(GRBL_METH*meth);

// Execute the homing cycle
void st_go_home(GRBL_METH*meth);
					
// 将该函数放入定时器中断中，驱动步进电机io，速度增减，动作区块弹出等等，
// 关闭定时器中断也会在这个函数中自动调用
void TimeInter(GRBL_METH*meth);
// extern void plan_discard_current_block();
void TimeInterComp(GRBL_METH*meth);
#endif
