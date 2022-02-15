
#ifndef stepper_h
#define stepper_h 

#include "GrblMeth.h"

// Initialize and start the stepper motor subsystem
// void st_init();

// Block until all buffered steps are executed
void st_synchronize(GRBL_METH*meth);

// Execute the homing cycle
void st_go_home(GRBL_METH*meth);
					
// 将该函数放入定时器中断中，用于驱动步进电机io，速度增减，动作区块弹出等等，
// 关闭定时器中断也会在这个函数中自动调用
void GrblTimeInter(GRBL_METH*meth);
// 这个是用于将控制步进电机的io电平在一段时间之后，输出低电平用
void GrblTimeInterComp(GRBL_METH*meth);
extern void plan_buffer_line(GRBL_METH*meth,double x, double y, double z, double feed_rate, int32_t invert_feed_rate);
extern void plan_discard_current_block(GRBL_METH*meth);
extern block_t *plan_get_current_block(GRBL_METH*meth);
#endif
