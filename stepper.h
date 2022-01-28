
#ifndef stepper_h
#define stepper_h 

#include "GrblMeth.h"

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize(GRBL_METH*meth);

// Execute the homing cycle
void st_go_home(GRBL_METH*meth);
					
// ���ú������붨ʱ���ж��У������������io���ٶ��������������鵯���ȵȣ�
// �رն�ʱ���ж�Ҳ��������������Զ�����
void TimeInter(GRBL_METH*meth);
// extern void plan_discard_current_block();
void TimeInterComp(GRBL_METH*meth);
#endif
