
#ifndef spindle_control_h
#define spindle_control_h 
#include "GrblMeth.h"

void spindle_init();
void spindle_run(int32_t direction, uint32_t rpm);
void spindle_stop();

#endif
