

#ifndef gcode_h
#define gcode_h
#include "GrblMeth.h"

void gc_init(GRBL_METH *meth);
// Execute one block of rs275/ngc/g-code
GC_STA gc_execute_line(GRBL_METH *meth,uint8_t *line);

// 启动grbl模块运行，从解析第一行g代码开始
void GrblStart(GRBL_METH *meth);
// 暂停grbl模块运行
void GrblPause(GRBL_METH *meth);
// 恢复grbl模块运行
void GrblResume(GRBL_METH *meth);

// 停止grbl模块运行，
void GrblStop(GRBL_METH *meth);

extern void plan_buffer_line(GRBL_METH*meth,double x, double y, double z, double feed_rate, int32_t invert_feed_rate);
#endif
