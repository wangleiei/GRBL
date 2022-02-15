

#ifndef serial_h
#define serial_h
#include "GrblMeth.h"

// 数据包输入处理，由这里开始处理代码，放在主循环中
int8_t SpProcess(GRBL_METH *meth);

#endif
