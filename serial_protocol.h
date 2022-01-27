

#ifndef serial_h
#define serial_h
#include "GrblMeth.h"

void SpInit(GRBL_METH *meth);
// 数据包输入处理，由这里开始处理代码，放在主循环中
void SpProcess(GRBL_METH *meth);

#endif
