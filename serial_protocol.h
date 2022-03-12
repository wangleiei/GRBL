

#ifndef serial_h
#define serial_h
#include "GrblMeth.h"

// 数据包输入处理，由这里开始处理代码，放在主循环中
int8_t SpProcess(GRBL_METH *meth);


// 用于判断grbl的动作是否执行完成
// 2:没有执行动作(都没有开始)
// 1:执行完成
// 0：已经开始执行,但是未完成
uint8_t GrblActionComplete(GRBL_METH*meth);
#endif
