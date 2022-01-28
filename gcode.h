

#ifndef gcode_h
#define gcode_h
#include "GrblMeth.h"

#define GCSTATUS_OK 0
#define GCSTATUS_BAD_NUMBER_FORMAT 1
#define GCSTATUS_EXPECTED_COMMAND_LETTER 2
#define GCSTATUS_UNSUPPORTED_STATEMENT 3
#define GCSTATUS_FLOATING_POINT_ERROR 4

void gc_init(GRBL_METH *meth);
// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(GRBL_METH *meth,uint8_t *line);

#endif
