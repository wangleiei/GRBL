

#ifndef settings_h
#define settings_h

#include "GrblMeth.h"

#define GRBL_VERSION "0.6b"

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 2

extern Settings settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init(GRBL_METH *meth);

// 打印grbl相关参数
void GrblPrintSettings(GRBL_METH *meth);

// A helper method to set new settings from command line
void settings_store_setting(GRBL_METH *meth,uint8_t parameter, double value);

// Default settings (used when resetting eeprom-settings)
// #define MICROSTEPS 8
// #define DEFAULT_X_STEPS_PER_MM (94.488188976378*MICROSTEPS)
// #define DEFAULT_Y_STEPS_PER_MM (94.488188976378*MICROSTEPS)
// #define DEFAULT_Z_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define DEFAULT_X_STEPS_PER_MM 5
#define DEFAULT_Y_STEPS_PER_MM 5
#define DEFAULT_Z_STEPS_PER_MM 5
#define DEFAULT_STEP_PULSE_MICROSECONDS 30 //微秒
// 这个是用来画圆形时候用，
// 会将圆形转换成一小截的线段，这个线段的长度就是 DEFAULT_MM_PER_ARC_SEGMENT
#define DEFAULT_MM_PER_ARC_SEGMENT 0.1
// #define DEFAULT_SEEK_FEEDRATE 480.0 //寻线速度，毫米/分钟
// #define DEFAULT_FEEDRATE 480.0
#define DEFAULT_SEEK_FEEDRATE 120//寻线速度，毫米/分钟
#define DEFAULT_FEEDRATE 60 //切割速度，毫米/分钟
#define DEFAULT_ACCELERATION (DEFAULT_FEEDRATE/100.0)
#define DEFAULT_MAX_JERK 50.0
#define DEFAULT_STEPPING_INVERT_MASK 0

extern GRBL_METH GrblMeth;

#endif
