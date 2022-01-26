#ifndef GRBL_H
#define GRBL_H

#ifdef __cplusplus
   extern "C"{ //在最顶层调用h文件使用，
#endif 

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

typedef struct GRBL_METH{
	int8_t (*ReadChar)(void);//从串口得到一个字节的数据,没有数据时候返回-1
	void (*XAxisPwmL)(void);
	void (*XAxisPwmH)(void);
	void (*XAxisDir_L)(void);
	void (*XAxisDir_H)(void);
	void (*YAxisPwmL)(void);
	void (*YAxisPwmH)(void);
	void (*YAxisDir_L)(void);
	void (*YAxisDir_H)(void);
	void (*ZAxisPwmL)(void);
	void (*ZAxisPwmH)(void);
	void (*ZAxisDir_L)(void);
	void (*ZAxisDir_H)(void);
	// 打印日志使用
	void (*printPgmString)(uint8_t*);
	void (*printByte)(uint8_t);
}GRBL_METH;

#include "stepper.h"
#include "serial_protocol.h"
#include "spindle_control.h"
#include "settings.h"
#include "gcode.h"
#include "motion_control.h"
#include "planner.h"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define M_PI 3.14159

// Updated default pin-assignments from 0.6 onwards 
// (see bottom of file for a copy of the old config)

// #define STEPPERS_ENABLE_DDR     DDRB
// #define STEPPERS_ENABLE_PORT    PORTB
#define STEPPERS_ENABLE_BIT         0

// #define STEPPING_DDR       DDRD
// #define STEPPING_PORT      PORTD
#define X_STEP_BIT           2
#define Y_STEP_BIT           3
#define Z_STEP_BIT           4
#define X_DIRECTION_BIT      5
#define Y_DIRECTION_BIT      6
#define Z_DIRECTION_BIT      7

// #define LIMIT_DDR      DDRB
// #define LIMIT_PORT     PORTB
#define X_LIMIT_BIT          1
#define Y_LIMIT_BIT          2
#define Z_LIMIT_BIT          3

// #define SPINDLE_ENABLE_DDR DDRB
// #define SPINDLE_ENABLE_PORT PORTB
#define SPINDLE_ENABLE_BIT 4

// #define SPINDLE_DIRECTION_DDR DDRB
// #define SPINDLE_DIRECTION_PORT PORTB
#define SPINDLE_DIRECTION_BIT 5

// The temporal resolution of the acceleration management subsystem. Higher number
// give smoother acceleration but may impact performance
#define ACCELERATION_TICKS_PER_SECOND 40L

#define clear_vector(a) memset(a, 0, sizeof(a))
#define max(a,b) (((a) > (b)) ? (a) : (b))


void printInteger(int n);
void printFloat(float n);

extern GRBL_METH GrblMeth;




#ifdef __cplusplus
}
#endif

#endif