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
#define BLOCK_BUFFER_SIZE 20 //存放接下来要执行动作的环形队列,20个动作
// Current global settings (persisted in EEPROM from byte 1 onwards)
typedef struct {
  double steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  double default_feed_rate;
  double default_seek_rate;
  uint8_t invert_mask;
  double mm_per_arc_segment;
  double acceleration;
  double max_jerk;
} settings_t;

typedef struct {
	uint8_t status_code;

	uint8_t motion_mode;             /* {G0, G1, G2, G3, G80} */
	uint8_t inverse_feed_rate_mode;  /* G93, G94 */
	uint8_t inches_mode;             /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
	uint8_t absolute_mode;           /* 0 = relative motion, 1 = absolute motion {G90, G91} */
	uint8_t program_flow;
	int32_t spindle_direction;
	double feed_rate, seek_rate;     /* Millimeters/second */
	double position[3];              /* Where the interpreter considers the tool to be at this point in the code */
	uint8_t tool;
	int16_t spindle_speed;           /* RPM/100 */
	uint8_t plane_axis_0; 
	uint8_t plane_axis_1; 
	uint8_t plane_axis_2;            // The axes of the selected plane  
} parser_state_t;

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
	// Fields used by the bresenham algorithm for tracing the line
	uint32_t steps_x, steps_y, steps_z; // Step count along each axis
	uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
	int32_t  step_event_count;          // 每次移动的步数，以下x，y，z轴中最大步数为准
	uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute
	
	// Fields used by the motion planner to manage acceleration
	double speed_x, speed_y, speed_z;   // Nominal mm/minute for each axis
	double nominal_speed;               // 该动作区块执行动作时候的额定速度
	double millimeters;                 // The total travel of this block in mm
	double entry_factor;                // The factor representing the change in speed at the start of this trapezoid.
																			// (The end of the curren speed trapezoid is defined by the entry_factor of the
																			// next block)
	
	// Settings for the trapezoid generator
	uint32_t initial_rate;              // The jerk-adjusted step rate at start of block  
	uint32_t final_rate;                // The minimal rate at exit
	int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
	uint32_t accelerate_until;          // The index of the step event on which to stop acceleration
	uint32_t decelerate_after;          // The index of the step event on which to start decelerating
	
} block_t;
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
	// 关键参数设置
	settings_t settings;
	parser_state_t gc;
	// 用于存放动作队列
	block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions,是一个存放接下来要执行动作的环形队列
	volatile int32_t block_buffer_head;           // Index of the next block to be pushed
	volatile int32_t block_buffer_tail;           // Index of the block to process now
	// The current position of the tool in absolute steps
	// 应该是当前坐标距离0点的步数
	int32_t position[3];
	uint8_t acceleration_manager_enabled;   //用来标记轴是否需要加速运动

	// stepper.c中实现算法所用的中间变量
	// Counter variables for the bresenham line tracer
	int32_t st_counter_x;
	int32_t st_counter_y;
	int32_t st_counter_z;
	uint32_t step_events_completed; // 在一个动作区块中已经执行完了的步数
	block_t *current_block;  // 当前运行中的动作区块指针
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


void printInteger(int32_t n);
void printFloat(float n);

extern GRBL_METH GrblMeth;




#ifdef __cplusplus
}
#endif

#endif