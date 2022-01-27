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

#include "gcode.h"
#include "motion_control.h"
#include "planner.h"
#include "serial_protocol.h"
#include "serial_protocol.h"
#include "spindle_control.h"
#include "stepper.h"
#define LINE_BUFFER_SIZE 50//一个数据包的缓存长度 单位：字节
#define BLOCK_BUFFER_SIZE 20 //存放接下来要执行动作的环形队列,20个动作
// Current global settings (persisted in EEPROM from byte 1 onwards)
typedef struct {
  double steps_per_mm[3];//分别对应x,y,z轴运行1毫米需要多少个脉冲
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
	// uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
	int32_t  step_event_count;          // 每次移动的步数，以下x，y，z轴中最大步数为准
	uint32_t nominal_rate;              // 得到一个值，表示每个动作区块上最长步数/min 单位（步数/min）
	
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
	uint32_t accelerate_until;          // 用来标记在哪一个step上停止加速 单位step
	uint32_t decelerate_after;          // The index of the step event on which to start decelerating
	
} block_t;
typedef struct GRBL_METH{
	// 从串口得到数据包,一般来说Gcode和配置code都是一行字符，所以一个数据包是以\r\n结束
	// 数据指针指向接受缓存，当得到数据之后，复制数据到这个缓存区域
	// maxlen代表的是缓冲区长度
	// 返回-1:没有数据
	// 返回>0:复制之后的数据长度（可能实际数据长度比maxlen大，但是返回maxlen）
	int8_t (*ReadCmd)(uint8_t*,uint8_t maxlen);
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
	void (*DisableTimeInter)(void);//禁止定时器中断
	void (*EnableTimeInter)(void);//使能定时器中断，溢出中断
	double (*SetTimeInterMs)(double timems);//设置定时器中断周期 单位ms，会返回实际中断间隔时间
	// 对于io口需要输出高电平之后再拉低，这个时间应该交给比较捕获中断做，就是溢出中断发生之后，在x个计数周期之后发生溢出中断
	void (*SetTimeCompInterUs)(double timeus);//设置一个在定时器中断发生 timeus 之后的另外一个中断。
	// 打印日志使用
	void (*printPgmString)(uint8_t*);
	void (*printByte)(uint8_t);
	
	uint8_t line[LINE_BUFFER_SIZE];
	uint8_t char_counter;//
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
	uint32_t trapezoid_adjusted_rate;// 该值用于调整每分钟有多少个定时器中断 单位（step/min）
	double ms_per_step_event;        // 两次定时器中断间隔的指令周期，现在更新成两次中断时候的定时器计数周期数
	double trapezoid_tick_ms_counter;//用来计算时间（ms）
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

#define STEPPERS_ENABLE_BIT         0

#define X_LIMIT_BIT          1
#define Y_LIMIT_BIT          2
#define Z_LIMIT_BIT          3

// The temporal resolution of the acceleration management subsystem. Higher number
// give smoother acceleration but may impact performance
// 这个主要控制 加速 减速 的计算频率，值代表每秒中计算多少次
// #define ACCELERATION_TICKS_PER_SECOND 40L
// 代表每隔多久毫秒计算一次加速度，值低就更加加速过程更加平滑，但是也更可能影响性能
#define ACCELERATION_TICKS_MS_PER_MS 25.0
// 这个代表最小定时器中断频率，1200次/分钟
#define MINIMUM_STEPS_PER_MINUTE 1200 // The stepper subsystem will never run slower than this, exept when sleeping

#define clear_vector(a) memset(a, 0, sizeof(a))
#define max(a,b) (((a) > (b)) ? (a) : (b))


void printInteger(int32_t n);
void printFloat(float n);

extern GRBL_METH GrblMeth;

/*-----------------------------------------------------对外函数-----------------------------------------------------*/
extern void TimeInter(GRBL_METH*meth);
extern void TimeInterComp(GRBL_METH*meth);

extern void SpInit(GRBL_METH *meth);
// 数据包输入处理，由这里开始处理代码，放在主循环中
extern void SpProcess(GRBL_METH *meth);
	#ifdef __cplusplus
	}
	#endif
#endif