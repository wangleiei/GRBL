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

#define LINE_BUFFER_SIZE 50//一个数据包的缓存长度 单位：字节
#define BLOCK_BUFFER_SIZE 20 //存放接下来要执行动作的环形队列,20个动作

typedef enum {LaserCutMode = 1,CNCMode }GRBL_MODE;
typedef enum {GC_Ok = 0,
			GC_BadNumberFormat,
			GC_ExpectedCommandLetter,
			GC_UnsupportedStatement,
			GC_FloatingPointError,
			GC_CodeStart,//找到开头的'%'
			GC_CodeEnd,
			GC_CodeEndWarn,//g代码结尾符号 '%'过多，文件应该只有两个一个开头，一个结尾
			GC_TOOBUSY,//grbl安排一个动作时候，发生占用动作区块太多，以至于用完所有的动作区块都无法安排好这个动作，需要在

			}GC_STA;

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
  uint8_t SettingsSum;//用于存储过程中校验使用
} Settings;

typedef struct {
	GC_STA status_code;

	uint8_t motion_mode;             /* {G0, G1, G2, G3, G80} */
	uint8_t inverse_feed_rate_mode;  /* G93, G94 */
	uint8_t inches_mode;             /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
	uint8_t absolute_mode;           /* 0 = relative motion, 1 = absolute motion {G90, G91} */
	uint8_t program_flow;
	int32_t spindle_direction;
	uint8_t laserpower;//激光输出功率0-100%
	double feed_rate, seek_rate;     /* 毫米/秒 */
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
	uint32_t nominal_rate;              // 得到一个值，表示每个动作区块上最长步数/一分钟 单位（步数/min）
	
	// Fields used by the motion planner to manage acceleration
	double speed_x, speed_y, speed_z;   // Nominal mm/minute for each axis
	double nominal_speed;               // 该动作区块执行动作时候的额定速度，由X,Y,Z轴的几何距离s算出
	double millimeters;                 // The total travel of this block in mm
	double entry_factor;                // （进入速度系数）The factor representing the change in speed at the start of this trapezoid.
										// (The end of the curren speed trapezoid is defined by the entry_factor of the
										// next block)
	
	// Settings for the trapezoid generator
	uint32_t initial_rate;              // The jerk-adjusted step rate at start of block  单位（步数/min） 
	uint32_t final_rate;                // The minimal rate at exit 单位（步数/min）
	int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
	uint32_t accelerate_until;          // 用来标记在哪一个step上停止加速 单位step
	uint32_t decelerate_after;          // The index of the step event on which to start decelerating
	void(*dir_X)(void);
	void(*dir_Y)(void);
	void(*dir_Z)(void);

	// 0-100的功率控制。0：是关闭意思
	// void(*BlockLaserControl)(uint8_t percent);
	uint8_t LaserPowerPercent;
} block_t;
typedef struct GRBL_METH{
	uint8_t line[LINE_BUFFER_SIZE];
	uint8_t char_counter;//
	// 关键参数设置
	Settings settings;
	parser_state_t gc;
	// 用于存放动作队列
	block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions,是一个存放接下来要执行动作的环形队列
	volatile int32_t block_buffer_head;           // 未来要执行的动作区块序号
	volatile int32_t block_buffer_tail;           // 当前要执行（读）的动作区块序号
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

	GRBL_MODE GrblMode;
	// 0:初始化时候
	// 1：收到一个来自gcode文件开始的标记
	// 2：收到一个来自gcode文件结束的标记
	// >2：说明收到标记'%'数量超过2，这是不合理的，因为abviewer生成的gcode文件应该只有两个'%'
	uint8_t GCodeEndFlag;
	// 从串口得到数据包,一般来说Gcode和配置code都是一行字符，所以一个数据包是以\r\n结束
	// 数据指针指向接受缓存，当得到数据之后，复制数据到这个缓存区域
	// maxlen代表的是缓冲区长度
	// 返回-1:没有数据
	// 返回>0:复制之后的数据长度（可能实际数据长度比maxlen大，但是返回maxlen）
	int8_t (*ReadCmd)(uint8_t*,uint8_t maxlen);
	void (*XAxisPwmL)(void);
	void (*XAxisPwmH)(void);
	void (*XAxisDirBack)(void);
	void (*XAxisDirGo)(void);
	void (*YAxisPwmL)(void);
	void (*YAxisPwmH)(void);
	void (*YAxisDirBack)(void);
	void (*YAxisDirGo)(void);
	void (*ZAxisPwmL)(void);
	void (*ZAxisPwmH)(void);
	void (*ZAxisDirBack)(void);
	void (*ZAxisDirGo)(void);
	void (*DisableTimeInter)(void);//禁止定时器中断
	void (*EnableTimeInter)(void);//使能定时器中断，溢出中断
	double (*SetTimeInterMs)(double timems);//设置定时器中断周期 单位ms，会返回实际中断间隔时间
	// 对于io口需要输出高电平之后再拉低，这个时间应该交给比较捕获中断做，就是溢出中断发生之后，在x个计数周期之后发生溢出中断
	void (*SetTimeCompInterUs)(double timeus);//设置一个在定时器中断发生 timeus 之后的另外一个中断。
	// 打印日志使用
	void (*SendString)(uint8_t*);
	// 用于获取永久存储数据的函数,GRBL用来存储一些设置参数,必须存放len个字节
	// -1：获取失败
	// 0:获取成功
	int8_t (*ReadNoMissingData)(uint8_t*buffer,uint8_t len);
	// 存放永久存储数据，
	// -1：存放失败
	// 0:存放成功
	int8_t (*SaveNoMissingData)(uint8_t*buffer,uint8_t len);
	// 限位开关函数
	// 1：触碰到行程开关
	// 0：没有触碰到行程开关
	uint8_t (*IsTouchX)(void);
	uint8_t (*IsTouchY)(void);
	uint8_t (*IsTouchZ)(void);
	// 主轴雕刻使用
	void(*spindle_run)(int32_t direction, uint32_t rpm);
	void(*spindle_stop)	(void);
	// 激光雕刻使用
	void(*LaserControl)(uint8_t percent);//0-100的功率控制。0：是关闭意思
	uint8_t LaserOpenFlag;//0:激光关闭，1：激光开启
	// 吹气泵控制
	void (*DisableAirPumb)();//禁止空气泵
	void (*EnableAirPumb)();//启动空气泵
}GRBL_METH;

#include "gcode.h"
#include "motion_control.h"
#include "planner.h"
#include "serial_protocol.h"
#include "serial_protocol.h"
#include "stepper.h"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define M_PI 3.14159

// The temporal resolution of the acceleration management subsystem. Higher number
// give smoother acceleration but may impact performance
// 这个主要控制 加速 减速 的计算频率，值代表每秒中计算多少次
// #define ACCELERATION_TICKS_PER_SECOND 40L
// 代表每隔多久毫秒计算一次加速度，值低就更加加速过程更加平滑，但是也更可能影响性能
#define ACCELERATION_TICKS_MS_PER_MS 25.0

/*----------------------------------对外函数接口开始----------------------------------*/
// 将该函数放入定时器中断中，用于驱动步进电机io，速度增减，动作区块弹出等等，
// 关闭定时器中断也会在这个函数中自动调用
// 一般是放在定时器的溢出中断
extern void GrblTimeInter(GRBL_METH*meth);
// 这个是用于将控制步进电机的io电平在一段时间之后，输出低电平用
// 一般是放在GrblTimeInter同一个定时器下的捕获中断
extern void GrblTimeInterComp(GRBL_METH*meth);

// 数据包输入处理，由这里开始处理代码，放在主循环中
extern int8_t SpProcess(GRBL_METH *meth);
extern void GrblInit(GRBL_METH*meth,
	// 从串口得到数据包,一般来说Gcode和配置code都是一行字符，所以一个数据包是以\r\n结束
	// 数据指针指向接受缓存，当得到数据之后，复制数据到这个缓存区域
	// maxlen代表的是缓冲区长度
	// 返回-1:没有数据
	// 返回>0:复制之后的数据长度（可能实际数据长度比maxlen大，但是返回maxlen）
	int8_t (*ReadCmd)(uint8_t*,uint8_t maxlen),
	void (*XAxisPwmL)(void),
	void (*XAxisPwmH)(void),
	void (*XAxisDirBack)(void),
	void (*XAxisDirGo)(void),
	void (*YAxisPwmL)(void),
	void (*YAxisPwmH)(void),
	void (*YAxisDirBack)(void),
	void (*YAxisDirGo)(void),
	void (*ZAxisPwmL)(void),
	void (*ZAxisPwmH)(void),
	void (*ZAxisDirBack)(void),
	void (*ZAxisDirGo)(void),
	void (*DisableTimeInter)(void),//禁止定时器中断
	void (*EnableTimeInter)(void),//使能定时器中断，溢出中断
	double (*SetTimeInterMs)(double timems),//设置定时器中断周期 单位ms，会返回实际中断间隔时间
	// 对于io口需要输出高电平之后再拉低，这个时间应该交给比较捕获中断做，就是溢出中断发生之后，在x个计数周期之后发生溢出中断
	void (*SetTimeCompInterUs)(double timeus),//设置一个在定时器中断发生 timeus 之后的另外一个中断。
	// 打印日志使用
	void (*SendString)(uint8_t*),
	// 用于获取永久存储数据的函数,GRBL用来存储一些设置参数,必须存放len个字节
	// -1：获取失败
	// 0:获取成功
	int8_t (*ReadNoMissingData)(uint8_t*buffer,uint8_t len),
	// 存放永久存储数据，
	// -1：存放失败
	// 0:存放成功
	int8_t (*SaveNoMissingData)(uint8_t*buffer,uint8_t len),
	// 限位开关函数
	// 1：触碰到行程开关
	// 0：没有触碰到行程开关
	uint8_t (*IsTouchX)(void),
	uint8_t (*IsTouchY)(void),
	uint8_t (*IsTouchZ)(void),
	// 主轴雕刻使用
	void(*spindle_run)(int32_t direction, uint32_t rpm),
	void(*spindle_stop)	(void),
	// 吹气泵控制
	void (*DisableAirPumb)(),//禁止空气泵
	void (*EnableAirPumb)(),//启动空气泵
	// 0-100的功率控制。0：是关闭意思
	void(*LaserControl)(uint8_t percent),
	// 0:激光模式
	// 1：CNC模式
	uint8_t GrblMode);
// 启动grbl模块运行，从解析第一行g代码开始
extern void GrblStart(GRBL_METH *meth);
// 暂停grbl模块运行
extern void GrblPause(GRBL_METH *meth);

// 恢复grbl模块运行
extern void GrblResume(GRBL_METH *meth);

// 停止grbl模块运行，
extern void GrblStop(GRBL_METH *meth);

// 打印grbl相关参数
extern void GrblPrintSettings(GRBL_METH *meth);

// 用于判断grbl的动作是否执行完成
// 2:没有执行动作(都没有开始)
// 1:执行完成
// 0：已经开始执行,但是未完成
extern uint8_t GrblActionComplete(GRBL_METH*meth);

/*----------------------------------对外函数接口结束----------------------------------*/

// G代码在线预览，还能提醒这个gcode是哪一个图的
// https://ncviewer.com

// 标准的Mcode，G代码说明
// http://linuxcnc.org/docs/2.6/html/gcode/m-code.html#sec:M3-M4-M5
// http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G17-G18-G19

// 新增激光模式，功率是由M10代码控制，是 ABVIEWER 软件上生成的
// 格式：M10 Q20
// ABVIEW会添加一个M11指令，不知道是干啥的

// 增加暂停与恢复功能
// G5暂停
// G6恢复

// 打算增加一个判断gcode继续接受的功能，如果动作区块已经存满，gcode仍然继续发出命令，
// 那就不能处理更多指令，grbl内部有判断，当动作区块满时候，会有死循环这样操作，由于动作区块是在
// 定时器中断中处理，在这里减动作区块

// 我发现GRBL的主轴控制部分与xyz轴控制部分是分离的，就有可能造成主轴与Xyz轴不同步

// 打算将寻线速度改成参数配置，切割速度由G代码中F代码控制

// Grbl模块是按照 ABVIEW生成的文件执行,这个文件开头和结尾有个有一个 '%'
// 模块中用来标记g代码开始与结束,

// 如果inventor生成 inventor版本的dwg，经过 ABVIEWER 转换得到g代码不带有 旋转 I 指令，这个可以
// 而cad版本的dwg则会有 -I 指令，这个I指令会导致 一直在grbl模块中循环，极其有可能
// 要么找到不带 -I 指令的g代码，要么grbl模块对I指令特殊处理
#ifdef __cplusplus
}
#endif

#endif