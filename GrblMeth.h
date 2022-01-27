#ifndef GRBL_H
#define GRBL_H

	#ifdef __cplusplus
	   extern "C"{ //��������h�ļ�ʹ�ã�
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
#define LINE_BUFFER_SIZE 50//һ�����ݰ��Ļ��泤�� ��λ���ֽ�
#define BLOCK_BUFFER_SIZE 20 //��Ž�����Ҫִ�ж����Ļ��ζ���,20������
// Current global settings (persisted in EEPROM from byte 1 onwards)
typedef struct {
  double steps_per_mm[3];//�ֱ��Ӧx,y,z������1������Ҫ���ٸ�����
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
	int32_t  step_event_count;          // ÿ���ƶ��Ĳ���������x��y��z���������Ϊ׼
	uint32_t nominal_rate;              // �õ�һ��ֵ����ʾÿ�����������������/min ��λ������/min��
	
	// Fields used by the motion planner to manage acceleration
	double speed_x, speed_y, speed_z;   // Nominal mm/minute for each axis
	double nominal_speed;               // �ö�������ִ�ж���ʱ��Ķ�ٶ�
	double millimeters;                 // The total travel of this block in mm
	double entry_factor;                // The factor representing the change in speed at the start of this trapezoid.
																			// (The end of the curren speed trapezoid is defined by the entry_factor of the
																			// next block)
	
	// Settings for the trapezoid generator
	uint32_t initial_rate;              // The jerk-adjusted step rate at start of block  
	uint32_t final_rate;                // The minimal rate at exit
	int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
	uint32_t accelerate_until;          // �����������һ��step��ֹͣ���� ��λstep
	uint32_t decelerate_after;          // The index of the step event on which to start decelerating
	
} block_t;
typedef struct GRBL_METH{
	// �Ӵ��ڵõ����ݰ�,һ����˵Gcode������code����һ���ַ�������һ�����ݰ�����\r\n����
	// ����ָ��ָ����ܻ��棬���õ�����֮�󣬸������ݵ������������
	// maxlen������ǻ���������
	// ����-1:û������
	// ����>0:����֮������ݳ��ȣ�����ʵ�����ݳ��ȱ�maxlen�󣬵��Ƿ���maxlen��
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
	void (*DisableTimeInter)(void);//��ֹ��ʱ���ж�
	void (*EnableTimeInter)(void);//ʹ�ܶ�ʱ���жϣ�����ж�
	double (*SetTimeInterMs)(double timems);//���ö�ʱ���ж����� ��λms���᷵��ʵ���жϼ��ʱ��
	// ����io����Ҫ����ߵ�ƽ֮�������ͣ����ʱ��Ӧ�ý����Ƚϲ����ж�������������жϷ���֮����x����������֮��������ж�
	void (*SetTimeCompInterUs)(double timeus);//����һ���ڶ�ʱ���жϷ��� timeus ֮�������һ���жϡ�
	// ��ӡ��־ʹ��
	void (*printPgmString)(uint8_t*);
	void (*printByte)(uint8_t);
	
	uint8_t line[LINE_BUFFER_SIZE];
	uint8_t char_counter;//
	// �ؼ���������
	settings_t settings;
	parser_state_t gc;
	// ���ڴ�Ŷ�������
	block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions,��һ����Ž�����Ҫִ�ж����Ļ��ζ���
	volatile int32_t block_buffer_head;           // Index of the next block to be pushed
	volatile int32_t block_buffer_tail;           // Index of the block to process now
	// The current position of the tool in absolute steps
	// Ӧ���ǵ�ǰ�������0��Ĳ���
	int32_t position[3];
	uint8_t acceleration_manager_enabled;   //����������Ƿ���Ҫ�����˶�

	// stepper.c��ʵ���㷨���õ��м����
	// Counter variables for the bresenham line tracer
	int32_t st_counter_x;
	int32_t st_counter_y;
	int32_t st_counter_z;
	uint32_t step_events_completed; // ��һ�������������Ѿ�ִ�����˵Ĳ���
	block_t *current_block;  // ��ǰ�����еĶ�������ָ��
	uint32_t trapezoid_adjusted_rate;// ��ֵ���ڵ���ÿ�����ж��ٸ���ʱ���ж� ��λ��step/min��
	double ms_per_step_event;        // ���ζ�ʱ���жϼ����ָ�����ڣ����ڸ��³������ж�ʱ��Ķ�ʱ������������
	double trapezoid_tick_ms_counter;//��������ʱ�䣨ms��
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
// �����Ҫ���� ���� ���� �ļ���Ƶ�ʣ�ֵ����ÿ���м�����ٴ�
// #define ACCELERATION_TICKS_PER_SECOND 40L
// ����ÿ����ú������һ�μ��ٶȣ�ֵ�;͸��Ӽ��ٹ��̸���ƽ��������Ҳ������Ӱ������
#define ACCELERATION_TICKS_MS_PER_MS 25.0
// ���������С��ʱ���ж�Ƶ�ʣ�1200��/����
#define MINIMUM_STEPS_PER_MINUTE 1200 // The stepper subsystem will never run slower than this, exept when sleeping

#define clear_vector(a) memset(a, 0, sizeof(a))
#define max(a,b) (((a) > (b)) ? (a) : (b))


void printInteger(int32_t n);
void printFloat(float n);

extern GRBL_METH GrblMeth;

/*-----------------------------------------------------���⺯��-----------------------------------------------------*/
extern void TimeInter(GRBL_METH*meth);
extern void TimeInterComp(GRBL_METH*meth);

extern void SpInit(GRBL_METH *meth);
// ���ݰ����봦�������￪ʼ������룬������ѭ����
extern void SpProcess(GRBL_METH *meth);
	#ifdef __cplusplus
	}
	#endif
#endif