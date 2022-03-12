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

#define LINE_BUFFER_SIZE 50//һ�����ݰ��Ļ��泤�� ��λ���ֽ�
#define BLOCK_BUFFER_SIZE 20 //��Ž�����Ҫִ�ж����Ļ��ζ���,20������

typedef enum {LaserCutMode = 1,CNCMode }GRBL_MODE;
typedef enum {GC_Ok = 0,
			GC_BadNumberFormat,
			GC_ExpectedCommandLetter,
			GC_UnsupportedStatement,
			GC_FloatingPointError,
			GC_CodeStart,//�ҵ���ͷ��'%'
			GC_CodeEnd,
			GC_CodeEndWarn,//g�����β���� '%'���࣬�ļ�Ӧ��ֻ������һ����ͷ��һ����β
			GC_TOOBUSY,//grbl����һ������ʱ�򣬷���ռ�ö�������̫�࣬�������������еĶ������鶼�޷����ź������������Ҫ��

			}GC_STA;

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
  uint8_t SettingsSum;//���ڴ洢������У��ʹ��
} Settings;

typedef struct {
	GC_STA status_code;

	uint8_t motion_mode;             /* {G0, G1, G2, G3, G80} */
	uint8_t inverse_feed_rate_mode;  /* G93, G94 */
	uint8_t inches_mode;             /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
	uint8_t absolute_mode;           /* 0 = relative motion, 1 = absolute motion {G90, G91} */
	uint8_t program_flow;
	int32_t spindle_direction;
	uint8_t laserpower;//�����������0-100%
	double feed_rate, seek_rate;     /* ����/�� */
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
	uint32_t nominal_rate;              // �õ�һ��ֵ����ʾÿ�����������������/һ���� ��λ������/min��
	
	// Fields used by the motion planner to manage acceleration
	double speed_x, speed_y, speed_z;   // Nominal mm/minute for each axis
	double nominal_speed;               // �ö�������ִ�ж���ʱ��Ķ�ٶȣ���X,Y,Z��ļ��ξ���s���
	double millimeters;                 // The total travel of this block in mm
	double entry_factor;                // �������ٶ�ϵ����The factor representing the change in speed at the start of this trapezoid.
										// (The end of the curren speed trapezoid is defined by the entry_factor of the
										// next block)
	
	// Settings for the trapezoid generator
	uint32_t initial_rate;              // The jerk-adjusted step rate at start of block  ��λ������/min�� 
	uint32_t final_rate;                // The minimal rate at exit ��λ������/min��
	int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
	uint32_t accelerate_until;          // �����������һ��step��ֹͣ���� ��λstep
	uint32_t decelerate_after;          // The index of the step event on which to start decelerating
	void(*dir_X)(void);
	void(*dir_Y)(void);
	void(*dir_Z)(void);

	// 0-100�Ĺ��ʿ��ơ�0���ǹر���˼
	// void(*BlockLaserControl)(uint8_t percent);
	uint8_t LaserPowerPercent;
} block_t;
typedef struct GRBL_METH{
	uint8_t line[LINE_BUFFER_SIZE];
	uint8_t char_counter;//
	// �ؼ���������
	Settings settings;
	parser_state_t gc;
	// ���ڴ�Ŷ�������
	block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions,��һ����Ž�����Ҫִ�ж����Ļ��ζ���
	volatile int32_t block_buffer_head;           // δ��Ҫִ�еĶ����������
	volatile int32_t block_buffer_tail;           // ��ǰҪִ�У������Ķ����������
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

	GRBL_MODE GrblMode;
	// 0:��ʼ��ʱ��
	// 1���յ�һ������gcode�ļ���ʼ�ı��
	// 2���յ�һ������gcode�ļ������ı��
	// >2��˵���յ����'%'��������2�����ǲ�����ģ���Ϊabviewer���ɵ�gcode�ļ�Ӧ��ֻ������'%'
	uint8_t GCodeEndFlag;
	// �Ӵ��ڵõ����ݰ�,һ����˵Gcode������code����һ���ַ�������һ�����ݰ�����\r\n����
	// ����ָ��ָ����ܻ��棬���õ�����֮�󣬸������ݵ������������
	// maxlen������ǻ���������
	// ����-1:û������
	// ����>0:����֮������ݳ��ȣ�����ʵ�����ݳ��ȱ�maxlen�󣬵��Ƿ���maxlen��
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
	void (*DisableTimeInter)(void);//��ֹ��ʱ���ж�
	void (*EnableTimeInter)(void);//ʹ�ܶ�ʱ���жϣ�����ж�
	double (*SetTimeInterMs)(double timems);//���ö�ʱ���ж����� ��λms���᷵��ʵ���жϼ��ʱ��
	// ����io����Ҫ����ߵ�ƽ֮�������ͣ����ʱ��Ӧ�ý����Ƚϲ����ж�������������жϷ���֮����x����������֮��������ж�
	void (*SetTimeCompInterUs)(double timeus);//����һ���ڶ�ʱ���жϷ��� timeus ֮�������һ���жϡ�
	// ��ӡ��־ʹ��
	void (*SendString)(uint8_t*);
	// ���ڻ�ȡ���ô洢���ݵĺ���,GRBL�����洢һЩ���ò���,������len���ֽ�
	// -1����ȡʧ��
	// 0:��ȡ�ɹ�
	int8_t (*ReadNoMissingData)(uint8_t*buffer,uint8_t len);
	// ������ô洢���ݣ�
	// -1�����ʧ��
	// 0:��ųɹ�
	int8_t (*SaveNoMissingData)(uint8_t*buffer,uint8_t len);
	// ��λ���غ���
	// 1���������г̿���
	// 0��û�д������г̿���
	uint8_t (*IsTouchX)(void);
	uint8_t (*IsTouchY)(void);
	uint8_t (*IsTouchZ)(void);
	// ������ʹ��
	void(*spindle_run)(int32_t direction, uint32_t rpm);
	void(*spindle_stop)	(void);
	// ������ʹ��
	void(*LaserControl)(uint8_t percent);//0-100�Ĺ��ʿ��ơ�0���ǹر���˼
	uint8_t LaserOpenFlag;//0:����رգ�1�����⿪��
	// �����ÿ���
	void (*DisableAirPumb)();//��ֹ������
	void (*EnableAirPumb)();//����������
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
// �����Ҫ���� ���� ���� �ļ���Ƶ�ʣ�ֵ����ÿ���м�����ٴ�
// #define ACCELERATION_TICKS_PER_SECOND 40L
// ����ÿ����ú������һ�μ��ٶȣ�ֵ�;͸��Ӽ��ٹ��̸���ƽ��������Ҳ������Ӱ������
#define ACCELERATION_TICKS_MS_PER_MS 25.0

/*----------------------------------���⺯���ӿڿ�ʼ----------------------------------*/
// ���ú������붨ʱ���ж��У����������������io���ٶ��������������鵯���ȵȣ�
// �رն�ʱ���ж�Ҳ��������������Զ�����
// һ���Ƿ��ڶ�ʱ��������ж�
extern void GrblTimeInter(GRBL_METH*meth);
// ��������ڽ����Ʋ��������io��ƽ��һ��ʱ��֮������͵�ƽ��
// һ���Ƿ���GrblTimeInterͬһ����ʱ���µĲ����ж�
extern void GrblTimeInterComp(GRBL_METH*meth);

// ���ݰ����봦�������￪ʼ������룬������ѭ����
extern int8_t SpProcess(GRBL_METH *meth);
extern void GrblInit(GRBL_METH*meth,
	// �Ӵ��ڵõ����ݰ�,һ����˵Gcode������code����һ���ַ�������һ�����ݰ�����\r\n����
	// ����ָ��ָ����ܻ��棬���õ�����֮�󣬸������ݵ������������
	// maxlen������ǻ���������
	// ����-1:û������
	// ����>0:����֮������ݳ��ȣ�����ʵ�����ݳ��ȱ�maxlen�󣬵��Ƿ���maxlen��
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
	void (*DisableTimeInter)(void),//��ֹ��ʱ���ж�
	void (*EnableTimeInter)(void),//ʹ�ܶ�ʱ���жϣ�����ж�
	double (*SetTimeInterMs)(double timems),//���ö�ʱ���ж����� ��λms���᷵��ʵ���жϼ��ʱ��
	// ����io����Ҫ����ߵ�ƽ֮�������ͣ����ʱ��Ӧ�ý����Ƚϲ����ж�������������жϷ���֮����x����������֮��������ж�
	void (*SetTimeCompInterUs)(double timeus),//����һ���ڶ�ʱ���жϷ��� timeus ֮�������һ���жϡ�
	// ��ӡ��־ʹ��
	void (*SendString)(uint8_t*),
	// ���ڻ�ȡ���ô洢���ݵĺ���,GRBL�����洢һЩ���ò���,������len���ֽ�
	// -1����ȡʧ��
	// 0:��ȡ�ɹ�
	int8_t (*ReadNoMissingData)(uint8_t*buffer,uint8_t len),
	// ������ô洢���ݣ�
	// -1�����ʧ��
	// 0:��ųɹ�
	int8_t (*SaveNoMissingData)(uint8_t*buffer,uint8_t len),
	// ��λ���غ���
	// 1���������г̿���
	// 0��û�д������г̿���
	uint8_t (*IsTouchX)(void),
	uint8_t (*IsTouchY)(void),
	uint8_t (*IsTouchZ)(void),
	// ������ʹ��
	void(*spindle_run)(int32_t direction, uint32_t rpm),
	void(*spindle_stop)	(void),
	// �����ÿ���
	void (*DisableAirPumb)(),//��ֹ������
	void (*EnableAirPumb)(),//����������
	// 0-100�Ĺ��ʿ��ơ�0���ǹر���˼
	void(*LaserControl)(uint8_t percent),
	// 0:����ģʽ
	// 1��CNCģʽ
	uint8_t GrblMode);
// ����grblģ�����У��ӽ�����һ��g���뿪ʼ
extern void GrblStart(GRBL_METH *meth);
// ��ͣgrblģ������
extern void GrblPause(GRBL_METH *meth);

// �ָ�grblģ������
extern void GrblResume(GRBL_METH *meth);

// ֹͣgrblģ�����У�
extern void GrblStop(GRBL_METH *meth);

// ��ӡgrbl��ز���
extern void GrblPrintSettings(GRBL_METH *meth);

// �����ж�grbl�Ķ����Ƿ�ִ�����
// 2:û��ִ�ж���(��û�п�ʼ)
// 1:ִ�����
// 0���Ѿ���ʼִ��,����δ���
extern uint8_t GrblActionComplete(GRBL_METH*meth);

/*----------------------------------���⺯���ӿڽ���----------------------------------*/

// G��������Ԥ���������������gcode����һ��ͼ��
// https://ncviewer.com

// ��׼��Mcode��G����˵��
// http://linuxcnc.org/docs/2.6/html/gcode/m-code.html#sec:M3-M4-M5
// http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G17-G18-G19

// ��������ģʽ����������M10������ƣ��� ABVIEWER ��������ɵ�
// ��ʽ��M10 Q20
// ABVIEW�����һ��M11ָ���֪���Ǹ�ɶ��

// ������ͣ��ָ�����
// G5��ͣ
// G6�ָ�

// ��������һ���ж�gcode�������ܵĹ��ܣ�������������Ѿ�������gcode��Ȼ�����������
// �ǾͲ��ܴ������ָ�grbl�ڲ����жϣ�������������ʱ�򣬻�����ѭ���������������ڶ�����������
// ��ʱ���ж��д������������������

// �ҷ���GRBL��������Ʋ�����xyz����Ʋ����Ƿ���ģ����п������������Xyz�᲻ͬ��

// ���㽫Ѱ���ٶȸĳɲ������ã��и��ٶ���G������F�������

// Grblģ���ǰ��� ABVIEW���ɵ��ļ�ִ��,����ļ���ͷ�ͽ�β�и���һ�� '%'
// ģ�����������g���뿪ʼ�����,

// ���inventor���� inventor�汾��dwg������ ABVIEWER ת���õ�g���벻���� ��ת I ָ��������
// ��cad�汾��dwg����� -I ָ����Iָ��ᵼ�� һֱ��grblģ����ѭ���������п���
// Ҫô�ҵ����� -I ָ���g���룬Ҫôgrblģ���Iָ�����⴦��
#ifdef __cplusplus
}
#endif

#endif