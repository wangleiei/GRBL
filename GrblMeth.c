#include "GrblMeth.h"

#ifdef __cplusplus
   extern "C"{ //��������h�ļ�ʹ�ã�
#endif 
extern void settings_init(GRBL_METH *meth);
void GrblInit(GRBL_METH*meth,
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
	// 0-100�Ĺ��ʿ��ơ�0���ǹر���˼
	void(*LaserControl)(uint8_t percent),
	// 0:����ģʽ
	// 1��CNCģʽ
	uint8_t GrblMode){

	memset((uint8_t*)meth,0,sizeof(GRBL_METH));
	if(0 == GrblMode){
		meth->GrblMode = LaserCutMode;
	}else if(1 == GrblMode){
		meth->GrblMode = CNCMode;
	}

	meth->ReadCmd = ReadCmd;
	meth->XAxisPwmL = XAxisPwmL;
	meth->XAxisPwmH = XAxisPwmH;
	meth->XAxisDirBack = XAxisDirBack;
	meth->XAxisDirGo = XAxisDirGo;
	meth->YAxisPwmL = YAxisPwmL;
	meth->YAxisPwmH = YAxisPwmH;
	meth->YAxisDirBack = YAxisDirBack;
	meth->YAxisDirGo = YAxisDirGo;
	meth->ZAxisPwmL = ZAxisPwmL;
	meth->ZAxisPwmH = ZAxisPwmH;
	meth->ZAxisDirBack = ZAxisDirBack;
	meth->ZAxisDirGo = ZAxisDirGo;
	meth->DisableTimeInter = DisableTimeInter;
	meth->EnableTimeInter = EnableTimeInter;
	meth->SetTimeInterMs = SetTimeInterMs;
	meth->SetTimeCompInterUs = SetTimeCompInterUs;
	meth->SendString = SendString;
	meth->IsTouchX = IsTouchX;
	meth->IsTouchY = IsTouchY;
	meth->IsTouchZ = IsTouchZ;
	meth->spindle_run = spindle_run;
	meth->spindle_stop = spindle_stop;
	meth->LaserControl = LaserControl;
	meth->ReadNoMissingData = ReadNoMissingData;
	meth->SaveNoMissingData = SaveNoMissingData;

	meth->gc.status_code = GC_Ok;

	settings_init(meth);
	gc_init(meth);
}

#ifdef __cplusplus
}
#endif