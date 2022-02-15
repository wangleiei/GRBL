#include "GrblMeth.h"

#ifdef __cplusplus
   extern "C"{ //在最顶层调用h文件使用，
#endif 
extern void settings_init(GRBL_METH *meth);
void GrblInit(GRBL_METH*meth,
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
	// 0-100的功率控制。0：是关闭意思
	void(*LaserControl)(uint8_t percent),
	// 0:激光模式
	// 1：CNC模式
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