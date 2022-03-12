#include "serial_protocol.h"
#include "gcode.h"
extern GC_STA gc_execute_line(GRBL_METH *meth,uint8_t *line);

// 去掉line缓存区的空格，将小写字符转换成大写字符
static void str_cg(uint8_t*buffer);

// 用于在激光模式下控制提前/延时 启动/关闭气泵，
static void APDeal(GRBL_METH *meth);
static void status_message(GRBL_METH *meth,GC_STA status_code);
static void status_message(GRBL_METH *meth,GC_STA status_code){
	uint8_t buf[50] = {0};

	switch(status_code){
		case GC_Ok:
				// meth->SendString("ok\n\r"); 
		break;
		case GC_BadNumberFormat:
				meth->SendString("错误: 数字格式不正确\n\r"); break;
		case GC_UnsupportedStatement:
				meth->SendString("错误: 错误G M F 代码\n\r"); break;
		case GC_ExpectedCommandLetter:
				meth->SendString("错误: 不支持的参数\n\r"); break;
		case GC_FloatingPointError:
				meth->SendString("错误: 浮点数错误\n\r"); break;
		case GC_CodeStart:meth->SendString("开始解析G代码\n\r");break;
		case GC_CodeEnd:meth->SendString("解析G代码结束\n\r");break;
		case GC_CodeEndWarn:meth->SendString("来着abviewer14的g代码有多余的 '%',应该头尾各一个的\n\r");break;
		case GC_TOOBUSY:break;
		default:
			sprintf(buf,"错误: %d\r\n",status_code);
			meth->SendString(buf);
		}
}
// 用于在激光模式下控制提前/延时 启动/关闭气泵，
static void APDeal(GRBL_METH *meth){
	uint32_t limmit_mm = 20;//20mm处关闭气泵
	int32_t x_mm = 0,y_mm = 0,z_mm = 0,a_mm = 0;
	uint8_t next_id = 0,i = 0;
	block_t * b_t = meth->current_block;//这里是为了防止中断中更改为空，对接下来的判断影响

	// 用于在激光模式下，添加吹气功能
	if(b_t != 0){
	
		if(b_t->LaserPowerPercent != 0){
			meth->EnableAirPumb();
			return ;
		}
		if(b_t->millimeters <= limmit_mm*2.5){return;}
		// 切线动作区块
		for(i = 0;i < BLOCK_BUFFER_SIZE;i ++){
			if(b_t == &(meth->block_buffer[i])){
				break;
			}
		}
		if(i == BLOCK_BUFFER_SIZE){return ;}//不该啊
		
  		x_mm = lround(b_t->steps_x * (meth->settings.steps_per_mm[X_AXIS]));
  		y_mm = lround(b_t->steps_y * (meth->settings.steps_per_mm[Y_AXIS]));
  		z_mm = lround(b_t->steps_z * (meth->settings.steps_per_mm[Z_AXIS]));		
		a_mm = sqrt(x_mm*x_mm+y_mm*y_mm+z_mm*z_mm);
		//  (meth->current_block->millimeters)
		// next_id = (i+1)%BLOCK_BUFFER_SIZE;
		if((a_mm >= limmit_mm) && (a_mm <= 1.5*limmit_mm)){
			meth->DisableAirPumb();
		// }else if((meth->block_buffer[next_id].LaserPowerPercent != 0) && (a_mm > 1.5*limmit_mm)){
		}else if(a_mm > 1.5*limmit_mm){
			meth->EnableAirPumb();
		}
	}
}
// 2:数据暂时无法处理，稍后处理，动作区块满
// 1:处理成功，
// 0：没有数据处理
// -1：处理失败
// 数据包输入处理，由这里开始处理代码，放在主循环中
int8_t SpProcess(GRBL_METH *meth){
	static uint8_t cmd_planoverflag = 1;
	static uint8_t last_cmd_line[sizeof(meth->line)] = {0};
	static int8_t rec_count_last = 0;
	GC_STA gc_sta;
	int8_t rec_count = 0;
	if(1 == GrblActionComplete(meth)){
		meth->SendString("grbl动作执行完成\r\n");
		GrblStop(meth);
	}

	APDeal(meth);

	if( ((meth->block_buffer_head == BLOCK_BUFFER_SIZE) && (meth->block_buffer_tail == 0)) || 
		((meth->block_buffer_tail - meth->block_buffer_head) == 1) ){
		// meth->SendString("grbl 动作区块满！！！");
		return 2;
	}
	if(1 == cmd_planoverflag){
		rec_count = meth->ReadCmd(meth->line,sizeof(meth->line));
		if(rec_count <= 0){ return 0;}
		if(rec_count >= sizeof(meth->line)){rec_count = sizeof(meth->line) - 1;}
		memcpy(last_cmd_line,meth->line,sizeof(meth->line));//命令备份，如果一个cmd没有一次安排完，那就由
		rec_count_last = rec_count;
	}else{
		memcpy(meth->line,last_cmd_line,sizeof(meth->line));
		rec_count = rec_count_last;
	}


	str_cg(meth->line);
	meth->line[rec_count] = 0;
	gc_sta = gc_execute_line(meth,meth->line);

	if(GC_TOOBUSY == gc_sta){
		cmd_planoverflag = 0;
	}else{
		cmd_planoverflag = 1;
	}

 	status_message(meth,gc_sta);
 	memset(meth->line,0,LINE_BUFFER_SIZE);
 	if(GC_Ok == gc_sta){
 		return 1;
 	}else{
 		return -1;
 	}
 	
}
// 去掉line缓存区的空格，将小写字符转换成大写字符
static void str_cg(uint8_t*buffer){
	uint8_t buf[LINE_BUFFER_SIZE] = {0};
	uint8_t i = 0,j = 0;
	for(i = 0,j = 0;i < LINE_BUFFER_SIZE;i ++){

		if(buffer[i] <= ' '){continue;} // 去掉非显示字符
		if((buffer[i] >= 'a') && (buffer[i] <= 'z')){//小写转大写
			buf[j] = buffer[i]-'a'+'A';
		}else{
			buf[j] = buffer[i];
		}
		j++;
	}
	memset(buffer,0,LINE_BUFFER_SIZE);
	memcpy(buffer,buf,LINE_BUFFER_SIZE);
}


// 用于判断grbl的动作是否执行完成
// 2:没有执行动作(都没有开始)
// 1:执行完成
// 0：已经开始执行,但是未完成
uint8_t GrblActionComplete(GRBL_METH*meth){
	// g代码解析肯定在动作区块执行前，如果g代码解析到最后
	// 动作区块也没有数据了，应该就是执行完成
	if((meth->GCodeEndFlag == 2) && (meth->block_buffer_head == meth->block_buffer_tail)){
		return 1;
	}
	if((meth->gc.status_code == GC_CodeStart) || (meth->gc.status_code == GC_TOOBUSY)){
		return 0;
	}
	return 2;
}