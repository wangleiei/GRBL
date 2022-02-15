#include "serial_protocol.h"
#include "gcode.h"
extern GC_STA gc_execute_line(GRBL_METH *meth,uint8_t *line);

// 去掉line缓存区的空格，将小写字符转换成大写字符
static void str_cg(uint8_t*buffer);
// 用于判断grbl的动作是否执行完成
// 1:执行完成
// 0：执行未完成
static uint8_t GrblActionComplete(GRBL_METH*meth);
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
		default:
			sprintf(buf,"错误: %d\r\n",status_code);
			meth->SendString(buf);
		}
}
// 2:数据暂时无法处理，稍后处理，动作区块满
// 1:处理成功，
// 0：没有数据处理
// -1：处理失败
// 数据包输入处理，由这里开始处理代码，放在主循环中
int8_t SpProcess(GRBL_METH *meth){
	GC_STA gc_sta;
	int8_t rec_count = 0;
	if(1 == GrblActionComplete(meth)){
		meth->SendString("grbl动作执行完成\r\n");
		GrblStop(meth);
	}
	if( ((meth->block_buffer_head == BLOCK_BUFFER_SIZE) && (meth->block_buffer_tail == 0)) || 
		((meth->block_buffer_tail - meth->block_buffer_head) == 1) ){
		// meth->SendString("grbl 动作区块满！！！");
		return 2;
	}	
	rec_count = meth->ReadCmd(meth->line,sizeof(meth->line));
	if(rec_count <= 0){ return 0;}
	if(rec_count >= sizeof(meth->line)){rec_count = sizeof(meth->line) - 1;}


	str_cg(meth->line);
	meth->line[rec_count] = 0;
	gc_sta = gc_execute_line(meth,meth->line);

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
// 1:执行完成
// 0：执行未完成
static uint8_t GrblActionComplete(GRBL_METH*meth){
	// g代码解析肯定在动作区块执行前，如果g代码解析到最后
	// 动作区块也没有数据了，应该就是执行完成
	if((meth->GCodeEndFlag == 2) && (meth->block_buffer_head == meth->block_buffer_tail)){
		return 1;
	}
	return 0;
}