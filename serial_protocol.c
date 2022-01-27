#include "serial_protocol.h"
#include "gcode.h"
extern uint8_t gc_execute_line(uint8_t *line);

static void status_message(int32_t status_code);
// 去掉line缓存区的空格，将小写字符转换成大写字符
static void str_cg(uint8_t*buffer);
static void status_message(GRBL_METH *meth,int32_t status_code);
static void status_message(GRBL_METH *meth,int32_t status_code){
	switch(status_code){
		case GCSTATUS_OK:
				meth->printPgmString("ok\n\r"); break;
		case GCSTATUS_BAD_NUMBER_FORMAT:
				meth->printPgmString("错误: 数字格式不正确\n\r"); break;
		case GCSTATUS_EXPECTED_COMMAND_LETTER:
				meth->printPgmString("错误: 错误命令标识\n\r"); break;
		case GCSTATUS_UNSUPPORTED_STATEMENT:
				meth->printPgmString("错误: 不支持的参数\n\r"); break;
		case GCSTATUS_FLOATING_POINT_ERROR:
				meth->printPgmString("错误: 浮点数错误\n\r"); break;
		default:
				meth->printPgmString("错误: ");
				printInteger(status_code);
				meth->printPgmString("\n\r");
		}
}
void SpInit(GRBL_METH *meth) {
	meth->printPgmString("\r\nGrbl ");
	meth->printPgmString("GRBL_VERSION");  
	meth->printPgmString("\r\n");  
}
// 数据包输入处理，由这里开始处理代码，放在主循环中
void SpProcess(GRBL_METH *meth){
	uint8_t rec_count = 0;
	rec_count = meth->ReadCmd(meth->line,sizeof(meth->line));
	if(rec_count <= 0){ return;}
	if(rec_count >= sizeof(meth->line)){rec_count = sizeof(meth->line) - 1;}

	str_cg(meth->line);
	meth->line[rec_count] = 0;
 	status_message(meth,gc_execute_line(meth,meth->line));
}
// 去掉line缓存区的空格，将小写字符转换成大写字符
static void str_cg(uint8_t*buffer){
	uint8_t buf[sizeof(meth->line)] = {0};
	uint8_t i = 0,j = 0;
	for(i = 0,j = 0;i < sizeof(meth->line);i ++){

		if(buffer[i] <= ' '){break;} // 去掉非显示字符
		if((buffer[i] >= 'a') && (buffer[i] <= 'z')){//小写转大写
			buf[j] = buffer[i]-'a'+'A';
		}else{
			buf[j] = buffer[i];
		}
		j++;
	}
	memset(buffer,0,sizeof(meth->line));
	memcpy(buffer,buf,sizeof(meth->line));
}