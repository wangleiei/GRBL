#include "serial_protocol.h"
#include "gcode.h"
extern GC_STA gc_execute_line(GRBL_METH *meth,uint8_t *line);

// ȥ��line�������Ŀո񣬽�Сд�ַ�ת���ɴ�д�ַ�
static void str_cg(uint8_t*buffer);
// �����ж�grbl�Ķ����Ƿ�ִ�����
// 1:ִ�����
// 0��ִ��δ���
static uint8_t GrblActionComplete(GRBL_METH*meth);
static void status_message(GRBL_METH *meth,GC_STA status_code);
static void status_message(GRBL_METH *meth,GC_STA status_code){
	uint8_t buf[50] = {0};

	switch(status_code){
		case GC_Ok:
				// meth->SendString("ok\n\r"); 
		break;
		case GC_BadNumberFormat:
				meth->SendString("����: ���ָ�ʽ����ȷ\n\r"); break;
		case GC_UnsupportedStatement:
				meth->SendString("����: ����G M F ����\n\r"); break;
		case GC_ExpectedCommandLetter:
				meth->SendString("����: ��֧�ֵĲ���\n\r"); break;
		case GC_FloatingPointError:
				meth->SendString("����: ����������\n\r"); break;
		case GC_CodeStart:meth->SendString("��ʼ����G����\n\r");break;
		case GC_CodeEnd:meth->SendString("����G�������\n\r");break;
		case GC_CodeEndWarn:meth->SendString("����abviewer14��g�����ж���� '%',Ӧ��ͷβ��һ����\n\r");break;
		default:
			sprintf(buf,"����: %d\r\n",status_code);
			meth->SendString(buf);
		}
}
// 2:������ʱ�޷������Ժ�������������
// 1:����ɹ���
// 0��û�����ݴ���
// -1������ʧ��
// ���ݰ����봦�������￪ʼ������룬������ѭ����
int8_t SpProcess(GRBL_METH *meth){
	GC_STA gc_sta;
	int8_t rec_count = 0;
	if(1 == GrblActionComplete(meth)){
		meth->SendString("grbl����ִ�����\r\n");
		GrblStop(meth);
	}
	if( ((meth->block_buffer_head == BLOCK_BUFFER_SIZE) && (meth->block_buffer_tail == 0)) || 
		((meth->block_buffer_tail - meth->block_buffer_head) == 1) ){
		// meth->SendString("grbl ����������������");
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
// ȥ��line�������Ŀո񣬽�Сд�ַ�ת���ɴ�д�ַ�
static void str_cg(uint8_t*buffer){
	uint8_t buf[LINE_BUFFER_SIZE] = {0};
	uint8_t i = 0,j = 0;
	for(i = 0,j = 0;i < LINE_BUFFER_SIZE;i ++){

		if(buffer[i] <= ' '){continue;} // ȥ������ʾ�ַ�
		if((buffer[i] >= 'a') && (buffer[i] <= 'z')){//Сдת��д
			buf[j] = buffer[i]-'a'+'A';
		}else{
			buf[j] = buffer[i];
		}
		j++;
	}
	memset(buffer,0,LINE_BUFFER_SIZE);
	memcpy(buffer,buf,LINE_BUFFER_SIZE);
}

// �����ж�grbl�Ķ����Ƿ�ִ�����
// 1:ִ�����
// 0��ִ��δ���
static uint8_t GrblActionComplete(GRBL_METH*meth){
	// g��������϶��ڶ�������ִ��ǰ�����g������������
	// ��������Ҳû�������ˣ�Ӧ�þ���ִ�����
	if((meth->GCodeEndFlag == 2) && (meth->block_buffer_head == meth->block_buffer_tail)){
		return 1;
	}
	return 0;
}