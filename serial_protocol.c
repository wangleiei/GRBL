#include "serial_protocol.h"
#include "gcode.h"
extern GC_STA gc_execute_line(GRBL_METH *meth,uint8_t *line);

// ȥ��line�������Ŀո񣬽�Сд�ַ�ת���ɴ�д�ַ�
static void str_cg(uint8_t*buffer);

// �����ڼ���ģʽ�¿�����ǰ/��ʱ ����/�ر����ã�
static void APDeal(GRBL_METH *meth);
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
		case GC_TOOBUSY:break;
		default:
			sprintf(buf,"����: %d\r\n",status_code);
			meth->SendString(buf);
		}
}
// �����ڼ���ģʽ�¿�����ǰ/��ʱ ����/�ر����ã�
static void APDeal(GRBL_METH *meth){
	uint32_t limmit_mm = 20;//20mm���ر�����
	int32_t x_mm = 0,y_mm = 0,z_mm = 0,a_mm = 0;
	uint8_t next_id = 0,i = 0;
	block_t * b_t = meth->current_block;//������Ϊ�˷�ֹ�ж��и���Ϊ�գ��Խ��������ж�Ӱ��

	// �����ڼ���ģʽ�£���Ӵ�������
	if(b_t != 0){
	
		if(b_t->LaserPowerPercent != 0){
			meth->EnableAirPumb();
			return ;
		}
		if(b_t->millimeters <= limmit_mm*2.5){return;}
		// ���߶�������
		for(i = 0;i < BLOCK_BUFFER_SIZE;i ++){
			if(b_t == &(meth->block_buffer[i])){
				break;
			}
		}
		if(i == BLOCK_BUFFER_SIZE){return ;}//���ð�
		
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
// 2:������ʱ�޷������Ժ�������������
// 1:����ɹ���
// 0��û�����ݴ���
// -1������ʧ��
// ���ݰ����봦�������￪ʼ������룬������ѭ����
int8_t SpProcess(GRBL_METH *meth){
	static uint8_t cmd_planoverflag = 1;
	static uint8_t last_cmd_line[sizeof(meth->line)] = {0};
	static int8_t rec_count_last = 0;
	GC_STA gc_sta;
	int8_t rec_count = 0;
	if(1 == GrblActionComplete(meth)){
		meth->SendString("grbl����ִ�����\r\n");
		GrblStop(meth);
	}

	APDeal(meth);

	if( ((meth->block_buffer_head == BLOCK_BUFFER_SIZE) && (meth->block_buffer_tail == 0)) || 
		((meth->block_buffer_tail - meth->block_buffer_head) == 1) ){
		// meth->SendString("grbl ����������������");
		return 2;
	}
	if(1 == cmd_planoverflag){
		rec_count = meth->ReadCmd(meth->line,sizeof(meth->line));
		if(rec_count <= 0){ return 0;}
		if(rec_count >= sizeof(meth->line)){rec_count = sizeof(meth->line) - 1;}
		memcpy(last_cmd_line,meth->line,sizeof(meth->line));//����ݣ����һ��cmdû��һ�ΰ����꣬�Ǿ���
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
// 2:û��ִ�ж���(��û�п�ʼ)
// 1:ִ�����
// 0���Ѿ���ʼִ��,����δ���
uint8_t GrblActionComplete(GRBL_METH*meth){
	// g��������϶��ڶ�������ִ��ǰ�����g������������
	// ��������Ҳû�������ˣ�Ӧ�þ���ִ�����
	if((meth->GCodeEndFlag == 2) && (meth->block_buffer_head == meth->block_buffer_tail)){
		return 1;
	}
	if((meth->gc.status_code == GC_CodeStart) || (meth->gc.status_code == GC_TOOBUSY)){
		return 0;
	}
	return 2;
}