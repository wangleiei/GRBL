#include "settings.h"

void settings_reset(GRBL_METH *meth) {
    meth->settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
    meth->settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
    meth->settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
    meth->settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
    meth->settings.default_feed_rate = DEFAULT_FEEDRATE;
    meth->settings.default_seek_rate = DEFAULT_SEEK_FEEDRATE;
    meth->settings.acceleration = DEFAULT_ACCELERATION;
    meth->settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
    meth->settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
    meth->settings.max_jerk = DEFAULT_MAX_JERK;
}

void GrblPrintSettings(GRBL_METH *meth) {
    uint8_t buf[50] = {0};
    sprintf(buf,"$0 = %f (steps/mm)X��\r\n",meth->settings.steps_per_mm[X_AXIS]);
    meth->SendString(buf);
    sprintf(buf,"$1 = %f (steps/mm)Y��\r\n",meth->settings.steps_per_mm[Y_AXIS]);
    meth->SendString(buf);
    sprintf(buf,"$2 = %f (steps/mm)Z��\r\n",meth->settings.steps_per_mm[Z_AXIS]);
    meth->SendString(buf);
    sprintf(buf,"$3 = %d (��������IO�ߵ�ƽʱ��us)\r\n",meth->settings.pulse_microseconds);
    meth->SendString(buf);
    sprintf(buf,"$4 = %f (mm/min Ĭ���и�����)\r\n",meth->settings.default_feed_rate);
    meth->SendString(buf);
    sprintf(buf,"$5 = %f (mm/min Ĭ��Ѱ������)\r\n",meth->settings.default_seek_rate);
    meth->SendString(buf);
    sprintf(buf,"$6 = %f (mm/arc segment)\r\n",meth->settings.mm_per_arc_segment);
    meth->SendString(buf);
    sprintf(buf,"$7 = %d (mm/arc segment)\r\n",meth->settings.invert_mask);
    meth->SendString(buf);
    sprintf(buf,"$8 = %f (mm/sec^2 ���ٶ�)\r\n",meth->settings.acceleration);
    meth->SendString(buf);
    sprintf(buf,"$9 = %f (mm/min ��������ڵ�任�ٶ�)\r\n",meth->settings.max_jerk);
    meth->SendString(buf);
    meth->SendString("\r\n��'$x=value'���ò��� ���� '$'��ӡ��ز���\r\n");
}

// A helper method to set settings from command line
void settings_store_setting(GRBL_METH *meth,uint8_t parameter, double value){
    // uint8_t data_buf[sizeof(meth->settings)] = {0};
    Settings SettingsTemp;
    uint8_t SettingsSum = 0;
    uint16_t i = 0;
    switch(parameter){
        case 0:case 1:case 2:meth->settings.steps_per_mm[parameter] = value; break;
        case 3: meth->settings.pulse_microseconds = round(value); break;
        case 4: meth->settings.default_feed_rate = value; break;
        case 5: meth->settings.default_seek_rate = value; break;
        case 6: meth->settings.mm_per_arc_segment = value; break;
        case 7: meth->settings.invert_mask = trunc(value); break;
        case 8: meth->settings.acceleration = value; break;
        case 9: meth->settings.max_jerk = fabs(value); break;
        default: meth->SendString("δ֪����\r\n");return;
    }
    for(i = 0;i < (sizeof(meth->settings) - sizeof(meth->settings.SettingsSum));i ++){
        SettingsSum += (uint8_t)*((uint8_t*)&(meth->settings) + i);
    }
    
    meth->settings.SettingsSum = SettingsSum;

    memcpy((uint8_t*)&SettingsTemp,(uint8_t*)&(meth->settings),sizeof(meth->settings));

    if(meth->SaveNoMissingData((uint8_t*)&SettingsTemp,sizeof(meth->settings)) < 0){
        meth->SendString("�洢����ʧ��\r\n");
        return;
    }

    if(meth->ReadNoMissingData((uint8_t*)&SettingsTemp,sizeof(meth->settings)) < 0){
        meth->SendString("�洢����ʧ��-У��ʱ���޷���ȡ\r\n");
        return;            
    }
    if(meth->settings.SettingsSum != SettingsTemp.SettingsSum){
        meth->SendString("�洢����ʧ��-У�鲻ͨ��\r\n");
        return;                    
    }

    meth->SendString("�洢���óɹ�\r\n");
}

// Initialize the config subsystem
void settings_init(GRBL_METH *meth) {
    uint8_t data_buf[sizeof(meth->settings)] = {0};

    if(meth->ReadNoMissingData(data_buf,sizeof(meth->settings)) < 0){
        meth->SendString("����: �޷���ȡ���ã�ʹ��Ĭ������\r\n");
        settings_reset(meth);

        if(meth->SaveNoMissingData((uint8_t*)&(meth->settings),sizeof(meth->settings)) < 0){
            meth->SendString("�洢Ĭ�ϲ���ʧ��\r\n");
        }else{
            meth->SendString("�洢Ĭ�ϲ����ɹ�\r\n");
        }        
    }else{
        memcpy((uint8_t*)&(meth->settings),data_buf,sizeof(meth->settings));
    }
    GrblPrintSettings(meth);
}
