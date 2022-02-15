// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef planner_h
#define planner_h
								 
#include "GrblMeth.h"

			
// Initialize the motion plan subsystem      
void plan_init(GRBL_METH*meth);

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(GRBL_METH*meth,double x, double y, double z, double feed_rate, int32_t invert_feed_rate);

// ˵������������飨��block_buffer_tail�����õ���ʹ����ɣ�����и�����ִ�еĶ������飬�͸�������
void plan_discard_current_block(GRBL_METH*meth);

// �õ�����Ķ������飬���û�з���null
block_t *plan_get_current_block(GRBL_METH*meth);

// Enables or disables acceleration-management for upcoming blocks
void plan_set_acceleration_manager_enabled(GRBL_METH*meth,int32_t enabled);

// Is acceleration-management currently enabled?
int32_t plan_is_acceleration_manager_enabled(GRBL_METH*meth);

#endif