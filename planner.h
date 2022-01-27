/*
	planner.h - buffers movement commands and manages the acceleration profile plan
	Part of Grbl

	The MIT License (MIT)

	Grbl(tm) - Embedded CNC g-code interpreter and motion-controller
	Copyright (c) 2009-2011 Simen Svale Skogsrud

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef planner_h
#define planner_h
								 
#include "GrblMeth.h"

			
// Initialize the motion plan subsystem      
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(GRBL_METH*meth,double x, double y, double z, double feed_rate, int32_t invert_feed_rate);

// 说明这个动作区块（以block_buffer_tail索引得到）使用完成，如果有更多能执行的动作区块，就更新索引
void plan_discard_current_block();

// 得到最近的动作区块，如果没有返回null
block_t *plan_get_current_block(GRBL_METH*meth) ;

// Enables or disables acceleration-management for upcoming blocks
void plan_set_acceleration_manager_enabled(int32_t enabled);

// Is acceleration-management currently enabled?
int32_t plan_is_acceleration_manager_enabled();

#endif