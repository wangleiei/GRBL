/*
	motion_control.c - high level interface for issuing motion commands
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

#include "motion_control.h"


void mc_dwell(uint32_t milliseconds) 
{
	st_synchronize();
	// _delay_ms(milliseconds);
}

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
// position is a pointer to a vector representing the current position in millimeters.

// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(GRBL_METH*meth,double theta, double angular_travel, double radius, double linear_travel, int32_t axis_1, int32_t axis_2, 
	int32_t axis_linear, double feed_rate, int32_t invert_feed_rate, double *position)
{      
	int32_t acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
	plan_set_acceleration_manager_enabled(0); // disable acceleration management for the duration of the arc
	double millimeters_of_travel = hypot(angular_travel*radius, labs(linear_travel));
	if (millimeters_of_travel == 0.0) { return; }
	uint16_t segments = ceil(millimeters_of_travel/settings.mm_per_arc_segment);
	// Multiply inverse feed_rate to compensate for the fact that this movement is approximated
	// by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
	// all segments.
	if (invert_feed_rate) { feed_rate *= segments; }
	// The angular motion for each segment
	double theta_per_segment = angular_travel/segments;
	// The linear motion for each segment
	double linear_per_segment = linear_travel/segments;
	// Compute the center of this circle
	double center_x = position[axis_1]-sin(theta)*radius;
	double center_y = position[axis_2]-cos(theta)*radius;
	// a vector to track the end point of each segment
	double target[3];
	int32_t i;
	// Initialize the linear axis
	target[axis_linear] = position[axis_linear];
	for (i=0; i<=segments; i++) {
		target[axis_linear] += linear_per_segment;
		theta += theta_per_segment;
		target[axis_1] = center_x+sin(theta)*radius;
		target[axis_2] = center_y+cos(theta)*radius;
		plan_buffer_line(meth,target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate, invert_feed_rate);
	}
	plan_set_acceleration_manager_enabled(acceleration_manager_was_enabled);
}

void mc_go_home()
{
	st_go_home();
}
