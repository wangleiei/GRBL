

#include "motion_control.h"


void mc_dwell(GRBL_METH*meth,uint32_t milliseconds) 
{
	st_synchronize(meth);
	// _delay_ms(milliseconds);
}

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
// position is a pointer to a vector representing the current position in millimeters.

// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
// 0:正常执行一个命令，动作区块不会被填满
// 1：执行一个命令，动作区块会被填满，（再填一次就会被填满，然后在plan_buffer_line 中失去响应）
int32_t mc_arc(GRBL_METH*meth,double theta, double angular_travel, double radius, double linear_travel, int32_t axis_1, int32_t axis_2, 
	int32_t axis_linear, double feed_rate, int32_t invert_feed_rate, double *position){      
	int32_t acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled(meth);
	 
	static double millimeters_of_travel = 0.0;
	static double theta_per_segment = 0.0;
	static double theta_in = 0.0;
	static int32_t axis_linear_in = 0;
	static int32_t invert_feed_rate_in = 0;
	static double feed_rate_in = 0.0;
	static double linear_per_segment = 0.0;
	static double center_x = 0.0;
	static double center_y = 0.0;
	static double target[3] = {0.0};
	static int32_t i = 0;
	static uint16_t segments = 0;
	static uint8_t toobusy = 0;
	int32_t next_buffer_head = 0;//用来判断

	plan_set_acceleration_manager_enabled(meth,0); // disable acceleration management for the duration of the arc\


	if(toobusy == 0){
		axis_linear_in = axis_linear;
		millimeters_of_travel = hypot(angular_travel*radius, labs(linear_travel));
		if (millimeters_of_travel == 0.0) { return 0; }

		invert_feed_rate_in = invert_feed_rate;
		feed_rate_in = feed_rate;

		segments = ceil(millimeters_of_travel/meth->settings.mm_per_arc_segment);
		if (invert_feed_rate_in) { feed_rate_in *= segments; }

		theta_in = theta;
		// The angular motion for each segment
		theta_per_segment = angular_travel/segments;
		// The linear motion for each segment
		linear_per_segment = linear_travel/segments;
		// Compute the center of this circle
		center_x = position[axis_1]-sin(theta_in)*radius;
		center_y = position[axis_2]-cos(theta_in)*radius;
		// Initialize the linear axis
		target[axis_linear_in] = position[axis_linear_in];
	}
	// Multiply inverse feed_rate_in to compensate for the fact that this movement is approximated
	// by a number of discrete segments. The inverse feed_rate_in should be correct for the sum of 
	// all segments.

	
	for (; i <= segments; i ++) {
		// 动作区块存放快满了，退出这次的动作安排，并且记录下
		// 当值相等时候，说明未来要执行的动作区块已经饱和，无法处理更多的动作
		next_buffer_head = (meth->block_buffer_head + 1) % BLOCK_BUFFER_SIZE;
		if(meth->block_buffer_tail == next_buffer_head){
			toobusy = 1;
			break;
		}

		target[axis_linear_in] += linear_per_segment;
		theta_in += theta_per_segment;
		target[axis_1] = center_x + sin(theta_in)*radius;
		target[axis_2] = center_y + cos(theta_in)*radius;
		plan_buffer_line(meth,target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate_in, invert_feed_rate_in);

		// 
	}
	if(i > segments){
		toobusy = 0;
		i = 0;
	}
	plan_set_acceleration_manager_enabled(meth,acceleration_manager_was_enabled);
	if(toobusy == 1){
		return 1;
	}
	return 0;
}
void mc_go_home(GRBL_METH*meth){
	st_go_home(meth);
}
