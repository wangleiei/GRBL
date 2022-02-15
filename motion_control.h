

#ifndef motion_control_h
#define motion_control_h
#include "GrblMeth.h"

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
// #define plan_buffer_line(m,x, y, z, feed_rate, invert_feed_rate) plan_buffer_line(m,x, y, z, feed_rate, invert_feed_rate) 
// NOTE: Although this function structurally belongs in this module, there is nothing to do but
// to forward the request to the planner. For efficiency the function is implemented with a macro.

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
void mc_arc(GRBL_METH*meth,double theta, double angular_travel, double radius, double linear_travel, int32_t axis_1, int32_t axis_2, 
	int32_t axis_linear, double feed_rate, int32_t invert_feed_rate, double *position);

// Dwell for a couple of time units
void mc_dwell(GRBL_METH*meth,uint32_t milliseconds);

// Send the tool home (not implemented)
void mc_go_home(GRBL_METH*meth);
extern void plan_buffer_line(GRBL_METH*meth,double x, double y, double z, double feed_rate, int32_t invert_feed_rate);
extern void plan_set_acceleration_manager_enabled(GRBL_METH*meth,int32_t enabled);
extern int32_t plan_is_acceleration_manager_enabled(GRBL_METH*meth);
#endif
