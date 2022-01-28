#include "gcode.h"

#define MM_PER_INCH (25.4)

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2

#define MOTION_MODE_SEEK 0 // G0 
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS  2

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1

// static parser_state_t gc;

#define FAIL(status) meth->gc.status_code = status;

static int32_t read_double(GRBL_METH *meth,uint8_t *line,               //  <- string: line of RS274/NGC code being processed
										 int32_t *char_counter,        //  <- pointer to a counter for position on the line 
										 double *double_ptr); //  <- pointer to double to be read                  

static int32_t next_statement(GRBL_METH *meth,uint8_t *letter, double *double_ptr, uint8_t *line, int32_t *char_counter) ;
static void select_plane(GRBL_METH *meth,uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);

static void select_plane(GRBL_METH *meth,uint8_t axis_0, uint8_t axis_1, uint8_t axis_2){
	meth->gc.plane_axis_0 = axis_0;
	meth->gc.plane_axis_1 = axis_1;
	meth->gc.plane_axis_2 = axis_2;
}

void gc_init(GRBL_METH *meth) {
	memset(&meth->gc, 0, sizeof(meth->gc));
	meth->gc.feed_rate = meth->settings.default_feed_rate/60;
	meth->gc.seek_rate = meth->settings.default_seek_rate/60;
	select_plane(meth,X_AXIS, Y_AXIS, Z_AXIS);
	meth->gc.absolute_mode = 1;
}

// Find the angle in radians of deviance from the positive y axis. negative angles to the left of y-axis, 
// positive to the right.
double theta(double x, double y)
{
	double theta = atan(x/fabs(y));
	if (y>0) {
		return(theta);
	} else {
		if (theta>0) 
		{
			return(M_PI-theta);
		} else {
			return(-M_PI-theta);
		}
	}
}
// 执行G代码
// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace).
uint8_t gc_execute_line(GRBL_METH *meth,uint8_t *line){
	int32_t char_counter = 0;  
	uint8_t letter = 0;
	double value = 0.0;
	double unit_converted_value = 0.0;
	double inverse_feed_rate = -1; // negative inverse_feed_rate means no inverse_feed_rate specified
	double x = 0.0;
	double y = 0.0;
	double h_x2_div_d = 0.0;
	double theta_start = 0.0;
	double theta_end = 0.0;
	double angular_travel = 0.0;
	double radius= 0.0;
	double depth= 0.0;
	int32_t radius_mode = 0;
	
	uint8_t absolute_override = 0;          /* 1 = absolute motion for this block only {G53} */
	uint8_t next_action = NEXT_ACTION_DEFAULT;  /* The action that will be taken by the parsed line */
	
	double target[3], offset[3];  
	
	double p = 0, r = 0;
	int32_t int_value;
	
	memset(target,0,sizeof(target));
	memset(offset,0,sizeof(offset));

	meth->gc.status_code = GCSTATUS_OK;
	
	// Disregard comments and block delete
	if (line[0] == '(') { return(meth->gc.status_code); }
	if (line[0] == '/') { char_counter++; } // ignore block delete  
	
	// If the line starts with an '$' it is a configuration-command，配置代码
	if (line[0] == '$') { 
		// Parameter lines are on the form '$4=374.3' or '$' to dump current settings
		char_counter = 1;
		if(line[char_counter] == 0) {
				settings_dump(meth); 
				return(GCSTATUS_OK); 
		}
		read_double(meth,line,&char_counter, &p);
		if(line[char_counter++] != '=') {
				return(GCSTATUS_UNSUPPORTED_STATEMENT); 
		}
		read_double(meth,line, &char_counter, &value);
		if(line[char_counter] != 0) {
				return(GCSTATUS_UNSUPPORTED_STATEMENT); 
		}
		settings_store_setting(meth,p, value);
		return(meth->gc.status_code);
	}
	
	/* We'll handle this as g-code. First: parse all statements */
	// G0X10.782Y26.491 
	// Pass 1: Commands,应该是一些参数类型的命令
	while(next_statement(meth,&letter, &value, line, &char_counter)) {
		int_value = trunc(value);//舍取小数位后的整数。
		switch(letter) {
			case 'G':
				switch(int_value) {
					case 0: meth->gc.motion_mode = MOTION_MODE_SEEK; break;
					case 1: meth->gc.motion_mode = MOTION_MODE_LINEAR; break;
					case 2: meth->gc.motion_mode = MOTION_MODE_CW_ARC; break;
					case 3: meth->gc.motion_mode = MOTION_MODE_CCW_ARC; break;
					case 4: next_action = NEXT_ACTION_DWELL; break;
					case 17: select_plane(meth,X_AXIS, Y_AXIS, Z_AXIS); break;
					case 18: select_plane(meth,X_AXIS, Z_AXIS, Y_AXIS); break;
					case 19: select_plane(meth,Y_AXIS, Z_AXIS, X_AXIS); break;
					case 20: meth->gc.inches_mode = 1; break;
					case 21: meth->gc.inches_mode = 0; break;
					case 28: case 30: next_action = NEXT_ACTION_GO_HOME; break;
					case 53: absolute_override = 1; break;
					case 80: meth->gc.motion_mode = MOTION_MODE_CANCEL; break;
					case 90: meth->gc.absolute_mode = 1; break;
					case 91: meth->gc.absolute_mode = 0; break;
					case 93: meth->gc.inverse_feed_rate_mode = 1; break;
					case 94: meth->gc.inverse_feed_rate_mode = 0; break;
					default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
				}
			break;
			
			case 'M':
				switch(int_value) {
					case 0: case 1: meth->gc.program_flow = PROGRAM_FLOW_PAUSED; break;
					case 2: case 30: case 60: meth->gc.program_flow = PROGRAM_FLOW_COMPLETED; break;
					case 3: meth->gc.spindle_direction = 1; break;
					case 4: meth->gc.spindle_direction = -1; break;
					case 5: meth->gc.spindle_direction = 0; break;
					default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
				}            
			break;
			case 'T': meth->gc.tool = trunc(value); break;
		}
		if(meth->gc.status_code) { break; }
	}
	
	// If there were any errors parsing this line, we will return right away with the bad news
	if (meth->gc.status_code) { return(meth->gc.status_code); }

	char_counter = 0;
	memset(offset,0,sizeof(offset));
	memcpy(target, meth->gc.position, sizeof(target)); // i.e. target = meth->gc.position

	// Pass 2: Parameters
	while(next_statement(meth,&letter, &value, line, &char_counter)) {
		int_value = trunc(value);
		unit_converted_value = meth->gc.inches_mode ? (value * MM_PER_INCH) : value;//单位全部换算成毫米
		switch(letter) {
			case 'F': 
			if (meth->gc.inverse_feed_rate_mode) {
				inverse_feed_rate = unit_converted_value; // seconds per motion for this motion only
			} else {          
				if (meth->gc.motion_mode == MOTION_MODE_SEEK) {
					meth->gc.seek_rate = unit_converted_value/60;
				} else {
					meth->gc.feed_rate = unit_converted_value/60; // millimeters pr second
				}
			}
			break;
			case 'I': case 'J': case 'K': offset[letter-'I'] = unit_converted_value; break;
			case 'P': p = value; break;
			case 'R': r = unit_converted_value; radius_mode = 1; break;
			case 'S': meth->gc.spindle_speed = value; break;
			case 'X': case 'Y': case 'Z':
			if (meth->gc.absolute_mode || absolute_override) {
				target[letter - 'X'] = unit_converted_value;
			} else {
				target[letter - 'X'] += unit_converted_value;
			}
			break;
		}
	}
	
	// If there were any errors parsing this line, we will return right away with the bad news
	if (meth->gc.status_code) { return(meth->gc.status_code); }
		
	// Update spindle state
	if (meth->gc.spindle_direction) {
		meth->spindle_run(meth->gc.spindle_direction, meth->gc.spindle_speed);
	} else {
		meth->spindle_stop();
	}
	
	// Perform any physical actions
	switch (next_action) {
		case NEXT_ACTION_GO_HOME: mc_go_home(meth); break;
		case NEXT_ACTION_DWELL: mc_dwell(meth,trunc(p*1000)); break;
		case NEXT_ACTION_DEFAULT: 
		switch (meth->gc.motion_mode) {
			case MOTION_MODE_CANCEL: break;
			case MOTION_MODE_SEEK:
			mc_line(meth,target[X_AXIS], target[Y_AXIS], target[Z_AXIS], meth->gc.seek_rate, 0);
			break;
			case MOTION_MODE_LINEAR:
			mc_line(meth,target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
				(meth->gc.inverse_feed_rate_mode) ? inverse_feed_rate : meth->gc.feed_rate, meth->gc.inverse_feed_rate_mode);
			break;
			case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
			if (radius_mode) {
				/* 
					We need to calculate the center of the circle that has the designated radius and passes
					through both the current position and the target position. This method calculates the following
					set of equations where [x,y] is the vector from current to target position, d == magnitude of 
					that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
					the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the 
					length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point 
					[i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.
					
					d^2 == x^2 + y^2
					h^2 == r^2 - (d/2)^2
					i == x/2 - y/d*h
					j == y/2 + x/d*h
					
																															 O <- [i,j]
																														-  |
																									r      -     |
																											-        |
																									 -           | h
																								-              |
																	[0,0] ->  C -----------------+--------------- T  <- [x,y]
																						| <------ d/2 ---->|
										
					C - Current position
					T - Target position
					O - center of circle that pass through both C and T
					d - distance from C to T
					r - designated radius
					h - distance from center of CT to O
					
					Expanding the equations:

					d -> sqrt(x^2 + y^2)
					h -> sqrt(4 * r^2 - x^2 - y^2)/2
					i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
					j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
				 
					Which can be written:
					
					i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
					j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
					
					Which we for size and speed reasons optimize to:

					h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
					i = (x - (y * h_x2_div_d))/2
					j = (y + (x * h_x2_div_d))/2
					
				*/
				
				// Calculate the change in position along each selected axis
				x = target[meth->gc.plane_axis_0]-meth->gc.position[meth->gc.plane_axis_0];
				y = target[meth->gc.plane_axis_1]-meth->gc.position[meth->gc.plane_axis_1];
				
				memset(offset,0,sizeof(offset));
				h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)
				// If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
				// real CNC, and thus - for practical reasons - we will terminate promptly:
				if(isnan(h_x2_div_d)) { FAIL(GCSTATUS_FLOATING_POINT_ERROR); return(meth->gc.status_code); }
				// Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
				if (meth->gc.motion_mode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }
				
				/* The counter clockwise circle lies to the left of the target direction. When offset is positive,
					 the left hand circle will be generated - when it is negative the right hand circle is generated.
					 
					 
																												 T  <-- Target position
																												 
																												 ^ 
							Clockwise circles with this center         |          Clockwise circles with this center will have
							will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
																							 \         |          /   
				center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
																												 |
																												 |
																												 
																												 C  <-- Current position                                 */
								

				// Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!), 
				// even though it is advised against ever generating such circles in a single line of g-code. By 
				// inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
				// travel and thus we get the unadvisably long arcs as prescribed.
				if (r < 0) { h_x2_div_d = -h_x2_div_d; }        
				// Complete the operation by calculating the actual center of the arc
				offset[meth->gc.plane_axis_0] = (x-(y*h_x2_div_d))/2;
				offset[meth->gc.plane_axis_1] = (y+(x*h_x2_div_d))/2;
			} 
			
			/*
				 This segment sets up an clockwise or counterclockwise arc from the current position to the target position around 
				 the center designated by the offset vector. All theta-values measured in radians of deviance from the positive 
				 y-axis. 

														| <- theta == 0
													* * *                
												*       *                                               
											*           *                                             
											*     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)                                          
											*   /                                                     
												C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))

			*/
						
			// calculate the theta (angle) of the current point
			theta_start = theta(-offset[meth->gc.plane_axis_0], -offset[meth->gc.plane_axis_1]);
			// calculate the theta (angle) of the target point
			theta_end = theta(target[meth->gc.plane_axis_0] - offset[meth->gc.plane_axis_0] - meth->gc.position[meth->gc.plane_axis_0], 
				 target[meth->gc.plane_axis_1] - offset[meth->gc.plane_axis_1] - meth->gc.position[meth->gc.plane_axis_1]);
			// ensure that the difference is positive so that we have clockwise travel
			if (theta_end < theta_start) { theta_end += 2*M_PI; }
			angular_travel = theta_end-theta_start;
			// Invert angular motion if the g-code wanted a counterclockwise arc
			if (meth->gc.motion_mode == MOTION_MODE_CCW_ARC) {
				angular_travel = angular_travel-2*M_PI;
			}
			// Find the radius
			radius = hypot(offset[meth->gc.plane_axis_0], offset[meth->gc.plane_axis_1]);
			// Calculate the motion along the depth axis of the helix
			depth = target[meth->gc.plane_axis_2]-meth->gc.position[meth->gc.plane_axis_2];
			// Trace the arc
			mc_arc(meth,theta_start, angular_travel, radius, depth, meth->gc.plane_axis_0, meth->gc.plane_axis_1, meth->gc.plane_axis_2, 
				(meth->gc.inverse_feed_rate_mode) ? inverse_feed_rate : meth->gc.feed_rate, meth->gc.inverse_feed_rate_mode,
				meth->gc.position);
			// Finish off with a line to make sure we arrive exactly where we think we are
			mc_line(meth,target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
				(meth->gc.inverse_feed_rate_mode) ? inverse_feed_rate : meth->gc.feed_rate, meth->gc.inverse_feed_rate_mode);
			break;
		}    
	}
	
	// As far as the parser is concerned, the position is now == target. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	memcpy(meth->gc.position, target, sizeof(double)*3); // meth->gc.position[] = target[];
	return(meth->gc.status_code);
}

// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
static int32_t next_statement(GRBL_METH *meth,uint8_t *letter, double *double_ptr, uint8_t *line, int32_t *char_counter){
	if (line[*char_counter] == 0) {
		return(0); // No more statements
	}
	
	*letter = line[*char_counter];
	if((*letter < 'A') || (*letter > 'Z')) {
		FAIL(GCSTATUS_EXPECTED_COMMAND_LETTER);
		return(0);
	}
	(*char_counter)++;
	if (!read_double(meth,line, char_counter, double_ptr)) {
		return(0);
	};
	return(1);
}

int32_t read_double(GRBL_METH *meth,uint8_t *line,               //!< string: line of RS274/NGC code being processed
										 int32_t *char_counter,   //!< pointer to a counter for position on the line 
										 double *double_ptr)  //!< pointer to double to be read                  
{
	uint8_t *start = line + *char_counter;
	uint8_t *end = 0;
	
	*double_ptr = strtod(start, &end);//字符串转换为一个浮点数
	if(end == start) { 
		FAIL(GCSTATUS_BAD_NUMBER_FORMAT); 
		return(0); 
	};

	*char_counter = end - line;
	return(1);
}

/* 
	Intentionally not supported:

	- Canned cycles
	- Tool radius compensation
	- A,B,C-axes
	- Multiple coordinate systems
	- Evaluation of expressions
	- Variables
	- Multiple home locations
	- Probing
	- Override control

	 group 0 = {G10, G28, G30, G92, G92.1, G92.2, G92.3} (Non modal G-codes)
	 group 8 = {M7, M8, M9} coolant (special case: M7 and M8 may be active at the same time)
	 group 9 = {M48, M49} enable/disable feed and speed override switches
	 group 12 = {G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3} coordinate system selection
	 group 13 = {G61, G61.1, G64} path control mode
*/

