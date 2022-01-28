
/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*  
	Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
	
	s == speed, a == acceleration, t == time, d == distance

	Basic definitions:

		Speed[s_, a_, t_] := s + (a*t) 
		Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t],求速度对时间的积分

	Distance to reach a specific speed with a constant acceleration:

		Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
			d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()

	Speed after a given distance of travel with constant acceleration:

		Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
			m -> Sqrt[2 a d + s^2]    

		DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]

	When to start braking (di) to reach a specified destionation speed (s2) after accelerating
	from initial speed s1 without ever stopping at a plateau:

		Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
			di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()

		IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
*/
#include "planner.h"
static void calculate_trapezoid_for_block(block_t *block, double entry_factor, double exit_factor);
static double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration);
static void planner_forward_pass(GRBL_METH*meth);
static void planner_recalculate_trapezoids(GRBL_METH*meth);
static void planner_recalculate(GRBL_METH*meth);

#define max(a,b) ( ( (a) > (b) ) ? (a) : (b) )

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
static double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration){
	return(
		(target_rate*target_rate-initial_rate*initial_rate)/
		(2L*acceleration)
	);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

/*                        + <- some maximum rate we don't care about
												 /|\
												/ | \                    
											 /  |  + <- final_rate     
											/   |  |                   
		 initial_rate -> +----+--+                   
												   ^  ^                   
												   |  |                   
									intersection_distance  distance                                                                           */

double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
	return(
		(2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
		(4*acceleration)
	);
}


// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.

/*                                                                              
																		 +--------+   <- nominal_rate
																		/          \                                
		nominal_rate*entry_factor ->   +            \                               
																	 |             + <- nominal_rate*exit_factor  
																	 +-------------+                              
																			 time -->                                 
*/                                                                              

static void calculate_trapezoid_for_block(block_t *block, double entry_factor, double exit_factor) {
	int32_t acceleration_per_minute = 0;
	int32_t accelerate_steps = 0;	
	int32_t decelerate_steps = 0;
	int32_t plateau_steps = 0;
	block->initial_rate = ceil(block->nominal_rate*entry_factor);
	block->final_rate = ceil(block->nominal_rate*exit_factor);
	acceleration_per_minute = block->rate_delta*1000.0*60.0/ACCELERATION_TICKS_MS_PER_MS;//steps/minute
	// acceleration_per_minute = block->rate_delta*ACCELERATION_TICKS_PER_SECOND*60.0;//steps/minute
	accelerate_steps = 
		ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_minute));
	decelerate_steps = 
		floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration_per_minute));

	// Calculate the size of Plateau of Nominal Rate. 
	plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
	
	// Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
	// have to use intersection_distance() to calculate when to abort acceleration and start braking 
	// in order to reach the final_rate exactly at the end of this block.
	if (plateau_steps < 0) {  
		accelerate_steps = ceil(
			intersection_distance(block->initial_rate, block->final_rate, acceleration_per_minute, block->step_event_count));
		plateau_steps = 0;
	}  
	
	block->accelerate_until = accelerate_steps;
	block->decelerate_after = accelerate_steps+plateau_steps;
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
double max_allowable_speed(double acceleration, double target_velocity, double distance) {//返回值单位 应该是mm/min
	return(sqrt(target_velocity*target_velocity-2*acceleration*60*60*distance));
}
// 该函数指的是在两个区块之间速度的变化,是通过计算两个动作区块之间的各轴速度得到，返回结果的单位是（mm/minute）
// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
double junction_jerk(block_t *before, block_t *after) {
	return(sqrt(
		pow(before->speed_x-after->speed_x, 2)+
		pow(before->speed_y-after->speed_y, 2)+
		pow(before->speed_z-after->speed_z, 2))
	);
}

// Calculate a braking factor to reach baseline speed which is max_jerk/2, e.g. the 
// speed under which you cannot exceed max_jerk no matter what you do.
double factor_for_safe_speed(GRBL_METH *meth,block_t *block) {
	return(meth->settings.max_jerk/block->nominal_speed);  
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(GRBL_METH *meth,block_t *previous, block_t *current, block_t *next) {
	double entry_factor = 1.0;
	double exit_factor = 0.0,jerk = 0.0,max_entry_speed = 0.0,max_entry_factor = 0.0;
	if(!current) { return; }

	if (next) {
		exit_factor = next->entry_factor;
	} else {
		exit_factor = factor_for_safe_speed(meth,current);
	}
	
	// 计算当前动作区块的进入速度因子，由前动作区块得到
	if (previous) {
		// Reduce speed so that junction_jerk is within the maximum allowed
		// 该函数指的是在两个区块之间速度的变化,是通过计算两个动作区块之间的各轴速度得到，返回结果的单位是（mm/minute）
		jerk = junction_jerk(previous, current);//这个词代表节点的加加速度？，
		if (jerk > meth->settings.max_jerk) {
			entry_factor = (meth->settings.max_jerk/jerk);
		} 
		// If the required deceleration across the block is too rapid, reduce the entry_factor accordingly.
		if (entry_factor > exit_factor) {
			max_entry_speed = max_allowable_speed(-meth->settings.acceleration,current->nominal_speed*exit_factor, 
				current->millimeters);
			max_entry_factor = max_entry_speed/current->nominal_speed;
			if (max_entry_factor < entry_factor) {
				entry_factor = max_entry_factor;
			}
		}    
	} else {
		entry_factor = factor_for_safe_speed(meth,current);
	}
		
	// Store result
	current->entry_factor = entry_factor;
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass(GRBL_METH *meth) {
	int8_t block_index = meth->block_buffer_head;
	block_t *block[3] = {NULL, NULL, NULL};
	while(block_index != meth->block_buffer_tail) {    
		block_index--;
		if(block_index < 0) {
			block_index = BLOCK_BUFFER_SIZE-1;
		}
		block[2]= block[1];
		block[1]= block[0];
		block[0] = &(meth->block_buffer[block_index]);
		planner_reverse_pass_kernel(meth,block[0], block[1], block[2]);
	}
	planner_reverse_pass_kernel(meth,NULL, block[0], block[1]);
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(GRBL_METH *meth,block_t *previous, block_t *current, block_t *next) {
	if(!current) { return; }
	if(previous) {
		// If the previous block is an acceleration block, but it is not long enough to 
		// complete the full speed change within the block, we need to adjust out entry
		// speed accordingly. Remember current->entry_factor equals the exit factor of 
		// the previous block.
		if(previous->entry_factor < current->entry_factor) {
			double max_entry_speed = max_allowable_speed(-meth->settings.acceleration,
				current->nominal_speed*previous->entry_factor, previous->millimeters);
			double max_entry_factor = max_entry_speed/current->nominal_speed;
			if (max_entry_factor < current->entry_factor) {
				current->entry_factor = max_entry_factor;
			}
		}
	}
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
static void planner_forward_pass(GRBL_METH*meth){
	int8_t block_index = meth->block_buffer_tail;
	block_t *block[3] = {NULL, NULL, NULL};
	
	while(block_index != meth->block_buffer_head) {
		block[0] = block[1];
		block[1] = block[2];
		block[2] = &(meth->block_buffer[block_index]);
		planner_forward_pass_kernel(meth,block[0],block[1],block[2]);
		block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
	}
	planner_forward_pass_kernel(meth,block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
static void planner_recalculate_trapezoids(GRBL_METH*meth){
	int8_t block_index = meth->block_buffer_tail;
	block_t *current;
	block_t *next = NULL;
	
	while(block_index != meth->block_buffer_head) {
		current = next;
		next = &(meth->block_buffer[block_index]);
		if (current) {
			calculate_trapezoid_for_block(current, current->entry_factor, next->entry_factor);      
		}
		block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
	}
	calculate_trapezoid_for_block(next, next->entry_factor, factor_for_safe_speed(meth,next));
}

// Recalculates the motion plan according to the following 算法:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

static void planner_recalculate(GRBL_METH*meth){
	planner_reverse_pass(meth);
	planner_forward_pass(meth);
	planner_recalculate_trapezoids(meth);
}

void plan_init(GRBL_METH*meth){
	meth->block_buffer_head = 0;
	meth->block_buffer_tail = 0;
	plan_set_acceleration_manager_enabled(meth,1);
	memset(meth->position,0,sizeof(meth->position));
}

void plan_set_acceleration_manager_enabled(GRBL_METH*meth,int32_t enabled) {
	if ((!!meth->acceleration_manager_enabled) != (!!enabled)) {
		st_synchronize(meth);
		meth->acceleration_manager_enabled = !!enabled;
	}
}

int32_t plan_is_acceleration_manager_enabled(GRBL_METH*meth) {
	return(meth->acceleration_manager_enabled);
}
// 说明这个动作区块（以block_buffer_tail索引得到）使用完成，如果有更多能执行的动作区块，就更新索引
void plan_discard_current_block(GRBL_METH*meth) {
	if (meth->block_buffer_head != meth->block_buffer_tail) {
		meth->block_buffer_tail = (meth->block_buffer_tail + 1) % BLOCK_BUFFER_SIZE;  
	}
}

block_t *plan_get_current_block(GRBL_METH*meth) {
	if (meth->block_buffer_head == meth->block_buffer_tail) {
		return(NULL); 
	}
	return(&(meth->block_buffer[meth->block_buffer_tail]));
}

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
// feed_rate单位是 毫米/秒
// 大概功能是得到一条线的执行到X,Y,Z位置需要的步数
// x，y，z:单位毫米

void plan_buffer_line(GRBL_METH*meth,double x, double y, double z, double feed_rate, int32_t invert_feed_rate) {
	// The target position of the tool in absolute steps
	
	// 计算每轴上的步数
	int32_t target[3] = {0};
	int32_t next_buffer_head = 0;
	uint32_t microseconds = 0;
	double delta_x_mm = 0.0;
	double delta_y_mm = 0.0;
	double delta_z_mm = 0.0;
	double multiplier = 0.0;
	double travel_per_step = 0.0;
	double safe_speed_factor = 0.0;
	block_t *block = 0;

	target[X_AXIS] = lround(x*meth->settings.steps_per_mm[X_AXIS]);
	target[Y_AXIS] = lround(y*meth->settings.steps_per_mm[Y_AXIS]);
	target[Z_AXIS] = lround(z*meth->settings.steps_per_mm[Z_AXIS]);  //浮点约等整数   
	
	// Calculate the buffer head after we push this byte
	next_buffer_head = (meth->block_buffer_head + 1) % BLOCK_BUFFER_SIZE;	
	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Rest here until there is room in the buffer.
	while(meth->block_buffer_tail == next_buffer_head) { ; }//这是干啥用?
	// Prepare to set up new block
	block = &(meth->block_buffer[meth->block_buffer_head]);
	// Number of steps for each axis
	block->steps_x = labs(target[X_AXIS]-meth->position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-meth->position[Y_AXIS]);
	block->steps_z = labs(target[Z_AXIS]-meth->position[Z_AXIS]);
	block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));
	// Bail if this is a zero-length block
	if (block->step_event_count == 0) { return; };
	
	delta_x_mm = (target[X_AXIS]-meth->position[X_AXIS])/meth->settings.steps_per_mm[X_AXIS];
	delta_y_mm = (target[Y_AXIS]-meth->position[Y_AXIS])/meth->settings.steps_per_mm[Y_AXIS];
	delta_z_mm = (target[Z_AXIS]-meth->position[Z_AXIS])/meth->settings.steps_per_mm[Z_AXIS];
	block->millimeters = sqrt((delta_x_mm*delta_x_mm) + (delta_y_mm*delta_y_mm) + (delta_z_mm*delta_z_mm));
	
	
	
	if (!invert_feed_rate) {//计算最长完成时间（毫秒）
		microseconds = lround((block->millimeters/feed_rate)*1000000);
	} else {
		microseconds = lround(60000000.0/feed_rate);
	}

	// 计算每轴上的速度（mm/minute）
	multiplier = 60.0*1000000.0/microseconds;//得到1/分钟
	block->speed_x = delta_x_mm * multiplier;
	block->speed_y = delta_y_mm * multiplier;
	block->speed_z = delta_z_mm * multiplier; 
	block->nominal_speed = block->millimeters * multiplier;
	block->nominal_rate = ceil(block->step_event_count * multiplier);  
	block->entry_factor = 0.0;
	
	// 为梯形速度模块计算加速度. Depending on the slope of the line
	// average travel per step event changes. For a line along one axis the travel per step event
	// is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
	// axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
	// To generate trapezoids with contant acceleration between blocks the rate_delta must be computed 
	// specifically for each line to compensate for this phenomenon:
	travel_per_step = block->millimeters/block->step_event_count;//计算速度(mm/step)
	block->rate_delta = ceil(
		// ((meth->settings.acceleration*60.0)/(ACCELERATION_TICKS_PER_SECOND))/ // acceleration mm/sec/sec per acceleration_tick
		((meth->settings.acceleration*60.0)/(1000.0/ACCELERATION_TICKS_MS_PER_MS))/ // acceleration mm/sec/sec per acceleration_tick
		travel_per_step);// convert to: acceleration steps/min/acceleration_tick    
	if (meth->acceleration_manager_enabled) {
		// compute a preliminary conservative acceleration trapezoid
		safe_speed_factor = factor_for_safe_speed(meth,block);//
		calculate_trapezoid_for_block(block, safe_speed_factor, safe_speed_factor); 
	} else {
		block->initial_rate = block->nominal_rate;
		block->final_rate = block->nominal_rate;
		block->accelerate_until = 0;
		block->decelerate_after = block->step_event_count;
		block->rate_delta = 0;
	}
	
	// 得到方向
	if (target[X_AXIS] < meth->position[X_AXIS]) {
		meth->XAxisDirGo();
	}else{
		meth->XAxisDirBack();
	}
	if (target[Y_AXIS] < meth->position[Y_AXIS]) {
		meth->YAxisDirGo();
	}else{
		meth->YAxisDirBack();
	}
	if (target[Z_AXIS] < meth->position[Z_AXIS]) {
		meth->ZAxisDirGo();
	}else{
		meth->ZAxisDirBack();
	}
	
	// Move buffer head
	meth->block_buffer_head = next_buffer_head;    
	// Update position 
	memcpy(meth->position, target, sizeof(target)); // meth->position[] = target[]
	
	if (meth->acceleration_manager_enabled) { planner_recalculate(meth); }  
	meth->EnableTimeInter();
}

