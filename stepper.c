


#include "stepper.h"
#include "planner.h"
extern void plan_discard_current_block(GRBL_METH*meth);
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

// #define TICKS_PER_MICROSECOND 1//(F_CPU/1000000) //F_CPU的值是以hz为单位，该宏代表1us有多少运行周期

// #define CYCLES_PER_ACCELERATION_TICK 2//((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)//应该是代表每隔多少指令周期改变一次速度

static inline void trapezoid_generator_reset(GRBL_METH*meth);
static void set_step_events_per_minute(GRBL_METH*meth,uint32_t steps_per_minute);
// Variables used by the trapezoid generation
//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate by trapezoid_generator_tick()
//  that is called ACCELERATION_TICKS_PER_SECOND times per second.

void set_step_events_per_minute(GRBL_METH*meth,uint32_t steps_per_minute);

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
static inline void trapezoid_generator_reset(GRBL_METH*meth) {
	meth->trapezoid_adjusted_rate = current_block->initial_rate;  
	meth->trapezoid_tick_ms_counter = 0; // Always start a new trapezoid with a full acceleration tick
	set_step_events_per_minute(meth,meth->trapezoid_adjusted_rate);
}
// 用来控制定时器中断产生的速度，能控制加速-稳定速度-减速阶段的中断频率
// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
inline void trapezoid_generator_tick(GRBL_METH*meth) {     
	if (meth->current_block) {
		if (meth->step_events_completed < meth->current_block->accelerate_until) {
			meth->trapezoid_adjusted_rate += meth->current_block->rate_delta;
			if (meth->trapezoid_adjusted_rate > meth->current_block->nominal_rate ) {
				meth->trapezoid_adjusted_rate = meth->current_block->nominal_rate;
			}
			set_step_events_per_minute(meth,meth->trapezoid_adjusted_rate);
		} else if (meth->step_events_completed > meth->current_block->decelerate_after) {
			// NOTE: We will only reduce speed if the result will be > 0. This catches small
			// rounding errors that might leave steps hanging after the last trapezoid tick.
			if (meth->trapezoid_adjusted_rate > meth->current_block->rate_delta) {
				meth->trapezoid_adjusted_rate -= meth->current_block->rate_delta;
			}
			if (meth->trapezoid_adjusted_rate < meth->current_block->final_rate) {
				meth->trapezoid_adjusted_rate = meth->current_block->final_rate;
			}        
			set_step_events_per_minute(meth,meth->trapezoid_adjusted_rate);
		} else {
			// Make sure we cruise at exactly nominal rate
			if (meth->trapezoid_adjusted_rate != meth->current_block->nominal_rate) {
				meth->trapezoid_adjusted_rate = meth->current_block->nominal_rate;
				set_step_events_per_minute(meth,meth->trapezoid_adjusted_rate);
			}
		}
	}
}

// grbl驱动中断，由 config_step_timer 函数控制中断频率，从动作区块缓冲中弹出动作，然后产生步进电机管脚脉冲
// 还和一个管脚复位中断搭配（Stepper Port Reset Interrupt），借此来产生完整的脉冲
void TimeInter(GRBL_METH*meth) {        
	// If there is no current block, attempt to pop one from the buffer
	if (meth->current_block == NULL) {
		// Anything in the buffer?
		meth->current_block = plan_get_current_block(meth);
		if (meth->current_block != NULL) {
			trapezoid_generator_reset(meth);// step_event_count = 20步，steps_x = 20
			meth->st_counter_x = -(meth->current_block->step_event_count >> 1);//-10步
			meth->st_counter_y = meth->st_counter_x;
			meth->st_counter_z = meth->st_counter_x;
			meth->step_events_completed = 0;
		} else {
			meth->DisableTimeInter();
		}
	} 

	if (meth->current_block != NULL) {
		// meth->XAxisPwmL();
		// meth->YAxisPwmL();
		// meth->ZAxisPwmL();
		// 如果单独实现某一个输出引脚的值，就需要执行多次才能完成多个轴的运动，
		// 这会产生阶梯状锯齿运动轨迹，这在CNC中是不允许的。下面我们详细分析IO映射和操作。 		
		// 我觉有可能会影响
		meth->st_counter_x += meth->current_block->steps_x;//10
		if (meth->st_counter_x > 0) {
			meth->XAxisPwmH();
			meth->st_counter_x -= meth->current_block->step_event_count;//10-20=-10
		}
		meth->st_counter_y += meth->current_block->steps_y;
		if (meth->st_counter_y > 0) {
			meth->YAxisPwmH();
			meth->st_counter_y -= meth->current_block->step_event_count;
		}
		meth->st_counter_z += meth->current_block->steps_z;
		if (meth->st_counter_z > 0) {
			meth->ZAxisPwmH();
			meth->st_counter_z -= meth->current_block->step_event_count;
		}
		// If current block is finished, reset pointer 
		meth->step_events_completed += 1;
		if (meth->step_events_completed >= meth->current_block->step_event_count) {
			meth->current_block = NULL;//说明这个区块的的步数已经用完，动作区块
			plan_discard_current_block();
		}
	} else {
		meth->XAxisPwmL();
		meth->YAxisPwmL();
		meth->ZAxisPwmL();
	}          

	// 代码假设定时器中断触发的频率远高于trapezoid_generator_tick触发的频率，
	meth->trapezoid_tick_ms_counter += meth->ms_per_step_event;//实际中断时间相加，单位ms
	if(meth->trapezoid_tick_ms_counter > ACCELERATION_TICKS_MS_PER_MS) {//经过一个加速tick的运行周期之后，改变一次中断频率
		meth->trapezoid_tick_ms_counter -= ACCELERATION_TICKS_MS_PER_MS;
		trapezoid_generator_tick(meth);//用于控制速度
	}
}
// io拉低的中断，
void TimeInterComp(GRBL_METH*meth){
	meth->XAxisPwmL();
	meth->YAxisPwmL();
	meth->ZAxisPwmL();
}
// This interrupt is set up by SIG_OUTPUT_COMPARE1A when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
// SIGNAL(TIMER2_OVF_vect)
// {
//   // reset stepping pins (leave the direction pins)
//   STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
// }

// Block until all buffered steps are executed
void st_synchronize()
{
	while(plan_get_current_block()) { ; }    
}

// 配置每隔多少次计数周期来一次定时器1中断，返回实际时间上最接近的计数周期数量
// 如果改成设置中断频率可以吗？
// uint32_t config_step_timer(uint32_t cycles){
// 	return(actual_cycles);
// }
// 设置每分钟有多少个定时器中断？，是的，然后会在中断中产生一次步进电机的脉冲输出
static void set_step_events_per_minute(GRBL_METH*meth,uint32_t steps_per_minute) {
	if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) {
		steps_per_minute = MINIMUM_STEPS_PER_MINUTE; 
	}
	// meth->ms_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*60)/steps_per_minute);
	// 每分钟有多少tick/每分钟有多少步 得到 每一步有多少tick

	meth->ms_per_step_event = meth->SetTimeInterMs(steps_per_minute/60);
}
void st_go_home(){
	// Todo: Perform the homing cycle
}
