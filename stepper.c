/*
	stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
	 and Philipp Tiefenbacher. */

#include "stepper.h"
#include "planner.h"
extern void plan_discard_current_block();
extern block_t *plan_get_current_block();
// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#define TICKS_PER_MICROSECOND 1//(F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK 2//((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

#define MINIMUM_STEPS_PER_MINUTE 1200 // The stepper subsystem will never run slower than this, exept when sleeping

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  //TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() //TIMSK1 &= ~(1<<OCIE1A)


// Variables used by The Stepper Driver Interrupt
static uint8_t out_bits;        // The next stepping-bits to be output

static volatile int32_t busy; // 1 when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

// Variables used by the trapezoid generation
static uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
static uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
																							// pace without allocating a separate timer
static uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator

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

void set_step_events_per_minute(uint32_t steps_per_minute);

void st_wake_up() {
	// ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
inline void trapezoid_generator_reset() {
	trapezoid_adjusted_rate = current_block->initial_rate;  
	trapezoid_tick_cycle_counter = 0; // Always start a new trapezoid with a full acceleration tick
	set_step_events_per_minute(trapezoid_adjusted_rate);
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
inline void trapezoid_generator_tick(GRBL_METH*meth) {     
	if (current_block) {
		if (meth->step_events_completed < current_block->accelerate_until) {
			trapezoid_adjusted_rate += current_block->rate_delta;
			if (trapezoid_adjusted_rate > current_block->nominal_rate ) {
				trapezoid_adjusted_rate = current_block->nominal_rate;
			}
			set_step_events_per_minute(trapezoid_adjusted_rate);
		} else if (meth->step_events_completed > current_block->decelerate_after) {
			// NOTE: We will only reduce speed if the result will be > 0. This catches small
			// rounding errors that might leave steps hanging after the last trapezoid tick.
			if (trapezoid_adjusted_rate > current_block->rate_delta) {
				trapezoid_adjusted_rate -= current_block->rate_delta;
			}
			if (trapezoid_adjusted_rate < current_block->final_rate) {
				trapezoid_adjusted_rate = current_block->final_rate;
			}        
			set_step_events_per_minute(trapezoid_adjusted_rate);
		} else {
			// Make sure we cruise at exactly nominal rate
			if (trapezoid_adjusted_rate != current_block->nominal_rate) {
				trapezoid_adjusted_rate = current_block->nominal_rate;
				set_step_events_per_minute(trapezoid_adjusted_rate);
			}
		}
	}
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is  executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse.
void PwmInter(GRBL_METH*meth) {        
	// TODO: Check if the busy-flag can be eliminated by just disabeling this interrupt while we are in it
	
	if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt

	busy = 1;
	// sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
				 // ((We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
				 // at exactly the right time even if we occasionally spend a lot of time inside this handler.))
		
	// If there is no current block, attempt to pop one from the buffer
	if (current_block == NULL) {
		// Anything in the buffer?
		current_block = plan_get_current_block();
		if (current_block != NULL) {
			trapezoid_generator_reset();// step_event_count = 20步，steps_x = 20
			meth->st_counter_x = -(current_block->step_event_count >> 1);//-10步
			meth->st_counter_y = meth->st_counter_x;
			meth->st_counter_z = meth->st_counter_x;
			meth->step_events_completed = 0;
		} else {
			DISABLE_STEPPER_DRIVER_INTERRUPT();
		}    
	} 

	if (current_block != NULL) {
		out_bits = current_block->direction_bits;
		meth->XAxisPwmL();
		meth->YAxisPwmL();
		meth->ZAxisPwmL();
		meth->st_counter_x += current_block->steps_x;//10
		if (meth->st_counter_x > 0) {
			meth->XAxisPwmH();
			meth->st_counter_x -= current_block->step_event_count;//10-20=-10
		}
		meth->st_counter_y += current_block->steps_y;
		if (meth->st_counter_y > 0) {
			meth->YAxisPwmH();
			meth->st_counter_y -= current_block->step_event_count;
		}
		meth->st_counter_z += current_block->steps_z;
		if (meth->st_counter_z > 0) {
			meth->ZAxisPwmH();
			meth->st_counter_z -= current_block->step_event_count;
		}
		// If current block is finished, reset pointer 
		meth->step_events_completed += 1;
		if (meth->step_events_completed >= current_block->step_event_count) {
			current_block = NULL;
			plan_discard_current_block();
		}
	} else {
		out_bits = 0;
		meth->XAxisPwmL();
		meth->XAxisDir_L();
		meth->YAxisPwmL();
		meth->YAxisDir_L();
		meth->ZAxisPwmL();
		meth->ZAxisDir_L();
	}          
	out_bits ^= settings.invert_mask;
	
	// In average this generates a trapezoid_generator_tick every CYCLES_PER_ACCELERATION_TICK by keeping track
	// of the number of elapsed cycles. The code assumes that step_events occur significantly more often than
	// trapezoid_generator_ticks as they well should. 
	trapezoid_tick_cycle_counter += cycles_per_step_event;
	if(trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
		trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
		trapezoid_generator_tick();
	}
	
	busy=0;
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

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
uint32_t config_step_timer(uint32_t cycles)
{
	uint16_t ceiling;
	uint16_t prescaler;
	uint32_t actual_cycles = 0;
	// if (cycles <= 0xffffL) {
	// 	ceiling = cycles;
 //    prescaler = 0; // prescaler: 0
 //    actual_cycles = ceiling;
	// } else if (cycles <= 0x7ffffL) {
 //    ceiling = cycles >> 3;
 //    prescaler = 1; // prescaler: 8
 //    actual_cycles = ceiling * 8L;
	// } else if (cycles <= 0x3fffffL) {
	// 	ceiling =  cycles >> 6;
 //    prescaler = 2; // prescaler: 64
 //    actual_cycles = ceiling * 64L;
	// } else if (cycles <= 0xffffffL) {
	// 	ceiling =  (cycles >> 8);
 //    prescaler = 3; // prescaler: 256
 //    actual_cycles = ceiling * 256L;
	// } else if (cycles <= 0x3ffffffL) {
	// 	ceiling = (cycles >> 10);
 //    prescaler = 4; // prescaler: 1024
 //    actual_cycles = ceiling * 1024L;    
	// } else {
	//   // Okay, that was slower than we actually go. Just set the slowest speed
	// 	ceiling = 0xffff;
 //    prescaler = 4;
 //    actual_cycles = 0xffff * 1024;
	// }
	// // Set prescaler
 //  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | ((prescaler+1)<<CS10);
 //  // Set ceiling
 //  OCR1A = ceiling;
	return(actual_cycles);
}

void set_step_events_per_minute(uint32_t steps_per_minute) {
	if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
	cycles_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*60)/steps_per_minute);
	// 每分钟有多少tick/每分钟有多少步 得到 每一步有多少tick
}

void st_go_home()
{
	// Todo: Perform the homing cycle
}
