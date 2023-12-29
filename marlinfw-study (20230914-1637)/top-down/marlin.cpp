#include "marlin.h"
#include <math.h>
#include <algorithm> // min()
using namespace std;

MarlinState marlin_state = MF_INITIALIZING;
bool IsRunning() { return marlin_state >= MF_RUNNING; }
bool IsStopped() { return marlin_state == MF_STOPPED; }



// planner
Planner planner;

block_t Planner::block_buffer[BLOCK_BUFFER_SIZE];
volatile uint8_t Planner::block_buffer_head,    // Index of the next block to be pushed
Planner::block_buffer_nonbusy, // Index of the first non-busy block
Planner::block_buffer_planned, // Index of the optimally planned block
Planner::block_buffer_tail;    // Index of the busy block, if any
uint16_t Planner::cleaning_buffer_counter;      // A counter to disable queuing of blocks
uint8_t Planner::delay_before_delivering;       // This counter delays delivery of blocks when queue becomes empty to allow the opportunity of merging blocks

planner_settings_t Planner::settings;           // Initialized by settings.load()
xyze_long_t Planner::position{ 0 };
xyze_float_t Planner::previous_speed;
float Planner::previous_nominal_speed;

Planner::Planner() { init(); }
void Planner::init() {
    float temp[NUM_AXES] = DEFAULT_AXIS_STEPS_PER_UNIT;
    settings.axis_steps_per_mm[0] = temp[0];
    settings.axis_steps_per_mm[1] = temp[1];
    settings.axis_steps_per_mm[2] = temp[2];
    position.reset();
    previous_speed.reset();
    previous_nominal_speed = 0;
    clear_block_buffer();
    delay_before_delivering = 0;
}
void Planner::set_position_mm(const xyze_pos_t &pos) {
    xyze_pos_t machine = pos;
    set_machine_position_mm(machine);
}
void Planner::set_machine_position_mm(const abce_pos_t & pos) {
    position.set(
        lroundf(pos.x * settings.axis_steps_per_mm[X_AXIS]),
        lroundf(pos.y * settings.axis_steps_per_mm[Y_AXIS]),
        lroundf(pos.z * settings.axis_steps_per_mm[Z_AXIS]));
}



// motion
extern xyze_pos_t current_position = { 0, 1, 2 };;
extern xyz_pos_t home_offset{ X_HOME_POS, Y_HOME_POS, Z_HOME_POS };



void sync_plan_position(void) {
	planner.set_position_mm(current_position);
}



// stepper
Stepper stepper; // Singleton

// public
stepper_flags_t Stepper::axis_enabled; // {0}

// private
block_t* Stepper::current_block; // (= nullptr) A pointer to the block currently being traced
axis_bits_t Stepper::last_direction_bits, // = 0
            Stepper::axis_did_move; // = 0
bool Stepper::abort_current_block;
uint32_t Stepper::acceleration_time, Stepper::deceleration_time;
uint8_t Stepper::steps_per_isr;
//constexpr uint8_t Stepper::oversampling_factor;
xyze_long_t Stepper::delta_error{ 0 };
xyze_long_t Stepper::advance_dividend{ 0 };
uint32_t Stepper::advance_divisor = 0,
        Stepper::step_events_completed = 0, // The number of step events executed in the current block
        Stepper::accelerate_until,          // The count at which to stop accelerating
        Stepper::decelerate_after,          // The count at which to start decelerating
        Stepper::step_event_count;          // The total event count for the current block
int32_t Stepper::ticks_nominal = -1;
uint32_t Stepper::acc_step_rate; // needed for deceleration start point
xyze_long_t Stepper::endstops_trigsteps;
xyze_long_t Stepper::count_position{ 0 };
xyze_int8_t Stepper::count_direction{ 0 };

void Stepper::init() {
    // init dir and enable pins
    // Init Step Pins
    HAL_timer_start(MF_TIMER_STEP, 122); // Init Stepper ISR to 122 Hz for quick starting
    //wake_up();
    //sei();
    set_directions(0x07); // Init direction bits for first moves
}//init()
/*
#define SET_STEP_DIR(A)                       \
  if (motor_direction(_AXIS(A))) {            \
    A##_APPLY_DIR(INVERT_##A##_DIR, false);   \
    count_direction[_AXIS(A)] = -1;           \
  }                                           \
  else {                                      \
    A##_APPLY_DIR(!INVERT_##A##_DIR, false);  \
    count_direction[_AXIS(A)] = 1;            \
  }
*/
void Stepper::SET_STEP_DIR(const AxisEnum axis) {
}
/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREXZ: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 *   COREYZ: Y_AXIS=B_AXIS and Z_AXIS=C_AXIS
 */
void Stepper::set_directions() {
    /*
    DIR_WAIT_BEFORE();
    TERN_(HAS_X_DIR, SET_STEP_DIR(X)); // A
    TERN_(HAS_Y_DIR, SET_STEP_DIR(Y)); // B
    TERN_(HAS_Z_DIR, SET_STEP_DIR(Z)); // C
    DIR_WAIT_AFTER();
    */
}//set_directions()


void Stepper::isr() {
    static uint32_t nextMainISR = 0;  // Interval until the next main Stepper Pulse phase (0 = Now)
#ifndef __AVR__
    // Disable interrupts, to avoid ISR preemption while we reprogram the period
    // (AVR enters the ISR with global interrupts disabled, so no need to do it here)
    //hal.isr_off();
#endif
    // Program timer compare for the maximum period, so it does NOT
    // flag an interrupt while this ISR is running - So changes from small
    // periods to big periods are respected and the timer does not reset to 0
    HAL_timer_set_compare(MF_TIMER_STEP, hal_timer_t(HAL_TIMER_TYPE_MAX));

    hal_timer_t next_isr_ticks = 0; // Count of ticks for the next ISR
    uint8_t max_loops = 10; // Limit the amount of iterations
    hal_timer_t min_ticks;
    do {
        //hal.isr_on(); // Enable ISRs to reduce USART processing latency
        if (!nextMainISR) pulse_phase_isr(); // 0 = Do coordinated axes Stepper pulses

        // ^== Time critical. NOTHING besides pulse generation should be above here!!!

        if (!nextMainISR) nextMainISR = block_phase_isr();  // Manage acc/deceleration, get next block

        // Get the interval to the next ISR call
        const uint32_t interval = min(uint32_t(HAL_TIMER_TYPE_MAX), nextMainISR);

        // Compute remaining time for each ISR phase
        //     NEVER : The phase is idle
        //      Zero : The phase will occur on the next ISR call
        //  Non-zero : The phase will occur on a future ISR call
        nextMainISR -= interval;

        /**
         * This needs to avoid a race-condition caused by interleaving
         * of interrupts required by both the LA and Stepper algorithms.
         *
         * Assume the following tick times for stepper pulses:
         *   Stepper ISR (S):  1 1000 2000 3000 4000
         *   Linear Adv. (E): 10 1010 2010 3010 4010
         *
         * The current algorithm tries to interleave them, giving:
         *  1:S 10:E 1000:S 1010:E 2000:S 2010:E 3000:S 3010:E 4000:S 4010:E
         *
         * Ideal timing would yield these delta periods:
         *  1:S  9:E  990:S   10:E  990:S   10:E  990:S   10:E  990:S   10:E
         *
         * But, since each event must fire an ISR with a minimum duration, the
         * minimum delta might be 900, so deltas under 900 get rounded up:
         *  900:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E
         *
         * It works, but divides the speed of all motors by half, leading to a sudden
         * reduction to 1/2 speed! Such jumps in speed lead to lost steps (not even
         * accounting for double/quad stepping, which makes it even worse).
         */
         // Compute the tick count for the next ISR
        next_isr_ticks += interval;
        /**
         * The following section must be done with global interrupts disabled.
         * We want nothing to interrupt it, as that could mess the calculations
         * we do for the next value to program in the period register of the
         * stepper timer and lead to skipped ISRs (if the value we happen to program
         * is less than the current count due to something preempting between the
         * read and the write of the new period value).
         */
        //hal.isr_off();

        /**
         * Get the current tick value + margin
         * Assuming at least 6µs between calls to this ISR...
         * On AVR the ISR epilogue+prologue is estimated at 100 instructions - Give 8µs as margin
         * On ARM the ISR epilogue+prologue is estimated at 20 instructions - Give 1µs as margin
         */
        hal_timer_t margin = STEPPER_TIMER_TICKS_PER_US;
#ifdef __AVR__
        margin = 8 * STEPPER_TIMER_TICKS_PER_US;
#endif
        min_ticks = HAL_timer_get_count(MF_TIMER_STEP) + margin;

        /**
         * NB: If for some reason the stepper monopolizes the MPU, eventually the
         * timer will wrap around (and so will 'next_isr_ticks'). So, limit the
         * loop to 10 iterations. Beyond that, there's no way to ensure correct pulse
         * timing, since the MCU isn't fast enough.
         */
        if (!--max_loops) next_isr_ticks = min_ticks;

        // Advance pulses if not enough time to wait for the next ISR
    } while (next_isr_ticks < min_ticks);

    // Now 'next_isr_ticks' contains the period to the next Stepper ISR - And we are
    // sure that the time has not arrived yet - Warrantied by the scheduler

    // Set the next ISR to fire at the proper time
    HAL_timer_set_compare(MF_TIMER_STEP, hal_timer_t(next_isr_ticks));

    // Don't forget to finally reenable interrupts on non-AVR.
    // AVR automatically calls sei() for us on Return-from-Interrupt.
#ifndef __AVR__
    //hal.isr_on();
#endif
}//isr

// optimized math for AVR
#define  MultiU24X32toH16(A, B) (A*B)
#ifdef CPU_32_BIT
#define STEP_MULTIPLY(A,B) MultiU32X24toH32(A, B)
#else
#define STEP_MULTIPLY(A,B) MultiU24X32toH16(A, B)
#endif

/**
 * This phase of the ISR should ONLY create the pulses for the steppers.
 * This prevents jitter caused by the interval between the start of the
 * interrupt and the start of the pulses. DON'T add any logic ahead of the
 * call to this method that might cause variation in the timing. The aim
 * is to keep pulse timing as regular as possible.
 */
void Stepper::pulse_phase_isr() {
    // If we must abort the current block, do so!
    if (abort_current_block) {
        abort_current_block = false;
        if (current_block) {
            discard_current_block();
        }
    }
    // If there is no current block, do nothing
    if (!current_block || step_events_completed >= step_event_count) return;
    // Count of pending loops and events for this iteration
    const uint32_t pending_events = step_event_count - step_events_completed;
    uint8_t events_to_do = min(pending_events, (uint32_t)steps_per_isr);
    // Just update the value we will get at the end of the loop
    step_events_completed += events_to_do;
    //
    xyze_bool_t step_needed{ 0 };
    do {
#define _APPLY_STEP(AXIS, INV, ALWAYS) AXIS ##_APPLY_STEP(INV, ALWAYS)
#define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

    // Start an active pulse if needed


        // Determine if a pulse is needed using Bresenham, PULSE_PREP
        for (int i = 0; i<3; i++) {
            delta_error[i] += advance_dividend[i];
            step_needed[i] = (delta_error[i] >= 0);
            if (step_needed[i]) delta_error[i] -= advance_divisor;
        }
        // Start an active pulse if needed, PULSE_START
        for (int i = 0; i < 3; i++) {
            if (step_needed[i]) {
                //_APPLY_STEP(AXIS, _INVERT_STEP_PIN(AXIS), 0);
            }
        }
        // Stop an active pulse if needed, PULSE_STOP
        for (int i = 0; i < 3; i++) {
            if (step_needed[i]) {
                //_APPLY_STEP(AXIS, _INVERT_STEP_PIN(AXIS), 0);
            }
        }
    } while (--events_to_do);
}//pulse_phase_isr()

// This is the last half of the stepper interrupt: This one processes and
// properly schedules blocks from the planner. This is executed after creating
// the step pulses, so it is not time critical, as pulses are already done.
uint32_t Stepper::block_phase_isr() {
    // If no queued movements, just wait 1ms for the next block
    uint32_t interval = (STEPPER_TIMER_RATE) / 1000UL;
    // If there is a current block
    if (current_block) {
        // If current block is finished, reset pointer and finalize state
        if (step_events_completed >= step_event_count) {
            discard_current_block();
        }
        else {
            // Step events not completed yet...

            // Are we in acceleration phase ?
            if (step_events_completed <= accelerate_until) { // Calculate new timer value
                acc_step_rate = STEP_MULTIPLY(acceleration_time, current_block->acceleration_rate)
                              + current_block->initial_rate;
                if (acc_step_rate > current_block->nominal_rate)
                    acc_step_rate = current_block->nominal_rate;

                // acc_step_rate is in steps/second

                // step_rate to timer interval and steps per stepper isr
                interval = calc_timer_interval(acc_step_rate << oversampling_factor, steps_per_isr);
                acceleration_time += interval;
            }
            // Are we in Deceleration phase ?
            else if (step_events_completed > decelerate_after) {
                uint32_t step_rate;
                // Using the old trapezoidal control
                step_rate = STEP_MULTIPLY(deceleration_time, current_block->acceleration_rate);
                if (step_rate < acc_step_rate) { // Still decelerating?
                    step_rate = acc_step_rate - step_rate;
                    if (step_rate > current_block->final_rate)
                        step_rate = current_block->final_rate;
                }
                else
                    step_rate = current_block->final_rate;
                // step_rate to timer interval and steps per stepper isr
                interval = calc_timer_interval(step_rate << oversampling_factor, steps_per_isr);
                deceleration_time += interval;
            }
            else {  // Must be in cruise phase otherwise
                // Calculate the ticks_nominal for this nominal speed, if not done yet
                if (ticks_nominal < 0) {
                    // step_rate to timer interval and loops for the nominal speed
                    ticks_nominal = calc_timer_interval(current_block->nominal_rate << oversampling_factor, steps_per_isr);
                }
                // The timer interval is just the nominal value for the nominal speed
                interval = ticks_nominal;
            }
        }
    }
    else { // !current_block
    }

    // If there is no current block at this point, attempt to pop one from the buffer
    // and prepare its movement
    if (!current_block) {
        // Anything in the buffer?
        if ((current_block = planner.get_current_block())) {
            // Sync block? Sync the stepper counts or fan speeds and return
            while (current_block->is_sync()) {
                TERN_(LASER_SYNCHRONOUS_M106_M107, if (current_block->is_fan_sync()) planner.sync_fan_speeds(current_block->fan_speed));

                if (!(current_block->is_fan_sync() || current_block->is_pwr_sync())) _set_position(current_block->position);

                discard_current_block();

                // Try to get a new block
                if (!(current_block = planner.get_current_block()))
                    return interval; // No more queued movements!
            }

            // For non-inline cutter, grossly apply power

            // Flag all moving axes for proper endstop handling
            axis_bits_t axis_bits = 0;
            if (!!current_block->steps.a) SBI(axis_bits, A_AXIS);
            if (!!current_block->steps.b) SBI(axis_bits, B_AXIS);
            if (!!current_block->steps.c) SBI(axis_bits, C_AXIS);
            axis_did_move = axis_bits;

            // No acceleration / deceleration time elapsed so far
            acceleration_time = deceleration_time = 0;

            // Based on the oversampling factor, do the calculations
            step_event_count = current_block->step_event_count << oversampling_factor;

            // Initialize Bresenham delta errors to 1/2
            delta_error = -int32_t(step_event_count);

            // Calculate Bresenham dividends and divisors
            advance_dividend = (current_block->steps << 1).asLong();
            advance_divisor = step_event_count << 1;

            // No step events completed so far
            step_events_completed = 0;

            // Compute the acceleration and deceleration points
            accelerate_until = current_block->accelerate_until << oversampling_factor;
            decelerate_after = current_block->decelerate_after << oversampling_factor;

            // Initialize the trapezoid generator from the current block.
            if (current_block->direction_bits != last_direction_bits)
                set_directions(current_block->direction_bits);
            }
            // If the endstop is already pressed, endstop interrupts won't invoke
            // endstop_triggered and the move will grind. So check here for a
            // triggered endstop, which marks the block for discard on the next ISR.
            //endstops.update();

            // Mark the time_nominal as not calculated yet
            ticks_nominal = -1;

            // Set as deceleration point the initial rate of the block
            acc_step_rate = current_block->initial_rate;

            // Calculate the initial timer interval
            interval = calc_timer_interval(current_block->initial_rate << oversampling_factor, steps_per_isr);
            acceleration_time += interval;
        }
    }
    // Return the interval to wait
    return interval;
}//block_phase_isr()



// timer
uint32_t TIMSK1, OCR1A, OCR0A, TNCT1, TNCT0;
void HAL_timer_start(const uint8_t timer_num, const uint32_t) {
    switch (timer_num) {
    case MF_TIMER_STEP:
        // timer 1, 122 Hz
        break;
    case MF_TIMER_TEMP:
        // timer 0, ? Hz
        // Use timer0 for temperature measurement
        // Interleave temperature interrupt with millies interrupt
        break;
    }
}//HAL_timer_start()
void HAL_timer_set_compare(const uint8_t timer, const uint32_t compare) {
    if (timer == MF_TIMER_STEP) OCR1A = compare;
    else if (timer == MF_TIMER_TEMP) OCR0A = compare;
}//HAL_timer_set_compare()
uint32_t HAL_timer_get_compare(const uint8_t timer) {
    if (timer == MF_TIMER_STEP) return OCR1A;
    else if (timer == MF_TIMER_TEMP) return OCR0A;
    return 0;
}//HAL_timer_get_compare()
uint32_t HAL_timer_get_count(const uint8_t timer) {
    if (timer == MF_TIMER_STEP) return OCR1A;
    else if (timer == MF_TIMER_TEMP) return OCR0A;
    return 0;
}//HAL_timer_get_count()



//marlin.cpp

