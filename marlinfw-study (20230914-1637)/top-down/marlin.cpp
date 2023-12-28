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

// private
axis_bits_t Stepper::last_direction_bits, // = 0
Stepper::axis_did_move; // = 0

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
        hal.isr_off();

        /**
         * Get the current tick value + margin
         * Assuming at least 6µs between calls to this ISR...
         * On AVR the ISR epilogue+prologue is estimated at 100 instructions - Give 8µs as margin
         * On ARM the ISR epilogue+prologue is estimated at 20 instructions - Give 1µs as margin
         */
        min_ticks = HAL_timer_get_count(MF_TIMER_STEP) + hal_timer_t(TERN(__AVR__, 8, 1) * (STEPPER_TIMER_TICKS_PER_US));

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
    hal.isr_on();
#endif
}//isr



// timer
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

int OCR1A, OCR0A;
int TNCT1, TNCT0;
void HAL_timer_set_compare(int timer, int compare) {
    if (timer == MF_TIMER_STEP) {
        OCR1A = compare;
    }
    else if (timer == MF_TIMER_TEMP) {
        OCR0A = compare;
    }
}//HAL_timer_set_compare()


//marlin.cpp

