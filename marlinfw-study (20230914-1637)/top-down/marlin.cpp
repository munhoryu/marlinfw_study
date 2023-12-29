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
    // Direct Stepping page?
    const bool is_page = current_block->is_page();

    do {
#define _APPLY_STEP(AXIS, INV, ALWAYS) AXIS ##_APPLY_STEP(INV, ALWAYS)
#define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

        // Determine if a pulse is needed using Bresenham
#define PULSE_PREP(AXIS) do{ \
      delta_error[_AXIS(AXIS)] += advance_dividend[_AXIS(AXIS)]; \
      step_needed[_AXIS(AXIS)] = (delta_error[_AXIS(AXIS)] >= 0); \
      if (step_needed[_AXIS(AXIS)]) \
        delta_error[_AXIS(AXIS)] -= advance_divisor; \
    }while(0)

    // With input shaping, direction changes can happen with almost only
    // AWAIT_LOW_PULSE() and  DIR_WAIT_BEFORE() between steps. To work around
    // the TMC2208 / TMC2225 shutdown bug (#16076), add a half step hysteresis
    // in each direction. This results in the position being off by half an
    // average half step during travel but correct at the end of each segment.
#if AXIS_DRIVER_TYPE_X(TMC2208) || AXIS_DRIVER_TYPE_X(TMC2208_STANDALONE) || \
        AXIS_DRIVER_TYPE_X(TMC5160) || AXIS_DRIVER_TYPE_X(TMC5160_STANDALONE)
#define HYSTERESIS_X 64
#else
#define HYSTERESIS_X 0
#endif
#if AXIS_DRIVER_TYPE_Y(TMC2208) || AXIS_DRIVER_TYPE_Y(TMC2208_STANDALONE) || \
        AXIS_DRIVER_TYPE_Y(TMC5160) || AXIS_DRIVER_TYPE_Y(TMC5160_STANDALONE)
#define HYSTERESIS_Y 64
#else
#define HYSTERESIS_Y 0
#endif
#define _HYSTERESIS(AXIS) HYSTERESIS_##AXIS
#define HYSTERESIS(AXIS) _HYSTERESIS(AXIS)

#define PULSE_PREP_SHAPING(AXIS, DELTA_ERROR, DIVIDEND) do{ \
      if (step_needed[_AXIS(AXIS)]) { \
        DELTA_ERROR += (DIVIDEND); \
        if ((MAXDIR(AXIS) && DELTA_ERROR <= -(64 + HYSTERESIS(AXIS))) || (MINDIR(AXIS) && DELTA_ERROR >= (64 + HYSTERESIS(AXIS)))) { \
          { USING_TIMED_PULSE(); START_TIMED_PULSE(); AWAIT_LOW_PULSE(); } \
          TBI(last_direction_bits, _AXIS(AXIS)); \
          DIR_WAIT_BEFORE(); \
          SET_STEP_DIR(AXIS); \
          DIR_WAIT_AFTER(); \
        } \
        step_needed[_AXIS(AXIS)] = DELTA_ERROR <= -(64 + HYSTERESIS(AXIS)) || DELTA_ERROR >= (64 + HYSTERESIS(AXIS)); \
        if (step_needed[_AXIS(AXIS)]) \
          DELTA_ERROR += MAXDIR(AXIS) ? -128 : 128; \
      } \
    }while(0)

    // Start an active pulse if needed
#define PULSE_START(AXIS) do{ \
      if (step_needed[_AXIS(AXIS)]) { \
        count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        _APPLY_STEP(AXIS, !_INVERT_STEP_PIN(AXIS), 0); \
      } \
    }while(0)

    // Stop an active pulse if needed
#define PULSE_STOP(AXIS) do { \
      if (step_needed[_AXIS(AXIS)]) { \
        _APPLY_STEP(AXIS, _INVERT_STEP_PIN(AXIS), 0); \
      } \
    }while(0)

#if ENABLED(DIRECT_STEPPING)
  // Direct stepping is currently not ready for HAS_I_AXIS
        if (is_page) {

#if STEPPER_PAGE_FORMAT == SP_4x4D_128

#define PAGE_SEGMENT_UPDATE(AXIS, VALUE) do{   \
                 if ((VALUE) <  7) SBI(dm, _AXIS(AXIS)); \
            else if ((VALUE) >  7) CBI(dm, _AXIS(AXIS)); \
            page_step_state.sd[_AXIS(AXIS)] = VALUE;     \
            page_step_state.bd[_AXIS(AXIS)] += VALUE;    \
          }while(0)

#define PAGE_PULSE_PREP(AXIS) do{ \
            step_needed[_AXIS(AXIS)] =      \
              pgm_read_byte(&segment_table[page_step_state.sd[_AXIS(AXIS)]][page_step_state.segment_steps & 0x7]); \
          }while(0)

            switch (page_step_state.segment_steps) {
            case DirectStepping::Config::SEGMENT_STEPS:
                page_step_state.segment_idx += 2;
                page_step_state.segment_steps = 0;
                // fallthru
            case 0: {
                const uint8_t low = page_step_state.page[page_step_state.segment_idx],
                    high = page_step_state.page[page_step_state.segment_idx + 1];
                axis_bits_t dm = last_direction_bits;

                PAGE_SEGMENT_UPDATE(X, low >> 4);
                PAGE_SEGMENT_UPDATE(Y, low & 0xF);
                PAGE_SEGMENT_UPDATE(Z, high >> 4);
                PAGE_SEGMENT_UPDATE(E, high & 0xF);

                if (dm != last_direction_bits)
                    set_directions(dm);

            } break;

            default: break;
            }

            PAGE_PULSE_PREP(X);
            PAGE_PULSE_PREP(Y);
            PAGE_PULSE_PREP(Z);
            TERN_(HAS_EXTRUDERS, PAGE_PULSE_PREP(E));

            page_step_state.segment_steps++;

#elif STEPPER_PAGE_FORMAT == SP_4x2_256

#define PAGE_SEGMENT_UPDATE(AXIS, VALUE) \
            page_step_state.sd[_AXIS(AXIS)] = VALUE; \
            page_step_state.bd[_AXIS(AXIS)] += VALUE;

#define PAGE_PULSE_PREP(AXIS) do{ \
            step_needed[_AXIS(AXIS)] =      \
              pgm_read_byte(&segment_table[page_step_state.sd[_AXIS(AXIS)]][page_step_state.segment_steps & 0x3]); \
          }while(0)

            switch (page_step_state.segment_steps) {
            case DirectStepping::Config::SEGMENT_STEPS:
                page_step_state.segment_idx++;
                page_step_state.segment_steps = 0;
                // fallthru
            case 0: {
                const uint8_t b = page_step_state.page[page_step_state.segment_idx];
                PAGE_SEGMENT_UPDATE(X, (b >> 6) & 0x3);
                PAGE_SEGMENT_UPDATE(Y, (b >> 4) & 0x3);
                PAGE_SEGMENT_UPDATE(Z, (b >> 2) & 0x3);
                PAGE_SEGMENT_UPDATE(E, (b >> 0) & 0x3);
            } break;
            default: break;
            }

            PAGE_PULSE_PREP(X);
            PAGE_PULSE_PREP(Y);
            PAGE_PULSE_PREP(Z);
            TERN_(HAS_EXTRUDERS, PAGE_PULSE_PREP(E));

            page_step_state.segment_steps++;

#elif STEPPER_PAGE_FORMAT == SP_4x1_512

#define PAGE_PULSE_PREP(AXIS, BITS) do{             \
            step_needed[_AXIS(AXIS)] = (steps >> BITS) & 0x1; \
            if (step_needed[_AXIS(AXIS)])                     \
              page_step_state.bd[_AXIS(AXIS)]++;              \
          }while(0)

            uint8_t steps = page_step_state.page[page_step_state.segment_idx >> 1];
            if (page_step_state.segment_idx & 0x1) steps >>= 4;

            PAGE_PULSE_PREP(X, 3);
            PAGE_PULSE_PREP(Y, 2);
            PAGE_PULSE_PREP(Z, 1);
            PAGE_PULSE_PREP(E, 0);

            page_step_state.segment_idx++;

#else
#error "Unknown direct stepping page format!"
#endif
        }

#endif // DIRECT_STEPPING

        if (!is_page) {
            // Determine if pulses are needed
#if HAS_X_STEP
            PULSE_PREP(X);
#endif
#if HAS_Y_STEP
            PULSE_PREP(Y);
#endif
#if HAS_Z_STEP
            PULSE_PREP(Z);
#endif
#if HAS_I_STEP
            PULSE_PREP(I);
#endif
#if HAS_J_STEP
            PULSE_PREP(J);
#endif
#if HAS_K_STEP
            PULSE_PREP(K);
#endif
#if HAS_U_STEP
            PULSE_PREP(U);
#endif
#if HAS_V_STEP
            PULSE_PREP(V);
#endif
#if HAS_W_STEP
            PULSE_PREP(W);
#endif

#if EITHER(HAS_E0_STEP, MIXING_EXTRUDER)
            PULSE_PREP(E);

#if ENABLED(LIN_ADVANCE)
            if (step_needed.e && current_block->la_advance_rate) {
                // don't actually step here, but do subtract movements steps
                // from the linear advance step count
                step_needed.e = false;
                la_advance_steps--;
            }
#endif
#endif

#if HAS_SHAPING
            // record an echo if a step is needed in the primary bresenham
            const bool x_step = TERN0(INPUT_SHAPING_X, shaping_x.enabled && step_needed[X_AXIS]),
                y_step = TERN0(INPUT_SHAPING_Y, shaping_y.enabled && step_needed[Y_AXIS]);
            if (x_step || y_step)
                ShapingQueue::enqueue(x_step, TERN0(INPUT_SHAPING_X, shaping_x.forward), y_step, TERN0(INPUT_SHAPING_Y, shaping_y.forward));

            // do the first part of the secondary bresenham
#if ENABLED(INPUT_SHAPING_X)
            if (shaping_x.enabled)
                PULSE_PREP_SHAPING(X, shaping_x.delta_error, shaping_x.factor1 * (shaping_x.forward ? 1 : -1));
#endif
#if ENABLED(INPUT_SHAPING_Y)
            if (shaping_y.enabled)
                PULSE_PREP_SHAPING(Y, shaping_y.delta_error, shaping_y.factor1 * (shaping_y.forward ? 1 : -1));
#endif
#endif
        }

#if ISR_MULTI_STEPS
        if (firstStep)
            firstStep = false;
        else
            AWAIT_LOW_PULSE();
#endif

        // Pulse start
#if HAS_X_STEP
        PULSE_START(X);
#endif
#if HAS_Y_STEP
        PULSE_START(Y);
#endif
#if HAS_Z_STEP
        PULSE_START(Z);
#endif
#if HAS_I_STEP
        PULSE_START(I);
#endif
#if HAS_J_STEP
        PULSE_START(J);
#endif
#if HAS_K_STEP
        PULSE_START(K);
#endif
#if HAS_U_STEP
        PULSE_START(U);
#endif
#if HAS_V_STEP
        PULSE_START(V);
#endif
#if HAS_W_STEP
        PULSE_START(W);
#endif

#if ENABLED(MIXING_EXTRUDER)
        if (step_needed.e) {
            count_position[E_AXIS] += count_direction[E_AXIS];
            E_STEP_WRITE(mixer.get_next_stepper(), !INVERT_E_STEP_PIN);
        }
#elif HAS_E0_STEP
        PULSE_START(E);
#endif

        TERN_(I2S_STEPPER_STREAM, i2s_push_sample());

        // TODO: need to deal with MINIMUM_STEPPER_PULSE over i2s
#if ISR_MULTI_STEPS
        START_TIMED_PULSE();
        AWAIT_HIGH_PULSE();
#endif

        // Pulse stop
#if HAS_X_STEP
        PULSE_STOP(X);
#endif
#if HAS_Y_STEP
        PULSE_STOP(Y);
#endif
#if HAS_Z_STEP
        PULSE_STOP(Z);
#endif
#if HAS_I_STEP
        PULSE_STOP(I);
#endif
#if HAS_J_STEP
        PULSE_STOP(J);
#endif
#if HAS_K_STEP
        PULSE_STOP(K);
#endif
#if HAS_U_STEP
        PULSE_STOP(U);
#endif
#if HAS_V_STEP
        PULSE_STOP(V);
#endif
#if HAS_W_STEP
        PULSE_STOP(W);
#endif

#if ENABLED(MIXING_EXTRUDER)
        if (step_needed.e) E_STEP_WRITE(mixer.get_stepper(), INVERT_E_STEP_PIN);
#elif HAS_E0_STEP
        PULSE_STOP(E);
#endif

#if ISR_MULTI_STEPS
        if (events_to_do) START_TIMED_PULSE();
#endif

    } while (--events_to_do);
}//pulse_phase_isr()



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

