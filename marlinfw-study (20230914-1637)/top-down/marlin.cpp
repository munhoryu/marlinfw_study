#include "marlin.h"
#include <math.h>

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
void Stepper::init() {
    // init dir and enable pins
    // Init Step Pins

    HAL_timer_start(MF_TIMER_STEP, 122); // Init Stepper ISR to 122 Hz for quick starting
    //wake_up();
    //sei();

    // Init direction bits for first moves
    set_directions(0x07);
}
