#include "marlin.h"
#include <math.h>
#include <algorithm> // min()
using namespace std;

MarlinState marlin_state = MF_INITIALIZING;
bool IsRunning() { return marlin_state >= MF_RUNNING; }
bool IsStopped() { return marlin_state == MF_STOPPED; }

// queue
 // Frequently used G-code strings
const char G28_STR[] ="G28";
GCodeQueue::SerialState GCodeQueue::serial_state[NUM_SERIAL] = { 0 };
GCodeQueue::RingBuffer GCodeQueue::ring_buffer = { 0 };
//BUFFER_MONITORING
// Next Injected PROGMEM Command pointer. (nullptr == empty)
// Internal commands are enqueued ahead of serial / SD commands.
PGM_P GCodeQueue::injected_commands_P; // = nullptr
// Injected SRAM Commands
char GCodeQueue::injected_commands[64]; // = { 0 }
void GCodeQueue::RingBuffer::commit_command(bool skip_ok) {
    commands[index_w].skip_ok = skip_ok;
    advance_pos(index_w, 1);
}
/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
bool GCodeQueue::RingBuffer::enqueue(const char* cmd, bool skip_ok/*=true*/) {
    if (*cmd == ';' || length >= BUFSIZE) return false;
    strcpy_s(commands[index_w].buffer, cmd);
    commit_command(skip_ok);
    return true;
}
/**
 * Enqueue with Serial Echo
 * Return true if the command was consumed
 */
bool GCodeQueue::enqueue_one(const char* const cmd) {
    //SERIAL_ECHOLNPGM("enqueue_one(\"", cmd, "\")");
    if (*cmd == 0 || ISEOL(*cmd)) return true;
    if (ring_buffer.enqueue(cmd)) {
        //SERIAL_ECHO_MSG(STR_ENQUEUEING, cmd, "\"");
        return true;
    }
    return false;
}
/**
 * Process the next "immediate" command from PROGMEM.
 * Return 'true' if any commands were processed.
 */
bool GCodeQueue::process_injected_command_P() {
    if (!injected_commands_P) return false;
    char c;
    size_t i = 0;
    while ((c = *(&injected_commands_P[i])) && c != '\n') i++;
    // Extract current command and move pointer to next command
    //char cmd[i + 1];
    char cmd[64];
    memcpy(cmd, injected_commands_P, i);
    cmd[i] = '\0';
    injected_commands_P = c ? injected_commands_P + i + 1 : nullptr;
    // Execute command if non-blank
    if (i) {
        //parser.parse(cmd);
        //gcode.process_parsed_command();
    }
    return true;
}
/**
 * Process the next "immediate" command from SRAM.
 * Return 'true' if any commands were processed.
 */
bool GCodeQueue::process_injected_command() {
    if (injected_commands[0] == '\0') return false;
    char c;
    size_t i = 0;
    while ((c = injected_commands[i]) && c != '\n') i++;
    // Execute a non-blank command
    if (i) {
        injected_commands[i] = '\0';
        //parser.parse(injected_commands);
        //gcode.process_parsed_command();
    }
    // Copy the next command into place
    for (
        uint8_t d = 0, s = i + !!c;                     // dst, src
        (injected_commands[d] = injected_commands[s]);  // copy, exit if 0
        d++, s++                                        // next dst, src
        );
    return true;
}
/**
 * Enqueue and return only when commands are actually enqueued.
 * Never call this from a G-code handler!
 */
void GCodeQueue::enqueue_one_now(const char* const cmd) { while (!enqueue_one(cmd)) idle(); }
//void GCodeQueue::enqueue_one_now(FSTR_P const fcmd) { while (!enqueue_one(fcmd)) idle(); }
/**
 * Attempt to enqueue a single G-code command
 * and return 'true' if successful.
 */
/*
bool GCodeQueue::enqueue_one(FSTR_P const fcmd) {
    size_t i = 0;
    PGM_P p = FTOP(fcmd);
    char c;
    while ((c = pgm_read_byte(&p[i])) && c != '\n') i++;
    char cmd[i + 1];
    memcpy_P(cmd, p, i);
    cmd[i] = '\0';
    return ring_buffer.enqueue(cmd);
}
*/
/**
 * Enqueue from program memory and return only when commands are actually enqueued
 * Never call this from a G-code handler!
 */
void GCodeQueue::enqueue_now_P(PGM_P const pgcode) {
    size_t i = 0;
    PGM_P p = pgcode;
    for (;;) {
        char c;
        while ((c = *(&p[i])) && c != '\n') i++;
        //char cmd[i + 1];
        char cmd[64];
        memcpy(cmd, p, i);
        cmd[i] = '\0';
        enqueue_one_now(cmd);
        if (!c) break;
        p += i + 1;
    }
}
/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void GCodeQueue::RingBuffer::ok_to_send() {
    CommandLine& command = commands[index_r];
    if (command.skip_ok) return;
    //SERIAL_ECHOPGM(STR_OK);
    //SERIAL_EOL();
}
/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void GCodeQueue::flush_and_request_resend(const serial_index_t serial_ind) {
    //SERIAL_FLUSH();
    //SERIAL_ECHOLNPGM(STR_RESEND, serial_state[serial_ind.index].last_N + 1);
    //SERIAL_ECHOLNPGM(STR_OK);
}
static bool serial_data_available(serial_index_t index) {
    /*
    const int a = SERIAL_IMPL.available(index);
#if ENABLED(RX_BUFFER_MONITOR) && RX_BUFFER_SIZE
    if (a > RX_BUFFER_SIZE - 2) {
        PORT_REDIRECT(SERIAL_PORTMASK(index));
        SERIAL_ERROR_MSG("RX BUF overflow, increase RX_BUFFER_SIZE: ", a);
    }
#endif
    return a > 0;
    */
    return false;
}
inline int read_serial(const serial_index_t index) { 
    //return SERIAL_IMPL.read(index);
    return '/0';
}
/*
void GCodeQueue::gcode_line_error(FSTR_P const ferr, const serial_index_t serial_ind) {
    PORT_REDIRECT(SERIAL_PORTMASK(serial_ind)); // Reply to the serial port that sent the command
    SERIAL_ERROR_START();
    SERIAL_ECHOLNF(ferr, serial_state[serial_ind.index].last_N);
    while (read_serial(serial_ind) != -1) { } // Clear out the RX buffer. Why don't use flush here ?
    flush_and_request_resend(serial_ind);
    serial_state[serial_ind.index].count = 0;
}
*/
FORCE_INLINE bool is_M29(const char* const cmd) {  // matches "M29" & "M29 ", but not "M290", etc
    //const char* const m29 = strstr_P(cmd, PSTR("M29"));
    //return m29 && !NUMERIC(m29[3]);
    return false;
}

#define PS_NORMAL 0
#define PS_EOL    1
#define PS_QUOTED 2
#define PS_PAREN  3
#define PS_ESC    4
inline void process_stream_char(const char c, uint8_t& sis, char(&buff)[MAX_CMD_SIZE], int& ind) {
    if (sis == PS_EOL) return;    // EOL comment or overflow
    else if (sis == PS_PAREN) { // Inline comment
        if (c == ')') sis = PS_NORMAL;
        return;
    }
    else if (sis >= PS_ESC)       // End escaped char
        sis -= PS_ESC;
    else if (c == '\\') {         // Start escaped char
        sis += PS_ESC;
        if (sis == PS_ESC) return;  // Keep if quoting
    }
    else if (sis == PS_QUOTED) {
        if (c == '"') sis = PS_NORMAL; // End quoted string
    }
    else if (c == '"')          // Start quoted string
        sis = PS_QUOTED;
    else if (c == ';') {          // Start end-of-line comment
        sis = PS_EOL;
        return;
    }
    else if (c == '(') {        // Start inline comment
        sis = PS_PAREN;
        return;
    }
    // Backspace erases previous characters
    if (c == 0x08) {
        if (ind) buff[--ind] = '\0';
    }
    else {
        buff[ind++] = c;
        if (ind >= MAX_CMD_SIZE - 1)
            sis = PS_EOL;             // Skip the rest on overflow
    }
}

/**
 * Handle a line being completed. For an empty line
 * keep sensor readings going and watchdog alive.
 */
inline bool process_line_done(uint8_t& sis, char(&buff)[MAX_CMD_SIZE], int& ind) {
    sis = PS_NORMAL;                    // "Normal" Serial Input State
    buff[ind] = '\0';                   // Of course, I'm a Terminator.
    const bool is_empty = (ind == 0);   // An empty line?
    if (is_empty)
        //thermalManager.task();            // Keep sensors satisfied
        int i = 0;
    else
        ind = 0;                          // Start a new line
    return is_empty;                    // Inform the caller
}
/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
void GCodeQueue::get_serial_commands() {
    //#if ENABLED(BINARY_FILE_TRANSFER)
    // If the command buffer is empty for too long,
    // send "wait" to indicate Marlin is still waiting.

    // Loop while serial characters are incoming and the queue is not full
    for (bool hadData = true; hadData;) {
        // Unless a serial port has data, this will exit on next iteration
        hadData = false;
        // Check if the queue is full and exit if it is.
        if (ring_buffer.full()) return;
        // No data for this port ? Skip it
        if (!serial_data_available(0)) continue;
        // Ok, we have some data to process, let's make progress here
        hadData = true;
        const int c = read_serial(0);
        if (c < 0) {
            // This should never happen, let's log it
            PORT_REDIRECT(SERIAL_PORTMASK(p));     // Reply to the serial port that sent the command
            // Crash here to get more information why it failed
            BUG_ON("SP available but read -1");
            SERIAL_ERROR_MSG(STR_ERR_SERIAL_MISMATCH);
            SERIAL_FLUSH();
            continue;
        }
        const char serial_char = (char)c;
        SerialState& serial = serial_state[p];
        if (ISEOL(serial_char)) {
            // Reset our state, continue if the line was empty
            if (process_line_done(serial.input_state, serial.line_buffer, serial.count))
                continue;
            char* command = serial.line_buffer;
            while (*command == ' ') command++;                   // Skip leading spaces
            char* npos = (*command == 'N') ? command : nullptr;  // Require the N parameter to start the line
            if (npos) {
                const bool M110 = !!strstr_P(command, PSTR("M110"));
                if (M110) {
                    char* n2pos = strchr(command + 4, 'N');
                    if (n2pos) npos = n2pos;
                }
                const long gcode_N = strtol(npos + 1, nullptr, 10);
                    // The line number must be in the correct sequence.
                    if (gcode_N != serial.last_N + 1 && !M110) {
                        // A request-for-resend line was already in transit so we got two - oops!
                        if (WITHIN(gcode_N, serial.last_N - 1, serial.last_N)) continue;
                        // A corrupted line or too high, indicating a lost line
                        gcode_line_error(F(STR_ERR_LINE_NO), p);
                        break;
                    }

                    char* apos = strrchr(command, '*');
                    if (apos) {
                        uint8_t checksum = 0, count = uint8_t(apos - command);
                        while (count) checksum ^= command[--count];
                        if (strtol(apos + 1, nullptr, 10) != checksum) {
                            gcode_line_error(F(STR_ERR_CHECKSUM_MISMATCH), p);
                            break;
                        }
                    }
                    else {
                        gcode_line_error(F(STR_ERR_NO_CHECKSUM), p);
                        break;
                    }

                    serial.last_N = gcode_N;
                }
#if ENABLED(SDSUPPORT)
                // Pronterface "M29" and "M29 " has no line number
                else if (card.flag.saving && !is_M29(command)) {
                    gcode_line_error(F(STR_ERR_NO_CHECKSUM), p);
                    break;
                }
#endif

                //
                // Movement commands give an alert when the machine is stopped
                //

                if (IsStopped()) {
                    char* gpos = strchr(command, 'G');
                    if (gpos) {
                        switch (strtol(gpos + 1, nullptr, 10)) {
                        case 0 ... 1:
                        TERN_(ARC_SUPPORT, case 2 ... 3:)
                        TERN_(BEZIER_CURVE_SUPPORT, case 5:)
                            PORT_REDIRECT(SERIAL_PORTMASK(p));     // Reply to the serial port that sent the command
                                                       SERIAL_ECHOLNPGM(STR_ERR_STOPPED);
                                                       LCD_MESSAGE(MSG_STOPPED);
                                                       break;
                        }
                    }
                }

#if DISABLED(EMERGENCY_PARSER)
                // Process critical commands early
                if (command[0] == 'M') switch (command[3]) {
                case '8': if (command[2] == '0' && command[1] == '1') { wait_for_heatup = false; TERN_(HAS_MARLINUI_MENU, wait_for_user = false); } break;
                case '2': if (command[2] == '1' && command[1] == '1') kill(FPSTR(M112_KILL_STR), nullptr, true); break;
                case '0': if (command[1] == '4' && command[2] == '1') quickstop_stepper(); break;
                }
#endif

#if NO_TIMEOUTS > 0
                last_command_time = ms;
#endif

                // Add the command to the queue
                ring_buffer.enqueue(serial.line_buffer, false OPTARG(HAS_MULTI_SERIAL, p));
            }
            else
                process_stream_char(serial_char, serial.input_state, serial.line_buffer, serial.count);

    } // queue has space, serial has data
}




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
uint32_t Planner::max_acceleration_steps_per_s2[DISTINCT_AXES]; // (steps/s^2) Derived from mm_per_s2
float Planner::mm_per_step[DISTINCT_AXES];      // (mm) Millimeters per step
//float Planner::junction_deviation_mm;         // (mm) M205 J

// private:
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
/**
 * Get the current block for processing
 * and mark the block as busy.
 * Return nullptr if the buffer is empty
 * or if there is a first-block delay.
 * WARNING: Called from Stepper ISR context!
 */
block_t* Planner::get_current_block() {
    // Get the number of moves in the planner queue so far
    const uint8_t nr_moves = movesplanned();
    // If there are any moves queued ...
    if (nr_moves) {
        // If there is still delay of delivery of blocks running, decrement it
        if (delay_before_delivering) {
            --delay_before_delivering;
            // If the number of movements queued is less than 3, and there is still time
            //  to wait, do not deliver anything
            if (nr_moves < 3 && delay_before_delivering) return nullptr;
            delay_before_delivering = 0;
        }
        // If we are here, there is no excuse to deliver the block
        block_t* const block = &block_buffer[block_buffer_tail];
        // No trapezoid calculated? Don't execute yet.
        if (block->flag.recalculate) return nullptr;
        // We can't be sure how long an active block will take, so don't count it.
        //TERN_(HAS_WIRED_LCD, block_buffer_runtime_us -= block->segment_time_us);
        // As this block is busy, advance the nonbusy block pointer
        block_buffer_nonbusy = next_block_index(block_buffer_tail);
        // Push block_buffer_planned pointer, if encountered.
        if (block_buffer_tail == block_buffer_planned)
            block_buffer_planned = block_buffer_nonbusy;
        // Return the block
        return block;
    }
    // The queue became empty
    //TERN_(HAS_WIRED_LCD, clear_block_buffer_runtime()); // paranoia. Buffer is empty now - so reset accumulated time to zero.
    return nullptr;
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
bool Planner::busy() {
    return (has_blocks_queued() || cleaning_buffer_counter);
}
void Planner::finish_and_disable() {
    while (has_blocks_queued() || cleaning_buffer_counter) idle();
    stepper.disable_all_steppers();
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from ABC to XYZ.
 */
float Planner::get_axis_position_mm(const AxisEnum axis) {
    float axis_steps;
    axis_steps = (float)stepper.position(axis);
    return axis_steps * mm_per_step[axis];
}

/**
 * Block until the planner is finished processing
 */
void Planner::synchronize() { while (busy()) idle(); }



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

#define ENABLE_AXIS_X()
#define ENABLE_AXIS_Y()
#define ENABLE_AXIS_Z()
#define DISABLE_AXIS_X()
#define DISABLE_AXIS_Y()
#define DISABLE_AXIS_Z()
void Stepper::enable_axis(const AxisEnum axis) {
    switch (axis) {
    case X_AXIS: ENABLE_AXIS_X(); break;
    case Y_AXIS: ENABLE_AXIS_Y(); break;
    case Z_AXIS: ENABLE_AXIS_Z(); break;
        default: break;
    }
    mark_axis_enabled(axis);
}
bool Stepper::disable_axis(const AxisEnum axis) {
    mark_axis_disabled(axis);
    // If all the axes that share the enabled bit are disabled
    const bool can_disable = can_axis_disable(axis);
    if (can_disable) {
        switch (axis) {
        case X_AXIS: DISABLE_AXIS_X(); break;
        case Y_AXIS: DISABLE_AXIS_Y(); break;
        case Z_AXIS: DISABLE_AXIS_Z(); break;
        default: break;
        }
    }
    return can_disable;
}
void Stepper::enable_all_steppers() {
    enable_axis(X_AXIS); enable_axis(Y_AXIS); enable_axis(Z_AXIS);
}
void Stepper::disable_all_steppers() {
    disable_axis(X_AXIS); disable_axis(Y_AXIS); disable_axis(Z_AXIS);
}


// Check if the given block is busy or not - Must not be called from ISR contexts
// The current_block could change in the middle of the read by an Stepper ISR, so
// we must explicitly prevent that!
bool Stepper::is_block_busy(const block_t* const block) {
#ifdef __AVR__
    // A SW memory barrier, to ensure GCC does not overoptimize loops
#define sw_barrier() asm volatile("": : :"memory");

// Keep reading until 2 consecutive reads return the same value,
// meaning there was no update in-between caused by an interrupt.
// This works because stepper ISRs happen at a slower rate than
// successive reads of a variable, so 2 consecutive reads with
// the same value means no interrupt updated it.
    block_t* vold, * vnew = current_block;
    sw_barrier();
    do {
        vold = vnew;
        vnew = current_block;
        sw_barrier();
    } while (vold != vnew);
#else
    block_t* vnew = current_block;
#endif
    // Return if the block is busy or not
    return block == vnew;
}
/**
 * Set the stepper positions directly in steps
 * The input is based on the typical per-axis XYZE steps.
 * For CORE machines XYZ needs to be translated to ABC.
 * This allows get_axis_position_mm to correctly
 * derive the current XYZE position later on.
 */
void Stepper::_set_position(const abce_long_t& spos) {
    // default non-h-bot planning
    count_position = spos;
}
// Get a stepper's position in steps.
int32_t Stepper::position(const AxisEnum axis) {
#ifdef __AVR__
    // Protect the access to the position. Only required for AVR, as
    //  any 32bit CPU offers atomic access to 32bit variables
    const bool was_enabled = suspend();
#endif
    const int32_t v = count_position[axis];
#ifdef __AVR__
    // Reenable Stepper ISR
    if (was_enabled) wake_up();
#endif
    return v;
}
// Set the current position in steps
void Stepper::set_position(const xyze_long_t& spos) {
    planner.synchronize();
    const bool was_enabled = suspend();
    _set_position(spos);
    if (was_enabled) wake_up();
}
void Stepper::set_axis_position(const AxisEnum a, const int32_t& v) {
    planner.synchronize();
#ifdef __AVR__
    // Protect the access to the position. Only required for AVR, as
    //  any 32bit CPU offers atomic access to 32bit variables
    const bool was_enabled = suspend();
#endif
    count_position[a] = v;
#ifdef __AVR__
    // Reenable Stepper ISR
    if (was_enabled) wake_up();
#endif
}

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

const uint16_t speed_lookuptable_fast[256][2] = {
  { 62500, 55556}, { 6944, 3268}, { 3676, 1176}, { 2500, 607}, { 1893, 369}, { 1524, 249}, { 1275, 179}, { 1096, 135},
  { 961, 105}, { 856, 85}, { 771, 69}, { 702, 58}, { 644, 49}, { 595, 42}, { 553, 37}, { 516, 32},
  { 484, 28}, { 456, 25}, { 431, 23}, { 408, 20}, { 388, 19}, { 369, 16}, { 353, 16}, { 337, 14},
  { 323, 13}, { 310, 11}, { 299, 11}, { 288, 11}, { 277, 9}, { 268, 9}, { 259, 8}, { 251, 8},
  { 243, 8}, { 235, 7}, { 228, 6}, { 222, 6}, { 216, 6}, { 210, 6}, { 204, 5}, { 199, 5},
  { 194, 5}, { 189, 4}, { 185, 4}, { 181, 4}, { 177, 4}, { 173, 4}, { 169, 4}, { 165, 3},
  { 162, 3}, { 159, 4}, { 155, 3}, { 152, 3}, { 149, 2}, { 147, 3}, { 144, 3}, { 141, 2},
  { 139, 3}, { 136, 2}, { 134, 2}, { 132, 3}, { 129, 2}, { 127, 2}, { 125, 2}, { 123, 2},
  { 121, 2}, { 119, 1}, { 118, 2}, { 116, 2}, { 114, 1}, { 113, 2}, { 111, 2}, { 109, 1},
  { 108, 2}, { 106, 1}, { 105, 2}, { 103, 1}, { 102, 1}, { 101, 1}, { 100, 2}, { 98, 1},
  { 97, 1}, { 96, 1}, { 95, 2}, { 93, 1}, { 92, 1}, { 91, 1}, { 90, 1}, { 89, 1},
  { 88, 1}, { 87, 1}, { 86, 1}, { 85, 1}, { 84, 1}, { 83, 0}, { 83, 1}, { 82, 1},
  { 81, 1}, { 80, 1}, { 79, 1}, { 78, 0}, { 78, 1}, { 77, 1}, { 76, 1}, { 75, 0},
  { 75, 1}, { 74, 1}, { 73, 1}, { 72, 0}, { 72, 1}, { 71, 1}, { 70, 0}, { 70, 1},
  { 69, 0}, { 69, 1}, { 68, 1}, { 67, 0}, { 67, 1}, { 66, 0}, { 66, 1}, { 65, 0},
  { 65, 1}, { 64, 1}, { 63, 0}, { 63, 1}, { 62, 0}, { 62, 1}, { 61, 0}, { 61, 1},
  { 60, 0}, { 60, 0}, { 60, 1}, { 59, 0}, { 59, 1}, { 58, 0}, { 58, 1}, { 57, 0},
  { 57, 1}, { 56, 0}, { 56, 0}, { 56, 1}, { 55, 0}, { 55, 1}, { 54, 0}, { 54, 0},
  { 54, 1}, { 53, 0}, { 53, 0}, { 53, 1}, { 52, 0}, { 52, 0}, { 52, 1}, { 51, 0},
  { 51, 0}, { 51, 1}, { 50, 0}, { 50, 0}, { 50, 1}, { 49, 0}, { 49, 0}, { 49, 1},
  { 48, 0}, { 48, 0}, { 48, 1}, { 47, 0}, { 47, 0}, { 47, 0}, { 47, 1}, { 46, 0},
  { 46, 0}, { 46, 1}, { 45, 0}, { 45, 0}, { 45, 0}, { 45, 1}, { 44, 0}, { 44, 0},
  { 44, 0}, { 44, 1}, { 43, 0}, { 43, 0}, { 43, 0}, { 43, 1}, { 42, 0}, { 42, 0},
  { 42, 0}, { 42, 1}, { 41, 0}, { 41, 0}, { 41, 0}, { 41, 0}, { 41, 1}, { 40, 0},
  { 40, 0}, { 40, 0}, { 40, 0}, { 40, 1}, { 39, 0}, { 39, 0}, { 39, 0}, { 39, 0},
  { 39, 1}, { 38, 0}, { 38, 0}, { 38, 0}, { 38, 0}, { 38, 1}, { 37, 0}, { 37, 0},
  { 37, 0}, { 37, 0}, { 37, 0}, { 37, 1}, { 36, 0}, { 36, 0}, { 36, 0}, { 36, 0},
  { 36, 1}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 1},
  { 34, 0}, { 34, 0}, { 34, 0}, { 34, 0}, { 34, 0}, { 34, 1}, { 33, 0}, { 33, 0},
  { 33, 0}, { 33, 0}, { 33, 0}, { 33, 0}, { 33, 1}, { 32, 0}, { 32, 0}, { 32, 0},
  { 32, 0}, { 32, 0}, { 32, 0}, { 32, 0}, { 32, 1}, { 31, 0}, { 31, 0}, { 31, 0},
  { 31, 0}, { 31, 0}, { 31, 0}, { 31, 1}, { 30, 0}, { 30, 0}, { 30, 0}, { 30, 0}
};
const uint16_t speed_lookuptable_slow[256][2] = {
  { 62500, 12500}, { 50000, 8334}, { 41666, 5952}, { 35714, 4464}, { 31250, 3473}, { 27777, 2777}, { 25000, 2273}, { 22727, 1894},
  { 20833, 1603}, { 19230, 1373}, { 17857, 1191}, { 16666, 1041}, { 15625, 920}, { 14705, 817}, { 13888, 731}, { 13157, 657},
  { 12500, 596}, { 11904, 541}, { 11363, 494}, { 10869, 453}, { 10416, 416}, { 10000, 385}, { 9615, 356}, { 9259, 331},
  { 8928, 308}, { 8620, 287}, { 8333, 269}, { 8064, 252}, { 7812, 237}, { 7575, 223}, { 7352, 210}, { 7142, 198},
  { 6944, 188}, { 6756, 178}, { 6578, 168}, { 6410, 160}, { 6250, 153}, { 6097, 145}, { 5952, 139}, { 5813, 132},
  { 5681, 126}, { 5555, 121}, { 5434, 115}, { 5319, 111}, { 5208, 106}, { 5102, 102}, { 5000, 99}, { 4901, 94},
  { 4807, 91}, { 4716, 87}, { 4629, 84}, { 4545, 81}, { 4464, 79}, { 4385, 75}, { 4310, 73}, { 4237, 71},
  { 4166, 68}, { 4098, 66}, { 4032, 64}, { 3968, 62}, { 3906, 60}, { 3846, 59}, { 3787, 56}, { 3731, 55},
  { 3676, 53}, { 3623, 52}, { 3571, 50}, { 3521, 49}, { 3472, 48}, { 3424, 46}, { 3378, 45}, { 3333, 44},
  { 3289, 43}, { 3246, 41}, { 3205, 41}, { 3164, 39}, { 3125, 39}, { 3086, 38}, { 3048, 36}, { 3012, 36},
  { 2976, 35}, { 2941, 35}, { 2906, 33}, { 2873, 33}, { 2840, 32}, { 2808, 31}, { 2777, 30}, { 2747, 30},
  { 2717, 29}, { 2688, 29}, { 2659, 28}, { 2631, 27}, { 2604, 27}, { 2577, 26}, { 2551, 26}, { 2525, 25},
  { 2500, 25}, { 2475, 25}, { 2450, 23}, { 2427, 24}, { 2403, 23}, { 2380, 22}, { 2358, 22}, { 2336, 22},
  { 2314, 21}, { 2293, 21}, { 2272, 20}, { 2252, 20}, { 2232, 20}, { 2212, 20}, { 2192, 19}, { 2173, 18},
  { 2155, 19}, { 2136, 18}, { 2118, 18}, { 2100, 17}, { 2083, 17}, { 2066, 17}, { 2049, 17}, { 2032, 16},
  { 2016, 16}, { 2000, 16}, { 1984, 16}, { 1968, 15}, { 1953, 16}, { 1937, 14}, { 1923, 15}, { 1908, 15},
  { 1893, 14}, { 1879, 14}, { 1865, 14}, { 1851, 13}, { 1838, 14}, { 1824, 13}, { 1811, 13}, { 1798, 13},
  { 1785, 12}, { 1773, 13}, { 1760, 12}, { 1748, 12}, { 1736, 12}, { 1724, 12}, { 1712, 12}, { 1700, 11},
  { 1689, 12}, { 1677, 11}, { 1666, 11}, { 1655, 11}, { 1644, 11}, { 1633, 10}, { 1623, 11}, { 1612, 10},
  { 1602, 10}, { 1592, 10}, { 1582, 10}, { 1572, 10}, { 1562, 10}, { 1552, 9}, { 1543, 10}, { 1533, 9},
  { 1524, 9}, { 1515, 9}, { 1506, 9}, { 1497, 9}, { 1488, 9}, { 1479, 9}, { 1470, 9}, { 1461, 8},
  { 1453, 8}, { 1445, 9}, { 1436, 8}, { 1428, 8}, { 1420, 8}, { 1412, 8}, { 1404, 8}, { 1396, 8},
  { 1388, 7}, { 1381, 8}, { 1373, 7}, { 1366, 8}, { 1358, 7}, { 1351, 7}, { 1344, 8}, { 1336, 7},
  { 1329, 7}, { 1322, 7}, { 1315, 7}, { 1308, 6}, { 1302, 7}, { 1295, 7}, { 1288, 6}, { 1282, 7},
  { 1275, 6}, { 1269, 7}, { 1262, 6}, { 1256, 6}, { 1250, 7}, { 1243, 6}, { 1237, 6}, { 1231, 6},
  { 1225, 6}, { 1219, 6}, { 1213, 6}, { 1207, 6}, { 1201, 5}, { 1196, 6}, { 1190, 6}, { 1184, 5},
  { 1179, 6}, { 1173, 5}, { 1168, 6}, { 1162, 5}, { 1157, 5}, { 1152, 6}, { 1146, 5}, { 1141, 5},
  { 1136, 5}, { 1131, 5}, { 1126, 5}, { 1121, 5}, { 1116, 5}, { 1111, 5}, { 1106, 5}, { 1101, 5},
  { 1096, 5}, { 1091, 5}, { 1086, 4}, { 1082, 5}, { 1077, 5}, { 1072, 4}, { 1068, 5}, { 1063, 4},
  { 1059, 5}, { 1054, 4}, { 1050, 4}, { 1046, 5}, { 1041, 4}, { 1037, 4}, { 1033, 5}, { 1028, 4},
  { 1024, 4}, { 1020, 4}, { 1016, 4}, { 1012, 4}, { 1008, 4}, { 1004, 4}, { 1000, 4}, { 996, 4},
  { 992, 4}, { 988, 4}, { 984, 4}, { 980, 4}, { 976, 4}, { 972, 4}, { 968, 3}, { 965, 3}
};

// Calculate timer interval, with all limits applied.
uint32_t Stepper::calc_timer_interval(uint32_t step_rate) {
#ifdef CPU_32_BIT
    // In case of high-performance processor, it is able to calculate in real-time
    return uint32_t(STEPPER_TIMER_RATE) / step_rate;
#else
    // AVR is able to keep up at 30khz Stepping ISR rate.
    constexpr uint32_t min_step_rate = (F_CPU) / 500000U;
    if (step_rate <= min_step_rate) {
        step_rate = 0;
        uintptr_t *table_address = (uintptr_t *)&speed_lookuptable_slow[0][0];
        return uint16_t(*table_address);
    }
    else {
        step_rate -= min_step_rate; // Correct for minimal speed
        if (step_rate >= 0x0800) {  // higher step rate
            const uint8_t rate_mod_256 = (step_rate & 0x00FF);
            const uintptr_t *table_address = (uintptr_t *)(&speed_lookuptable_fast[uint8_t(step_rate >> 8)][0]),
                gain = uint16_t(*(table_address + 2));
            //return uint16_t(*(table_address)) - MultiU8X16toH16(rate_mod_256, gain);
            return uint16_t(*(table_address)) - rate_mod_256*gain;
        }
        else { // lower step rates
            uintptr_t *table_address = (uintptr_t *)(&speed_lookuptable_slow[0][0]);
            table_address += (step_rate >> 1) & 0xFFFC;
            return uint16_t(*table_address)
                - ((uint16_t(*(table_address + 2)) * uint8_t(step_rate & 0x0007)) >> 3);
        }
    }
#endif
}

// Get the timer interval and the number of loops to perform per tick
uint32_t Stepper::calc_timer_interval(uint32_t step_rate, uint8_t& loops) {
    uint8_t multistep = 1;
    // The stepping frequency limits for each multistepping rate
    static const uint32_t limit[] = {
      (MAX_STEP_ISR_FREQUENCY_1X),
      (MAX_STEP_ISR_FREQUENCY_2X >> 1),
      (MAX_STEP_ISR_FREQUENCY_4X >> 2),
      (MAX_STEP_ISR_FREQUENCY_8X >> 3),
      (MAX_STEP_ISR_FREQUENCY_16X >> 4),
      (MAX_STEP_ISR_FREQUENCY_32X >> 5),
      (MAX_STEP_ISR_FREQUENCY_64X >> 6),
      (MAX_STEP_ISR_FREQUENCY_128X >> 7)
    };
    // Select the proper multistepping
    uint8_t idx = 0;
    while (idx < 7 && step_rate >(uint32_t)*(&limit[idx])) {
        step_rate >>= 1;
        multistep <<= 1;
        ++idx;
    };
    if (step_rate > uint32_t(MAX_STEP_ISR_FREQUENCY_1X))
        step_rate = uint32_t(MAX_STEP_ISR_FREQUENCY_1X);
    loops = multistep;
    return calc_timer_interval(step_rate);
}

// This is the last half of the stepper interrupt: This one processes and
// properly schedules blocks from the planner. This is executed after creating
// the step pulses, so it is not time critical, as pulses are already done.


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
                _set_position(current_block->position);
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



// marlin
inline void manage_inactivity(const bool no_stepper_sleep = false) {
    queue.get_available_commands();
    //const millis_t ms = millis();
    // Prevent steppers timing-out
    const bool do_reset_timeout = no_stepper_sleep;
    // Reset both the M18/M84 activity timeout and the M85 max 'kill' timeout
    //if (do_reset_timeout) gcode.reset_stepper_timeout(ms);
    /*
    if (gcode.stepper_max_timed_out(ms)) {
        SERIAL_ERROR_START();
        SERIAL_ECHOPGM(STR_KILL_PRE);
        SERIAL_ECHOLNPGM(STR_KILL_INACTIVE_TIME, parser.command_ptr);
        kill();
    }
    */
    const bool has_blocks = planner.has_blocks_queued();  // Any moves in the planner?
    //if (has_blocks) gcode.reset_stepper_timeout(ms);      // Reset timeout for M18/M84, M85 max 'kill', and laser.
    // M18 / M84 : Handle steppers inactive time timeout
    /*
    if (gcode.stepper_inactive_time) {
        static bool already_shutdown_steppers; // = false
        if (!has_blocks && !do_reset_timeout && gcode.stepper_inactive_timeout()) {
            if (!already_shutdown_steppers) {
                already_shutdown_steppers = true;
                // Individual axes will be disabled if configured
                stepper.disable_axis(X_AXIS);
                stepper.disable_axis(Y_AXIS);
                stepper.disable_axis(Z_AXIS);
            }
        }
        else
            already_shutdown_steppers = false;
    }
    */
    // Limit check_axes_activity frequency to 10Hz
    /*
    static millis_t next_check_axes_ms = 0;
    if (ELAPSED(ms, next_check_axes_ms)) {
        planner.check_axes_activity();
        next_check_axes_ms = ms + 100UL;
    }
    */
}//manage_inactivity()

/**
 * Standard idle routine keeps the machine alive:
 *  - Core Marlin activities
 *  - Manage heaters (and Watchdog)
 *  - Max7219 heartbeat, animation, etc.
 *
 *  Only after setup() is complete:
 *  - Handle filament runout sensors
 *  - Run HAL idle tasks
 *  - Handle Power-Loss Recovery
 *  - Run StallGuard endstop checks
 *  - Handle SD Card insert / remove
 *  - Handle USB Flash Drive insert / remove
 *  - Announce Host Keepalive state (if any)
 *  - Update the Print Job Timer state
 *  - Update the Beeper queue
 *  - Read Buttons and Update the LCD
 *  - Run i2c Position Encoders
 *  - Auto-report Temperatures / SD Status
 *  - Update the Průša MMU2
 *  - Handle Joystick jogging
 */
void idle(const bool no_stepper_sleep/*=false*/) {
    manage_inactivity(no_stepper_sleep); // core marlin activites
    //thermalManager.task(); // Manage Heaters (and Watchdog)
    // Return if setup() isn't completed
    if (marlin_state == MF_INITIALIZING) goto IDLE_DONE;
    //(void)check_tool_sensor_stats(active_extruder, true);
    //hal.idletask();
IDLE_DONE:
    //idle_depth--;
    return;
}//idle()




 
 //marlin.cpp

