//#pragma once
#include <stdint.h>


enum MarlinState : uint8_t {
	MF_INITIALIZING = 0,
	MF_STOPPED,
	MF_KILLED,
	MF_RUNNING,
	MF_SD_COMPLETE,
	MF_PAUSED,
	MF_WAITING,
};
extern MarlinState marlin_state;

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define MAX_AXIS 3


template<typename T>
struct POSITION {
	union {
		struct { T x, y, z; };
		T pos[MAX_AXIS];
	};
	void reset(void) { x = y = z = 0; }
	void set(const T x, const T y, const T z) { this->x = x; this->y = y; this->z = z; }
	POSITION<T>& operator+=(const POSITION<T> &rs) { x += rs.x; y += rs.y; z += rs.z; return *this; }
	POSITION<T>& operator-=(const POSITION<T> &rs) { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
};//POSITION
typedef POSITION<float> position_float_t;
typedef POSITION<int32_t> position_int32_t;
typedef POSITION<uint32_t> position_uint32_t;
#define X_HOME_POS 100
#define Y_HOME_POS 200
#define Z_HOME_POS 300
extern position_float_t current_position;
extern position_float_t home_offset;

#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400}


// planner
/*
typedef struct {
	union {
		uint8_t bits;
		struct {
			bool recalculate : 1;
			bool nomical_length : 1;
			bool continued : 1;
			bool sync_position : 1;
		};
	};
	void clear(void) { bits = 0; }
} block_flags_t;
*/
/**
 * struct block_t
 *
 * A single entry in the planner buffer.
 * Tracks linear movement over multiple axes.
 *
 * The "nominal" values are as-specified by G-code, and
 * may never actually be reached due to acceleration limits.
 */
typedef struct PlannerBlock {
	union {
		position_uint32_t steps;	// Step count along each axis
		position_int32_t position;	// New position to force when this sync block is executed
	};
	uint32_t step_event_count;		// The number of step events required to complete this block
} block_t;//PlannerBlock
#define BLOCK_BUFFER_SIZE 16
#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

typedef struct {
	float axis_steps_per_mm[MAX_AXIS];
} planner_settings_t;


class Planner {
	public:
		/**
		 * The move buffer, calculated in stepper steps
		 *
		 * block_buffer is a ring buffer...
		 *
		 *             head,tail : indexes for write,read
		 *            head==tail : the buffer is empty
		 *            head!=tail : blocks are in the buffer
		 *   head==(tail-1)%size : the buffer is full
		 *
		 *  Writer of head is Planner::buffer_segment().
		 *  Reader of tail is Stepper::isr(). Always consider tail busy / read-only
		 */
		static block_t block_buffer[BLOCK_BUFFER_SIZE];
		static volatile uint8_t block_buffer_head,      // Index of the next block to be pushed
								block_buffer_nonbusy,   // Index of the first non busy block
								block_buffer_planned,   // Index of the optimally planned block
								block_buffer_tail;      // Index of the busy block, if any
		static uint16_t cleaning_buffer_counter;        // A counter to disable queuing of blocks
		static uint8_t delay_before_delivering;         // This counter delays delivery of blocks when queue becomes empty to allow the opportunity of merging blocks

		static planner_settings_t settings;
		static position_int32_t position; // [step]
	private:
		// Speed of previous path line segment
		static position_float_t previous_speed;
		// Nominal speed of previous path line segment (mm/s)^2
		static float previous_nominal_speed;

	public:
		Planner();
		void init();

	private:	
		// Get the index of the next / previous block in the ring buffer
		static constexpr uint8_t next_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index + 1); }
		static constexpr uint8_t prev_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index - 1); }
	public:
		// Number of moves currently in the planner including the busy block, if any
		static uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail); }
		// Number of nonbusy moves currently in the planner
		static uint8_t nonbusy_movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_nonbusy); }
		// Remove all blocks from the buffer
		static void clear_block_buffer() { block_buffer_nonbusy = block_buffer_planned = block_buffer_head = block_buffer_tail = 0; }
		// Check if movement queue is full
		static bool is_full() { return block_buffer_tail == next_block_index(block_buffer_head); }
		// Get count of movement slots free
		static uint8_t moves_free() { return BLOCK_BUFFER_SIZE - 1 - movesplanned(); }

		static void set_position_mm(const position_float_t& pos);
		static void set_machine_position_mm(const position_float_t& pos);
	private:


};//class Panner
extern Planner planner;


// motion
void sync_plan_position(void);
