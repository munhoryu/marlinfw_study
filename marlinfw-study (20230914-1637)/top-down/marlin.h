﻿//#pragma once
#include <stdint.h>
#include <string.h>

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
void idle(const bool no_stepper_sleep=false);


// stepper
typedef uint8_t axis_bits_t;
enum AxisEnum : uint8_t {
	//X_AXIS_ENUM, Y_AXIS_ENUM, Z_AXIS_ENUM, MAX_AXIS_ENUM
	//, ALL_AXES_ENUM = 0xFE, NO_AXIS_ENUM = 0xFF
	X_AXIS, Y_AXIS, Z_AXIS, E_AXIS,
	A_AXIS = X_AXIS, B_AXIS = Y_AXIS, C_AXIS = Z_AXIS
};



// macros
#define _BV(b) (1 << (b))
#define TEST(n,b) (!!((n)&_BV(b)))
#define SBI(A,B) (A |= _BV(B))
#define CBI(A,B) (A &= ~_BV(B))
#define _MIN(a,b) ((a)<(b)?(a):(b))
//#define _MIN3(a,b,c) (_MIN(_MIN(a,b),(c))
//#define _MIN4(a,b,c,d) (_MIN(_MIN(a,b),_MIN(c,d)))
#define _MAX(a,b) ((a)>(b)?(a):(b))
//#define _MAX3(a,b,c) (_MAX(_MAX(a,b),(c)))
//#define _MAX4(a,b,c,d) (_MAX(_MAX(a,b),_MAX(c,d)))
#define WITHIN(N,L,H)       ((N) >= (L) && (N) <= (H))
#define ISEOL(C)            ((C) == '\n' || (C) == '\r')
#define NUMERIC(a)          WITHIN(a, '0', '9')
#define DECIMAL(a)          (NUMERIC(a) || a == '.')
#define HEXCHR(a)           (NUMERIC(a) ? (a) - '0' : WITHIN(a, 'a', 'f') ? ((a) - 'a' + 10)  : WITHIN(a, 'A', 'F') ? ((a) - 'A' + 10) : -1)
#define NUMERIC_SIGNED(a)   (NUMERIC(a) || (a) == '-' || (a) == '+')
#define DECIMAL_SIGNED(a)   (DECIMAL(a) || (a) == '-' || (a) == '+')
#define COUNT(a)            (sizeof(a)/sizeof(*a))
#define ZERO(a)             memset((void*)a,0,sizeof(a))
#define COPY(a,b) do{ \
    static_assert(sizeof(a[0]) == sizeof(b[0]), "COPY: '" STRINGIFY(a) "' and '" STRINGIFY(b) "' types (sizes) don't match!"); \
    memcpy(&a[0],&b[0],_MIN(sizeof(a),sizeof(b))); \
  }while(0)


template <bool, class L, class R>
struct IF { typedef R type; };
template <class L, class R>
struct IF<true, L, R> { typedef L type; };
template<size_t N>
struct Flags {
	typedef typename IF<(N > 8), uint16_t, uint8_t>::type bits_t;
	typedef struct { bool b0 : 1, b1 : 1, b2 : 1, b3 : 1, b4 : 1, b5 : 1, b6 : 1, b7 : 1; } N8;
	typedef struct { bool b0 : 1, b1 : 1, b2 : 1, b3 : 1, b4 : 1, b5 : 1, b6 : 1, b7 : 1, b8 : 1, b9 : 1, b10 : 1, b11 : 1, b12 : 1, b13 : 1, b14 : 1, b15 : 1; } N16;
	union {
		bits_t b;
		typename IF<(N > 8), N16, N8>::type flag;
	};
	void reset() { b = 0; }
	void set(const int n, const bool onoff) { onoff ? set(n) : clear(n); }
	void set(const int n) { b |= (bits_t)_BV(n); }
	void clear(const int n) { b &= ~(bits_t)_BV(n); }
	bool test(const int n) const { return TEST(b, n); }
	bool operator[](const int n) { return test(n); }
	bool operator[](const int n) const { return test(n); }
	int size() const { return sizeof(b); }
};
typedef struct AxisFlags {
	union {
		struct Flags<3> flags;
		struct { bool x : 1, y : 1, z : 1; };
	};
	void reset() { flags.reset(); }
	void set(const int n) { flags.set(n); }
	void set(const int n, const bool onoff) { flags.set(n, onoff); }
	void clear(const int n) { flags.clear(n); }
	bool test(const int n) const { return flags.test(n); }
	bool operator[](const int n) { return flags[n]; }
	bool operator[](const int n) const { return flags[n]; }
	int size() const { return sizeof(flags); }
} axis_flags_t;


//#define X_AXIS 0
//#define Y_AXIS 1
//#define Z_AXIS 2
#define XY 2
#define NUM_AXES 3
#define LOGICAL_AXES 4
#define DISTINCT_AXES 4
//#define A_AXIS X_AXIS
//#define B_AXIS Y_AXIS
//#define C_AXIS Z_AXIS

// types.h
typedef float feedRate_t;
// On AVR pointers are only 2 bytes so use 'const float &' for 'const float'
#ifdef __AVR__
typedef const float& const_float_t;
#else
typedef const float const_float_t;
#endif
typedef const_float_t const_feedRate_t;
typedef const_float_t const_celsius_float_t;
// Conversion macros
#define MMM_TO_MMS(MM_M) feedRate_t(static_cast<float>(MM_M) / 60.0f)
#define MMS_TO_MMM(MM_S) (static_cast<float>(MM_S) * 60.0f)

// Helpers
#define _RECIP(N) ((N) ? 1.0f / static_cast<float>(N) : 0.0f)
#define _ABS(N) ((N) < 0 ? -(N) : (N))
#define _LS(N)  (N = (T)(uint32_t(N) << p))
#define _RS(N)  (N = (T)(uint32_t(N) >> p))
#define FORCE_INLINE
#define FI FORCE_INLINE

// Forward declarations
template<typename T> struct XYval;
template<typename T> struct XYZval;
template<typename T> struct XYZEval;
typedef struct XYval<bool>          xy_bool_t;
typedef struct XYZval<bool>        xyz_bool_t;
typedef struct XYZEval<bool>      xyze_bool_t;
typedef struct XYval<char>          xy_char_t;
typedef struct XYZval<char>        xyz_char_t;
typedef struct XYZEval<char>      xyze_char_t;
typedef struct XYval<unsigned char>     xy_uchar_t;
typedef struct XYZval<unsigned char>   xyz_uchar_t;
typedef struct XYZEval<unsigned char> xyze_uchar_t;
typedef struct XYval<int8_t>        xy_int8_t;
typedef struct XYZval<int8_t>      xyz_int8_t;
typedef struct XYZEval<int8_t>    xyze_int8_t;
typedef struct XYval<uint8_t>      xy_uint8_t;
typedef struct XYZval<uint8_t>    xyz_uint8_t;
typedef struct XYZEval<uint8_t>  xyze_uint8_t;
typedef struct XYval<int16_t>        xy_int_t;
typedef struct XYZval<int16_t>      xyz_int_t;
typedef struct XYZEval<int16_t>    xyze_int_t;
typedef struct XYval<uint16_t>      xy_uint_t;
typedef struct XYZval<uint16_t>    xyz_uint_t;
typedef struct XYZEval<uint16_t>  xyze_uint_t;
typedef struct XYval<int32_t>       xy_long_t;
typedef struct XYZval<int32_t>     xyz_long_t;
typedef struct XYZEval<int32_t>   xyze_long_t;
typedef struct XYval<uint32_t>     xy_ulong_t;
typedef struct XYZval<uint32_t>   xyz_ulong_t;
typedef struct XYZEval<uint32_t> xyze_ulong_t;
typedef struct XYZval<volatile int32_t>   xyz_vlong_t;
typedef struct XYZEval<volatile int32_t> xyze_vlong_t;
typedef struct XYval<float>        xy_float_t;
typedef struct XYZval<float>      xyz_float_t;
typedef struct XYZEval<float>    xyze_float_t;

typedef struct XYval<feedRate_t>     xy_feedrate_t;
typedef struct XYZval<feedRate_t>   xyz_feedrate_t;
typedef struct XYZEval<feedRate_t> xyze_feedrate_t;

typedef xy_uint8_t xy_byte_t;
typedef xyz_uint8_t xyz_byte_t;
typedef xyze_uint8_t xyze_byte_t;

typedef xyz_long_t abc_long_t;
typedef xyze_long_t abce_long_t;
typedef xyz_ulong_t abc_ulong_t;
typedef xyze_ulong_t abce_ulong_t;

typedef xy_float_t xy_pos_t;
typedef xyz_float_t xyz_pos_t;
typedef xyze_float_t xyze_pos_t;

typedef xy_float_t ab_float_t;
typedef xyz_float_t abc_float_t;
typedef xyze_float_t abce_float_t;

typedef ab_float_t ab_pos_t;
typedef abc_float_t abc_pos_t;
typedef abce_float_t abce_pos_t;

// External conversion methods
/*
void toLogical(xy_pos_t& raw);
void toLogical(xyz_pos_t& raw);
void toLogical(xyze_pos_t& raw);
void toNative(xy_pos_t& lpos);
void toNative(xyz_pos_t& lpos);
void toNative(xyze_pos_t& lpos);
*/
// paired xy
template<typename T>
struct XYval {
	union {
		struct { T x, y; };
		struct { T a, b; };
		T pos[2];
	};
	// Set all to 0
	FI void reset() { x = y = 0; }
	// Setters taking struct types and arrays
	FI void set(const T px) { x = px; }
	FI void set(const T px, const T py) { x = px; y = py; }
	FI void set(const T(&arr)[XY]) { x = arr[0]; y = arr[1]; }
	FI void set(const T(&arr)[NUM_AXES]) { x = arr[0]; y = arr[1]; }
	FI T magnitude()    const { return (T)sqrtf(x * x + y * y); }
	FI operator T* ()	{ return pos; }
	FI operator bool()	{ return x || y; }
	FI T small()        const { return _MIN(x, y); }
	FI T large()        const { return _MAX(x, y); }
	// Explicit copy and copies with conversion
	FI XYval<T>           copy() const { return *this; }
	FI XYval<T>            ABS() const { return { T(_ABS(x)), T(_ABS(y)) }; }
	FI XYval<int16_t>    asInt() { return { int16_t(x), int16_t(y) }; }
	FI XYval<int16_t>    asInt() const { return { int16_t(x), int16_t(y) }; }
	FI XYval<int32_t>   asLong() { return { int32_t(x), int32_t(y) }; }
	FI XYval<int32_t>   asLong() const { return { int32_t(x), int32_t(y) }; }
	FI XYval<int32_t>   ROUNDL() { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
	FI XYval<int32_t>   ROUNDL() const { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
	FI XYval<float>    asFloat() { return { static_cast<float>(x), static_cast<float>(y) }; }
	FI XYval<float>    asFloat() const { return { static_cast<float>(x), static_cast<float>(y) }; }
	FI XYval<float> reciprocal() const { return { _RECIP(x), _RECIP(y) }; }
	// Marlin workspace shifting is done with G92 and M206
	FI XYval<float>  asLogical() const { XYval<float> o = asFloat(); toLogical(o); return o; }
	FI XYval<float>   asNative() const { XYval<float> o = asFloat(); toNative(o);  return o; }
	// Cast to a type with more fields by making a new object
	FI operator XYZval<T>()  { return { x, y }; }
	FI operator XYZval<T>()  const { return { x, y }; }
	FI operator XYZEval<T>() { return { x, y }; }
	FI operator XYZEval<T>() const { return { x, y }; }
	// Accessor via an AxisEnum (or any integer) [index]
	FI       T& operator[](const int n) { return pos[n]; }
	FI const T& operator[](const int n) const { return pos[n]; }
	// Assignment operator overrides do the expected thing
	FI XYval<T>& operator= (const T v) { set(v, v); return *this; }
	FI XYval<T>& operator= (const XYZval<T>& rs) { set(rs.x, rs.y); return *this; }
	FI XYval<T>& operator= (const XYZEval<T>& rs) { set(rs.x, rs.y); return *this; }
	// Override other operators to get intuitive behaviors
	FI XYval<T>  operator+ (const XYval<T>& rs)	const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator+ (const XYval<T>& rs) { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator- (const XYval<T>& rs) const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator- (const XYval<T>& rs) { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator* (const XYval<T>& rs) const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator* (const XYval<T>& rs) { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYval<T>& rs) const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYval<T>& rs) { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZval<T>& rs)const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator- (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator- (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZEval<T>& rs)	{ XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator- (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator- (const XYZEval<T>& rs)	{ XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZEval<T>& rs)	{ XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZEval<T>& rs)	{ XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator* (const float& p)         const { XYval<T> ls = *this; ls.x *= p; ls.y *= p; return ls; }
	FI XYval<T>  operator* (const float& p)			{ XYval<T> ls = *this; ls.x *= p; ls.y *= p; return ls; }
	FI XYval<T>  operator* (const int& p)           const { XYval<T> ls = *this; ls.x *= p; ls.y *= p; return ls; }
	FI XYval<T>  operator* (const int& p)			{ XYval<T> ls = *this; ls.x *= p; ls.y *= p; return ls; }
	FI XYval<T>  operator/ (const float& p)         const { XYval<T> ls = *this; ls.x /= p; ls.y /= p; return ls; }
	FI XYval<T>  operator/ (const float& p)			{ XYval<T> ls = *this; ls.x /= p; ls.y /= p; return ls; }
	FI XYval<T>  operator/ (const int& p)           const { XYval<T> ls = *this; ls.x /= p; ls.y /= p; return ls; }
	FI XYval<T>  operator/ (const int& p)			{ XYval<T> ls = *this; ls.x /= p; ls.y /= p; return ls; }
	FI XYval<T>  operator>>(const int& p)           const { XYval<T> ls = *this; _RS(ls.x); _RS(ls.y); return ls; }
	FI XYval<T>  operator>>(const int& p)			{ XYval<T> ls = *this; _RS(ls.x); _RS(ls.y); return ls; }
	FI XYval<T>  operator<<(const int& p)           const { XYval<T> ls = *this; _LS(ls.x); _LS(ls.y); return ls; }
	FI XYval<T>  operator<<(const int& p)			{ XYval<T> ls = *this; _LS(ls.x); _LS(ls.y); return ls; }
	FI const XYval<T> operator-()                   const { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
	FI XYval<T>       operator-()					{ XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
	// Modifier operators
	FI XYval<T>& operator+=(const XYval<T>& rs) { x += rs.x; y += rs.y; return *this; }
	FI XYval<T>& operator-=(const XYval<T>& rs) { x -= rs.x; y -= rs.y; return *this; }
	FI XYval<T>& operator*=(const XYval<T>& rs) { x *= rs.x; y *= rs.y; return *this; }
	FI XYval<T>& operator/=(const XYval<T>& rs) { x /= rs.x; y /= rs.y; return *this; }
	FI XYval<T>& operator+=(const XYZval<T>& rs) { x += rs.x; y += rs.y; return *this; }
	FI XYval<T>& operator-=(const XYZval<T>& rs) { x -= rs.x; y -= rs.y; return *this; }
	FI XYval<T>& operator*=(const XYZval<T>& rs) { x *= rs.x; y *= rs.y; return *this; }
	FI XYval<T>& operator/=(const XYZval<T>& rs) { x /= rs.x; y /= rs.y; return *this; }
	FI XYval<T>& operator+=(const XYZEval<T>& rs) { x += rs.x; y += rs.y; return *this; }
	FI XYval<T>& operator-=(const XYZEval<T>& rs) { x -= rs.x; y -= rs.y; return *this; }
	FI XYval<T>& operator*=(const XYZEval<T>& rs) { x *= rs.x; y *= rs.y; return *this; }
	FI XYval<T>& operator/=(const XYZEval<T>& rs) { x /= rs.x; y /= rs.y; return *this; }
	FI XYval<T>& operator*=(const float& p) { x *= p; y *= p; return *this; }
	FI XYval<T>& operator*=(const int& p) { x *= p; y *= p; return *this; }
	FI XYval<T>& operator>>=(const int& p) { _RS(x); _RS(y); return *this; }
	FI XYval<T>& operator<<=(const int& p) { _LS(x); _LS(y); return *this; }
	// Exact comparisons. For floats a "NEAR" operation may be better.
	FI bool operator==(const XYval<T>& rs)   const { return x == rs.x && y == rs.y; }
	FI bool operator==(const XYZval<T>& rs)  const { return x == rs.x && y == rs.y; }
	FI bool operator==(const XYZEval<T>& rs) const { return x == rs.x && y == rs.y; }
	FI bool operator!=(const XYval<T>& rs)   const { return !operator==(rs); }
	FI bool operator!=(const XYZval<T>& rs)  const { return !operator==(rs); }
	FI bool operator!=(const XYZEval<T>& rs) const { return !operator==(rs); }
};//XYval

// linear axies
template<typename T>
struct XYZval {
	union {
		struct { T x, y, z; };
		struct { T a, b, c; };
		T pos[3];
	};
	// Set all to 0
	FI void reset() { x = y = z = 0; }
	// Setters taking struct types and arrays
	FI void set(const XYval<T> pxy) { x = pxy.x; y = pxy.y; }
	FI void set(const XYval<T> pxy, const T pz) { x = pxy.x; y = pxy.y; z = pz; }
	FI void set(const T(&arr)[NUM_AXES]) { x = arr[0]; y = arr[1]; z = arr[2]; }
	// Setter for all individual args
	FI void set(const T x, const T y, const T z) { a = x; b = y; c = z; }
	// Setters with fewer elements leave the rest untouched
	FI void set(const T px) { x = px; }
	FI void set(const T px, const T py) { x = px; y = py; }
	//FI void set(const T px, const T py, const T pz) { x = px; y = py; z = pz; }
	// Length reduced to one dimension
	FI T magnitude() const { return (T)sqrtf(x * x + y * y + z * z); }
	// Pointer to the data as a simple array
	FI operator T* () { return pos; }
	// If any element is true then it's true
	FI operator bool() { return x || y || z; }
	FI T small() const { return _MIN3(x, y, z); }
	FI T large() const { return _MAX3(x, y, z); }
	// Explicit copy and copies with conversion
	FI XYZval<T>          copy() const { XYZval<T> o = *this; return o; }
	FI XYZval<T>           ABS() const { return { T(_ABS(x)), T(_ABS(y)), T(_ABS(z)) }; }
	FI XYZval<int16_t>   asInt() { return { int16_t(x), int16_t(y), int16_t(z) }; }
	FI XYZval<int16_t>   asInt() const { return { int16_t(x), int16_t(y), int16_t(z) }; }
	FI XYZval<int32_t>  asLong() { return { int32_t(x), int32_t(y), int32_t(z) }; }
	FI XYZval<int32_t>  asLong() const { return { int32_t(x), int32_t(y), int32_t(z) }; }
	FI XYZval<int32_t>  ROUNDL() { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)) }; }
	FI XYZval<int32_t>  ROUNDL() const { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)) }; }
	FI XYZval<float>   asFloat() { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) }; }
	FI XYZval<float>   asFloat() const { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) }; }
	FI XYZval<float> reciprocal() const { return { _RECIP(x), _RECIP(y), _RECIP(z) }; }
	// Marlin workspace shifting is done with G92 and M206
	FI XYZval<float> asLogical() const { XYZval<float> o = asFloat(); toLogical(o); return o; }
	FI XYZval<float>  asNative() const { XYZval<float> o = asFloat(); toNative(o);  return o; }
	// In-place cast to types having fewer fields
	FI operator XYval<T>& () { return *(XYval<T>*)this; }
	FI operator const XYval<T>& () const { return *(const XYval<T>*)this; }
	// Cast to a type with more fields by making a new object
	FI operator XYZEval<T>() const { return { x, y, z }; }
	// Accessor via an AxisEnum (or any integer) [index]
	FI       T& operator[](const int n) { return pos[n]; }
	FI const T& operator[](const int n)          const { return pos[n]; }
	// Assignment operator overrides do the expected thing
	FI XYZval<T>& operator= (const T v) { set(ARRAY_N_1(NUM_AXES, v)); return *this; }
	FI XYZval<T>& operator= (const XYval<T>& rs) { set(rs.x, rs.y); return *this; }
	FI XYZval<T>& operator= (const XYZEval<T>& rs) { set(NUM_AXIS_ELEM(rs)); return *this; }
	// Override other operators to get intuitive behaviors
	FI XYZval<T>  operator+ (const XYval<T>& rs) const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYZval<T>  operator+ (const XYval<T>& rs) { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYZval<T>  operator- (const XYval<T>& rs) const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYZval<T>  operator- (const XYval<T>& rs) { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYZval<T>  operator* (const XYval<T>& rs) const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYZval<T>  operator* (const XYval<T>& rs) { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYZval<T>  operator/ (const XYval<T>& rs) const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYZval<T>  operator/ (const XYval<T>& rs) { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYZval<T>  operator+ (const XYZval<T>& rs) const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
	FI XYZval<T>  operator+ (const XYZval<T>& rs) { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
	FI XYZval<T>  operator- (const XYZval<T>& rs) const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
	FI XYZval<T>  operator- (const XYZval<T>& rs) { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
	FI XYZval<T>  operator* (const XYZval<T>& rs) const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZval<T>  operator* (const XYZval<T>& rs) { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZval<T>  operator/ (const XYZval<T>& rs) const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
	FI XYZval<T>  operator/ (const XYZval<T>& rs) { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
	FI XYZval<T>  operator+ (const XYZEval<T>& rs) const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
	FI XYZval<T>  operator+ (const XYZEval<T>& rs) { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
	FI XYZval<T>  operator- (const XYZEval<T>& rs) const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
	FI XYZval<T>  operator- (const XYZEval<T>& rs) { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
	FI XYZval<T>  operator* (const XYZEval<T>& rs) const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZval<T>  operator* (const XYZEval<T>& rs) { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZval<T>  operator/ (const XYZEval<T>& rs) const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
	FI XYZval<T>  operator/ (const XYZEval<T>& rs) { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
	FI XYZval<T>  operator* (const float& p)       const { XYZval<T> ls = *this; ls.x *= p; ls.y *= p; ls.z *= p; return ls; }
	FI XYZval<T>  operator* (const float& p)	   { XYZval<T> ls = *this; ls.x *= p; ls.y *= p; ls.z *= p; return ls; }
	FI XYZval<T>  operator* (const int& p)         const { XYZval<T> ls = *this; ls.x *= p; ls.y *= p; ls.z *= p; return ls; }
	FI XYZval<T>  operator* (const int& p)		   { XYZval<T> ls = *this; ls.x *= p; ls.y *= p; ls.z *= p; return ls; }
	FI XYZval<T>  operator/ (const float& p)       const { XYZval<T> ls = *this; ls.x /= p; ls.y /= p; ls.z /= p; return ls; }
	FI XYZval<T>  operator/ (const float& p) { XYZval<T> ls = *this; ls.x /= p; ls.y /= p; ls.z /= p; return ls; }
	FI XYZval<T>  operator/ (const int& p)         const { XYZval<T> ls = *this; ls.x /= p; ls.y /= p; ls.z /= p; return ls; }
	FI XYZval<T>  operator/ (const int& p)		   { XYZval<T> ls = *this; ls.x /= p; ls.y /= p; ls.z /= p; return ls; }
	FI XYZval<T>  operator>>(const int& p)         const { XYZval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); return ls; }
	FI XYZval<T>  operator>>(const int& p)		   { XYZval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); return ls; }
	FI XYZval<T>  operator<<(const int& p)         const { XYZval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); return ls; }
	FI XYZval<T>  operator<<(const int& p)		   { XYZval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); }
	FI const XYZval<T> operator-()                 const { XYZval<T> o = *this; o.x = -x; o.y = -y; o.z = -z; return o; }
	FI XYZval<T>       operator-() { XYZval<T> o = *this; o.x = -x; o.y = -y; o.z = -z; return o; }
	// Modifier operators
	FI XYZval<T>& operator+=(const XYval<T>& rs) { x += rs.x; y += rs.y; return *this; }
	FI XYZval<T>& operator-=(const XYval<T>& rs) { x -= rs.x; y -= rs.y; return *this; }
	FI XYZval<T>& operator*=(const XYval<T>& rs) { x *= rs.x; y *= rs.y; return *this; }
	FI XYZval<T>& operator/=(const XYval<T>& rs) { x /= rs.x; y /= rs.y; return *this; }
	FI XYZval<T>& operator+=(const XYZval<T>& rs) { x += rs.x; y += rs.y; z += rs.z; return *this; }
	FI XYZval<T>& operator-=(const XYZval<T>& rs) { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
	FI XYZval<T>& operator*=(const XYZval<T>& rs) { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
	FI XYZval<T>& operator/=(const XYZval<T>& rs) { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
	FI XYZval<T>& operator+=(const XYZEval<T>& rs) { x += rs.x; y += rs.y; z += rs.z; return *this; }
	FI XYZval<T>& operator-=(const XYZEval<T>& rs) { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
	FI XYZval<T>& operator*=(const XYZEval<T>& rs) { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
	FI XYZval<T>& operator/=(const XYZEval<T>& rs) { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
	FI XYZval<T>& operator*=(const float& p) { x *= p, y *= p, z *= p; return *this; }
	FI XYZval<T>& operator*=(const int& p) { x *= p, y *= p, z *= p; return *this; }
	FI XYZval<T>& operator>>=(const int& p) { _RS(x); _RS(y); _RS(z); return *this; }
	FI XYZval<T>& operator<<=(const int& p) { _LS(x); _LS(y); _LS(z); return *this; }
	// Exact comparisons. For floats a "NEAR" operation may be better.
	FI bool       operator==(const XYZEval<T>& rs) const { return true && x == rs.x && y == rs.y && z == rs.z; }
	FI bool       operator!=(const XYZEval<T>& rs) const { return !operator==(rs); }
};//XYZval

// logical axes
template<typename T>
struct XYZEval {
	union {
		struct { T x, y, z, e; };
		struct { T a, b, c, _e; };
		T pos[LOGICAL_AXES];
	};
	// Reset all to 0
	FI void reset() { x = y = z = e = 0; }
	// Setters taking struct types and arrays
	FI void set(const XYval<T> pxy) { x = pxy.x; y = pxy.y; }
	FI void set(const XYZval<T> pxyz) { set(pxyz.x, pxyz.y, pxyz.z); }
	FI void set(const XYval<T> pxy, const T pz) { set(pxy.x, pxy.y); z = pz; }
	FI void set(const T(&arr)[NUM_AXES]) { x = arr[0]; y = arr[1]; z = arr[2]; }
	FI void set(const T(&arr)[LOGICAL_AXES]) { e = arr[LOGICAL_AXES - 1]; x = arr[0]; y = arr[1]; z = arr[2]; }
	FI void set(const XYval<T> pxy, const T pz, const T pe) { set(pxy, pz); e = pe; }
	FI void set(const XYZval<T> pxyz, const T pe) { set(pxyz); e = pe; }
	FI void set(const T x, const T y, const T z, const T e) { a = x; b = y; c = z; _e = e; }
	// Setter for all individual args
	FI void set(const T x, const T y, const T z) { a = x; b = y; c = z; }
	// Setters with fewer elements leave the rest untouched
	FI void set(const T px) { x = px; }
	FI void set(const T px, const T py) { x = px; y = py; }
	FI T magnitude() const { return (T)sqrtf(x * x + y * y + z * z + e * e); }
	FI operator T* () { return pos; }
	FI operator bool() { return 0 || x || y || z || e; }
	FI T small()					const { return _MIN4(x, y, z, e); }
	FI T large()					const { return _MAX4(x, y, z, e); }
	// Explicit copy and copies with conversion
	FI XYZEval<T>          copy()	const { XYZEval<T> v = *this; return v; }
	FI XYZEval<T>           ABS()	const { return { T(_ABS(x)), T(_ABS(y)), T(_ABS(z)), T(_ABS(e)) }; }
	FI XYZEval<int16_t>   asInt()	{ return { int16_t(x), int16_t(y), int16_t(z), int16_t(e) }; }
	FI XYZEval<int16_t>   asInt()	const { return { int16_t(x), int16_t(y), int16_t(z), int16_t(e) }; }
	FI XYZEval<int32_t>  asLong() { return { int32_t(x), int32_t(y), int32_t(z), int32_t(e) }; }
	FI XYZEval<int32_t>  asLong()  const { return { int32_t(x), int32_t(y), int32_t(z), int32_t(e) }; }
	FI XYZEval<int32_t>  ROUNDL() { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(e)) }; }
	FI XYZEval<int32_t>  ROUNDL()  const { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(e)) }; }
	FI XYZEval<float>   asFloat() { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(e) }; }
	FI XYZEval<float>   asFloat()  const { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(e) }; }
	FI XYZEval<float> reciprocal() const { return { _RECIP(x), _RECIP(y), _RECIP(z), _RECIP(e) }; }
	// Marlin workspace shifting is done with G92 and M206
	FI XYZEval<float> asLogical()  const { XYZEval<float> o = asFloat(); toLogical(o); return o; }
	FI XYZEval<float>  asNative()  const { XYZEval<float> o = asFloat(); toNative(o);  return o; }
	// In-place cast to types having fewer fields
	FI operator XYval<T>& () { return *(XYval<T>*)this; }
	FI operator const XYval<T>& ()  const { return *(const XYval<T>*)this; }
	FI operator XYZval<T>& () { return *(XYZval<T>*)this; }
	FI operator const XYZval<T>& () const { return *(const XYZval<T>*)this; }
	// Accessor via an AxisEnum (or any integer) [index]
	FI       T& operator[](const int n) { return pos[n]; }
	FI const T& operator[](const int n)          const { return pos[n]; }
	// Assignment operator overrides do the expected thing
	FI XYZEval<T>& operator= (const T v) { set(v, v, v, v); return *this; }
	FI XYZEval<T>& operator= (const XYval<T>& rs) { set(rs.x, rs.y); return *this; }
	FI XYZEval<T>& operator= (const XYZval<T>& rs) { set(NUM_AXIS_ELEM(rs)); return *this; }
	// Override other operators to get intuitive behaviors
	FI XYZEval<T>  operator+ (const XYval<T>& rs)  const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYZEval<T>  operator+ (const XYval<T>& rs) { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYZEval<T>  operator- (const XYval<T>& rs)  const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYZEval<T>  operator- (const XYval<T>& rs) { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYZEval<T>  operator* (const XYval<T>& rs)  const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYZEval<T>  operator* (const XYval<T>& rs) { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYZEval<T>  operator/ (const XYval<T>& rs)  const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYZEval<T>  operator/ (const XYval<T>& rs) { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYZEval<T>  operator+ (const XYZval<T>& rs)  const { XYZval<T>  ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
	FI XYZEval<T>  operator+ (const XYZval<T>& rs) { XYZval<T>  ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z ; return ls; }
	FI XYZEval<T>  operator- (const XYZval<T>& rs)  const { XYZval<T>  ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z ; return ls; }
	FI XYZEval<T>  operator- (const XYZval<T>& rs) { XYZval<T>  ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
	FI XYZEval<T>  operator* (const XYZval<T>& rs)  const { XYZval<T>  ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZEval<T>  operator* (const XYZval<T>& rs) { XYZval<T>  ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZEval<T>  operator/ (const XYZval<T>& rs)  const { XYZval<T>  ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
	FI XYZEval<T>  operator/ (const XYZval<T>& rs) { XYZval<T>  ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
	FI XYZEval<T>  operator+ (const XYZEval<T>& rs) const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; ls.e += rs.e; return ls; }
	FI XYZEval<T>  operator+ (const XYZEval<T>& rs) { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; ls.e += rs.e; return ls; }
	FI XYZEval<T>  operator- (const XYZEval<T>& rs) const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; ls.e -= rs.e; return ls; }
	FI XYZEval<T>  operator- (const XYZEval<T>& rs) { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; ls.e -= rs.e; return ls; }
	FI XYZEval<T>  operator* (const XYZEval<T>& rs) const { XYZEval<T> ls = *this; ls.e *= rs.e, ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
	FI XYZEval<T>  operator* (const XYZEval<T>& rs) { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; ls.e *= rs.e; return ls; }
	FI XYZEval<T>  operator/ (const XYZEval<T>& rs) const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; ls.e /= rs.e; return ls; }
	FI XYZEval<T>  operator/ (const XYZEval<T>& rs) { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; ls.e /= rs.e; return ls; }
	FI XYZEval<T>  operator* (const float& p)       const { XYZEval<T> ls = *this; ls.e *= p, ls.x *= p; ls.y *= p; ls.z *= p; return ls; }
	FI XYZEval<T>  operator* (const float& p) { XYZEval<T> ls = *this; ls.x *= p, ls.y *= p, ls.z *= p; ls.e *= p; return ls; }
	FI XYZEval<T>  operator* (const int& p)         const { XYZEval<T> ls = *this; ls.x *= p, ls.y *= p, ls.z *= p; ls.e *= p; return ls; }
	FI XYZEval<T>  operator* (const int& p) { XYZEval<T> ls = *this; ls.x *= p, ls.y *= p, ls.z *= p; ls.e *= p; return ls; }
	FI XYZEval<T>  operator/ (const float& p)       const { XYZEval<T> ls = *this; ls.x /= p, ls.y /= p, ls.z /= p; ls.e /= p; return ls; }
	FI XYZEval<T>  operator/ (const float& p) { XYZEval<T> ls = *this; ls.x /= p, ls.y /= p, ls.z /= p; ls.e /= p; return ls; }
	FI XYZEval<T>  operator/ (const int& p)         const { XYZEval<T> ls = *this; ls.x /= p, ls.y /= p, ls.z /= p; ls.e /= p; return ls; }
	FI XYZEval<T>  operator/ (const int& p) { XYZEval<T> ls = *this; ls.x /= p, ls.y /= p, ls.z /= p; ls.e /= p; return ls; }
	FI XYZEval<T>  operator>>(const int& p)         const { XYZEval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); _RS(ls.e); return ls; }
	FI XYZEval<T>  operator>>(const int& p) { XYZEval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); _RS(ls.e); return ls; }
	FI XYZEval<T>  operator<<(const int& p)         const { XYZEval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); _LS(ls.e); return ls; }
	FI XYZEval<T>  operator<<(const int& p) { XYZEval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); _LS(ls.e); return ls; }
	FI const XYZEval<T> operator-()                 const { return { -x, -y, -z, -e }; }
	FI       XYZEval<T> operator-() { return { -x, -y, -z, -e }; }
	// Modifier operators
	FI XYZEval<T>& operator+=(const XYval<T>& rs) { x += rs.x; y += rs.y; return *this; }
	FI XYZEval<T>& operator-=(const XYval<T>& rs) { x -= rs.x; y -= rs.y; return *this; }
	FI XYZEval<T>& operator*=(const XYval<T>& rs) { x *= rs.x; y *= rs.y; return *this; }
	FI XYZEval<T>& operator/=(const XYval<T>& rs) { x /= rs.x; y /= rs.y; return *this; }
	FI XYZEval<T>& operator+=(const XYZval<T>& rs) { x += rs.x; y += rs.y; z += rs.z; return *this; }
	FI XYZEval<T>& operator-=(const XYZval<T>& rs) { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
	FI XYZEval<T>& operator*=(const XYZval<T>& rs) { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
	FI XYZEval<T>& operator/=(const XYZval<T>& rs) { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
	FI XYZEval<T>& operator+=(const XYZEval<T>& rs) { x += rs.x; y += rs.y; z += rs.z; e += rs.e; return *this; }
	FI XYZEval<T>& operator-=(const XYZEval<T>& rs) { x -= rs.x; y -= rs.y; z -= rs.z; e -= rs.e; return *this; }
	FI XYZEval<T>& operator*=(const XYZEval<T>& rs) { x *= rs.x; y *= rs.y; z *= rs.z; e *= rs.e; return *this; }
	FI XYZEval<T>& operator/=(const XYZEval<T>& rs) { x /= rs.x; y /= rs.y; z /= rs.z; e /= rs.e; return *this; }
	FI XYZEval<T>& operator*=(const T& p) { x *= p; y *= p; z *= p; e *= p; return *this; }
	FI XYZEval<T>& operator>>=(const int& p) { _RS(x); _RS(y); _RS(z); _RS(e); return *this; }
	FI XYZEval<T>& operator<<=(const int& p) { _LS(x); _LS(y); _LS(z); _LS(e); return *this; }
	// Exact comparisons. For floats a "NEAR" operation may be better.
	FI bool operator==(const XYZval<T>& rs) const { return true && x == rs.x && y == rs.y && z == rs.z; }
	FI bool operator==(const XYZEval<T>& rs) const { return true && x == rs.x && y == rs.y && z == rs.z && e == rs.e; }
	FI bool operator!=(const XYZval<T>& rs) const { return !operator==(rs); }
	FI bool operator!=(const XYZEval<T>& rs) const { return !operator==(rs); }
};//XYZEval



// queue.h - The G-code command queue, which holds commands before they
//           go to the parser and dispatcher.
// configuration_adv.h
// @section serial
// The ASCII buffer for serial input
#define MAX_CMD_SIZE 96

#define BUFSIZE 4
#define NUM_SERIAL 1

// serial_base.h
// Used in multiple places
// You can build it but not manipulate it.
// There are only few places where it's required to access the underlying member: GCodeQueue, SerialMask and MultiSerial
struct serial_index_t {
	// A signed index, where -1 is a special case meaning no action (neither output or input)
	int8_t  index;
	// Check if the index is within the range [a ... b]
	constexpr inline bool within(const int8_t a, const int8_t b) const { return WITHIN(index, a, b); }
	constexpr inline bool valid() const { return WITHIN(index, 0, 7); } // At most, 8 bits
	// Construction is either from an index
	constexpr serial_index_t(const int8_t index) : index(index) {}
	// Default to "no index"
	constexpr serial_index_t() : index(-1) {}
};

// pgmspaces.h
#define PGM_P const char *

class GCodeQueue {
public:
	 // The buffers per serial port.
	struct SerialState {
		/**
		 * GCode line number handling. Hosts may include line numbers when sending
		 * commands to Marlin, and lines will be checked for sequentiality.
		 * M110 N<int> sets the current line number.
		 */
		long last_N;
		int count;                      //!< Number of characters read in the current line of serial input
		char line_buffer[MAX_CMD_SIZE]; //!< The current line accumulator
		uint8_t input_state;            //!< The input state
	};
	static SerialState serial_state[NUM_SERIAL]; //!< Serial states for each serial port
	/**
	 * GCode Command Queue
	 * A simple (circular) ring buffer of BUFSIZE command strings.
	 * Commands are copied into this buffer by the command injectors
	 * (immediate, serial, sd card) and they are processed sequentially by
	 * the main loop. The gcode.process_next_command method parses the next
	 * command and hands off execution to individual handler functions.
	 */
	struct CommandLine {
		char buffer[MAX_CMD_SIZE];	//!< The command buffer
		bool skip_ok;               //!< Skip sending ok when command is processed?
	};
	// A handy ring buffer type
	struct RingBuffer {
		uint8_t length,             //!< Number of commands in the queue
			index_r,                //!< Ring buffer's read position
			index_w;                //!< Ring buffer's write position
		CommandLine commands[BUFSIZE];  //!< The ring buffer of commands
		inline serial_index_t command_port() const { return 0; }
		inline void clear() { length = index_r = index_w = 0; }
		void advance_pos(uint8_t& p, const int inc) { if (++p >= BUFSIZE) p = 0; length += inc; }
		void commit_command(bool skip_ok);
		bool enqueue(const char* cmd, bool skip_ok = true);
		void ok_to_send();
		inline bool full(uint8_t cmdCount = 1) const { return length > (BUFSIZE - cmdCount); }
		inline bool occupied() const { return length != 0; }
		inline bool empty() const { return !occupied(); }
		inline CommandLine& peek_next_command() { return commands[index_r]; }
		inline char* peek_next_command_string() { return peek_next_command().buffer; }
	};
	static RingBuffer ring_buffer;
	static void clear() { ring_buffer.clear(); }
	// Next Injected Command (PROGMEM) pointer. (nullptr == empty)
	// Internal commands are enqueued ahead of serial / SD commands.
	static PGM_P injected_commands_P;
	// Injected Commands (SRAM)
	static char injected_commands[64];
	// Enqueue command(s) to run from PROGMEM. Drained by process_injected_command_P().
	// Don't inject comments or use leading spaces!
	// Aborts the current PROGMEM queue so only use for one or two commands.
	static void inject_P(PGM_P const pgcode) { injected_commands_P = pgcode; }
	//static void inject(FSTR_P const fgcode) { inject_P(FTOP(fgcode)); }
	// Enqueue command(s) to run from SRAM. Drained by process_injected_command().
	// Aborts the current SRAM queue so only use for one or two commands.
	static void inject(const char* const gcode) {
		strncpy_s(injected_commands, gcode, sizeof(injected_commands) - 1);
	}
	// Enqueue and return only when commands are actually enqueued
	static void enqueue_one_now(const char* const cmd);
	//static void enqueue_one_now(FSTR_P const fcmd);
	// Attempt to enqueue a single G-code command
	// and return 'true' if successful.
	//static bool enqueue_one(FSTR_P const fcmd);
	// Enqueue with Serial Echo
	// Return true on success
	static bool enqueue_one(const char* cmd);
	// Enqueue from program memory and return only when commands are actually enqueued
	static void enqueue_now_P(PGM_P const pcmd);
	//static void enqueue_now(FSTR_P const fcmd) { enqueue_now_P(FTOP(fcmd)); }
	// Check whether there are any commands yet to be executed
	static bool has_commands_queued() { return ring_buffer.length || injected_commands_P || injected_commands[0]; }
	// Get the next command in the queue, optionally log it to SD, then dispatch it
	static void advance();
	// Run the entire queue in-place
	static void exhaust();
	/**
	 * Add to the circular command queue the next command from:
	 *  - The command-injection queue (injected_commands_P)
	 *  - The active serial input (usually USB)
	 *  - The SD card file being actively printed
	 */
	static void get_available_commands();
	/**
	 * Send an "ok" message to the host, indicating
	 * that a command was successfully processed.
	 * If ADVANCED_OK is enabled also include:
	 *   N<int>  Line number of the command, if any
	 *   P<int>  Planner space remaining
	 *   B<int>  Block queue space remaining
	 */
	static void ok_to_send() { ring_buffer.ok_to_send(); }
	/**
	 * Clear the serial line and request a resend of
	 * the next expected line number.
	 */
	static void flush_and_request_resend(const serial_index_t serial_ind);
	/**
	 * (Re)Set the current line number for the last received command
	 */
	static void set_current_line_number(long n) { serial_state[ring_buffer.command_port().index].last_N = n; }
	// BUFFER_MONITORING
private:
	static void get_serial_commands();
	//SDSUPPORT
	// Process the next "immediate" command (PROGMEM)
	static bool process_injected_command_P();
	// Process the next "immediate" command (SRAM)
	static bool process_injected_command();
	//static void gcode_line_error(FSTR_P const ferr, const serial_index_t serial_ind);
	friend class GcodeSuite;
};
extern GCodeQueue queue;
extern const char G28_STR[];



// motion
#define X_HOME_POS 100
#define Y_HOME_POS 200
#define Z_HOME_POS 300
extern xyze_pos_t current_position;
extern xyz_pos_t home_offset;
#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400}



// planner
// Planner block flags as boolean bit fields
enum BlockFlagBit {
	// Recalculate trapezoids on entry junction. For optimization.
	BLOCK_BIT_RECALCULATE,
	// Nominal speed always reached.
	// i.e., The segment is long enough, so the nominal speed is reachable if accelerating
	// from a safe speed (in consideration of jerking from zero speed).
	BLOCK_BIT_NOMINAL_LENGTH,
	// The block is segment 2+ of a longer move
	BLOCK_BIT_CONTINUED,
	// Sync the stepper counts from the block
	BLOCK_BIT_SYNC_POSITION
};

// Planner block flags as boolean bit fields
typedef struct _block_flags_t {
	union {
		uint8_t bits;
		struct {
			bool recalculate : 1;
			bool nominal_length : 1;
			bool continued : 1;
			bool sync_position : 1;
		};
	};
	void clear(void) volatile { bits = 0; }
	void apply(const uint8_t f) volatile { bits |= f; }
	void apply(const BlockFlagBit b) volatile { SBI(bits, b); }
	void reset(const BlockFlagBit b) volatile { bits = _BV(b); }
	void set_nominal(const bool n) volatile { recalculate = true; if (n) nominal_length = true; }
} block_flags_t;

/**
 * struct block_t
 * A single entry in the planner buffer.
 * Tracks linear movement over multiple axes.
 * The "nominal" values are as-specified by G-code, and
 * may never actually be reached due to acceleration limits.
 */
typedef struct PlannerBlock {
	volatile block_flags_t flag;
	bool is_sync() { return flag.sync_position; }
	bool is_move() { return !(is_sync()); }

	// Fields used by the motion planner to manage acceleration
	float nominal_speed,		// The nominal speed for this block in (mm/sec)
		entry_speed_sqr,        // Entry speed at previous-current junction in (mm/sec)^2
		max_entry_speed_sqr,    // Maximum allowable junction entry speed in (mm/sec)^2
		millimeters,            // The total travel of this block in mm
		acceleration;           // acceleration mm/sec^2
	union {
		abce_ulong_t steps;		// Step count along each axis
		abce_long_t position;	// New position to force when this sync block is executed
	};
	uint32_t step_event_count;	// The number of step events required to complete this block
	// Settings for the trapezoid generator
	uint32_t accelerate_until,	// The index of the step event on which to stop acceleration
		decelerate_after;       // The index of the step event on which to start decelerating
	uint32_t acceleration_rate;	// The acceleration rate used for acceleration calculation
	axis_bits_t direction_bits;	// The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
	uint32_t nominal_rate,		// The nominal step rate for this block in step_events/sec
		initial_rate,           // The jerk-adjusted step rate at start of block
		final_rate,             // The minimal rate at exit
		acceleration_steps_per_s2;// acceleration steps/sec^2
	void reset() { memset((char*)this, 0, sizeof(*this)); }
} block_t;//PlannerBlock

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))
#define BLOCK_BUFFER_SIZE 16
#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

typedef struct {
	uint32_t max_acceleration_mm_per_s2[DISTINCT_AXES], // (mm/s^2) M201 XYZE
		min_segment_time_us;							// (µs) M205 B
	float axis_steps_per_mm[DISTINCT_AXES];				// (steps) M92 XYZE - Steps per millimeter
	feedRate_t max_feedrate_mm_s[DISTINCT_AXES];		// (mm/s) M203 XYZE - Max speeds
	float acceleration,             // (mm/s^2) M204 S - Normal acceleration. DEFAULT ACCELERATION for all printing moves.
		retract_acceleration,       // (mm/s^2) M204 R - Retract acceleration. Filament pull-back and push-forward while standing still in the other axes
		travel_acceleration;        // (mm/s^2) M204 T - Travel acceleration. DEFAULT ACCELERATION for all NON printing moves.
	feedRate_t min_feedrate_mm_s,	// (mm/s) M205 S - Minimum linear feedrate
		min_travel_feedrate_mm_s;   // (mm/s) M205 T - Minimum travel feedrate
} planner_settings_t;

struct PlannerHints {
	float millimeters = 0.0;    // Move Length, if known, else 0.
	float inv_duration = 0.0;   // Reciprocal of the move duration, if known
	float curve_radius = 0.0;	// Radius of curvature of the motion path - to calculate cornering speed
	float safe_exit_speed_sqr = 0.0;// Square of the speed considered "safe" at the end of the segment
									// i.e., at or below the exit speed of the segment that the planner
									// would calculate if it knew the as-yet-unbuffered path
	PlannerHints(const_float_t mm = 0.0f) : millimeters(mm) {}
};

class Planner {
	public:
		/**
		 * The move buffer, calculated in stepper steps
		 * block_buffer is a ring buffer...
		 *             head,tail : indexes for write,read
		 *            head==tail : the buffer is empty
		 *            head!=tail : blocks are in the buffer
		 *   head==(tail-1)%size : the buffer is full
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
		static uint32_t max_acceleration_steps_per_s2[DISTINCT_AXES]; // (steps/s^2) Derived from mm_per_s2
		static float mm_per_step[DISTINCT_AXES];// Millimeters per step
		static xyze_long_t position; // [step]
	private:
		static xyze_float_t previous_speed;
		static float previous_nominal_speed; // Nominal speed of previous path line segment (mm/s)^2
		static uint32_t acceleration_long_cutoff; // Limit where 64bit math is necessary for acceleration calculation

	public:
		Planner();
		void init();
		// Recalculate steps/s^2 accelerations based on mm/s^2 settings
		static void refresh_acceleration_rates();
		// Recalculate 'position' and 'mm_per_step'.
		// Must be called whenever settings.axis_steps_per_mm changes!
		static void refresh_positioning();
		// For an axis set the Maximum Acceleration in mm/s^2
		static void set_max_acceleration(const AxisEnum axis, float inMaxAccelMMS2);
		// For an axis set the Maximum Feedrate in mm/s
		static void set_max_feedrate(const AxisEnum axis, float inMaxFeedrateMMS);
		// For an axis set the Maximum Jerk (instant change) in mm/s
		static void set_max_jerk(const AxisEnum, const_float_t) {}
		// Manage fans, paste pressure, etc.
		static void check_axes_activity();
		static float fade_scaling_factor_for_z(const_float_t) { return 1; }
		static bool leveling_active_at_z(const_float_t) { return true; }
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

		/**
		 * Planner::get_next_free_block
		 * - Get the next head indices (passed by reference)
		 * - Wait for the number of spaces to open up in the planner
		 * - Return the first head block
		 */
		static block_t* get_next_free_block(uint8_t& next_buffer_head, const uint8_t count = 1) {
			// Wait until there are enough slots free
			while (moves_free() < count) { idle(); }
			// Return the first available block
			next_buffer_head = next_block_index(block_buffer_head);
			return &block_buffer[block_buffer_head];
		}

		/**
		 * Planner::_buffer_steps
		 * Add a new linear movement to the buffer (in terms of steps).
		 *  target      - target position in steps units
		 *  fr_mm_s     - (target) speed of the move
		 *  extruder    - target extruder
		 *  hints       - parameters to aid planner calculations
		 * Returns true if movement was buffered, false otherwise
		 */
		static bool _buffer_steps(const xyze_long_t& target
			, feedRate_t fr_mm_s, const uint8_t extruder, const PlannerHints& hints
		);

		/**
		 * @brief Populate a block in preparation for insertion
		 * @details Populate the fields of a new linear movement block
		 *          that will be added to the queue and processed soon
		 *          by the Stepper ISR.
		 * @param block         A block to populate
		 * @param target        Target position in steps units
		 * @param target_float  Target position in native mm
		 * @param cart_dist_mm  The pre-calculated move lengths for all axes, in mm
		 * @param fr_mm_s       (target) speed of the move
		 * @param extruder      target extruder
		 * @param hints         parameters to aid planner calculations
		 * @return  true if movement is acceptable, false otherwise
		 */
		static bool _populate_block(block_t* const block, const xyze_long_t& target
			, feedRate_t fr_mm_s, const uint8_t extruder, const PlannerHints& hints
		);

		/**
		 * Planner::buffer_sync_block
		 * Add a block to the buffer that just updates the position
		 * @param sync_flag sets a condition bit to process additional items
		 * such as sync fan pwm or sync M3/M4 laser power into a queued block
		 */
		static void buffer_sync_block(const BlockFlagBit flag = BLOCK_BIT_SYNC_POSITION);

		/**
		 * Planner::buffer_segment
		 * Add a new linear movement to the buffer in axis units.
		 * Leveling and kinematics should be applied ahead of calling this.
		 *  a,b,c,e     - target positions in mm and/or degrees
		 *  fr_mm_s     - (target) speed of the move
		 *  extruder    - optional target extruder (otherwise active_extruder)
		 *  hints       - optional parameters to aid planner calculations
		 */
		static bool buffer_segment(const abce_pos_t& abce
			, const_feedRate_t fr_mm_s
			, const PlannerHints& hints = PlannerHints()
		);

		/**
		 * Add a new linear movement to the buffer.
		 * The target is cartesian. It's translated to
		 * delta/scara if needed.
		 *  cart         - target position in mm or degrees
		 *  fr_mm_s      - (target) speed of the move (mm/s)
		 *  extruder     - optional target extruder (otherwise active_extruder)
		 *  hints        - optional parameters to aid planner calculations
		 */
		static bool buffer_line(const xyze_pos_t& cart, const_feedRate_t fr_mm_s
			, const PlannerHints& hints = PlannerHints()
		);

		static void set_position_mm(const xyze_pos_t & pos);
		static void set_machine_position_mm(const abce_pos_t & pos);
		static float get_axis_position_mm(const AxisEnum axis);
		static abce_pos_t get_axis_positions_mm() {
			const abce_pos_t out = { get_axis_position_mm(A_AXIS), get_axis_position_mm(B_AXIS), get_axis_position_mm(C_AXIS), get_axis_position_mm(E_AXIS) };
			return out;
		}

		// Called to force a quick stop of the machine (for example, when
		// a Full Shutdown is required, or when endstops are hit)
		static void quick_stop();

		// Called when an endstop is triggered. Causes the machine to stop immediately
		static void endstop_triggered(const AxisEnum axis);

		// Triggered position of an axis in mm (not core-savvy)
		static float triggered_position_mm(const AxisEnum axis);

		// Blocks are queued, or we're running out moves, or the closed loop controller is waiting
		static bool busy();

		// Block until all buffered steps are executed / cleaned
		static void synchronize();

		// Wait for moves to finish and disable all steppers
		static void finish_and_disable();

		// Periodic handler to manage the cleaning buffer counter
		// Called from the Temperature ISR at ~1kHz
		static void isr() { if (cleaning_buffer_counter) --cleaning_buffer_counter; }

		// Does the buffer have any blocks queued?
		static bool has_blocks_queued() { return (block_buffer_head != block_buffer_tail); }
		/**
		 * Get the current block for processing
		 * and mark the block as busy.
		 * Return nullptr if the buffer is empty
		 * or if there is a first-block delay.
		 * WARNING: Called from Stepper ISR context!
		 */
		static block_t* get_current_block();

		/**
		 * "Release" the current block so its slot can be reused.
		 * Called when the current block is no longer needed.
		 */
		static void release_current_block() {
			if (has_blocks_queued())
				block_buffer_tail = next_block_index(block_buffer_tail);
		}


	private:
		// Get the index of the next / previous block in the ring buffer
		static constexpr uint8_t next_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index + 1); }
		static constexpr uint8_t prev_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index - 1); }


};//class Panner
extern Planner planner;



// motion
void sync_plan_position(void);



// stepper
/*
typedef uint8_t axis_bits_t;
enum AxisEnum : uint8_t {
	//X_AXIS_ENUM, Y_AXIS_ENUM, Z_AXIS_ENUM, MAX_AXIS_ENUM
	//, ALL_AXES_ENUM = 0xFE, NO_AXIS_ENUM = 0xFF
	X_AXIS, Y_AXIS, Z_AXIS, E_AXIS,
	A_AXIS = X_AXIS, B_AXIS = Y_AXIS, C_AXIS = Z_AXIS
};
*/
// And each stepper (start + stop pulse) takes in worst case
#define ISR_BASE_CYCLES  996UL
#define ISR_LOOP_BASE_CYCLES 32UL
#define ISR_STEPPER_CYCLES 88UL
// Add time for each stepper
#define ISR_X_STEPPER_CYCLES ISR_STEPPER_CYCLES
#define ISR_Y_STEPPER_CYCLES ISR_STEPPER_CYCLES
#define ISR_Z_STEPPER_CYCLES ISR_STEPPER_CYCLES
#define MIN_ISR_LOOP_CYCLES ( ISR_X_STEPPER_CYCLES + ISR_Y_STEPPER_CYCLES + ISR_Z_STEPPER_CYCLES)
// HAS_DRIVER(A4988)
#define MAXIMUM_STEPPER_RATE 500000
#define MINIMUM_STEPPER_PULSE 1
// Calculate the minimum MPU cycles needed per pulse to enforce, limited to the max stepper rate
#define _MIN_STEPPER_PULSE_CYCLES(N) _MAX(uint32_t((F_CPU) / (MAXIMUM_STEPPER_RATE)), ((F_CPU) / 500000UL) * (N))
#define MIN_STEPPER_PULSE_CYCLES _MIN_STEPPER_PULSE_CYCLES(uint32_t(MINIMUM_STEPPER_PULSE))

// The loop takes the base time plus the time for all the bresenham logic for R pulses plus the time
// between pulses for (R-1) pulses. But the user could be enforcing a minimum time so the loop time is:
#define ISR_LOOP_CYCLES(R) ((ISR_LOOP_BASE_CYCLES + MIN_ISR_LOOP_CYCLES + MIN_STEPPER_PULSE_CYCLES) * (R - 1) + _MAX(MIN_ISR_LOOP_CYCLES, MIN_STEPPER_PULSE_CYCLES))

// Now estimate the total ISR execution time in cycles given a step per ISR multiplier
#define ISR_EXECUTION_CYCLES(R) (((ISR_BASE_CYCLES + ISR_LOOP_CYCLES(R))) / (R))
// The maximum allowable stepping frequency when doing x128-x1 stepping (in Hz)
#define MAX_STEP_ISR_FREQUENCY_128X ((F_CPU) / ISR_EXECUTION_CYCLES(128))
#define MAX_STEP_ISR_FREQUENCY_64X  ((F_CPU) / ISR_EXECUTION_CYCLES(64))
#define MAX_STEP_ISR_FREQUENCY_32X  ((F_CPU) / ISR_EXECUTION_CYCLES(32))
#define MAX_STEP_ISR_FREQUENCY_16X  ((F_CPU) / ISR_EXECUTION_CYCLES(16))
#define MAX_STEP_ISR_FREQUENCY_8X   ((F_CPU) / ISR_EXECUTION_CYCLES(8))
#define MAX_STEP_ISR_FREQUENCY_4X   ((F_CPU) / ISR_EXECUTION_CYCLES(4))
#define MAX_STEP_ISR_FREQUENCY_2X   ((F_CPU) / ISR_EXECUTION_CYCLES(2))
#define MAX_STEP_ISR_FREQUENCY_1X   ((F_CPU) / ISR_EXECUTION_CYCLES(1))

typedef struct {
	union {
		uint8_t bits;
		struct {
			bool X : 1, Y : 1, Z : 1;
		};
	};
} stepper_flags_t;

class Stepper {
	//friend void stepperTask(void*);
	public:
	private:
		static block_t* current_block;			// A pointer to the block currently being traced
		static axis_bits_t last_direction_bits,	// The next stepping-bits to be output
			axis_did_move;						// Last Movement in the given direction is not null, as computed when the last movement was fetched from planner
		static bool abort_current_block;        // Signals to the stepper that current block should be aborted
		static uint32_t acceleration_time, deceleration_time; // time measured in Stepper Timer ticks
		static uint8_t steps_per_isr;			// Count of steps to perform per Stepper ISR call
		static constexpr uint8_t oversampling_factor = 0;
		// Delta error variables for the Bresenham line tracer
		static xyze_long_t delta_error;
		static xyze_long_t advance_dividend;
		static uint32_t advance_divisor,
			step_events_completed,  // The number of step events executed in the current block
			accelerate_until,       // The point from where we need to stop acceleration
			decelerate_after,       // The point from where we need to start decelerating
			step_event_count;       // The total event count for the current block
		
		static int32_t ticks_nominal;
		static uint32_t acc_step_rate; // needed for deceleration start point
		static xyze_long_t endstops_trigsteps; // Exact steps at which an endstop was triggered
		static xyze_long_t count_position; // Positions of stepper motors, in step units
		static xyze_int8_t count_direction; // Current stepper motor directions (+1 or -1)
	public:
        static void init(); // Initialize stepper hardware
        // Interrupt Service Routine and phases
        // The stepper subsystem goes to sleep when it runs out of things to execute.
        // Call this to notify the subsystem that it is time to go to work.
#define ENABLE_STEPPER_DRIVER_INTERRUPT() {;}
#define DISABLE_STEPPER_DRIVER_INTERRUPT() {;}
#define STEPPER_ISR_ENABLED() true
        static void wake_up() { ENABLE_STEPPER_DRIVER_INTERRUPT(); }
        static bool is_awake() { return STEPPER_ISR_ENABLED(); }
        static bool suspend() {
            const bool awake = is_awake();
            if (awake) DISABLE_STEPPER_DRIVER_INTERRUPT();
            return awake;
        }
        
        static void isr(); // The ISR scheduler
        static void pulse_phase_isr(); // The stepper pulse ISR phase
        static uint32_t block_phase_isr(); // The stepper block processing ISR phase
		
        static bool is_block_busy(const block_t* const block); // Check if the given block is busy or not - Must not be called from ISR contexts
        static int32_t position(const AxisEnum axis); // Get the position of a stepper, in steps
        static void set_position(const xyze_long_t &spos); //// Set the current position in steps
        static void set_axis_position(const AxisEnum a, const int32_t &v);
        static void report_a_position(const xyz_long_t &pos); // Report the positions of the steppers, in steps
        static void report_positions();
        // Discard current block and free any resources
        static void discard_current_block() {
            current_block = nullptr;
            axis_did_move = 0;
            planner.release_current_block();
            //TERN_(LIN_ADVANCE, la_interval = nextAdvanceISR = LA_ADV_NEVER);
        }
        // Quickly stop all steppers
        static void quick_stop() { abort_current_block = true; }
        // The direction of a single motor
		static bool motor_direction(const AxisEnum axis) { return !!((last_direction_bits) & (1 << axis)); }
        // The last movement direction was not null on the specified axis. Note that motor direction is not necessarily the same.
        static bool axis_is_moving(const AxisEnum axis) { return !!((axis_did_move) & (1<<axis)); }
        // Handle a triggered endstop
        static void endstop_triggered(const AxisEnum axis);
        // Triggered position of an axis in steps
        static int32_t triggered_position(const AxisEnum axis);

		static void microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2, const int8_t ms3);
		static void microstep_mode(const uint8_t driver, const uint8_t stepping);
		static void microstep_readings();

		static stepper_flags_t axis_enabled;  // Axis stepper(s) ENABLED states
        static bool axis_is_enabled(const AxisEnum axis) { return !!((axis_enabled.bits) & (1<<axis));
        }
        static void mark_axis_enabled(const AxisEnum axis) { axis_enabled.bits |= (1<<axis); }
        static void mark_axis_disabled(const AxisEnum axis) { axis_enabled.bits &= ~(1<<axis); }
        static bool can_axis_disable(const AxisEnum) {
            //return !any_enable_overlap() || !(axis_enabled.bits & enable_overlap[INDEX_OF_AXIS(axis, eindex)]);
			return true;
        }
        static void enable_axis(const AxisEnum axis);
        static bool disable_axis(const AxisEnum axis);
        static void enable_all_steppers();
        static void disable_all_steppers();
		static void SET_STEP_DIR(const AxisEnum axis);
		static void set_directions(); // Update direction states for all steppers
        // Set direction bits and update all stepper DIR states
        static void set_directions(const axis_bits_t bits) {
            last_direction_bits = bits;
            set_directions();
        }

	private:
		// Set the current position in steps
		static void _set_position(const abce_long_t &spos);
		// Calculate timing interval for the given step rate
		static uint32_t calc_timer_interval(uint32_t step_rate);
		static uint32_t calc_timer_interval(uint32_t step_rate, uint8_t &loops);

};//Stepper
extern Stepper stepper;



// timer
typedef uint16_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFF

#define F_CPU 16000000
#define HAL_TIMER_RATE	((F_CPU) / 8)    // i.e., 2MHz or 2.5MHz
#define MF_TIMER_STEP	1
#define MF_TIMER_PULSE  MF_TIMER_STEP
#define MF_TIMER_TEMP   0

#define STEPPER_TIMER_RATE			HAL_TIMER_RATE
#define STEPPER_TIMER_PRESCALE		8
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000)

#define PULSE_TIMER_RATE			STEPPER_TIMER_RATE
#define PULSE_TIMER_PRESCALE		STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US	STEPPER_TIMER_TICKS_PER_US

extern uint32_t TIMSK1, OCR1A, OCR0A, TNCT1, TNCT0;
//#define OCIE1A 0x01
//#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
//#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)
//#define STEPPER_ISR_ENABLED()             TEST(TIMSK1, OCIE1A)

void HAL_timer_start(const uint8_t timer_num, const uint32_t);
void HAL_timer_set_compare(const uint8_t timer, const uint32_t compare);
uint32_t HAL_timer_get_compare(const uint8_t timer);
uint32_t HAL_timer_get_count(const uint8_t timer);



//marlin.h
