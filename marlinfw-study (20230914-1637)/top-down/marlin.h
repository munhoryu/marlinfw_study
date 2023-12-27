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


#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define NUM_AXIS 3

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
	// Length reduced to one dimension
	FI T magnitude()                                const { return (T)sqrtf(x * x + y * y); }
	// Pointer to the data as a simple array
	FI operator T* () { return pos; }
	// If any element is true then it's true
	FI operator bool() { return x || y; }
	// Smallest element
	FI T small()                                    const { return _MIN(x, y); }
	// Largest element
	FI T large()                                    const { return _MAX(x, y); }
	// Explicit copy and copies with conversion
	FI XYval<T>           copy()                    const { return *this; }
	FI XYval<T>            ABS()                    const { return { T(_ABS(x)), T(_ABS(y)) }; }
	FI XYval<int16_t>    asInt() { return { int16_t(x), int16_t(y) }; }
	FI XYval<int16_t>    asInt()                    const { return { int16_t(x), int16_t(y) }; }
	FI XYval<int32_t>   asLong() { return { int32_t(x), int32_t(y) }; }
	FI XYval<int32_t>   asLong()                    const { return { int32_t(x), int32_t(y) }; }
	FI XYval<int32_t>   ROUNDL() { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
	FI XYval<int32_t>   ROUNDL()                    const { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
	FI XYval<float>    asFloat() { return { static_cast<float>(x), static_cast<float>(y) }; }
	FI XYval<float>    asFloat()                    const { return { static_cast<float>(x), static_cast<float>(y) }; }
	FI XYval<float> reciprocal()                    const { return { _RECIP(x),  _RECIP(y) }; }
	// Marlin workspace shifting is done with G92 and M206
	FI XYval<float>  asLogical()                    const { XYval<float> o = asFloat(); toLogical(o); return o; }
	FI XYval<float>   asNative()                    const { XYval<float> o = asFloat(); toNative(o);  return o; }
	// Cast to a type with more fields by making a new object
	FI operator XYZval<T>() { return { x, y }; }
	FI operator XYZval<T>()                         const { return { x, y }; }
	FI operator XYZEval<T>() { return { x, y }; }
	FI operator XYZEval<T>()                        const { return { x, y }; }
	// Accessor via an AxisEnum (or any integer) [index]
	FI       T& operator[](const int n) { return pos[n]; }
	FI const T& operator[](const int n)            const { return pos[n]; }
	// Assignment operator overrides do the expected thing
	FI XYval<T>& operator= (const T v) { set(v, v); return *this; }
	FI XYval<T>& operator= (const XYZval<T>& rs) { set(rs.x, rs.y); return *this; }
	FI XYval<T>& operator= (const XYZEval<T>& rs) { set(rs.x, rs.y); return *this; }
	// Override other operators to get intuitive behaviors
	FI XYval<T>  operator+ (const XYval<T>& rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator+ (const XYval<T>& rs) { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator- (const XYval<T>& rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator- (const XYval<T>& rs) { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator* (const XYval<T>& rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator* (const XYval<T>& rs) { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYval<T>& rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYval<T>& rs) { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator- (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator- (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZval<T>& rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZval<T>& rs) { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator+ (const XYZEval<T>& rs) { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
	FI XYval<T>  operator- (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator- (const XYZEval<T>& rs) { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator* (const XYZEval<T>& rs) { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZEval<T>& rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator/ (const XYZEval<T>& rs) { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
	FI XYval<T>  operator* (const float& p)         const { XYval<T> ls = *this; ls.x *= p;    ls.y *= p;    return ls; }
	FI XYval<T>  operator* (const float& p) { XYval<T> ls = *this; ls.x *= p;    ls.y *= p;    return ls; }
	FI XYval<T>  operator* (const int& p)           const { XYval<T> ls = *this; ls.x *= p;    ls.y *= p;    return ls; }
	FI XYval<T>  operator* (const int& p) { XYval<T> ls = *this; ls.x *= p;    ls.y *= p;    return ls; }
	FI XYval<T>  operator/ (const float& p)         const { XYval<T> ls = *this; ls.x /= p;    ls.y /= p;    return ls; }
	FI XYval<T>  operator/ (const float& p) { XYval<T> ls = *this; ls.x /= p;    ls.y /= p;    return ls; }
	FI XYval<T>  operator/ (const int& p)           const { XYval<T> ls = *this; ls.x /= p;    ls.y /= p;    return ls; }
	FI XYval<T>  operator/ (const int& p) { XYval<T> ls = *this; ls.x /= p;    ls.y /= p;    return ls; }
	FI XYval<T>  operator>>(const int& p)           const { XYval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    return ls; }
	FI XYval<T>  operator>>(const int& p) { XYval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    return ls; }
	FI XYval<T>  operator<<(const int& p)           const { XYval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    return ls; }
	FI XYval<T>  operator<<(const int& p) { XYval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    return ls; }
	FI const XYval<T> operator-()                   const { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
	FI XYval<T>       operator-() { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
	// Modifier operators
	FI XYval<T>& operator+=(const XYval<T>& rs) { x += rs.x; y += rs.y; return *this; }
	FI XYval<T>& operator-=(const XYval<T>& rs) { x -= rs.x; y -= rs.y; return *this; }
	FI XYval<T>& operator*=(const XYval<T>& rs) { x *= rs.x; y *= rs.y; return *this; }
	FI XYval<T>& operator/=(const XYval<T>& rs) { x /= rs.x; y /= rs.y; return *this; }
	FI XYval<T>& operator+=(const XYZval<T>& rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator-=(const XYZval<T>& rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator*=(const XYZval<T>& rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator/=(const XYZval<T>& rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator+=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator-=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator*=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator/=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, , , , , , , ); return *this; }
	FI XYval<T>& operator*=(const float& p) { x *= p;    y *= p;    return *this; }
	FI XYval<T>& operator*=(const int& p) { x *= p;    y *= p;    return *this; }
	FI XYval<T>& operator>>=(const int& p) { _RS(x);    _RS(y);    return *this; }
	FI XYval<T>& operator<<=(const int& p) { _LS(x);    _LS(y);    return *this; }
	// Exact comparisons. For floats a "NEAR" operation may be better.
	FI bool      operator==(const XYval<T>& rs)   const { return NUM_AXIS_GANG(x == rs.x, && y == rs.y, , , , , , , ); }
	FI bool      operator==(const XYZval<T>& rs)   const { return NUM_AXIS_GANG(x == rs.x, && y == rs.y, , , , , , , ); }
	FI bool      operator==(const XYZEval<T>& rs)   const { return NUM_AXIS_GANG(x == rs.x, && y == rs.y, , , , , , , ); }
	FI bool      operator!=(const XYval<T>& rs)   const { return !operator==(rs); }
	FI bool      operator!=(const XYZval<T>& rs)   const { return !operator==(rs); }
	FI bool      operator!=(const XYZEval<T>& rs)   const { return !operator==(rs); }
};

template<typename T>
struct XYZval {
	union {
		struct { T x, y, z };
		struct { T a, b, c };
		T pos[3];
	};
	// Set all to 0
	FI void reset() { x = y = z = 0; }
	// Setters taking struct types and arrays
	FI void set(const XYval<T> pxy) { x = pxy.x; y = pxy.y; }
	FI void set(const XYval<T> pxy, const T pz) { x = pxy.x; y = pxy.y; z = pz; }
	FI void set(const T(&arr)[NUM_AXES]) { x = arr[0]; y = arr[1]; z = arr[2]; }
	// Setter for all individual args
	FI void set(const T x, const T y, const T z) { a = x; , b = y; c = z; }

	// Setters with fewer elements leave the rest untouched
#if HAS_Y_AXIS
	FI void set(const T px) { x = px; }
#endif
#if HAS_Z_AXIS
	FI void set(const T px, const T py) { x = px; y = py; }
#endif
#if HAS_I_AXIS
	FI void set(const T px, const T py, const T pz) { x = px; y = py; z = pz; }
#endif
#if HAS_J_AXIS
	FI void set(const T px, const T py, const T pz, const T pi) { x = px; y = py; z = pz; i = pi; }
#endif
#if HAS_K_AXIS
	FI void set(const T px, const T py, const T pz, const T pi, const T pj) { x = px; y = py; z = pz; i = pi; j = pj; }
#endif
#if HAS_U_AXIS
	FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; }
#endif
#if HAS_V_AXIS
	FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; u = pu; }
#endif
#if HAS_W_AXIS
	FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu, const T pv) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; u = pu; v = pv; }
#endif

	// Length reduced to one dimension
	FI T magnitude()                               const { return (T)sqrtf(NUM_AXIS_GANG(x * x, +y * y, +z * z, +i * i, +j * j, +k * k, +u * u, +v * v, +w * w)); }
	// Pointer to the data as a simple array
	FI operator T* () { return pos; }
	// If any element is true then it's true
	FI operator bool() { return NUM_AXIS_GANG(x, || y, || z, || i, || j, || k, || u, || v, || w); }
	// Smallest element
	FI T small()                                   const { return _MIN(NUM_AXIS_LIST(x, y, z, i, j, k, u, v, w)); }
	// Largest element
	FI T large()                                   const { return _MAX(NUM_AXIS_LIST(x, y, z, i, j, k, u, v, w)); }

	// Explicit copy and copies with conversion
	FI XYZval<T>          copy()                   const { XYZval<T> o = *this; return o; }
	FI XYZval<T>           ABS()                   const { return NUM_AXIS_ARRAY(T(_ABS(x)), T(_ABS(y)), T(_ABS(z)), T(_ABS(i)), T(_ABS(j)), T(_ABS(k)), T(_ABS(u)), T(_ABS(v)), T(_ABS(w))); }
	FI XYZval<int16_t>   asInt() { return NUM_AXIS_ARRAY(int16_t(x), int16_t(y), int16_t(z), int16_t(i), int16_t(j), int16_t(k), int16_t(u), int16_t(v), int16_t(w)); }
	FI XYZval<int16_t>   asInt()                   const { return NUM_AXIS_ARRAY(int16_t(x), int16_t(y), int16_t(z), int16_t(i), int16_t(j), int16_t(k), int16_t(u), int16_t(v), int16_t(w)); }
	FI XYZval<int32_t>  asLong() { return NUM_AXIS_ARRAY(int32_t(x), int32_t(y), int32_t(z), int32_t(i), int32_t(j), int32_t(k), int32_t(u), int32_t(v), int32_t(w)); }
	FI XYZval<int32_t>  asLong()                   const { return NUM_AXIS_ARRAY(int32_t(x), int32_t(y), int32_t(z), int32_t(i), int32_t(j), int32_t(k), int32_t(u), int32_t(v), int32_t(w)); }
	FI XYZval<int32_t>  ROUNDL() { return NUM_AXIS_ARRAY(int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(i)), int32_t(LROUND(j)), int32_t(LROUND(k)), int32_t(LROUND(u)), int32_t(LROUND(v)), int32_t(LROUND(w))); }
	FI XYZval<int32_t>  ROUNDL()                   const { return NUM_AXIS_ARRAY(int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(i)), int32_t(LROUND(j)), int32_t(LROUND(k)), int32_t(LROUND(u)), int32_t(LROUND(v)), int32_t(LROUND(w))); }
	FI XYZval<float>   asFloat() { return NUM_AXIS_ARRAY(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(i), static_cast<float>(j), static_cast<float>(k), static_cast<float>(u), static_cast<float>(v), static_cast<float>(w)); }
	FI XYZval<float>   asFloat()                   const { return NUM_AXIS_ARRAY(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(i), static_cast<float>(j), static_cast<float>(k), static_cast<float>(u), static_cast<float>(v), static_cast<float>(w)); }
	FI XYZval<float> reciprocal()                  const { return NUM_AXIS_ARRAY(_RECIP(x), _RECIP(y), _RECIP(z), _RECIP(i), _RECIP(j), _RECIP(k), _RECIP(u), _RECIP(v), _RECIP(w)); }

	// Marlin workspace shifting is done with G92 and M206
	FI XYZval<float> asLogical()                   const { XYZval<float> o = asFloat(); toLogical(o); return o; }
	FI XYZval<float>  asNative()                   const { XYZval<float> o = asFloat(); toNative(o);  return o; }

	// In-place cast to types having fewer fields
	FI operator XYval<T>& () { return *(XYval<T>*)this; }
	FI operator const XYval<T>& ()                  const { return *(const XYval<T>*)this; }

	// Cast to a type with more fields by making a new object
	FI operator XYZEval<T>()                 const { return NUM_AXIS_ARRAY(x, y, z, i, j, k, u, v, w); }

	// Accessor via an AxisEnum (or any integer) [index]
	FI       T& operator[](const int n) { return pos[n]; }
	FI const T& operator[](const int n)          const { return pos[n]; }

	// Assignment operator overrides do the expected thing
	FI XYZval<T>& operator= (const T v) { set(ARRAY_N_1(NUM_AXES, v)); return *this; }
	FI XYZval<T>& operator= (const XYval<T>& rs) { set(rs.x, rs.y); return *this; }
	FI XYZval<T>& operator= (const XYZEval<T>& rs) { set(NUM_AXIS_ELEM(rs)); return *this; }

	// Override other operators to get intuitive behaviors
	FI XYZval<T>  operator+ (const XYval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x += rs.x, ls.y += rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator+ (const XYval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x += rs.x, ls.y += rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator- (const XYval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x -= rs.x, ls.y -= rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator- (const XYval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x -= rs.x, ls.y -= rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator* (const XYval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= rs.x, ls.y *= rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator* (const XYval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= rs.x, ls.y *= rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator/ (const XYval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= rs.x, ls.y /= rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator/ (const XYval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= rs.x, ls.y /= rs.y, , , , , , , ); return ls; }
	FI XYZval<T>  operator+ (const XYZval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x += rs.x, ls.y += rs.y, ls.z += rs.z, ls.i += rs.i, ls.j += rs.j, ls.k += rs.k, ls.u += rs.u, ls.v += rs.v, ls.w += rs.w); return ls; }
	FI XYZval<T>  operator+ (const XYZval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x += rs.x, ls.y += rs.y, ls.z += rs.z, ls.i += rs.i, ls.j += rs.j, ls.k += rs.k, ls.u += rs.u, ls.v += rs.v, ls.w += rs.w); return ls; }
	FI XYZval<T>  operator- (const XYZval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x -= rs.x, ls.y -= rs.y, ls.z -= rs.z, ls.i -= rs.i, ls.j -= rs.j, ls.k -= rs.k, ls.u -= rs.u, ls.v -= rs.v, ls.w -= rs.w); return ls; }
	FI XYZval<T>  operator- (const XYZval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x -= rs.x, ls.y -= rs.y, ls.z -= rs.z, ls.i -= rs.i, ls.j -= rs.j, ls.k -= rs.k, ls.u -= rs.u, ls.v -= rs.v, ls.w -= rs.w); return ls; }
	FI XYZval<T>  operator* (const XYZval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= rs.x, ls.y *= rs.y, ls.z *= rs.z, ls.i *= rs.i, ls.j *= rs.j, ls.k *= rs.k, ls.u *= rs.u, ls.v *= rs.v, ls.w *= rs.w); return ls; }
	FI XYZval<T>  operator* (const XYZval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= rs.x, ls.y *= rs.y, ls.z *= rs.z, ls.i *= rs.i, ls.j *= rs.j, ls.k *= rs.k, ls.u *= rs.u, ls.v *= rs.v, ls.w *= rs.w); return ls; }
	FI XYZval<T>  operator/ (const XYZval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= rs.x, ls.y /= rs.y, ls.z /= rs.z, ls.i /= rs.i, ls.j /= rs.j, ls.k /= rs.k, ls.u /= rs.u, ls.v /= rs.v, ls.w /= rs.w); return ls; }
	FI XYZval<T>  operator/ (const XYZval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= rs.x, ls.y /= rs.y, ls.z /= rs.z, ls.i /= rs.i, ls.j /= rs.j, ls.k /= rs.k, ls.u /= rs.u, ls.v /= rs.v, ls.w /= rs.w); return ls; }
	FI XYZval<T>  operator+ (const XYZEval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x += rs.x, ls.y += rs.y, ls.z += rs.z, ls.i += rs.i, ls.j += rs.j, ls.k += rs.k, ls.u += rs.u, ls.v += rs.v, ls.w += rs.w); return ls; }
	FI XYZval<T>  operator+ (const XYZEval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x += rs.x, ls.y += rs.y, ls.z += rs.z, ls.i += rs.i, ls.j += rs.j, ls.k += rs.k, ls.u += rs.u, ls.v += rs.v, ls.w += rs.w); return ls; }
	FI XYZval<T>  operator- (const XYZEval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x -= rs.x, ls.y -= rs.y, ls.z -= rs.z, ls.i -= rs.i, ls.j -= rs.j, ls.k -= rs.k, ls.u -= rs.u, ls.v -= rs.v, ls.w -= rs.w); return ls; }
	FI XYZval<T>  operator- (const XYZEval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x -= rs.x, ls.y -= rs.y, ls.z -= rs.z, ls.i -= rs.i, ls.j -= rs.j, ls.k -= rs.k, ls.u -= rs.u, ls.v -= rs.v, ls.w -= rs.w); return ls; }
	FI XYZval<T>  operator* (const XYZEval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= rs.x, ls.y *= rs.y, ls.z *= rs.z, ls.i *= rs.i, ls.j *= rs.j, ls.k *= rs.k, ls.u *= rs.u, ls.v *= rs.v, ls.w *= rs.w); return ls; }
	FI XYZval<T>  operator* (const XYZEval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= rs.x, ls.y *= rs.y, ls.z *= rs.z, ls.i *= rs.i, ls.j *= rs.j, ls.k *= rs.k, ls.u *= rs.u, ls.v *= rs.v, ls.w *= rs.w); return ls; }
	FI XYZval<T>  operator/ (const XYZEval<T>& rs) const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= rs.x, ls.y /= rs.y, ls.z /= rs.z, ls.i /= rs.i, ls.j /= rs.j, ls.k /= rs.k, ls.u /= rs.u, ls.v /= rs.v, ls.w /= rs.w); return ls; }
	FI XYZval<T>  operator/ (const XYZEval<T>& rs) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= rs.x, ls.y /= rs.y, ls.z /= rs.z, ls.i /= rs.i, ls.j /= rs.j, ls.k /= rs.k, ls.u /= rs.u, ls.v /= rs.v, ls.w /= rs.w); return ls; }
	FI XYZval<T>  operator* (const float& p)       const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= p, ls.y *= p, ls.z *= p, ls.i *= p, ls.j *= p, ls.k *= p, ls.u *= p, ls.v *= p, ls.w *= p); return ls; }
	FI XYZval<T>  operator* (const float& p) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= p, ls.y *= p, ls.z *= p, ls.i *= p, ls.j *= p, ls.k *= p, ls.u *= p, ls.v *= p, ls.w *= p); return ls; }
	FI XYZval<T>  operator* (const int& p)         const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= p, ls.y *= p, ls.z *= p, ls.i *= p, ls.j *= p, ls.k *= p, ls.u *= p, ls.v *= p, ls.w *= p); return ls; }
	FI XYZval<T>  operator* (const int& p) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x *= p, ls.y *= p, ls.z *= p, ls.i *= p, ls.j *= p, ls.k *= p, ls.u *= p, ls.v *= p, ls.w *= p); return ls; }
	FI XYZval<T>  operator/ (const float& p)       const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= p, ls.y /= p, ls.z /= p, ls.i /= p, ls.j /= p, ls.k /= p, ls.u /= p, ls.v /= p, ls.w /= p); return ls; }
	FI XYZval<T>  operator/ (const float& p) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= p, ls.y /= p, ls.z /= p, ls.i /= p, ls.j /= p, ls.k /= p, ls.u /= p, ls.v /= p, ls.w /= p); return ls; }
	FI XYZval<T>  operator/ (const int& p)         const { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= p, ls.y /= p, ls.z /= p, ls.i /= p, ls.j /= p, ls.k /= p, ls.u /= p, ls.v /= p, ls.w /= p); return ls; }
	FI XYZval<T>  operator/ (const int& p) { XYZval<T> ls = *this; NUM_AXIS_CODE(ls.x /= p, ls.y /= p, ls.z /= p, ls.i /= p, ls.j /= p, ls.k /= p, ls.u /= p, ls.v /= p, ls.w /= p); return ls; }
	FI XYZval<T>  operator>>(const int& p)         const { XYZval<T> ls = *this; NUM_AXIS_CODE(_RS(ls.x), _RS(ls.y), _RS(ls.z), _RS(ls.i), _RS(ls.j), _RS(ls.k), _RS(ls.u), _RS(ls.v), _RS(ls.w)); return ls; }
	FI XYZval<T>  operator>>(const int& p) { XYZval<T> ls = *this; NUM_AXIS_CODE(_RS(ls.x), _RS(ls.y), _RS(ls.z), _RS(ls.i), _RS(ls.j), _RS(ls.k), _RS(ls.u), _RS(ls.v), _RS(ls.w)); return ls; }
	FI XYZval<T>  operator<<(const int& p)         const { XYZval<T> ls = *this; NUM_AXIS_CODE(_LS(ls.x), _LS(ls.y), _LS(ls.z), _LS(ls.i), _LS(ls.j), _LS(ls.k), _LS(ls.u), _LS(ls.v), _LS(ls.w)); return ls; }
	FI XYZval<T>  operator<<(const int& p) { XYZval<T> ls = *this; NUM_AXIS_CODE(_LS(ls.x), _LS(ls.y), _LS(ls.z), _LS(ls.i), _LS(ls.j), _LS(ls.k), _LS(ls.u), _LS(ls.v), _LS(ls.w)); return ls; }
	FI const XYZval<T> operator-()                 const { XYZval<T> o = *this; NUM_AXIS_CODE(o.x = -x, o.y = -y, o.z = -z, o.i = -i, o.j = -j, o.k = -k, o.u = -u, o.v = -v, o.w = -w); return o; }
	FI XYZval<T>       operator-() { XYZval<T> o = *this; NUM_AXIS_CODE(o.x = -x, o.y = -y, o.z = -z, o.i = -i, o.j = -j, o.k = -k, o.u = -u, o.v = -v, o.w = -w); return o; }

	// Modifier operators
	FI XYZval<T>& operator+=(const XYval<T>& rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, , , , , , , ); return *this; }
	FI XYZval<T>& operator-=(const XYval<T>& rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, , , , , , , ); return *this; }
	FI XYZval<T>& operator*=(const XYval<T>& rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, , , , , , , ); return *this; }
	FI XYZval<T>& operator/=(const XYval<T>& rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, , , , , , , ); return *this; }
	FI XYZval<T>& operator+=(const XYZval<T>& rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, z += rs.z, i += rs.i, j += rs.j, k += rs.k, u += rs.u, v += rs.v, w += rs.w); return *this; }
	FI XYZval<T>& operator-=(const XYZval<T>& rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, z -= rs.z, i -= rs.i, j -= rs.j, k -= rs.k, u -= rs.u, v -= rs.v, w -= rs.w); return *this; }
	FI XYZval<T>& operator*=(const XYZval<T>& rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, z *= rs.z, i *= rs.i, j *= rs.j, k *= rs.k, u *= rs.u, v *= rs.v, w *= rs.w); return *this; }
	FI XYZval<T>& operator/=(const XYZval<T>& rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, z /= rs.z, i /= rs.i, j /= rs.j, k /= rs.k, u /= rs.u, v /= rs.v, w /= rs.w); return *this; }
	FI XYZval<T>& operator+=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, z += rs.z, i += rs.i, j += rs.j, k += rs.k, u += rs.u, v += rs.v, w += rs.w); return *this; }
	FI XYZval<T>& operator-=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, z -= rs.z, i -= rs.i, j -= rs.j, k -= rs.k, u -= rs.u, v -= rs.v, w -= rs.w); return *this; }
	FI XYZval<T>& operator*=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, z *= rs.z, i *= rs.i, j *= rs.j, k *= rs.k, u *= rs.u, v *= rs.v, w *= rs.w); return *this; }
	FI XYZval<T>& operator/=(const XYZEval<T>& rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, z /= rs.z, i /= rs.i, j /= rs.j, k /= rs.k, u /= rs.u, v /= rs.v, w /= rs.w); return *this; }
	FI XYZval<T>& operator*=(const float& p) { NUM_AXIS_CODE(x *= p, y *= p, z *= p, i *= p, j *= p, k *= p, u *= p, v *= p, w *= p);    return *this; }
	FI XYZval<T>& operator*=(const int& p) { NUM_AXIS_CODE(x *= p, y *= p, z *= p, i *= p, j *= p, k *= p, u *= p, v *= p, w *= p);    return *this; }
	FI XYZval<T>& operator>>=(const int& p) { NUM_AXIS_CODE(_RS(x), _RS(y), _RS(z), _RS(i), _RS(j), _RS(k), _RS(u), _RS(v), _RS(w));    return *this; }
	FI XYZval<T>& operator<<=(const int& p) { NUM_AXIS_CODE(_LS(x), _LS(y), _LS(z), _LS(i), _LS(j), _LS(k), _LS(u), _LS(v), _LS(w));    return *this; }

	// Exact comparisons. For floats a "NEAR" operation may be better.
	FI bool       operator==(const XYZEval<T>& rs) const { return true NUM_AXIS_GANG(&& x == rs.x, && y == rs.y, && z == rs.z, && i == rs.i, && j == rs.j, && k == rs.k, && u == rs.u, && v == rs.v, && w == rs.w); }
	FI bool       operator!=(const XYZEval<T>& rs) const { return !operator==(rs); }
};


#define X_HOME_POS 100
#define Y_HOME_POS 200
#define Z_HOME_POS 300
extern position_float_t current_position;
extern position_float_t home_offset;

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
	BLOCK_BIT_CONTINUED
};

// Planner block flags as boolean bit fields
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



// stepper
typedef uint8_t axis_bits_t;
enum AxisEnum : uint8_t {
	X_AXIS_ENUM, Y_AXIS_ENUM, Z_AXIS_ENUM, MAX_AXIS_ENUM
	, ALL_AXES_ENUM = 0xFE, NO_AXIS_ENUM = 0xFF
};

typedef struct {
	union {
		uint8_t bits;
		struct {
			bool X : 1, Y : 1, Z : 1;
		};
	};
} stepper_flags_t;

class Stepper {
	public:
	private:
		static block_t* current_block;			// A pointer to the block currently being traced
		static axis_bits_t	last_direction_bits,// The next stepping-bits to be output
			axis_did_move;		// Last Movement in the given direction is not null, as computed when the last movement was fetched from planner
		static bool abort_current_block;        // Signals to the stepper that current block should be aborted
		static uint32_t acceleration_time, deceleration_time; // time measured in Stepper Timer ticks
		static uint8_t steps_per_isr;			// Count of steps to perform per Stepper ISR call
		// Delta error variables for the Bresenham line tracer
		static position_int32_t delta_error;
		static position_int32_t advance_dividend;
		static uint32_t advance_divisor,
			step_events_completed,  // The number of step events executed in the current block
			accelerate_until,       // The point from where we need to stop acceleration
			decelerate_after,       // The point from where we need to start decelerating
			step_event_count;       // The total event count for the current block
		static int32_t ticks_nominal;
		static uint32_t acc_step_rate; // needed for deceleration start point
		// Exact steps at which an endstop was triggered
		static position_int32_t endstops_trigsteps;
		// Positions of stepper motors, in step units
		static position_int32_t count_position;
		// Current stepper motor directions (+1 or -1)
		static position_int8_t count_direction;
	public:
        // Initialize stepper hardware
        static void init();
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
        // The ISR scheduler
        static void isr();
        // The stepper pulse ISR phase
        static void pulse_phase_isr();
        // The stepper block processing ISR phase
        static uint32_t block_phase_isr();
        // Check if the given block is busy or not - Must not be called from ISR contexts
        static bool is_block_busy(const block_t* const block);
        // Get the position of a stepper, in steps
        static int32_t position(const AxisEnum axis);
        // Set the current position in steps
        static void set_position(const position_int32_t &spos);
        static void set_axis_position(const AxisEnum a, const int32_t &v);
        // Report the positions of the steppers, in steps
        static void report_a_position(const position_int32_t &pos);
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
        // Update direction states for all steppers
        static void set_directions();
        // Set direction bits and update all stepper DIR states
        static void set_directions(const axis_bits_t bits) {
            last_direction_bits = bits;
            set_directions();
        }

	private:
		// Set the current position in steps
		static void _set_position(const position_int32_t &spos);
		// Calculate timing interval for the given step rate
		static uint32_t calc_timer_interval(uint32_t step_rate);
		static uint32_t calc_timer_interval(uint32_t step_rate, uint8_t &loops);
};//Stepper
extern Stepper stepper;


// timer
#define F_CPU 16000000
#define HAL_TIMER_RATE	((F_CPU) / 8)    // i.e., 2MHz or 2.5MHz
#define MF_TIMER_STEP	1
#define MF_TIMER_PULSE  MF_TIMER_STEP
#define MF_TIMER_TEMP   0

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
