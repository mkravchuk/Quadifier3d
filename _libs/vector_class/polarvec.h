/***************************  complexvec.h   **********************************
* Author:        Agner Fog
* Date created:  2012-07-24
* Last modified: 2016-12-21
* Version:       1.26
* Project:       vector classes
* Description:
* Classes for complex number math:
* Polar2f:  One complex number consisting of two single precision floats
* Polar4f:  A vector of 2 complex numbers made from 4 single precision floats
* Complex8f:  A vector of 4 complex numbers made from 8 single precision floats
* Complex2d:  One complex number consisting of two double precision floats
* Complex4d:  A vector of 2 complex numbers made from 4 double precision floats
*
* (c) Copyright 2012-2016 GNU General Public License http://www.gnu.org/licenses
******************************************************************************/

// The Comples classes do not inherit from the corresponding vector classes
// because that would lead to undesired implicit conversions when calling
// functions that are defined only for the vector class. For example, if the
// sin function is defined for Vec2d but not for Complex2d then an attempt to
// call sin(Complex2d) would in fact call sin(Vec2d) which would be mathematically
// wrong. This is avoided by not using inheritance. Explicit conversion between
// these classes is still possible.

#ifndef POLARVEC_H
#define POLARVEC_H  126


#include "vectorclass.h"
#include <math.h>          // define math library functions

#ifdef VCL_NAMESPACE
namespace VCL_NAMESPACE {
#endif

/*****************************************************************************
*
*               Class Polar2f: one single precision complex polar number
*
*****************************************************************************/

class Polar2f {
protected:
    __m128 xmm; // vector of 4 single precision floats. Only the first two are used
public:
    // default constructor
    Polar2f() {
    }
    // construct from real and imaginary part
    Polar2f(float re, float im)
    {
        xmm = Vec4f(re, im, 0.f, 0.f);
    }
    // construct from real, no imaginary part
    Polar2f(int re_int)
    {
        float re = (float)re_int;
        xmm = _mm_load_ss(&re);
    }
    // construct from real, no imaginary part
    Polar2f(float re)
    {
        xmm = _mm_load_ss(&re);
    }
    // construct from real, no imaginary part
    Polar2f(double re_double)
    {
        float re = (float)re_double;
        xmm = _mm_load_ss(&re);
    }
    // Constructor to convert from type __m128 used in intrinsics:
    Polar2f(__m128 const & x) {
        xmm = x;
    }

    Polar2f(complex<float> const * p)
    {
        xmm = Vec4f().load_partial(2, reinterpret_cast<float const*>(p));
    }
    // Assignment operator to convert from type __m128 used in intrinsics:
    Polar2f & operator = (__m128 const & x) {
        xmm = x;
        return *this;
    }
    // Type cast operator to convert to __m128 used in intrinsics
    operator __m128() const {
        return xmm;
    }
    // Member function to convert to vector
    Vec4f to_vector() const {
        return xmm;
    }
    // Member function to load from array (unaligned)
    Polar2f & load(float const * p) {
        xmm = Vec4f().load_partial(2, p);
        return *this;
    }
    // Member function to load from array, aligned by 16
    Polar2f const & load_a(float const * p) {
        load(p);
        return *this;
    }
    void conjugate()
    {
        xmm = change_sign<0, 1, 0, 1>(xmm);
    }
    // Member function to store into array (unaligned)
    void store(float * p) const {
        Vec4f(xmm).store_partial(2, p);
    }
    void store(complex<float> * p) const
    {
        Vec4f(xmm).store_partial(2, reinterpret_cast<float*>(p));
    }
    // Member function to store into array, aligned by 16
    void store_a(float * p) const {
        store(p);
    }


    // get real part
    float real() const {
        return _mm_cvtss_f32(xmm);
    }
    // get imaginary part
    float imag() const {
        Vec4f t = permute4f<1,-256,-256,-256>(Vec4f(xmm));
        return Polar2f(t).real();
    }
    // Member function to extract real or imaginary part
    float extract(uint32_t index) const {
        float x[4];
        Vec4f(xmm).store(x);
        return x[index & 1];
    }

    complex<float> toComplex()
    {
        return complex<float>(real(), imag()); 
    }
};

/*****************************************************************************
*
*          Operators for Polar2f
*
*****************************************************************************/

// operator + : add
static inline Polar2f operator + (Polar2f const & a, Polar2f const & b) {
    return Polar2f(Vec4f(a) + Vec4f(b));
}

// operator += : add
static inline Polar2f & operator += (Polar2f & a, Polar2f const & b) {
    a = a + b;
    return a;
}

// operator - : subtract
static inline Polar2f operator - (Polar2f const & a, Polar2f const & b) {
    return Polar2f(Vec4f(a) - Vec4f(b));
}

// operator - : unary minus
static inline Polar2f operator - (Polar2f const & a) {
    return Polar2f(- Vec4f(a));
}




// operator -= : subtract
static inline Polar2f & operator -= (Polar2f & a, Polar2f const & b) {
    a = a - b;
    return a;
}


// operator * : complex multiply is defined as
// (a.re * b.re - a.im * b.im,  a.re * b.im + b.re * a.im)
// http://yangkunlun.blogspot.com/2011/09/fast-complex-multiply-with-sse.html    
static inline Polar2f operator * (Polar2f const & a, Polar2f const & b) {
    __m128 b_flip = _mm_shuffle_ps(b,b,0xB1);   // Swap b.re and b.im
    //__m128 a_im   = _mm_shuffle_ps(a,a,0xF5);   // Imag. part of a in both
    //__m128 a_re   = _mm_shuffle_ps(a,a,0xA0);   // Real part of a in both
    __m128 a_re = _mm_moveldup_ps(a);
    __m128 a_im = _mm_movehdup_ps(a);
    __m128 aib    = _mm_mul_ps(a_im, b_flip);   // (a.im*b.im, a.im*b.re)
//#ifdef __FMA__      // FMA3
//    return  _mm_fmaddsub_ps(a_re, b, aib);      // a_re * b +/- aib
//#elif defined (__FMA4__)  // FMA4
//    return  _mm_maddsub_ps(a_re, b, aib);       // a_re * b +/- aib
//#elif  INSTRSET >= 3  // SSE3
    __m128 arb    = _mm_mul_ps(a_re, b);        // (a.re*b.re, a.re*b.im)
    return _mm_addsub_ps(arb, aib);             // subtract/add
//#else
//    __m128 arb    = _mm_mul_ps(a_re, b);        // (a.re*b.re, a.re*b.im)
//    __m128 aib_m  = change_sign<1,0,1,0>(Vec4f(aib)); // change sign of low part
//    return _mm_add_ps(arb, aib_m);              // add
//#endif
}





// operator *= : multiply
static inline Polar2f & operator *= (Polar2f & a, Polar2f const & b) {
    a = a * b;
    return a;
}

// operator / : complex divide is defined as
// (a.re * b.re + a.im * b.im, b.re * a.im - a.re * b.im) / (b.re * b.re + b.im * b.im)
static inline Polar2f operator / (Polar2f const & a, Polar2f const & b) {
    // The following code is made similar to the operator * to enable common 
    // subexpression elimination in code that contains both operator * and 
    // operator / where one or both operands are the same
    __m128 a_re   = _mm_shuffle_ps(a,a,0);      // Real part of a in both
    __m128 arb    = _mm_mul_ps(a_re, b);        // (a.re*b.re, a.re*b.im)
    __m128 b_flip = _mm_shuffle_ps(b,b,1);      // Swap b.re and b.im
    __m128 a_im   = _mm_shuffle_ps(a,a,5);      // Imag. part of a in both
#ifdef __FMA__      // FMA3
    __m128 n      = _mm_fmsubadd_ps(a_im, b_flip, arb); 
#elif defined (__FMA4__)  // FMA4
    __m128 n      = _mm_msubadd_ps (a_im, b_flip, arb);
#else
    __m128 aib    = _mm_mul_ps(a_im, b_flip);   // (a.im*b.im, a.im*b.re)
    // The parameters for change_sign are made identical to the ones used in Polar4f so that the constant mask will share the same memory position
    __m128 arbm   = change_sign<0,1,0,1>(Vec4f(arb)); // change sign of high part
    __m128 n      = _mm_add_ps(aib, arbm);      // arbm + aib
#endif  // FMA
    __m128 bb     = _mm_mul_ps(b, b);           // (b.re*b.re, b.im*b.im)
#if INSTRSET >= 3  // SSE3
    __m128 bb2    = _mm_hadd_ps(bb,bb);         // (b.re*b.re + b.im*b.im) 
#else
    __m128 bb1    = _mm_shuffle_ps(bb,bb,1);
    __m128 bb2    = _mm_add_ss(bb,bb1);
#endif
    __m128 bb3    = _mm_shuffle_ps(bb2,bb2,0);  // copy into all positions to avoid division by zero
    return          _mm_div_ps(n, bb3);         // n / bb3
}

// operator /= : divide
static inline Polar2f & operator /= (Polar2f & a, Polar2f const & b) {
    a = a / b;
    return a;
}

// operator ~ : complex conjugate
// ~(a,b) = (a,-b)
static inline Polar2f operator ~ (Polar2f const & a) {
    // The parameters for change_sign are made identical to the ones used in Polar4f so that the constant mask will share the same memory position
    return Polar2f(change_sign<0,1,0,1>(Vec4f(a))); // change sign of high part
}

// operator == : returns true if a == b
static inline bool operator == (Polar2f const & a, Polar2f const & b) {
    Vec4fb t1 = Vec4f(a) == Vec4f(b);
#if INSTRSET >= 5   // SSE4.1 supported. Using PTEST
    Vec4fb t2 = _mm_shuffle_ps(t1, t1, 0x44);
    return horizontal_and(t2);
#else
    __m128i t2 = _mm_srli_epi64(_mm_castps_si128(t1),32);  // get 32 bits down
    __m128i t3 = _mm_and_si128(t2,_mm_castps_si128(t1));   // and 32 bits
    int     t4 = _mm_cvtsi128_si32(t3);                    // transfer 32 bits to integer
    return  t4 == -1;
#endif
}

// operator != : returns true if a != b
static inline bool operator != (Polar2f const & a, Polar2f const & b) {
    return !(a == b);
}

/*****************************************************************************
*
*          Operators mixing Polar2f and float
*
*****************************************************************************/

// operator + : add
static inline Polar2f operator + (Polar2f const & a, float b) {
    return _mm_add_ss(a, _mm_set_ss(b));
}
static inline Polar2f operator + (float a, Polar2f const & b) {
    return b + a;
}
static inline Polar2f & operator += (Polar2f & a, float & b) {
    a = a + b;
    return a;
}

// operator - : subtract
static inline Polar2f operator - (Polar2f const & a, float b) {
    return _mm_sub_ss(a, _mm_set_ss(b));
}
static inline Polar2f operator - (float a, Polar2f const & b) {
    return Polar2f(a) - b;
}
static inline Polar2f & operator -= (Polar2f & a, float & b) {
    a = a - b;
    return a;
}

// operator * : multiply
static inline Polar2f operator * (Polar2f const & a, float b)
{
    return _mm_mul_ps(a, _mm_set1_ps(b));
}
static inline Polar2f operator * (Polar2f const & a, double b)
{
    float f = (float)b;
    return _mm_mul_ps(a, _mm_set1_ps(f));
}
static inline Polar2f operator * (float a, Polar2f const & b) {
    return b * a;
}
static inline Polar2f & operator *= (Polar2f & a, float & b) {
    a = a * b;
    return a;
}
static inline Polar2f & operator *= (Polar2f & a, double & b)
{
    a = a * b;
    return a;
}

// operator / : divide
static inline Polar2f operator / (Polar2f const & a, float b) {
    return _mm_div_ps(a, _mm_set1_ps(b));
}

static inline Polar2f operator / (float a, Polar2f const & b) {
    Vec4f b2(b);
    Vec4f b3 = b2 * b2;
#if  INSTRSET >= 3  // SSE3
    __m128 t2 = _mm_hadd_ps(b3,b3);
#else
    __m128 t1 = _mm_shuffle_ps(b3,b3,1);
    __m128 t2 = _mm_add_ss(t1,b3);
#endif
    float  b4 = _mm_cvtss_f32(t2);
    return ~b * (a / b4);
}

static inline Polar2f & operator /= (Polar2f & a, float b) {
    a = a / b;
    return a;
}


/*****************************************************************************
*
*          Functions for Polar2f
*
*****************************************************************************/

// function abs: absolute value
// abs(a,b) = sqrt(a*a+b*b);
static inline float abs(Polar2f const & a) {
    Vec4f a1 = Vec4f(a);
    Vec4f a2 = a1 * a1;
#if  INSTRSET >= 3  // SSE3
    __m128 t2 = _mm_hadd_ps(a2,a2);
#else
    __m128 t1 = _mm_shuffle_ps(a2,a2,1);
    __m128 t2 = _mm_add_ss(t1,a2);
#endif
    float  a3 = _mm_cvtss_f32(t2);
    return sqrtf(a3);
}

// function sqrt: square root
static inline Polar2f sqrt(Polar2f const & a) {
    __m128 t1  = _mm_mul_ps(a,a);             // r*r, i*i
    __m128 t2  = _mm_shuffle_ps(t1,t1,0xB1);  // swap real and imaginary parts
    __m128 t3  = _mm_add_ps(t1,t2);           // pairwise horizontal sum
    __m128 t4  = _mm_sqrt_ps(t3);             // n = sqrt(r*r+i*i)
    __m128 t5  = _mm_shuffle_ps(a,a,0xA0);    // copy real part of a
    __m128 sbithi = _mm_castsi128_ps(constant4ui<0,0x80000000,0,0x80000000>());  // 0.0, -0.0, 0.0, -0.0
    __m128 t6  = _mm_xor_ps(t5, sbithi);      // r, -r
    __m128 t7  = _mm_add_ps(t4,t6);           // n+r, n-r
    __m128 t8  = _mm_sqrt_ps(t7);             // sqrt(n+r), sqrt(n-r)
    __m128 t9  = _mm_and_ps(a,sbithi);        // 0, signbit of i
    __m128 t10 = _mm_xor_ps(t8, t9);          // sqrt(n+r), sign(i)*sqrt(n-r)
    __m128 t11 = Vec4f(0.7071067811865f, 0.7071067811865f, 0.f, 0.f);  // 1/sqrt(2)
    return _mm_mul_ps(t10, t11);
}

// function select
static inline Polar2f select (bool s, Polar2f const & a, Polar2f const & b) {
    return s ? a : b;
}


/*****************************************************************************
*
*               Class Polar4f: two single precision complex numbers
*
*****************************************************************************/

class Polar4f {
protected:
    __m128 xmm; // vector of 4 single precision floats
public:
    // default constructor
    Polar4f() {
        xmm = Vec4f(0, 0, 0, 0);
    }

    Polar4f(complex<float> const * p)
    {
        xmm = Vec4f().load(reinterpret_cast<float const*>(p));
    }
    Polar4f(char const * p)
    {
        xmm = Vec4f().load(reinterpret_cast<float const*>(p));
    }
    Polar4f(Polar4f const * p)
    {
        xmm = Vec4f().load(reinterpret_cast<float const*>(p));
    }
    Polar4f(complex<float> const * p1, complex<float> const * p2)
    {
        __m128 a1 = Vec4f().load_partial(2, reinterpret_cast<float const*>(p1));
        __m128 a2 = Vec4f().load_partial(2, reinterpret_cast<float const*>(p2));
        xmm = _mm_movelh_ps(a1, a2);
    }

    // construct from real and imaginary parts
    Polar4f(float re0, float im0, float re1, float im1) {
        xmm = Vec4f(re0, im0, re1, im1);
    }
    // construct from real, no imaginary part
    Polar4f(float re)
    {
        xmm = Vec4f(re, 0.f, re, 0.f);
    }
    // construct from real, no imaginary part
    Polar4f(int re)
    {
        xmm = Vec4f((float)re, 0.f, (float)re, 0.f);
    }
    // construct from real and imaginary part, broadcast to all
    Polar4f(float re, float im) {
        xmm = Vec4f(re, im, re, im);
    }
    // construct from two Polar2f
    Polar4f(Polar2f const & a0, Polar2f const & a1) {
        xmm = _mm_movelh_ps(a0, a1);
    }
    // Constructor to convert from type __m128 used in intrinsics:
    Polar4f(__m128 const & x) {
        xmm = x;
    }
    // Assignment operator to convert from type __m128 used in intrinsics:
    Polar4f & operator = (__m128 const & x)
    {
        xmm = x;
        return *this;
    }
    // Assignment operator to convert from type __m128 used in intrinsics:
    //Polar4f & operator = (Polar2f const & x)
    //{
    //    xmm = Vec4f(x.real(), 0.f, x.imag(), 0.f);
    //    return *this;
    //}
    // Assignment operator to convert from type __m128 used in intrinsics:
    Polar4f & operator = (float const & re)
    {
        xmm = Vec4f(re, 0.f, re, 0.f);
        return *this;
    }
    // Assignment operator to convert from type __m128 used in intrinsics:
    Polar4f & operator = (complex<float> const & x)
    {
        xmm = Vec4f(x.real(), x.imag(), x.real(), x.imag());
        return *this;
    }

    // Type cast operator to convert to __m128 used in intrinsics
    operator __m128() const {
        return xmm;
    }
    // Member function to convert to vector
    Vec4f to_vector() const {
        return xmm;
    }
    // Member function to load from array (unaligned)
    Polar4f & load(float const * p) {
        xmm = Vec4f().load(p);
        return *this;
    }
    // Member function to load from array, aligned by 16
    Polar4f const & load_a(float const * p) {
        xmm = Vec4f().load_a(p);
        return *this;
    }
    // Member function to store into array (unaligned)
    void store(float * p) const
    {
        Vec4f(xmm).store(p);
    }
    void store(Polar4f * p) const
    {
        Vec4f(xmm).store(reinterpret_cast<float*>(p));
    }
    void store(complex<float> * p) const
    {
        Vec4f(xmm).store(reinterpret_cast<float*>(p));
    }
    void store_low(complex<float> * p) const
    {
        Polar2f(xmm).store(reinterpret_cast<float*>(p));
    }
    void store_high(complex<float> * p) const
    {
        get_high().store(p);
    }


    // Member function to store into array, aligned by 16
    void store_a(float * p) const
    {
        Vec4f(xmm).store_a(p);
    }
    void store_a(char * p) const
    {
        Vec4f(xmm).store_a(reinterpret_cast<float*>(p));
    }
    // Member functions to split into two Polar2f:
    Polar2f get_low() const {
        return Polar2f(Vec4f(xmm).cutoff(2));
    }
    Polar2f get_high() const {
        __m128 t = _mm_movehl_ps(_mm_setzero_ps(), xmm);
        return Polar2f(t);
    }
    // Member function to extract one real or imaginary part
    float extract(uint32_t index) const {
        float x[4];
        Vec4f(xmm).store(x);
        return x[index & 3];
    }

    // change sign of imaginary part
    void conjugate()
    {
        xmm = change_sign<0, 1, 0, 1>(xmm);
    }

    // change sign of real part
    void conjugateReal()
    {
        xmm = change_sign<1, 0, 1, 0>(xmm);
    }

    // change sign of imaginary part
    Polar4f getconjugated() const
    {
        return change_sign<0, 1, 0, 1>(xmm);
    }

    // change sign of real part
    Polar4f getconjugatedReal() const
    {
        return change_sign<1, 0, 1, 0>(xmm);
    }

    // from {real0, img0, real1, img1} return {real0, real0, real1, real1}
    Polar4f getOnlyReals() const
    {
        return Polar4f(_mm_moveldup_ps(xmm));
    }

    // from {real0, img0, real1, img1} return {img0, img0, img1, img1}
    Polar4f getOnlyImgs() const
    {
        return Polar4f(_mm_movehdup_ps(xmm));
    }

    // from {real0, img0, real1, img1} return {img0, real0, img1, real1}
    Polar4f getflipRealAndImg() const
    {
        return Polar4f(_mm_shuffle_ps(xmm, xmm, 0xB1));
    }

    void flipRealAndImg() 
    {
        _mm_shuffle_ps(xmm, xmm, 0xB1);
    }


    Polar2f real() const
    {
        float x[4];
        Vec4f(xmm).store(x);
        return Polar2f(x[0], x[2]);
    }
    Polar2f imag() const
    {
        float x[4];
        Vec4f(xmm).store(x);
        return Polar2f(x[1], x[3]);
    }
    
};

/*****************************************************************************
*
*          Operators for Polar4f
*
*****************************************************************************/

// operator + : add
static inline Polar4f operator + (Polar4f const & a, Polar4f const & b) {
    return Polar4f(Vec4f(a) + Vec4f(b));
}

// operator += : add
static inline Polar4f & operator += (Polar4f & a, Polar4f const & b) {
    a = a + b;
    return a;
}

// operator - : subtract
static inline Polar4f operator - (Polar4f const & a, Polar4f const & b) {
    return Polar4f(Vec4f(a) - Vec4f(b));
}

// operator - : unary minus
static inline Polar4f operator - (Polar4f const & a) {
    return Polar4f(- Vec4f(a));
}

// operator -= : subtract
static inline Polar4f & operator -= (Polar4f & a, Polar4f const & b) {
    a = a - b;
    return a;
}

// operator * : complex multiply is defined as
// (a.re * b.re - a.im * b.im,  a.re * b.im + b.re * a.im)
// http://yangkunlun.blogspot.com/2011/09/fast-complex-multiply-with-sse.html
static inline Polar4f operator * (Polar4f const & a, Polar4f const & b) {
    __m128 b_flip = _mm_shuffle_ps(b,b,0xB1);   // Swap b.re and b.im
    //__m128 a_im   = _mm_shuffle_ps(a,a,0xF5);   // Imag. part of a in both
    //__m128 a_re   = _mm_shuffle_ps(a,a,0xA0);   // Real part of a in both
    __m128 a_re = _mm_moveldup_ps(a);
    __m128 a_im = _mm_movehdup_ps(a);
    __m128 aib    = _mm_mul_ps(a_im, b_flip);   // (a.im*b.im, a.im*b.re)

//#ifdef __FMA__      // FMA3
//    return  _mm_fmaddsub_ps(a_re, b, aib);      // a_re * b +/- aib
//#elif defined (__FMA4__)  // FMA4
//    return  _mm_maddsub_ps (a_re, b, aib);      // a_re * b +/- aib
//#elif  INSTRSET >= 3  // SSE3
    __m128 arb    = _mm_mul_ps(a_re, b);        // (a.re*b.re, a.re*b.im)
    return _mm_addsub_ps(arb, aib);             // subtract/add
//#else
//    __m128 arb     = _mm_mul_ps(a_re, b);       // (a.re*b.re, a.re*b.im)
//    __m128 aib_m   = change_sign<1,0,1,0>(Vec4f(aib)); // change sign of low part
//    return _mm_add_ps(arb, aib_m);              // add
//#endif
}

// operator *= : multiply
static inline Polar4f & operator *= (Polar4f & a, Polar4f const & b) {
    a = a * b;
    return a;
}

// operator / : complex divide is defined as
// (a.re * b.re + a.im * b.im, b.re * a.im - a.re * b.im) / (b.re * b.re + b.im * b.im)
static inline Polar4f operator / (Polar4f const & a, Polar4f const & b) {
    // The following code is made similar to the operator * to enable common 
    // subexpression elimination in code that contains both operator * and 
    // operator / where one or both operands are the same
    __m128 a_re   = _mm_shuffle_ps(a,a,0xA0);   // Real part of a in both
    __m128 arb    = _mm_mul_ps(a_re, b);        // (a.re*b.re, a.re*b.im)
    __m128 b_flip = _mm_shuffle_ps(b,b,0xB1);   // Swap b.re and b.im
    __m128 a_im   = _mm_shuffle_ps(a,a,0xF5);   // Imag. part of a in both
#ifdef __FMA__      // FMA3
    __m128 n      = _mm_fmsubadd_ps(a_im, b_flip, arb); 
#elif defined (__FMA4__)  // FMA4
    __m128 n      = _mm_msubadd_ps (a_im, b_flip, arb);
#else
    __m128 aib    = _mm_mul_ps(a_im, b_flip);   // (a.im*b.im, a.im*b.re)
    __m128 arbm   = change_sign<0,1,0,1>(Vec4f(arb)); // change sign of high part
    __m128 n      = _mm_add_ps(arbm, aib);      // arbm + aib
#endif  // FMA
    __m128 bb     = _mm_mul_ps(b, b);           // (b.re*b.re, b.im*b.im)
    __m128 bb1    = _mm_shuffle_ps(bb,bb,0xB1); // Swap bb.re and bb.im
    __m128 bb2    = _mm_add_ps(bb,bb1);         // add pairwise into both positions
    return          _mm_div_ps(n, bb2);         // n / bb3
}

// operator /= : divide
static inline Polar4f & operator /= (Polar4f & a, Polar4f const & b) {
    a = a / b;
    return a;
}

// operator ~ : complex conjugate
// ~(a,b) = (a,-b)
static inline Polar4f operator ~ (Polar4f const & a) {
    return Polar4f(change_sign<0,1,0,1>(Vec4f(a))); // change sign of high part
}

// operator == : returns true if a == b
static inline Vec2db operator == (Polar4f const & a, Polar4f const & b) {
    Vec4fb t1 = Vec4f(a) == Vec4f(b);
    Vec4fb t2 = _mm_shuffle_ps(t1, t1, 0xB1); // swap real and imaginary parts
    return _mm_castps_pd(t1 & t2);
}

// operator != : returns true if a != b
//static inline Vec2db operator != (Polar4f const & a, Polar4f const & b) {
//    Vec4fb t1 = Vec4f(a) != Vec4f(b);
//    Vec4fb t2 = _mm_shuffle_ps(t1, t1, 0xB1); // swap real and imaginary parts
//    //bool res = Vec2db(_mm_castps_pd(t1 | t2));
//    return _mm_castps_pd(t1 | t2);
//}

static inline bool operator != (Polar4f const & a, Polar4f const & b)
{
    Vec4fb t1 = Vec4f(a) != Vec4f(b);
    Vec4fb t2 = _mm_shuffle_ps(t1, t1, 0xB1); // swap real and imaginary parts
    bool res = horizontal_or(Vec2db(_mm_castps_pd(t1 | t2)));
    return res;
}


/*****************************************************************************
*
*          Operators mixing Polar4f and float
*
*****************************************************************************/

// operator + : add
static inline Polar4f operator + (Polar4f const & a, float b) {
    return a + Polar4f(b);
}
static inline Polar4f operator + (float a, Polar4f const & b) {
    return b + a;
}
static inline Polar4f & operator += (Polar4f & a, float & b) {
    a = a + b;
    return a;
}

// operator - : subtract
static inline Polar4f operator - (Polar4f const & a, float b) {
    return a - Polar4f(b);
}
static inline Polar4f operator - (float a, Polar4f const & b) {
    return Polar4f(a) - b;
}
static inline Polar4f & operator -= (Polar4f & a, float & b) {
    a = a - b;
    return a;
}

// operator * : multiply
static inline Polar4f operator * (Polar4f const & a, float b) {
    return _mm_mul_ps(a, _mm_set1_ps(b));
}
static inline Polar4f operator * (float a, Polar4f const & b) {
    return b * a;
}
static inline Polar4f & operator *= (Polar4f & a, float & b) {
    a = a * b;
    return a;
}

// operator / : divide
static inline Polar4f operator / (Polar4f const & a, float b) {
    return _mm_div_ps(a, _mm_set1_ps(b));
}

static inline Polar4f operator / (float a, Polar4f const & b) {
    Vec4f b2(b);
    Vec4f b3 = b2 * b2;
    Vec4f t1 = _mm_shuffle_ps(b3,b3,0xB1); // swap real and imaginary parts
    Vec4f t2 = t1 + b3;
    Vec4f t3 = Vec4f(a) / t2;
    Vec4f t4 = Vec4f(~b) * t3;
    return Polar4f(t4);
}

static inline Polar4f & operator /= (Polar4f & a, float b) {
    a = a / b;
    return a;
}


/*****************************************************************************
*
*          Functions for Polar4f
*
*****************************************************************************/

// function abs: absolute value
// abs(a,b) = sqrt(a*a+b*b);
static inline Polar4f abs(Polar4f const & a) {
    Vec4f a1 = Vec4f(a);
    Vec4f a2 = a1 * a1;
    Vec4f t1 = _mm_shuffle_ps(a2,a2,0xB1); // swap real and imaginary parts
    Vec4f t2 = t1 + a2;
    Vec4f mask = _mm_castsi128_ps(constant4i<-1,0,-1,0>());
    Vec4f t3 = t2 & mask;                  // set imaginary parts to zero
    Vec4f t4 = sqrt(t3);
    return Polar4f(t4);
}

static inline Polar4f absPow2fast(Polar4f const & a)
{
    Vec4f a1 = Vec4f(a);
    Vec4f a2 = a1 * a1;
    Vec4f t1 = _mm_shuffle_ps(a2, a2, 0xB1); // swap real and imaginary parts
    Vec4f t2 = t1 + a2;
    return Polar4f(t2);
}


// function sqrt: square root
static inline Polar4f sqrt(Polar4f const & a) {
    __m128 t1  = _mm_mul_ps(a,a);             // r*r, i*i
    __m128 t2  = _mm_shuffle_ps(t1,t1,0xB1);  // swap real and imaginary parts
    __m128 t3  = _mm_add_ps(t1,t2);           // pairwise horizontal sum
    __m128 t4  = _mm_sqrt_ps(t3);             // n = sqrt(r*r+i*i)
    __m128 t5  = _mm_shuffle_ps(a,a,0xA0);    // copy real part of a
    __m128 sbithi = _mm_castsi128_ps(constant4ui<0,0x80000000,0,0x80000000>());  // 0.0, -0.0, 0.0, -0.0
    __m128 t6  = _mm_xor_ps(t5, sbithi);      // r, -r
    __m128 t7  = _mm_add_ps(t4,t6);           // n+r, n-r
    __m128 t8  = _mm_sqrt_ps(t7);             // sqrt(n+r), sqrt(n-r)
    __m128 t9  = _mm_and_ps(a,sbithi);        // 0, signbit of i
    __m128 t10 = _mm_xor_ps(t8, t9);          // sqrt(n+r), sign(i)*sqrt(n-r)
    __m128 t11 = Vec4f(0.7071067811865f);  // 1/sqrt(2)
    return _mm_mul_ps(t10, t11);
}

// function select
static inline Polar4f select (Vec2db const & s, Polar4f const & a, Polar4f const & b) {
    return Polar4f(select(_mm_castpd_ps(s), Vec4f(a), Vec4f(b)));
}



/*****************************************************************************
*
*          Mathematical functions
*
*****************************************************************************/




#ifdef VCL_NAMESPACE
}
#endif




#endif  // POLARVEC_H
