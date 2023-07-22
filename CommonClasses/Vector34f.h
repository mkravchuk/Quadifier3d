#pragma once
#include "CommonClassesShared.h"
#include <emmintrin.h> //  for SSE
#include <smmintrin.h> //  for SSE 4.1
#include "Memory.h"
#include "Vector.h"

namespace CommonClasses
{
    // Vector3f - 3D vector
    class Vector34f
    {
    protected:
        __m128 xmm; // Float vector
        //inline void zerolast()
        //{
        //    xmm.m128_f32[3] = 0;
        //}
    public:
        #pragma region Constructors
        // Default constructor
        Vector34f() = default;
        Vector34f(Vector<float, 3> v)
            : Vector34f(v(0), v(1), v(2))
        {
        }
        Vector34f(Vector<float, 4> v)
            : Vector34f(v(0), v(1), v(2))
        {
        }
        //Vector34f(MemoryLink<float> m)
        //    : Vector34f(m.data())
        //{
        //}
        Vector34f(const float* p)
        {
            load(p);
        }
        // Constructor to broadcast the same value into all elements
        Vector34f(float f)
        {
            xmm = _mm_setr_ps(f, f, f, 0);
        }
        // Constructor to build from all elements:
        Vector34f(float x, float y, float z)
        {
            xmm = _mm_setr_ps(x, y, z, 0);
        }
        // Constructor to broadcast the same value into all elements
        Vector34f(int x, int y, int z)
            : Vector34f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z))
        {
        }
        // Constructor to broadcast the same value into all elements
        Vector34f(double d)
            : Vector34f(static_cast<float>(d))
        {
        }
        // Constructor to build from all elements
        Vector34f(double x, double y, double z)
            : Vector34f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z))
        {
        }
        // Constructor to convert from type __m128 used in intrinsics:
        Vector34f(__m128 const & x)
        {
            xmm = x;
        }
        // Assignment operator to convert from type __m128 used in intrinsics:
        Vector34f & operator = (__m128 const & x)
        {
            xmm = x;
            return *this;
        }
        // Type cast operator to convert to __m128 used in intrinsics
        operator __m128() const
        {
            return xmm;
        }
        #pragma endregion 

        #pragma region Convert to
        operator Vector<float, 3>() const
        {
            return Vector<float, 3>(xmm.m128_f32[0], xmm.m128_f32[1], xmm.m128_f32[2]);
        }
        operator Vector<float, 4>() const
        {
            //return Vector<float, 4>(xmm.m128_f32[0], xmm.m128_f32[1], xmm.m128_f32[2], 0);
            Vector<float, 4> v;
            store(v.data());
            return v;
        }
        #pragma endregion 

        #pragma region Read/Write
        // Member function to load from array (unaligned)
        Vector34f & load(float const * p)
        {
            xmm = _mm_loadu_ps(p);
            return *this;
        }
        // Member function to store into array (unaligned)
        void store(float * p) const
        {
            _mm_storeu_ps(p, xmm);
        }
        CommonClassesINLINE float operator () (Index index) const
        {
            #if DEBUG
            assert(index >= 0 && index < 3 && "index is out of range");
            #endif
            return xmm.m128_f32[index];
        }
        CommonClassesINLINE float operator [] (Index index) const
        {
            #if DEBUG
            assert(index >= 0 && index < 3 && "index is out of range");
            #endif
            return xmm.m128_f32[index];
        }
        #pragma endregion 

        #pragma region Vector Operators

        // operator +  add
        friend inline Vector34f operator + (Vector34f const & a, Vector34f const & b)
        {
            return _mm_add_ps(a.xmm, b.xmm);
        }
        // operator -  subtract
        friend inline Vector34f operator - (Vector34f const & a, Vector34f const & b)
        {
            return _mm_sub_ps(a.xmm, b.xmm);
        }
        // operator *  multiply element-by-element (see also cross_product and dot_product)
        friend inline Vector34f operator * (Vector34f const & a, Vector34f const & b)
        {
            return _mm_mul_ps(a.xmm, b.xmm);
        }

        // operator +=  add
        friend inline Vector34f & operator += (Vector34f & a, Vector34f const & b)
        {
            a = a + b;
            return a;
        }
        // operator -=  subtract
        friend inline Vector34f & operator -= (Vector34f & a, Vector34f const & b)
        {
            a = a - b;
            return a;
        }
        // operator *=  multiply
        friend inline Vector34f & operator *= (Vector34f & a, Vector34f const & b)
        {
            a = a * b;
            return a;
        }

        // operator -  unary minus
        friend inline Vector34f operator - (Vector34f const & a)
        {
            return _mm_xor_ps(a, _mm_castsi128_ps(_mm_set1_epi32(0x80000000)));
        }

        // operator ==  returns true if a == b
        friend inline bool operator == (Vector34f const & a, Vector34f const & b)
        {
            __m128 a_eq_b = _mm_cmpeq_ps(a, b);  // a == b
            __m128 a_eq_b_3 = _mm_shuffle_ps(a_eq_b, a_eq_b, 0x24);  // ignore unused top element
            return _mm_movemask_ps(a_eq_b_3) == 0x0F; // Returns true if all bits are 1
        }

        // operator != : returns true if a != b
        friend inline bool operator != (Vector34f const & a, Vector34f const & b)
        {
            __m128 a_neq_b = _mm_cmpneq_ps(a, b); // a != b
            __m128 a_neq_b_3 = _mm_shuffle_ps(a_neq_b, a_neq_b, 0x24);  // ignore unused top element
            return _mm_movemask_ps(a_neq_b_3) == 0;  // Returns true if at least one bit is 1
        }

        #pragma endregion 

        #pragma region Scalar Operators

        // operator *  multiply
        friend inline Vector34f operator * (Vector34f const & a, float c)
        {
            return _mm_mul_ps(a, _mm_set1_ps(c));
        }
        // operator *  multiply
        friend inline Vector34f operator * (float c, Vector34f const & a)
        {
            return a * c;
        }
        // operator /  divide
        friend inline Vector34f operator / (Vector34f const & a, float c)
        {
            return _mm_div_ps(a, _mm_set1_ps(c));
        }

        // operator *=  multiply
        friend inline Vector34f & operator *= (Vector34f & a, float c)
        {
            a = a * c;
            return a;
        }
        // operator /=  divide
        friend inline Vector34f & operator /= (Vector34f & a, float c)
        {
            a = a / c;
            return a;
        }

        #pragma endregion 

        #pragma region Normalization
        inline float squaredNorm() const
        {
            return dot(*this);
        }
        inline float squaredNorm_sse41() const
        {
            return dot_sse41(*this);
        }
        inline float norm() const
        {
            return _mm_sqrt_ps(_mm_set1_ps(squaredNorm())).m128_f32[0];
        }
        // all 4 floats will have same value: norm of vector
        inline __m128 norm_sse41_simd() const
        {
            return _mm_sqrt_ps(_mm_dp_ps(xmm, xmm, 127));
        }
        inline float norm_sse41() const
        {
            return norm_sse41_simd().m128_f32[0];
        }
        inline float normPow2() const
        {
            return squaredNorm();
        }
        inline float normPow2_sse41() const
        {
            return squaredNorm_sse41();
        }
        inline void normalize()
        {
            // v0 - original - precise
            *this /= norm();
            // v1 - fast - but not precise
            //float rsqrt = _mm_rsqrt_ss(_mm_set1_ps(squaredNorm())).m128_f32[0];
            //*this *= rsqrt;
        }
        inline void normalize_sse41()
        {
            // v0 - original - precise
            //*this /= norm_sse41();
            // v0 - original - precise - optimized
            xmm = _mm_div_ps(xmm, norm_sse41_simd());
            // v1 - fast - but not precise
            //float rsqrt = _mm_rsqrt_ss(_mm_set1_ps(squaredNorm())).m128_f32[0];
            //*this *= rsqrt;
        }
        // do normalization using faster but not very precise method
        //inline void normalize_fast()
        //{
        //    // v0 - original - precise
        //    //*this /= norm();
        //    // v1 - fast - but not precise
        //    float rsqrt = _mm_rsqrt_ss(_mm_set1_ps(squaredNorm())).m128_f32[0];
        //    *this *= rsqrt;
        //}
        //// do normalization using faster but not very precise method
        //inline void normalize_fast_sse41()
        //{
        //    // v0 - original - precise
        //    //*this /= norm();
        //    // v1 - fast - but not precise
        //    float rsqrt = _mm_rsqrt_ss(_mm_dp_ps(*this, *this, 113)).m128_f32[0];
        //    *this *= rsqrt;
        //}
        inline Vector34f normalized() const
        {
            // v0 - original - precise
            return *this / norm();
            // v1 - fast - but not precise
            //float rsqrt = _mm_rsqrt_ss(_mm_set1_ps(squaredNorm())).m128_f32[0];
            //return *this * rsqrt;
        }
        inline Vector34f normalized_sse41() const
        {
            // v0 - original - precise
            //return *this / norm_sse41();
            // v0 - original - precise - optimized
            return Vector34f(_mm_div_ps(xmm, norm_sse41_simd()));
            // v1 - fast - but not precise
            //float rsqrt = _mm_rsqrt_ss(_mm_set1_ps(squaredNorm_sse41())).m128_f32[0];
            //return *this * rsqrt;
        }
        #pragma endregion  

        #pragma region Cross, Dot
        inline float dot(const Vector34f& v) const
        {
            Vector34f mult = (*this)*v;
            return mult.sum();
        }
        inline float dot_sse41(const Vector34f& v) const
        {
            return _mm_dp_ps(*this, v, 113).m128_f32[0];
        }
        inline Vector34f cross(const Vector34f& v) const
        {
            //taken from 'Investigating SSE Cross Product Performance':  http://threadlocalmutex.com/?p=8            

            // v0 - 4 shuffles
            //__m128 a_yzx = _mm_shuffle_ps(xmm, xmm, _MM_SHUFFLE(3, 0, 2, 1));
            //__m128 a_zxy = _mm_shuffle_ps(xmm, xmm, _MM_SHUFFLE(3, 1, 0, 2));
            //__m128 b_zxy = _mm_shuffle_ps(v.xmm, v.xmm, _MM_SHUFFLE(3, 1, 0, 1));
            //__m128 b_yzx = _mm_shuffle_ps(v.xmm, v.xmm, _MM_SHUFFLE(3, 0, 2, 1));
            //return _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));

            // v1 - 3 shuffles
            __m128 a_yzx = _mm_shuffle_ps(xmm, xmm, _MM_SHUFFLE(3, 0, 2, 1));
            __m128 b_yzx = _mm_shuffle_ps(v.xmm, v.xmm, _MM_SHUFFLE(3, 0, 2, 1));
            __m128 c = _mm_sub_ps(_mm_mul_ps(xmm, b_yzx), _mm_mul_ps(a_yzx, v.xmm));
            return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1));
        }
        #pragma endregion 

        #pragma region Agregators
        inline float sum() const
        {
            // v0 - add 4 floats
            //__m128 t1 = _mm_hadd_ps(xmm, xmm);
            //__m128 t2 = _mm_hadd_ps(t1, t1);
            //return _mm_cvtss_f32(t2);
            
            // v1 - add 3 floats - avoiding adding 4-th number, coz it may be some random number
            //__m128 t1 = _mm_hadd_ps(xmm, xmm);
            //float a0_plus_a1 = t1.m128_f32[0];
            //float a3 = t1.m128_f32[2];
            //return a0_plus_a1 + a3;

            // v2 - add 3 floats
            return xmm.m128_f32[0] + xmm.m128_f32[1] + xmm.m128_f32[2];
        }
        inline float minCoeff() const
        {
            // v0 - 4 floats
            //__m128 data = xmm;             /* [0, 1, 2, 3] */
            //__m128 low = _mm_movehl_ps(data, data); /* [2, 3, 2, 3] */
            //__m128 low_accum = _mm_min_ps(low, data); /* [0|2, 1|3, 2|2, 3|3] */
            //__m128 elem1 = _mm_shuffle_ps(low_accum, low_accum, _MM_SHUFFLE(1, 1, 1, 1)); /* [1|3, 1|3, 1|3, 1|3] */
            //__m128 accum = _mm_min_ss(low_accum, elem1);
            //return _mm_cvtss_f32(accum);

            // v1 - 3 floats
            float min01 = xmm.m128_f32[0] < xmm.m128_f32[1] ? xmm.m128_f32[0] : xmm.m128_f32[1];
            return xmm.m128_f32[2] < min01 ? xmm.m128_f32[2] : min01;
        }
        inline float maxCoeff() const
        {
            // v0 - 4 floats
            //__m128 data = xmm;             /* [0, 1, 2, 3] */
            //__m128 high = _mm_movehl_ps(data, data); /* [2, 3, 2, 3] */
            //__m128 high_accum = _mm_max_ps(high, data); /* [0|2, 1|3, 2|2, 3|3] */
            //__m128 elem1 = _mm_shuffle_ps(high_accum, high_accum, _MM_SHUFFLE(1, 1, 1, 1)); /* [1|3, 1|3, 1|3, 1|3] */
            //__m128 accum = _mm_max_ss(high_accum, elem1);
            //return _mm_cvtss_f32(accum);

            // v1 - 3 floats
            float max01 = xmm.m128_f32[0] > xmm.m128_f32[1] ? xmm.m128_f32[0] : xmm.m128_f32[1];
            return xmm.m128_f32[2] > max01 ? xmm.m128_f32[2] : max01;

        }
        //inline void minCoeff(Index& index) const
        //{

        //}
        //inline void maxCoeff(Index& index) const
        //{

        //}
        #pragma endregion 

        #pragma region Transformations
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Vector34f transpose() const
        {
            return *this;
        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Vector34f& eval()
        {
            return *this;
        }
        #pragma endregion 

        #pragma region Convertions
        std::string toString() const
        {
            return std::to_string(xmm.m128_f32[0]) + ", " + std::to_string(xmm.m128_f32[1]) + ", " + std::to_string(xmm.m128_f32[2]);
        }
        friend std::ostream& operator<<(std::ostream& os, const Vector34f& v)
        {
            os << v.toString();
            return os;
        }
        #pragma endregion 

    };
}
