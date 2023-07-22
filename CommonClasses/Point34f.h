#pragma once
#include "CommonClassesShared.h"
#include <emmintrin.h> //  for SSE
#include <smmintrin.h> //  for SSE 4.1
#include "Memory.h"
#include "Vector.h"
#include "Vector34f.h"

namespace CommonClasses
{
    // Point3f - 3D point
    class Point34f
    {
    protected:
        __m128 xmm; // Float point
    public:
        #pragma region Constructors
        // Default constructor
        Point34f() = default;
        Point34f(Vector<float, 3> v)
            : Point34f(v(0), v(1), v(2))
        {
        }
        Point34f(Vector<float, 4> v)
            : Point34f(v(0), v(1), v(2))
        {
        }
        //Point34f(MemoryLink<float> m)
            //: Point34f(m(0), m(1), m(2))
        //{
        //}
        Point34f(const float* p)
        {
            load(p);
        }
        // Constructor to broadcast the same value into all elements
        Point34f(float f)
        {
            xmm = _mm_set1_ps(f);
        }
        // Constructor to build from all elements:
        Point34f(float x, float y, float z)
        {
            xmm = _mm_setr_ps(x, y, z, 0);
        }
        // Constructor to broadcast the same value into all elements
        Point34f(int x, int y, int z)
        {
            xmm = _mm_setr_ps(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0);
        }
        // Constructor to broadcast the same value into all elements
        Point34f(double d)
        {
            xmm = _mm_set1_ps(static_cast<float>(d));
        }
        // Constructor to build from all elements:
        Point34f(double x, double y, double z)
        {
            xmm = _mm_setr_ps(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0);
        }
        // Constructor to convert from type __m128 used in intrinsics
        Point34f(__m128 const & x)
        {
            xmm = x;
        }
        // Assignment operator to convert from type __m128 used in intrinsics
        Point34f & operator = (__m128 const & x)
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
        Point34f & load(float const * p)
        {
            xmm = _mm_loadu_ps(p);
            return *this;
        }
        // Member function to store into array (unaligned)
        void store(float * p) const
        {
            _mm_storeu_ps(p, xmm);
        }
        float operator () (int index) const
        {
            #if DEBUG
            assert(index >= 0 && index < 3 && "index is out of range");
            #endif
            return xmm.m128_f32[index];
        }
        float operator [] (int index) const
        {
            #if DEBUG
            assert(index >= 0 && index < 3 && "index is out of range");
            #endif
            return xmm.m128_f32[index];
        }
        #pragma endregion 

        #pragma region Vector Operators

        // operator +  add Point34f and Vector34f
        friend inline Point34f operator + (Point34f const & a, Vector34f const & v)
        {
            return Point34f(_mm_add_ps(a.xmm, v));
        }
        // operator +  add
        friend inline Point34f operator + (Point34f const & a, Point34f const & b)
        {
            return _mm_add_ps(a.xmm, b.xmm);
        }
        // operator -  subtract Point34f and Vector34f
        friend inline Point34f operator - (Point34f const & a, Vector34f const & v)
        {
            return Point34f(_mm_sub_ps(a.xmm, v));
        }
        // operator -  subtract
        friend inline Vector34f operator - (Point34f const & a, Point34f const & b)
        {
            return Vector34f(_mm_sub_ps(a.xmm, b.xmm));
        }

        /* DOES vector operators valid on Point ?
        // operator *  multiply element-by-element (see also cross_product and dot_product)
        friend inline Point34f operator * (Point34f const & a, Point34f const & b)
        {
            return _mm_mul_ps(a.xmm, b.xmm);
        }*/

        // operator +=  add
        friend inline Point34f & operator += (Point34f & a, Point34f const & b)
        {
            a = a + b;
            return a;
        }
        friend inline Point34f & operator += (Point34f & a, Vector34f const & v)
        {
            a = a + v;
            return a;
        }
        /* DOES vector operators valid on Point ?
        // operator -=  subtract
        friend inline Point34f & operator -= (Point34f & a, Point34f const & b)
        {
            a = a - b;
            return a;
        }
        // operator *=  multiply
        friend inline Point34f & operator *= (Point34f & a, Point34f const & b)
        {
            a = a * b;
            return a;
        }
        */
        // operator -  unary minus
        friend inline Point34f operator - (Point34f const & a)
        {
            return _mm_xor_ps(a, _mm_castsi128_ps(_mm_set1_epi32(0x80000000)));
        }

        // operator ==  returns true if a == b
        friend inline bool operator == (Point34f const & a, Point34f const & b)
        {
            __m128 a_eq_b = _mm_cmpeq_ps(a, b);  // a == b
            __m128 a_eq_b_3 = _mm_shuffle_ps(a_eq_b, a_eq_b, 0x24);  // ignore unused top element
            return _mm_movemask_ps(a_eq_b_3) == 0x0F; // Returns true if all bits are 1
        }

        // operator != : returns true if a != b
        friend inline bool operator != (Point34f const & a, Point34f const & b)
        {
            __m128 a_neq_b = _mm_cmpneq_ps(a, b); // a != b
            __m128 a_neq_b_3 = _mm_shuffle_ps(a_neq_b, a_neq_b, 0x24);  // ignore unused top element
            return _mm_movemask_ps(a_neq_b_3) == 0;  // Returns true if at least one bit is 1
        }

        #pragma endregion 

        #pragma region Scalar Operators

        // operator *  multiply
        friend inline Point34f operator * (Point34f const & a, float c)
        {
            return _mm_mul_ps(a, _mm_set1_ps(c));
        }

        // operator *  multiply
        friend inline Point34f operator * (float c, Point34f const & a)
        {
            return a * c;
        }

        // operator /  divide
        friend inline Point34f operator / (Point34f const & a, float c)
        {
            return _mm_div_ps(a, _mm_set1_ps(c));
        }

        // operator *=  multiply
        friend inline Point34f & operator *= (Point34f & a, float c)
        {
            a = a * c;
            return a;
        }

        // operator /=  divide
        friend inline Point34f & operator /= (Point34f & a, float c)
        {
            a = a / c;
            return a;
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
        CommonClassesINLINE Point34f transpose() const
        {
            return *this;
        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Point34f& eval()
        {
            return *this;
        }
        #pragma endregion 

        #pragma region Convertions
        std::string toString() const
        {
            return std::to_string(xmm.m128_f32[0]) + ", " + std::to_string(xmm.m128_f32[1]) + ", " + std::to_string(xmm.m128_f32[2]);
        }
        friend std::ostream& operator<<(std::ostream& os, const Point34f& v)
        {
            os << v.toString();
            return os;
        }
        #pragma endregion 

    };
}
