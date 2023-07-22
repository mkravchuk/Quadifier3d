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
    class Matrix34f
    {
    protected:
        __m128 xmm[3]; // 3 rows

    public:
        #pragma region Constructors
        Matrix34f()
        {
        }
        Matrix34f(Vector34f v0, Vector34f v1, Vector34f v2)
        {
            xmm[0] = v0;
            xmm[1] = v1;
            xmm[2] = v2;
        }
        Matrix34f(Point34f v0, Point34f v1, Point34f v2)
        {
            xmm[0] = v0;
            xmm[1] = v1;
            xmm[2] = v2;
        }

        Matrix34f(float a00, float a01, float a02, float a10, float a11, float a12, float a20, float a21, float a22)
        {
            xmm[0] = Vector34f(a00, a01, a02);
            xmm[1] = Vector34f(a10, a11, a12);
            xmm[2] = Vector34f(a20, a21, a22);
        }
        #pragma endregion 

        #pragma region Dimension control
        constexpr Index rows() const
        {
            return 3;
        }
        constexpr Index cols() const
        {
            return 3;
        }
        #pragma endregion 

        #pragma region  Fill data
        void setConstant(float value)
        {
            Vector34f constant(value);
            xmm[0] = constant;
            xmm[1] = constant;
            xmm[2] = constant;
        }
        void setZero()
        {
            setConstant(0);
        }
        void setRandom()
        {
            xmm[0] = Vector34f(static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand()));
            xmm[1] = Vector34f(static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand()));
            xmm[2] = Vector34f(static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand()));
        }
        #pragma endregion 

        #pragma region Read/Write
        CommonClassesINLINE float&  operator()(Index row, Index col)
        {
            #if DEBUG
            assert(row >= 0 && row < 3 && "row index is out of range");
            assert(col >= 0 && col < 3 && "col index is out of range");
            #endif
            return xmm[row].m128_f32[col];
        }
        CommonClassesINLINE float  operator()(Index row, Index col) const
        {
            #if DEBUG
            assert(row >= 0 && row < 3 && "row index is out of range");
            assert(col >= 0 && col < 3 && "col index is out of range");
            #endif
            return xmm[row].m128_f32[col];
        }
        CommonClassesINLINE Vector34f col(Index col) const
        {
            #if DEBUG
            assert(col >= 0 && col < 3 && "row is out of range");
            #endif
            return Vector34f(xmm[0].m128_f32[col], xmm[1].m128_f32[col], xmm[2].m128_f32[col]);
        }
        CommonClassesINLINE Vector34f& row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < 3 && "row is out of range");
            #endif
            return *(Vector34f*)(&xmm[row]); // dont convert to static cast - will not compile
        }
        CommonClassesINLINE Vector34f row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < 3 && "row is out of range");
            #endif
            return Vector34f(xmm[row]);
        }
        #pragma endregion 


        #pragma region Matrix Operators

        // operator +    add
        friend CommonClassesINLINE  Matrix34f operator * (Matrix34f const & m, Matrix34f const & mult)
        {
            Vector34f row0 = m.row(0);
            Vector34f row1 = m.row(1);
            Vector34f row2 = m.row(2);
            Vector34f col0 = mult.col(0);
            Vector34f col1 = mult.col(1);
            Vector34f col2 = mult.col(2);

            auto r0 = Vector34f(row0.dot(col0), row0.dot(col1), row0.dot(col2));
            auto r1 = Vector34f(row1.dot(col0), row1.dot(col1), row1.dot(col2));
            auto r2 = Vector34f(row2.dot(col0), row2.dot(col1), row2.dot(col2));

            return Matrix34f(r0, r1, r2);
        }


        // operator +    add
        friend CommonClassesINLINE  Matrix34f operator + (Matrix34f const & m, Matrix34f const & m2)
        {
            return Matrix34f(m.row(0) + m2.row(0), m.row(1) + m2.row(1), m.row(2) + m2.row(2));
        }

        // operator -    minus
        friend CommonClassesINLINE  Matrix34f operator - (Matrix34f const & m, Matrix34f const & m2)
        {
            return Matrix34f(m.row(0) - m2.row(0), m.row(1) - m2.row(1), m.row(2) - m2.row(2));
        }

        // operator -    unary minus
        friend CommonClassesINLINE  Matrix34f operator - (Matrix34f const & m)
        {
            return Matrix34f(-m.row(0), -m.row(1), -m.row(2));
        }

        // operator ==   returns true if m == m2
        friend CommonClassesINLINE  bool operator == (Matrix34f const & m, Matrix34f const & m2)
        {
            return m.row(0) == m2.row(0) && m.row(1) == m2.row(1) && m.row(2) == m2.row(2);
        }

        // operator !=   returns true if m != m2
        friend CommonClassesINLINE  bool operator != (Matrix34f const & m, Matrix34f const & m2)
        {
            return !(m == m2);
        }

        #pragma endregion 

        #pragma region Vector Operators

        // operator *    multiply by vector
        friend CommonClassesINLINE Vector34f operator * (Matrix34f const & m, Vector34f const & v)
        {
            return Vector34f(m.row(0).dot(v), m.row(1).dot(v), m.row(2).dot(v));
        }

        #pragma endregion 

        #pragma region Scalar Operators

        // operator *    multiply by scalar
        friend CommonClassesINLINE  Matrix34f operator * (Matrix34f const & m, const float c)
        {
            Vector34f vc(c);
            return Matrix34f(m.row(0)*vc, m.row(1)*vc, m.row(2)*vc);
        }

        // operator *    multiply
        friend CommonClassesINLINE  Matrix34f operator * (float c, Matrix34f const & m)
        {
            return m * c;
        }

        // operator /     divide by scalar
        friend CommonClassesINLINE  Matrix34f operator / (Matrix34f const & m, const float c)
        {
            return Matrix34f(m.row(0)/c, m.row(1)/c, m.row(2)/c);
        }

        // operator *=   multiply
        friend CommonClassesINLINE  Matrix34f & operator *= (Matrix34f & m, float c)
        {
            Vector34f vc(c);
            m.row(0) *= vc;
            m.row(1) *= vc;
            m.row(2) *= vc;
            return m;
        }

        // operator *=   multiply
        friend CommonClassesINLINE  Matrix34f & operator /= (Matrix34f & m, float c)
        {
            m.row(0) /= c;
            m.row(1) /= c;
            m.row(2) /= c;
            return m;
        }

        #pragma endregion 


        #pragma region floatransformations
        Matrix34f transpose() const
        {
            return Matrix34f(col(0), col(1), col(2));
        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Matrix34f& eval()
        {
            return *this;
        }
        #pragma endregion  

        #pragma region Convertions
        Table<float, 3, 3> cast()
        {
            Table<float, 3, 3>  m;
            for (Index row = 0; row < 3; row++)
                for (Index col = 0; col < 3; col++)
                m[3*row+col] = xmm[row].m128_f32[col];
            return m;
        }
        std::string toString() const
        {
            std::string s = "";
            for (Index row = 0; row < 3; row++)
            {
                if (row != 0) s += "\n";
                for (Index col = 0; col < 3; col++)
                {
                    if (col != 0) s += ", ";
                    s += std::to_string(xmm[row].m128_f32[col]);
                }
            }
            return s;
        }
        friend std::ostream& operator<<(std::ostream& os, const Matrix34f& m)
        {
            os << m.toString();
            return os;
        }
        #pragma endregion  

        #pragma region Static Methods
        CommonClassesINLINE static Matrix34f Zero()
        {
            Matrix34f  m;
            m.setZero();
            return m;
        }
        CommonClassesINLINE static Matrix34f Constant(float value)
        {
            Matrix34f  m;
            m.setConstant(value);
            return m;
        }
        CommonClassesINLINE static Matrix34f Random()
        {
            Matrix34f  m;
            m.setRandom();
            return m;
        }
        static Matrix34f Identity()
        {
            return Matrix34f(1, 0, 0, 0, 1, 0, 0, 0, 1);
        }
        #pragma endregion 
    };
}