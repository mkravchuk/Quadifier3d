#pragma once
#include "CommonClassesShared.h"
#include "MemoryFixed.h"


namespace CommonClasses
{
    // Specialization - fixed vector (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T, Index SIZE = X>
    class Vector : public MemoryFixed<T, SIZE>
    {
    public: 
        #pragma region Constructors
        Vector() = default;
        Vector(const T* data)
        {
            for (Index i = 0; i < SIZE; i++)
            {
                m_data[i] = data[i];
            }
        }
        template <Index S = SIZE, std::enable_if_t<(S == 2)>* = 0>              // Method available only for Vector2     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Vector(const T x, const T y)
        {
            m_data[0] = x;
            m_data[1] = y;
        }
        template <Index S = SIZE, std::enable_if_t<(S == 3)>* = 0>              // Method available only for Vector3     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Vector(const T x, const T y, const T z)
        {
            m_data[0] = x;
            m_data[1] = y;
            m_data[2] = z;
        }
        template <Index S = SIZE, std::enable_if_t<(S == 4)>* = 0>              // Method available only for Vector4     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Vector(const T x, const T y, const T z, const T w)
        {
            m_data[0] = x;
            m_data[1] = y;
            m_data[2] = z;
            m_data[3] = w;
        }
        inline Vector<T, SIZE>& operator=(const T* data)
        {
            for (Index i = 0; i < SIZE; i++)
            {
                m_data[i] = data[i];
            }
            return *this;
        }
        #pragma endregion 

        #pragma region  Fill data (Math)
        using MemoryFixed<T, SIZE>::setConstant;
        #include "_IMemory_SingleValue_MathFillData"
        #pragma endregion 

        #pragma region Read/Write (row)
        // same as [] or () - because Vector is a Matrix with dimension <N,1>
        CommonClassesINLINE T& row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < size() && "row is out of range");
            #endif
            return m_data[row];
        }
        // same as [] or () - because Vector is a Matrix with dimension <N,1>
        CommonClassesINLINE T row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < size() && "row is out of range");
            #endif
            return m_data[row];
        }
        #pragma endregion 


        #pragma region Vector Operators

        // operator +  add
        friend CommonClassesINLINE  Vector<T, SIZE> operator + (Vector<T, SIZE> const & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) + b(0);
                v(1) = a(1) + b(1);
                return v;
            }
            else if (SIZE == 3)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) + b(0);
                v(1) = a(1) + b(1);
                v(2) = a(2) + b(2);
                return v;
            }
            else if (SIZE == 4)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) + b(0);
                v(1) = a(1) + b(1);
                v(2) = a(2) + b(2);
                v(3) = a(3) + b(3);
                return v;
            }
            else
            {
                Vector<T, SIZE> v;
                for (Index i = 0; i < SIZE; i++)
                {
                    v(i) = a(i) + b(i);
                }
                return v;
            }
        }
        // operator -  subtract
        friend CommonClassesINLINE  Vector<T, SIZE> operator - (Vector<T, SIZE> const & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) - b(0);
                v(1) = a(1) - b(1);
                return v;
            }
            else if (SIZE == 3)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) - b(0);
                v(1) = a(1) - b(1);
                v(2) = a(2) - b(2);
                return v;
            }
            else if (SIZE == 4)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) - b(0);
                v(1) = a(1) - b(1);
                v(2) = a(2) - b(2);
                v(3) = a(3) - b(3);
                return v;
            }
            else
            {
                Vector<T, SIZE> v;
                for (Index i = 0; i < SIZE; i++)
                {
                    v(i) = a(i) - b(i);
                }
                return v;
            }
        }
        // operator *  multiply element-by-element (see also cross_product and dot_product)
        friend CommonClassesINLINE  Vector<T, SIZE> operator * (Vector<T, SIZE> const & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) * b(0);
                v(1) = a(1) * b(1);
                return v;
            }
            else if (SIZE == 3)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) * b(0);
                v(1) = a(1) * b(1);
                v(2) = a(2) * b(2);
                return v;
            }
            else if (SIZE == 4)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) * b(0);
                v(1) = a(1) * b(1);
                v(2) = a(2) * b(2);
                v(3) = a(3) * b(3);
                return v;
            }
            else
            {
                Vector<T, SIZE> v;
                for (Index i = 0; i < SIZE; i++)
                {
                    v(i) = a(i) * b(i);
                }
                return v;
            }
        }

        // operator +=  add
        friend CommonClassesINLINE  Vector<T, SIZE> & operator += (Vector<T, SIZE> & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                a(0) += b(0); a(1) += b(1);  return a;
            }
            else if (SIZE == 3)
            {
                a(0) += b(0); a(1) += b(1); a(2) += b(2); return a;
            }
            else if (SIZE == 4)
            {
                a(0) += b(0); a(1) += b(1); a(2) += b(2); a(3) += b(3); return a;
            }
            else
            {
                for (Index i = 0; i < SIZE; i++) a(i) += b(i); return a;
            }
        }
        // operator -=  subtract
        friend CommonClassesINLINE  Vector<T, SIZE> & operator -= (Vector<T, SIZE> & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                a(0) -= b(0); a(1) -= b(1);  return a;
            }
            else if (SIZE == 3)
            {
                a(0) -= b(0); a(1) -= b(1); a(2) -= b(2); return a;
            }
            else if (SIZE == 4)
            {
                a(0) -= b(0); a(1) -= b(1); a(2) -= b(2); a(3) -= b(3); return a;
            }
            else
            {
                for (Index i = 0; i < SIZE; i++) a(i) -= b(i); return a;
            }
        }
        // operator *=  multiply
        friend CommonClassesINLINE  Vector<T, SIZE> & operator *= (Vector<T, SIZE> & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                a(0) *= b(0); a(1) *= b(1);   return a;
            }
            else if (SIZE == 3)
            {
                a(0) *= b(0); a(1) *= b(1); a(2) *= b(2);  return a;
            }
            else if (SIZE == 4)
            {
                a(0) *= b(0); a(1) *= b(1); a(2) *= b(2); a(3) *= b(3);  return a;
            }
            else
            {
                for (Index i = 0; i < SIZE; i++)
                {
                    a(i) *= b(i);
                }
                return a;
            }
        }

        // operator -  unary minus
        friend CommonClassesINLINE  Vector<T, SIZE> operator - (Vector<T, SIZE> const & a)
        {
            if (SIZE == 2)
            {
                Vector<T, SIZE> v;
                v(0) = -a(0);
                v(1) = -a(1);
                return v;
            }
            else if (SIZE == 3)
            {
                Vector<T, SIZE> v;
                v(0) = -a(0);
                v(1) = -a(1);
                v(2) = -a(2);
                return v;
            }
            else if (SIZE == 4)
            {
                Vector<T, SIZE> v;
                v(0) = -a(0);
                v(1) = -a(1);
                v(2) = -a(2);
                v(3) = -a(3);
                return v;
            }
            else
            {
                Vector<T, SIZE> v;
                for (Index i = 0; i < SIZE; i++)
                {
                    v(i) = -a(i);
                }
                return v;
            }
        }

        // operator ==  returns true if a == b
        friend CommonClassesINLINE  bool operator == (Vector<T, SIZE> const & a, Vector<T, SIZE> const & b)
        {
            if (SIZE == 2)
            {
                return a(0) == b(0) && a(1) == b(1);
            }
            else if (SIZE == 3)
            {
                return a(0) == b(0) && a(1) == b(1) && a(2) == b(2);
            }
            else if (SIZE == 4)
            {
                return a(0) == b(0) && a(1) == b(1) && a(2) == b(2) && a(3) == b(3);
            }
            else
            {
                for (Index i = 0; i < SIZE; i++)
                {
                    if (a(i) != b(i)) return false;
                }
                return true;
            }
        }

        // operator != : returns true if a != b
        friend CommonClassesINLINE  bool operator != (Vector<T, SIZE> const & a, Vector<T, SIZE> const & b)
        {
            return !(a == b);
        }

        #pragma endregion 

        #pragma region Scalar Operators

        // operator *  multiply
        friend CommonClassesINLINE  Vector<T, SIZE> operator * (Vector<T, SIZE> const & a, T c)
        {
            if (SIZE == 2)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) * c;
                v(1) = a(1) * c;
                return v;
            }
            else if (SIZE == 3)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) * c;
                v(1) = a(1) * c;
                v(2) = a(2) * c;
                return v;
            }
            else if (SIZE == 4)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) * c;
                v(1) = a(1) * c;
                v(2) = a(2) * c;
                v(3) = a(3) * c;
                return v;
            }
            else
            {
                Vector<T, SIZE> v;
                for (Index i = 0; i < SIZE; i++)
                {
                    v(i) = a(i) * c;
                }
                return v;
            }
        }
        // operator *  multiply
        friend CommonClassesINLINE  Vector<T, SIZE> operator * (T c, Vector<T, SIZE> const & a)
        {
            return a * c;
        }
        // operator /  divide
        friend CommonClassesINLINE  Vector<T, SIZE> operator / (Vector<T, SIZE> const & a, T c)
        {
            if (SIZE == 2)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) / c;
                v(1) = a(1) / c;
                return v;
            }
            else if (SIZE == 3)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) / c;
                v(1) = a(1) / c;
                v(2) = a(2) / c;
                return v;
            }
            else if (SIZE == 4)
            {
                Vector<T, SIZE> v;
                v(0) = a(0) / c;
                v(1) = a(1) / c;
                v(2) = a(2) / c;
                v(3) = a(3) / c;
                return v;
            }
            else
            {
                Vector<T, SIZE> v;
                for (Index i = 0; i < SIZE; i++)
                {
                    v(i) = a(i) / c;
                }
                return v;
            }
        }

        // operator *=  multiply
        friend CommonClassesINLINE  Vector<T, SIZE> & operator *= (Vector<T, SIZE> & a, T c)
        {
            if (SIZE == 2)
            {
                a(0) *= c; a(1) *= c;  return a;
            }
            else if (SIZE == 3)
            {
                a(0) *= c; a(1) *= c; a(2) *= c; return a;
            }
            else if (SIZE == 4)
            {
                a(0) *= c; a(1) *= c; a(2) *= c; a(3) *= c; return a;
            }
            else
            {
                for (Index i = 0; i < SIZE; i++) a(i) *= c; return a;
            }
        }
        // operator /=  divide
        friend CommonClassesINLINE  Vector<T, SIZE> & operator /= (Vector<T, SIZE> & a, T c)
        {
            if (SIZE == 2)
            {
                a(0) /= c; a(1) /= c;  return a;
            }
            else if (SIZE == 3)
            {
                a(0) /= c; a(1) /= c; a(2) /= c; return a;
            }
            else if (SIZE == 4)
            {
                a(0) /= c; a(1) /= c; a(2) /= c; a(3) /= c; return a;
            }
            else
            {
                for (Index i = 0; i < SIZE; i++) a(i) /= c; return a;
            }
        }

        #pragma endregion 

        #pragma region Normalization
        CommonClassesINLINE T squaredNorm() const
        {
            const Vector<T, SIZE>& v = *this;
            return v.dot(v);
        }
        CommonClassesINLINE T norm() const
        {
            return static_cast<T>(std::sqrt(squaredNorm()));
        }
        CommonClassesINLINE T normPow2() const
        {
            return squaredNorm();
        }
        CommonClassesINLINE void normalize()
        {
            *this /= norm();
        }
        CommonClassesINLINE Vector<T, SIZE> normalized() const
        {
            return *this / norm();
        }
        #pragma endregion  

        #pragma region Cross, Dot
        CommonClassesINLINE T dot(const Vector<T, SIZE>& v) const
        {
            if (SIZE == 2)
            {
                return m_data[0] * v[0] + m_data[1] * v[1];
            }
            else if (SIZE == 3)
            {
                return m_data[0] * v[0] + m_data[1] * v[1] + m_data[2] * v[2];
            }
            else if (SIZE == 4)
            {
                return m_data[0] * v[0] + m_data[1] * v[1] + m_data[2] * v[2] + m_data[3] * v[3];
            }
            else
            {
                return (*this*v).sum<T>();
            }
        }
        template <Index S = SIZE, std::enable_if_t<(S == 4)>* = 0>              // Method available only for Vector4        (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        CommonClassesINLINE Vector<T, SIZE> cross3(const Vector<T, SIZE>& v) const
        {
            T r0 = m_data[1] * v.m_data[2] - m_data[2] * v.m_data[1];
            T r1 = m_data[2] * v.m_data[0] - m_data[0] * v.m_data[2];
            T r2 = m_data[0] * v.m_data[1] - m_data[1] * v.m_data[0];
            return Vector<T, SIZE>(r0, r1, r2, 0);
        }
        template <Index S = SIZE, std::enable_if_t<(S == 3)>* = 0>              // Method available only for Vector3        (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        CommonClassesINLINE Vector<T, SIZE> cross(const Vector<T, SIZE>& v) const
        {
            T r0 = m_data[1] * v.m_data[2] - m_data[2] * v.m_data[1];
            T r1 = m_data[2] * v.m_data[0] - m_data[0] * v.m_data[2];
            T r2 = m_data[0] * v.m_data[1] - m_data[1] * v.m_data[0];
            return Vector<T, SIZE>(r0, r1, r2);
        }
        #pragma endregion 


        #pragma region Transformations
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Vector<T, SIZE> transpose() const
        {
            return *this;
        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Vector<T, SIZE>& eval()
        {
            return *this;
        }
         #pragma endregion 

        #pragma region Convertions
        template <class TCast>
        Vector<TCast, SIZE> cast()
        {
            Vector<TCast, SIZE>  v;
            for (Index i = 0; i < SIZE; i++) v[i] = static_cast<TCast>(m_data[i]);
            return v;
        }
        #include "_IMemory_toString"
        friend std::ostream& operator<<(std::ostream& os, const Vector<T, SIZE>& v)
        {
            os << v.toString();
            return os;
        }
        #pragma endregion 

        #pragma region Static Methods
        CommonClassesINLINE static Vector<T, SIZE> Zero()
        {
            Vector<T, SIZE> v;
            v.setZero();
            return v;
        }
        CommonClassesINLINE static Vector<T, SIZE> Constant(const T c)
        {
            Vector<T, SIZE> v;
            v.setConstant(c);
            return v;
        }
        CommonClassesINLINE static Vector<T, SIZE> Random()
        {
            Vector<T, SIZE> v;
            v.setRandom();
            return v;
        }
        #pragma endregion 

        #include "_IMemory_SingleValue_MathAgregators"
    };
}




#ifdef COMMONCLASSES_DEFINE_SHORTCUTS
namespace CommonClasses
{

    #pragma region Define   Vector2, Vector3, Vector4, Point2, Point3, Point4, Color2, Color3, Color4

    #define CommonClasses_MAKE_TYPEDEFS_FIXED2(Type, TypeSuffix)   \
typedef Vector<Type, 2>    Vector2##TypeSuffix;  \
typedef Vector<Type, 2>    Point2##TypeSuffix;  \
typedef Vector<Type, 2>    Color2##TypeSuffix;  \
typedef Vector<Type, 2>    RowVector2##TypeSuffix;  

    #define CommonClasses_MAKE_TYPEDEFS_FIXED3(Type, TypeSuffix)   \
typedef Vector<Type, 3>    Vector3##TypeSuffix;  \
typedef Vector<Type, 3>    Point3##TypeSuffix;  \
typedef Vector<Type, 3>    Color3##TypeSuffix;  \
typedef Vector<Type, 3>    RowVector3##TypeSuffix;  

    #define CommonClasses_MAKE_TYPEDEFS_FIXED4(Type, TypeSuffix)   \
typedef Vector<Type, 4>    Vector4##TypeSuffix;  \
typedef Vector<Type, 4>    Point4##TypeSuffix;  \
typedef Vector<Type, 4>    Color4##TypeSuffix;  \
typedef Vector<Type, 4>    RowVector4##TypeSuffix;  

    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_FIXED2(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_FIXED3(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_FIXED4(Type, TypeSuffix)

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_MAKE_TYPEDEFS_FIXED2
        #undef CommonClasses_MAKE_TYPEDEFS_FIXED3
        #undef CommonClasses_MAKE_TYPEDEFS_FIXED4
        #undef CommonClasses_DEF_ALL_SIZES

        #pragma endregion 
}
#endif
