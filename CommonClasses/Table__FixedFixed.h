#pragma once
#include "CommonClassesShared.h"
#include "MemoryFixed.h"
#include "Vector__Fixed.h"


namespace CommonClasses
{
    // Specialization - fixed rows and fixed cols (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T, Index ROWS, Index COLS>
    class Table : public MemoryFixed<T, ROWS*COLS>
    {
    protected:
        #include "_IMemory_toString_rows_cols"
    public:
        #pragma region Constructors
        Table()
            : MemoryFixed()
        {
        }
        Table(const T* data)
            : MemoryFixed(data)
        {
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C && R*C == 4)>* = 0>              // Method available only for 2x2 Matrixes     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table(T a00, T a01, T a10, T a11)
        {
            m_data[0] = a00; m_data[1] = a01;
            m_data[2] = a10; m_data[3] = a11;
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C && R*C == 9)>* = 0>              // Method available only for 3x3 Matrixes     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22)
        {
            m_data[0] = a00; m_data[1] = a01; m_data[2] = a02;
            m_data[3] = a10; m_data[4] = a11; m_data[5] = a12;
            m_data[6] = a20; m_data[7] = a21; m_data[8] = a22;
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C && R*C == 16)>* = 0>              // Method available only for 4x4 Matrixes   (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table(T a00, T a01, T a02, T a03, T a10, T a11, T a12, T a13, T a20, T a21, T a22, T a23, T a30, T a31, T a32, T a33)
        {
            m_data[0] = a00;   m_data[1] = a01;   m_data[2] = a02;   m_data[3] = a03;
            m_data[4] = a10;   m_data[5] = a11;   m_data[6] = a12;   m_data[7] = a13;
            m_data[8] = a20;   m_data[9] = a21;   m_data[10] = a22;  m_data[11] = a23;
            m_data[12] = a30;  m_data[13] = a31; m_data[14] = a32;  m_data[15] = a33;
        }
        #pragma endregion 

        #pragma region Dimension control
        constexpr Index rows() const
        {
            return ROWS;
        }
        constexpr Index cols() const
        {
            return COLS;
        }
        #pragma endregion 

        #pragma region  Fill data
        using MemoryFixed<T, ROWS*COLS>::setConstant;
        #include "_IMemory_SingleValue_MathFillData"
        #pragma endregion 

        #pragma region Read/Write
        template <Index C = COLS, std::enable_if_t<(C != 1)>* = 0>              // Method available only for Matrixes             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        CommonClassesINLINE T&  operator()(Index row, Index col)
        {
            #if DEBUG
            assert(row >= 0 && row < ROWS && "row index is out of range");
            assert(col >= 0 && col < COLS && "col index is out of range");
            #endif
            return m_data[COLS*row + col];
        }
        template <Index C = COLS, std::enable_if_t<(C != 1)>* = 0>              // Method available only for Matrixes             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        CommonClassesINLINE T  operator()(Index row, Index col) const
        {
            #if DEBUG
            assert(row >= 0 && row < ROWS && "row index is out of range");
            assert(col >= 0 && col < COLS && "col index is out of range");
            #endif
            return m_data[COLS*row + col];
        }
        template <Index C = COLS, std::enable_if_t<(C != 1)>* = 0>              // Method available only for Matrixes             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        CommonClassesINLINE Vector<T, ROWS> col(Index col) const
        {
            #if DEBUG
            assert(col >= 0 && col < COLS && "row is out of range");
            #endif
            Vector<T, ROWS> m;
            for (Index row = 0; row < ROWS; row++)
            {
                m[row] = m_data[COLS * row + col];
            }
            return m;
        }
        CommonClassesINLINE Vector<T, COLS>& row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < ROWS && "row is out of range");
            #endif
            return *(Vector<T, COLS>*)(&m_data[COLS*row]); // dont convert to static cast - will not compile
        }
        CommonClassesINLINE Vector<T, COLS> row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < ROWS && "row is out of range");
            #endif
            return Vector<T, COLS>(m_data + COLS * row);
        }
        #pragma endregion 

        #pragma region Table Operators

        // operator +    add
        friend CommonClassesINLINE  Table<T, ROWS, COLS> operator + (Table<T, ROWS, COLS> const & m, Table<T, ROWS, COLS> const & m2)
        {
            const T* x = m.data();
            const T* y = m2.data();
            Table<T, ROWS, COLS> r;
            for (Index i = 0; i < ROWS*COLS; i++) r[i] = x[i] + y[i];
            return r;
        }

        // operator -    minus
        friend CommonClassesINLINE  Table<T, ROWS, COLS> operator - (Table<T, ROWS, COLS> const & m, Table<T, ROWS, COLS> const & m2)
        {
            const T* x = m.data();
            const T* y = m2.data();
            Table<T, ROWS, COLS> r;
            for (Index i = 0; i < ROWS*COLS; i++) r[i] = x[i] - y[i];
            return r;
        }

        // operator -    unary minus
        friend CommonClassesINLINE  Table<T, ROWS, COLS> operator - (Table<T, ROWS, COLS> const & m)
        {
            const T* x = m.data();
            Table<T, ROWS, COLS> r;
            for (Index i = 0; i < ROWS*COLS; i++) r[i] = -x[i];
            return r;
        }

        // operator ==   returns true if m == m2
        friend CommonClassesINLINE  bool operator == (Table<T, ROWS, COLS> const & m, Table<T, ROWS, COLS> const & m2)
        {
            const T* x = m.data();
            const T* y = m2.data();
            if (ROWS*COLS == 2)
            {
                return x[0] == y[0] && x[1] == y[1];
            }
            else if (ROWS*COLS == 3)
            {
                return x[0] == y[0] && x[1] == y[1] && x[2] == y[2];
            }
            else if (ROWS*COLS == 4)
            {
                return x[0] == y[0] && x[1] == y[1] && x[2] == y[2] && x[3] == y[3];
            }
            else
            {
                for (Index i = 0; i < ROWS*COLS; i++) if (x[i] != y[i]) return false;
                return true;
            }
        }

        // operator !=   returns true if m != m2
        friend CommonClassesINLINE  bool operator != (Table<T, ROWS, COLS> const & m, Table<T, ROWS, COLS> const & m2)
        {
            return !(m == m2);
        }

        #pragma endregion 

        #pragma region Vector Operators

        // operator *    multiply by vector
        friend CommonClassesINLINE Vector<T, ROWS> operator * (Table<T, ROWS, COLS> const & m, Vector<T, COLS> const & v)
        {
            if (ROWS == COLS && ROWS == 2)
            {
                const T* a = m.data();
                T v0 = v(0);
                T v1 = v(1);
                Vector<T, ROWS> res;
                res[0] = a[0] * v0 + a[1] * v1;
                res[1] = a[2] * v0 + a[3] * v1;
                return res;
            }
            else if (ROWS == COLS && ROWS == 3)
            {
                const T* a = m.data();
                T v0 = v(0);
                T v1 = v(1);
                T v2 = v(2);
                Vector<T, ROWS> res;
                res[0] = a[0] * v0 + a[1] * v1 + a[2] * v2;
                res[1] = a[3] * v0 + a[4] * v1 + a[5] * v2;
                res[2] = a[6] * v0 + a[7] * v1 + a[8] * v2;
                return res;
            }
            else if (ROWS == COLS && ROWS == 4)
            {
                const T* a = m.data();
                T v0 = v(0);
                T v1 = v(1);
                T v2 = v(2);
                T v3 = v(3);
                Vector<T, ROWS> res;
                res[0] = a[0] * v0 + a[1] * v1 + a[2] * v2 + a[3] * v3;
                res[1] = a[4] * v0 + a[5] * v1 + a[6] * v2 + a[7] * v3;
                res[2] = a[8] * v0 + a[9] * v1 + a[10] * v2 + a[11] * v3;
                res[3] = a[12] * v0 + a[13] * v1 + a[14] * v2 + a[15] * v3;
                return res;
            }
            else
            {
                Vector<T, ROWS> mult_m_v;
                for (Index ri = 0; ri < ROWS; ri++)
                {
                    T sum = 0;
                    for (Index ci = 0; ci < COLS; ci++)
                    {
                        sum += m(ri, ci)* v(ci);
                    }
                    mult_m_v[ri] = sum;
                }
                return mult_m_v;
            }


        }

        #pragma endregion 

        #pragma region Scalar Operators

        // operator *    multiply by scalar
        friend CommonClassesINLINE  Table<T, ROWS, COLS> operator * (Table<T, ROWS, COLS> const & m, const T c)
        {
            const T* x = m.data();
            Table<T, ROWS, COLS> r;
            for (Index i = 0; i < ROWS*COLS; i++) r[i] = x[i] * c;
            return r;
        }

        // operator *    multiply
        friend CommonClassesINLINE  Table<T, ROWS, COLS> operator * (T c, Table<T, ROWS, COLS> const & m)
        {
            return m * c;
        }

        // operator /     divide by scalar
        friend CommonClassesINLINE  Table<T, ROWS, COLS> operator / (Table<T, ROWS, COLS> const & m, const T c)
        {
            const T* x = m.data();
            Table<T, ROWS, COLS> r;
            for (Index i = 0; i < ROWS*COLS; i++) r[i] = x[i] / c;
            return r;
        }

        // operator *=   multiply
        friend CommonClassesINLINE  Table<T, ROWS, COLS> & operator *= (Table<T, ROWS, COLS> & m, T c)
        {
            T* x = m.data();
            for (Index i = 0; i < ROWS*COLS; i++) x[i] *= c;
            return m;
        }

        // operator *=   multiply
        friend CommonClassesINLINE  Table<T, ROWS, COLS> & operator /= (Table<T, ROWS, COLS> & m, T c)
        {
            T* x = m.data();
            for (Index i = 0; i < ROWS*COLS; i++) x[i] /= c;
            return m;
        }

        #pragma endregion 


        #pragma region Transformations
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R != C || (R != 2 && R != 3 && R != 4))>* = 0>              // Method available only for Table             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table<T, COLS, ROWS> transpose() const
        {
            Table<T, COLS, ROWS> m;
            for (Index row = 0; row < ROWS; row++)
            {
                for (Index col = 0; col < COLS; col++)
                {
                    m.m_data[ROWS*col + row] = m_data[COLS*row + col];
                }
            }
            return m;
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C && R == 2)>* = 0>              // Method available only for Matrix2x2             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table<T, COLS, ROWS> transpose() const
        {
            return Table<T, COLS, ROWS>(m_data[0], m_data[2],
                m_data[1], m_data[3]);
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C && R == 3)>* = 0>              // Method available only for Matrix3x3             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table<T, COLS, ROWS> transpose() const
        {

            return Table<T, COLS, ROWS>(m_data[0], m_data[3], m_data[6],
                m_data[1], m_data[4], m_data[7],
                m_data[2], m_data[5], m_data[8]);
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C && R == 4)>* = 0>              // Method available only for Matrix4x4             (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        Table<T, COLS, ROWS> transpose() const
        {
            return Table<T, COLS, ROWS>(m_data[0], m_data[4], m_data[8], m_data[12],
                m_data[1], m_data[5], m_data[9], m_data[13],
                m_data[2], m_data[6], m_data[10], m_data[14],
                m_data[3], m_data[7], m_data[11], m_data[15]);

        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Table<T, ROWS, COLS>& eval()
        {
            return *this;
        }
        #pragma endregion  

        #pragma region Convertions
        template <class TCast>
        Table<TCast, ROWS, COLS> cast()
        {
            Table<TCast, ROWS, COLS>  m;
            for (Index i = 0; i < ROWS*COLS; i++) m[i] = static_cast<TCast>(m_data[i]);
            return m;
        }
        std::string toString() const
        {
            return toString_rows_cols(ROWS, COLS);
        }
        friend std::ostream& operator<<(std::ostream& os, const Table <T, ROWS, COLS>& m)
        {
            os << m.toString();
            return os;
        }
        #pragma endregion  

        #pragma region Static Methods
        CommonClassesINLINE static Table<T, ROWS, COLS> Zero()
        {
            Table<T, ROWS, COLS>  m;
            m.setZero();
            return m;
        }
        CommonClassesINLINE static Table<T, ROWS, COLS> Constant(T value)
        {
            Table<T, ROWS, COLS>  m;
            m.setConstant(value);
            return m;
        }
        CommonClassesINLINE static Table<T, ROWS, COLS> Random()
        {
            Table<T, ROWS, COLS>  m;
            m.setRandom();
            return m;
        }
        template <Index R = ROWS, Index C = COLS, std::enable_if_t<(R == C)>* = 0>              // Method available only for Matrixes     where ROWS=COLS        (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        static Table<T, ROWS, COLS> Identity()
        {
            if (ROWS == 2)
            {
                T a[2 * 2] = { 1, 0,    0, 1 };
                return Table<T, ROWS, COLS>(a);
            }
            else if (ROWS == 3)
            {
                T a[3 * 3] = { 1, 0, 0,    0, 1, 0,    0, 0, 1 };
                return Table<T, ROWS, COLS>(a);
            }
            else if (ROWS == 4)
            {
                T a[4 * 4] = { 1, 0, 0, 0,    0, 1, 0, 0,    0, 0, 1, 0,    0, 0, 0, 1 };
                return Table<T, ROWS, COLS>(a);
            }
            else
            {
                Table<T, ROWS, COLS> m;
                m.setZero();
                for (Index i = 0; i < ROWS; i++)
                {
                    m(i, i) = 1;
                }
                return m;
            }
        }
        #pragma endregion 

        #include "_IMemory_SingleValue_MathAgregators"
    };
}




#ifdef COMMONCLASSES_DEFINE_SHORTCUTS
namespace CommonClasses
{
    #pragma region Define   Matrix2, Matrix3, Matrix4

    #define CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, Size)         \
typedef Table<Type, Size, Size> Table##Size##TypeSuffix;  

    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, 2) \
CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, 3) \
CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, 4) 

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED
        #undef CommonClasses_DEF_ALL_SIZES

        #pragma endregion 
}
#endif
