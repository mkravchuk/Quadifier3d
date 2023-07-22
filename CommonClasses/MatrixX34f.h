#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"



namespace CommonClasses
{
    class MatrixX34__Info
    {
    public:
        Index Rows;
        MatrixX34__Info()
            : Rows(0)
        {
        }
    };

    // Specialization - dynamic rows and fixed cols (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class TRow>
    class MatrixX34f : public MemoryDynamic<float, MatrixX34__Info>
    {
    protected:
        #include "_IMemory_toString_rows_cols"

    public:
        #pragma region Constructors
        MatrixX34f()
            : MemoryDynamic()
        {
        }
        MatrixX34f(Index rows, Index cols)
        {
            #if DEBUG
            assert(cols == 3 && "Constructor() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, cols);
        }
        constexpr  Index size() const
        {
            return m_size;
        }
        constexpr  TRow* data()
        { 
            return (TRow*)m_data;
        }
        constexpr  const TRow* data() const
        {
            return (const TRow*)m_data;
        }
        #pragma endregion    

        #pragma region Dimension control
        CommonClassesINLINE Index rows() const
        {
            return m_info.Rows;
        }
        constexpr Index cols() const
        {
            return 3;
        }
        void resize(Index rows, Index cols)
        {
            #if DEBUG
            assert(cols == 3 && "resize() - COLS at class definition differs from new cols count");
            #endif
            m_info.Rows = rows;
            resize_internal(rows*4);  // each row will have 4 floats
        }
        void conservativeResize(Index rows, Index cols)
        {
            #if DEBUG
            assert(cols == 3 && "conservativeResize() - COLS at class definition differs from new cols count");
            #endif
            m_info.Rows = rows;
            conservativeResize_internal(rows*4); // each row will have 4 floats
        }
        #pragma endregion 

        #pragma region  Fill data
        void setConstant(TRow value)
        {
            TRow c(value);
            for (Index i = 0; i < rows(); i++)
            {
                row(i) = c;
            }
        }
        void setZero()
        {
            TRow c(0.f);
            for (Index i = 0; i < rows(); i++)
            {
                row(i) = c;
            }
        }
        void setRandom()
        {
            for (Index i = 0; i < rows(); i++)
            {                
                row(i) = TRow(static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand()));
            }
        }
        void setZero(Index rows, Index cols)
        {
            #if DEBUG
            assert(cols == 3 && "setZero() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, cols);
            setZero();
        }
        void setConstant(Index rows, Index cols, float value)
        {
            #if DEBUG
            assert(cols == 3 && "setConstant() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, cols);
            setConstant(value);
        }
        void setRandom(Index rows, Index cols)
        {
            #if DEBUG
            assert(cols == 3 && "setRandom() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, 3);
            setRandom();
        }
        #pragma endregion 

        #pragma region Read/Write
        CommonClassesINLINE float&  operator()(Index row, Index col)
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row index is out of range");
            assert(col >= 0 && col < 4 && "col index is out of range");
            #endif
            return m_data[row*4 + col];
        }
        CommonClassesINLINE float  operator()(Index row, Index col) const
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row index is out of range");
            assert(col >= 0 && col < 4 && "col index is out of range");
            #endif
            return m_data[row*4 + col];
        }

        CommonClassesINLINE TRow& row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row is out of range");
            #endif
            TRow& res = *(TRow*)(&m_data[row*4]);
            return res; // dont convert to static cast - will not compile
        }
        CommonClassesINLINE TRow row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row is out of range");
            #endif
            TRow res = *(TRow*)(&m_data[row * 4]);
            return res; // dont convert to static cast - will not compile
        }
        #pragma endregion 

        #pragma region Matrix Operators

        // operator -    unary minus
        friend CommonClassesINLINE  MatrixX34f<TRow> operator - (MatrixX34f<TRow> const & m)
        {
            MatrixX34f<TRow> r(m.rows(), 3);
            for (Index i = 0; i < m.rows(); i++)
            {
                TRow row = m.row(i);
                r.row(i) = -row;
            }
            return r;
        }

        #pragma endregion 

        #pragma region Transformations
        // does nothing. added just for Eigen portability
        CommonClassesINLINE MatrixX34f<TRow>& eval()
        {
            return *this;
        }
        #pragma endregion  

        #pragma region Convertions
        std::string toString() const
        {
            return toString_rows_cols(rows(), 3);
        }
        friend std::ostream& operator<<(std::ostream& os, const MatrixX34f<TRow>& m)
        {
            os << m.toString();
            return os;
        }
        #pragma endregion  

        #pragma region Static Methods
        static MatrixX34f<TRow> Zero(Index rows, Index cols = 3)
        {
            #if DEBUG
            assert(cols == 3 && "Zero() - COLS at class definition differs from new cols count");
            #endif
            MatrixX34f<TRow>  m(rows, 3);
            m.setZero();
            return m;
        }
        static MatrixX34f<TRow> Constant(Index rows, Index cols, TRow value)
        {
            #if DEBUG
            assert(cols == 3 && "Constant() - COLS at class definition differs from new cols count");
            #endif
            MatrixX34f<TRow>  m(rows, 3);
            m.setConstant(value);
            return m;
        }
        static MatrixX34f<TRow> Random(Index rows, Index cols = 3)
        {
            #if DEBUG
            assert(cols == 3 && "Random() - COLS at class definition differs from new cols count");
            #endif
            MatrixX34f<TRow>  m(rows, 3);
            m.setRandom();
            return m;
        }
        #pragma endregion 

        //#include "_IMemory_SingleValue_MathAgregators"
    };
}