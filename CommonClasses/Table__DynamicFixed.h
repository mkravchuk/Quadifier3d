#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"
#include "Table__FixedFixed.h" // must be included, because this class is specialization of Table__ROWS_COLS
#include "Vector__Fixed.h"

namespace CommonClasses
{
    class Table__Dynamic_COLS_Info
    {
    public:
        Index Rows;
        Table__Dynamic_COLS_Info()
            : Rows(0)
        {
        }
    };

    // Specialization - dynamic rows and fixed cols (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T, Index COLS>
    class Table<T, X, COLS> : public MemoryDynamic<T, Table__Dynamic_COLS_Info>
    {
    protected:
        #include "_IMemory_toString_rows_cols"
    public:
        #pragma region Constructors
        Table()
            : MemoryDynamic()
        {
        }
        Table(Index rows, Index cols)
        {
            #if DEBUG
            assert(COLS == cols && "Constructor() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, cols);
        }
        constexpr  T* data() 
        {
            return m_data;
        }
        constexpr  const T* data() const 
        {
            return m_data;
        }
        constexpr  Index size() const 
        {
            return m_size;
        }
        #pragma endregion    

        #pragma region Dimension control
        CommonClassesINLINE Index rows() const
        {
            return m_info.Rows;
        }
        constexpr Index cols() const
        {
            return COLS;
        }
        void resize(Index rows, Index cols)
        {
            #if DEBUG
            assert(COLS == cols && "resize() - COLS at class definition differs from new cols count");
            #endif
            m_info.Rows = rows;
            resize_internal(rows*COLS);
        }
        void conservativeResize(Index rows, Index cols)
        {
            #if DEBUG
            assert(COLS == cols && "conservativeResize() - COLS at class definition differs from new cols count");
            #endif
            m_info.Rows = rows;
            conservativeResize_internal(rows*COLS);
        }
        #pragma endregion 

        #pragma region  Fill data
        using MemoryDynamic<T, Table__Dynamic_COLS_Info>::setConstant;
        using MemoryDynamic<T, Table__Dynamic_COLS_Info>::setZero;
        #include "_IMemory_SingleValue_MathFillData"
        void setZero(Index rows, Index cols)
        {
            #if DEBUG
            assert(COLS == cols && "setZero() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, COLS);
            setZero();
        }
        void setConstant(Index rows, Index cols, T value)
        {
            #if DEBUG
            assert(COLS == cols && "setConstant() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, COLS);
            setConstant(value);
        }
        void setRandom(Index rows, Index cols)
        {
            #if DEBUG
            assert(COLS == cols && "setRandom() - COLS at class definition differs from new cols count");
            #endif
            resize(rows, COLS);
            setRandom();
        }
        #pragma endregion 

        #pragma region Read/Write
        CommonClassesINLINE T&  operator()(Index row, Index col)
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row index is out of range");
            assert(col >= 0 && col < COLS && "col index is out of range");
            #endif
            return m_data[COLS*row + col];
        }
        CommonClassesINLINE T  operator()(Index row, Index col) const
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row index is out of range");
            assert(col >= 0 && col < COLS && "col index is out of range");
            #endif
            return m_data[COLS*row + col];
        }

        CommonClassesINLINE Vector<T, COLS>& row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row is out of range");
            #endif
            Vector<T, COLS>& res = *(Vector<T, COLS>*)(&m_data[COLS*row]);
            return res; // dont convert to static cast - will not compile
        }
        CommonClassesINLINE Vector<T, COLS> row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row is out of range");
            #endif
            return Vector<T, COLS>(m_data + COLS * row);
        }
        #pragma endregion 


        #pragma region Transformations
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Table<T, X, COLS>& eval()
        {
            return *this;
        }
        #pragma endregion  

        #pragma region Convertions
        template <class TCast>
        Table<TCast, X, COLS> cast()
        {
            Table<TCast, X, COLS>  m;
            for (Index i = 0; i < size(); i++) m[i] = static_cast<TCast>(m_data[i]);
            return m;
        }
        std::string toString() const
        {
            return toString_rows_cols(rows(), COLS);
        }
        friend std::ostream& operator<<(std::ostream& os, const Table <T, X, COLS>& m)
        {
            os << m.toString();
            return os;
        }
        #pragma endregion  

        #pragma region Static Methods
        static Table<T, X, COLS> Zero(Index rows, Index cols = COLS)
        {
            #if DEBUG
            assert(COLS == cols && "Zero() - COLS at class definition differs from new cols count");
            #endif
            Table<T, X, COLS>  m(rows, COLS);
            m.setZero();
            return m;
        }
        static Table<T, X, COLS> Constant(Index rows, Index cols, T value)
        {
            #if DEBUG
            assert(COLS == cols && "Constant() - COLS at class definition differs from new cols count");
            #endif
            Table<T, X, COLS>  m(rows, COLS);
            m.setConstant(value);
            return m;
        }
        static Table<T, X, COLS> Random(Index rows, Index cols = COLS)
        {
            #if DEBUG
            assert(COLS == cols && "Random() - COLS at class definition differs from new cols count");
            #endif
            Table<T, X, COLS>  m(rows, COLS);
            m.setRandom();
            return m;
        }
        #pragma endregion 

        #include "_IMemory_SingleValue_MathAgregators"
    };
}



#ifdef COMMONCLASSES_DEFINE_SHORTCUTS
namespace CommonClasses
{
    #pragma region Define   MatrixX2, MatrixX3, MatrixX4

    #define CommonClasses_MAKE_TYPEDEFS_DYNAMIC_FIXED(Type, TypeSuffix, Size)         \
typedef Table<Type, X, Size> Table##X##Size##TypeSuffix;  

    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_DYNAMIC_FIXED(Type, TypeSuffix, 2) \
CommonClasses_MAKE_TYPEDEFS_DYNAMIC_FIXED(Type, TypeSuffix, 3) \
CommonClasses_MAKE_TYPEDEFS_DYNAMIC_FIXED(Type, TypeSuffix, 4) 

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_MAKE_TYPEDEFS_DYNAMIC_FIXED
        #undef CommonClasses_DEF_ALL_SIZES

        #pragma endregion 
}
#endif
