#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"
#include "Table__FixedFixed.h" // must be included, because this class is specialization of Table__ROWS_COLS
#include "MemoryLink.h"

namespace CommonClasses
{
    class Table__Dynamic_Dynamic_Info
    {
    public:
        Index Rows;
        Index Cols;
        Table__Dynamic_Dynamic_Info()
            : Rows(0), Cols(0)
        {
        }
    };
    
    // Specialization - dynamic rows and dynamic cols (no speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T>
    class Table<T, X, X> : public MemoryDynamic<T, Table__Dynamic_Dynamic_Info>
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
        CommonClassesINLINE Index cols() const
        {
            return m_info.Cols;
        }
        void resize(Index rows, Index cols)
        {
            m_info.Rows = rows;
            m_info.Cols = cols;
            resize_internal(rows*cols);
        }
        void conservativeResize(Index rows, Index cols)
        {
            m_info.Rows = rows;
            m_info.Cols = cols;
            conservativeResize_internal(rows*cols);
        }
        #pragma endregion 

        #pragma region  Fill data
        using MemoryDynamic<T, Table__Dynamic_Dynamic_Info>::setConstant;
        using MemoryDynamic<T, Table__Dynamic_Dynamic_Info>::setZero;
        #include "_IMemory_SingleValue_MathFillData"
        void setZero(Index rows, Index cols)
        {
            resize(rows, cols);
            setZero();
        }
        void setConstant(Index rows, Index cols, T value)
        {
            resize(rows, cols);
            setConstant(value);
        }
        void setRandom(Index rows, Index cols)
        {
            resize(rows, cols);
            setRandom();
        }
        #pragma endregion 

        #pragma region Read/Write
        CommonClassesINLINE T& operator()(Index row, Index col)
        {   
            #if DEBUG
            assert(row >= 0 && row < rows() && "row index is out of range");
            assert(col >= 0 && col < cols() && "col index is out of range");
            #endif
            return m_data[m_info.Cols*row + col];
        }
        CommonClassesINLINE T operator()(Index row, Index col) const
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row index is out of range");
            assert(col >= 0 && col < cols() && "col index is out of range");
            #endif
            return m_data[m_info.Cols*row + col];
        }
        CommonClassesINLINE MemoryLink<T> row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row is out of range");
            #endif
            return MemoryLink<T>(&m_data[m_info.Cols*row], m_info.Cols);
        }
        CommonClassesINLINE MemoryLink<T> row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < rows() && "row is out of range");
            #endif
            return MemoryLink<T>(&m_data[m_info.Cols*row], m_info.Cols);
        }
        #pragma endregion 


        #pragma region Transformations
        Table<T, X, X> transpose() const
        {
            Table<T, X, X> m(cols(), rows());
            for (Index row = 0; row < rows(); row++)
            {
                for (Index col = 0; col < cols(); col++)
                {
                    m.m_data[rows()*col + row] = m_data[cols()*row + col];
                }
            }
            return m;
        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Table<T, X, X>& eval()
        {
            return *this;
        }
        #pragma endregion  

        #pragma region Convertions
        template <class TCast>
        Table<TCast, X, X> cast()
        {
            Table<TCast, X, X>  m;
            for (Index i = 0; i < size(); i++) m[i] = static_cast<TCast>(m_data[i]);
            return m;
        }
        std::string toString() const
        {
            return toString_rows_cols(rows(), cols());
        }
        friend std::ostream& operator<<(std::ostream& os, const Table <T, X, X>& m)
        {
            os << m.toString();
            return os;
        }
        #pragma endregion  

        #pragma region Static Methods
        static Table<T, X, X> Zero(Index rows, Index cols)
        {
            Table<T, X, X>  m(rows, cols);
            m.setZero();
            return m;
        }
        static Table<T, X, X> Constant(Index rows, Index cols, T value)
        {
            Table<T, X, X>  m(rows, cols);
            m.setConstant(value);
            return m;
        }
        static Table<T, X, X> Random(Index rows, Index cols)
        {
            Table<T, X, X>  m(rows, cols);
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
    #pragma region Define   MatrixX

    #define CommonClasses_MAKE_TYPEDEFS_DYNAMICX2(Type, TypeSuffix)         \
typedef Table<Type, X, X> Table##X##TypeSuffix;  

    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_DYNAMICX2(Type, TypeSuffix) 

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_MAKE_TYPEDEFS_DYNAMICX2
        #undef CommonClasses_DEF_ALL_SIZES

        #pragma endregion 
}
#endif

