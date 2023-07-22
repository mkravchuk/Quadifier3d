#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"
#include "Table__FixedFixed.h" // must be included, because this class is specialization of Table__ROWS_COLS

namespace CommonClasses
{
    class Table__ROWS_Dynamic_Info
    {
    public: 
        Index Cols;
        Table__ROWS_Dynamic_Info()
            : Cols(0)
        {
        }
    };

    // Specialization - dynamic rows and fixed cols (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T, Index ROWS>
    class Table<T, ROWS, X> : public MemoryDynamic<T, Table__ROWS_Dynamic_Info>
    {
    public:
        #pragma region Constructors
        Table() = default;
        Table(Index rows, Index cols)
        {
            #if DEBUG
            assert(ROWS == rows && "Constructor() - ROWS at class definition differs from new rows count");
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

        // NOT IMPLEMENTED - we dont support such type of Table dimension
    };
}