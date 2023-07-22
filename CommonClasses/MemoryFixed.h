#pragma once
#include "CommonClassesShared.h"
#include "IMemory.h"

namespace CommonClasses
{
    template <class T, Index SIZE>
    class MemoryFixed
    {
    protected:
        T m_data[SIZE];
    public:
        #pragma region Constructors
        MemoryFixed() = default;
        MemoryFixed(const T* data)
        {
            for (Index i = 0; i < SIZE; i++) m_data[i] = data[i];
        }

        constexpr T* data() 
        {
            return m_data;
        }
        constexpr const T* data() const 
        {
            return m_data;
        }
        constexpr Index size() const 
        {
            return SIZE;
        }
        constexpr Index sizeofT() const
        {
            return sizeof(T);
        }
        #pragma endregion 

        #include "_IMemory_SingleValue_ReadWrite"
        #include "_IMemory_SingleValue_FillData"
        
    };
}