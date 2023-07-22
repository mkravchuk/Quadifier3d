#pragma once
#include "CommonClassesShared.h"
#include "Vector__Fixed.h"
#include "Vector__Dynamic.h"


namespace CommonClasses
{
    template <class T>
    class MemoryLink
    {
    protected:
        T * m_data;
        Index m_size;

    public:
        #pragma region Constructors
        // creates link to memory - so we can read from memory or save to memory
        MemoryLink(T* data, Index size)
            : m_data(data), m_size(size)
        {
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
            return m_size;
        }
        constexpr Index sizeofT() const
        {
            return sizeof(T);
        }
        #pragma endregion 

        #include "_IMemory_SingleValue_ReadWrite"
        #include "_IMemory_SingleValue_FillData"

        #pragma region Convert to
        template <Index SIZE>
        operator Vector<T, SIZE>() const
        {
            #if DEBUG
            assert(m_size == SIZE && "source MemoryLink size must be equal to Vector size");
            #endif
            return Vector<T, SIZE>(m_data);
        }
        operator Vector<T>() const
        {
            return Vector<T>(m_data, m_size);
        }
        #pragma endregion 

        #pragma region Convert from
        template <Index SIZE>
        CommonClassesINLINE MemoryLink<T>& operator=(const Vector<T, SIZE>& v)
        {
            #if DEBUG
            Index vector_size = SIZE;
            Index memoryLink_size = m_size;
            assert(vector_size == memoryLink_size && "source Vector size must be equal to MemoryLink size");
            #endif
            for (Index i = 0; i < SIZE; i++)
            {
                m_data[i] = v(i);
            }
            return *this;
        }
        CommonClassesINLINE MemoryLink<T>& operator=(const Vector<T>& v)
        {
            #if DEBUG
            Index vector_size = v.size();
            Index memoryLink_size = m_size;
            assert(vector_size == memoryLink_size && "source Vector size must be equal to MemoryLink size");
            #endif
            for (Index i = 0; i < v.size(); i++)
            {
                m_data[i] = v(i);
            }
            return *this;
        }
        CommonClassesINLINE MemoryLink<T>& operator=(const MemoryLink<T>& other)
        {
            Index source_size = other.size();
            #if DEBUG
            Index dest_size = m_size;
            assert(source_size == dest_size && "source MemoryLink size must be equal to MemoryLink size");
            #endif
            if (source_size == 3)
            {
                m_data[0] = other.m_data[0];
                m_data[1] = other.m_data[1];
                m_data[2] = other.m_data[2];
            }
            else if (source_size == 4)
            {
                m_data[0] = other.m_data[0];
                m_data[1] = other.m_data[1];
                m_data[2] = other.m_data[2];
                m_data[3] = other.m_data[3];
            }
            else if (source_size == 2)
            {
                m_data[0] = other.m_data[0];
                m_data[1] = other.m_data[1];
            }
            else
            {
                for (Index i = 0; i < source_size; i++)
                {
                    m_data[i] = other.m_data[i];
                }
            }
            return *this;
        }
        #pragma endregion 
    };
};