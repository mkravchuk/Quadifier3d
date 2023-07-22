#pragma once
#include "CommonClassesShared.h"
#include "IMemory.h"

namespace CommonClasses
{
    template <class T, class TInfo>
    class MemoryDynamic
    {
    protected:
        Index m_size;
        T * m_data;
        TInfo m_info;

        #pragma region  Dimension control
        __declspec(noinline)  void clearMemory_internal()
        {
            if (m_data != nullptr)
            {
                delete m_data;
            }
            m_size = 0;
            m_data = nullptr;
        }
        CommonClassesINLINE void resize_internal(Index size)
        {
            if (m_size == size) return;
            clearMemory_internal();
            m_size = size;
            //Index size_SSE = (size * sizeof(T) + 128 / 8) / sizeof(T); //reserve space for SSE commands, to be sure we will always access proper memory
            //m_data = new T[size_SSE];
            m_data = new T[m_size];
            //std::cout << "MemoryDynamic::resize_internal()  m_data=" << m_data << std::endl;
        }
        void conservativeResize_internal(Index sizeUpdated, bool allowInPlace = false)
        {
            if (sizeUpdated == m_size) return;
            if (allowInPlace && sizeUpdated != 0 && m_size - 10 <= sizeUpdated && sizeUpdated <= m_size)
            {
                m_size = sizeUpdated;
                return;
            }
            T* m_data_new = new T[sizeUpdated];
            if (m_data != nullptr)
            {
                Index min_size = (m_size < sizeUpdated) ? m_size : sizeUpdated;
                memmove(m_data_new, m_data, min_size * sizeof(T));
            }
            clearMemory_internal();
            m_size = sizeUpdated;
            m_data = m_data_new;
        }
        #pragma endregion 

    public:
        #pragma region Constructors
        MemoryDynamic()
            : m_size(0), m_data(nullptr), m_info(TInfo())
        {
        }
        MemoryDynamic(const T* data, Index size)
            : MemoryDynamic(data, size, TInfo())
        {

        }
        MemoryDynamic(const T* data, Index size, TInfo info)
            : m_size(size), m_data(nullptr)
        {
            resize_internal(size);
            memcpy(m_data, data, size * sizeof(T));
            m_info = info;
        }
        ~MemoryDynamic() // destructor (Rule of five)
        {
            //std::cout << "MemoryDynamic::~MemoryDynamic()  m_data=" << m_data << std::endl;
            clearMemory_internal();
        }
        MemoryDynamic(const MemoryDynamic& o)  // copy constructor (Rule of five)
            : m_size(0), m_data(nullptr)
        {
            resize_internal(o.size());
            //memcpy(m_data, o.m_data, m_size * sizeof(T)); // shallow copy
            for (int i = 0; i < m_size; i++)
            {
                m_data[i] = o.m_data[i]; // deep copy
            }
            m_info = o.m_info;
        }
        MemoryDynamic(MemoryDynamic&& o) noexcept // move constructor (Rule of five)
            : m_size(std::exchange(o.m_size, Index())),
            m_data(std::exchange(o.m_data, nullptr)),
            m_info(std::exchange(o.m_info, TInfo()))
        {
        }
        inline MemoryDynamic& operator=(const MemoryDynamic& other) // copy assignment (Rule of five)
        {
            return *this = MemoryDynamic(other);
        }
        inline MemoryDynamic& operator=(MemoryDynamic&& other) noexcept // move assignment (Rule of five)
        {
            std::swap(m_size, other.m_size);
            std::swap(m_data, other.m_data);
            std::swap(m_info, other.m_info);
            return *this;
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

        long long SizeOF() const
        {
            return sizeof(*this) + m_size * sizeofT();
        }
    };
}