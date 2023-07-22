#pragma once
#include "CommonClassesShared.h"
#include "List__Dynamic.h"

namespace CommonClasses
{
    // class for storing 'new T' pointers, but access them by reference
    // class T must have default constructor
    template <class T>
    class ReferencesList 
    {
    protected:
        List<T*> Objects;
    public:
        #pragma region Constructors
        ReferencesList()
        {
        }
        ~ReferencesList() // destructor (Rule of five)
        {
            Clear(); 
        }
        ReferencesList(const ReferencesList& o)  // copy constructor (Rule of five)
        {
            Reserve(o.size());
            //memcpy(m_data, o.m_data, m_size * sizeof(T));// shallow copy
            for (int i = 0; i < o.size(); i++)
            {
                auto& last = Add();
                last = o[i]; // deep copy
            }
        }
        ReferencesList(ReferencesList&& o) noexcept // move constructor (Rule of five)
            : m_size(std::exchange(o.m_size, Index())),
            m_data(std::exchange(o.m_data, nullptr)),
            m_info(std::exchange(o.m_info, ReferencesList_Dynamic_Info()))
        {
        }
        inline ReferencesList& operator=(const ReferencesList& other) // copy assignment (Rule of five)
        {
            return *this = ReferencesList(other);
        }
        inline ReferencesList& operator=(ReferencesList&& other) noexcept // move assignment (Rule of five)
        {
            std::swap(m_size, other.m_size);
            std::swap(m_data, other.m_data);
            std::swap(m_info, other.m_info);
            return *this;
        }
        constexpr Index size() const
        {
            return count;
        }

        #pragma endregion    

        #pragma region Read/Write (single value)
        CommonClassesINLINE T& operator()(Index index)
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            return *m_data[index];
        }
        CommonClassesINLINE const T& operator()(Index index) const
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            return *m_data[index];
        }
        CommonClassesINLINE T& operator[](Index index)
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            return *m_data[index];
        }
        CommonClassesINLINE const T& operator[](Index index) const
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            return *m_data[index];
        }
        #pragma endregion 

        void Reserve(Index size)
        {
            resize_internal(size);
            count = size;
        }


        T& Add()
        {
            Objects.Add(new T());
            return Objects.Last();
        }

        bool Remove(const T& obj)
        {
            auto index = std::find(Objects.begin(), Objects.end(), obj);
            if (index != Objects.end())
            {
                delete obj;
                Objects.erase(index);
                return true;
            }
            return false;
        }

        void Clear()
        {
            for (T* obj : Objects)
            {
                delete obj;
            }
            Objects.Clear();
        }



        
    };
}