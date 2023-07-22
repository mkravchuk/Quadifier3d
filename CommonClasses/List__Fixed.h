#pragma once
#include "CommonClassesShared.h"
#include "MemoryFixed.h"


namespace CommonClasses
{
    // Specialization - fixed list (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T, Index SIZE = X>
    class List : public MemoryFixed<T, SIZE>
    {
    public:
        #pragma region Constructors
        template <Index S = SIZE, std::enable_if_t<(S == 2)>* = 0>              // Method available only for List2     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        List(const T x, const T y)
        {
            m_data[0] = x;
            m_data[1] = y;
        }
        template <Index S = SIZE, std::enable_if_t<(S == 3)>* = 0>              // Method available only for List3     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        List(const T x, const T y, const T z)
        {
            m_data[0] = x;
            m_data[1] = y;
            m_data[2] = z;
        }
        template <Index S = SIZE, std::enable_if_t<(S == 4)>* = 0>              // Method available only for List4     (specialization for Vectors - improves intellisense)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
        List(const T x, const T y, const T z, const T w)
        {
            m_data[0] = x;
            m_data[1] = y;
            m_data[2] = z;
            m_data[3] = w;
        }
        inline List<T, SIZE>& operator=(const T* data)
        {
            for (Index i = 0; i < SIZE; i++)
            {
                m_data[i] = data[i];
            }
            return *this;
        }
        #pragma endregion 


        #pragma region Transformations
        template <class TCast>
        List<TCast, SIZE> cast()
        {
            List<TCast, SIZE>  c;
            for (Index i = 0; i < SIZE; i++) c[i] = static_cast<TCast>(m_data[i]);
            return c;
        }
        #pragma endregion  

        #pragma region Static Methods
        CommonClassesINLINE static List<T, SIZE> Constant(const T c)
        {
            List<T, SIZE> v;
            v.setConstant(c);
            return v;
        }
        #pragma endregion 
    };
}