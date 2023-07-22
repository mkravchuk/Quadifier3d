#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"
#include "List__Fixed.h"  // must be included, because this class is specialization of Vector__Fixed

namespace CommonClasses
{
    class List_Dynamic_Info
    {
    public:
        List_Dynamic_Info()
        {
        }
    };

    // Specialization - dynamic list (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T>
    class List<T, X> : public MemoryDynamic<T, List_Dynamic_Info>
    {
    public:
        #pragma region Constructors
        List()
            : MemoryDynamic()
        {
        }
        List(Index size)
        {
            resize(size);
        }
        #pragma endregion    

        #pragma region Dimension control
        void resize(Index size)
        {
            resize_internal(size);
        }
        void conservativeResize(Index size)
        {
            conservativeResize_internal(size);
        }
        #pragma endregion 

        #pragma region  Fill data
        using MemoryDynamic<T, List_Dynamic_Info>::setConstant;
        void setConstant(Index size, T value)
        {
            resize(size);
            setConstant(value);
        }
        #pragma endregion 


        #pragma region Transformations
        template <class TCast>
        List<TCast, X> cast()
        {
            List<TCast, X>  c;
            for (Index i = 0; i < m_size; i++) c[i] = static_cast<TCast>(m_data[i]);
            return c;
        }
        #pragma endregion  

        #pragma region Static Methods
        static List<T, X> Constant(Index size, T value)
        {
            List<T, X>  m(size);
            m.setConstant(value);
            return m;
        }
        #pragma endregion 
    };
}


