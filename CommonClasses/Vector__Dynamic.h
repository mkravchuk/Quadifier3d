#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"
#include "Vector__Fixed.h"  // must be included, because this class is specialization of Vector__Fixed

namespace CommonClasses
{
    class Vector_Dynamic_Info
    {
    public:
        Vector_Dynamic_Info()
        {
        }
    };

    // Specialization - dynamic vector (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    template <class T>
    class Vector<T, X> : public MemoryDynamic<T, Vector_Dynamic_Info>
    {
    public:
        #pragma region Constructors
        Vector() = default;
        Vector(const T* data, Index size)
            : MemoryDynamic(data, size, Vector_Dynamic_Info())
        {
        }
        Vector(Index size)
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

        #pragma region  Fill data (Math)
        using MemoryDynamic<T, Vector_Dynamic_Info>::setConstant;
        using MemoryDynamic<T, Vector_Dynamic_Info>::setZero;
        #include "_IMemory_SingleValue_MathFillData"
        void setZero(Index size)
        {
            resize(size);
            setZero();
        }
        void setConstant(Index size, T value)
        {
            resize(size);
            setConstant(value);
        }
        void setRandom(Index size)
        {
            resize(size);
            setRandom();
        }
        #pragma endregion 

        #pragma region Read/Write (row)
        // same as [] or () - because Vector is a Matrix with dimension <N,1>
        CommonClassesINLINE T& row(Index row)
        {
            #if DEBUG
            assert(row >= 0 && row < size() && "row is out of range");
            #endif
            return m_data[row];
        }
        // same as [] or () - because Vector is a Matrix with dimension <N,1>
        CommonClassesINLINE T row(Index row) const
        {
            #if DEBUG
            assert(row >= 0 && row < size() && "row is out of range");
            #endif
            return m_data[row];
        }
        #pragma endregion 


        #pragma region Transformations
        // does nothing. added just for Eigen portability
        Vector<T, X> transpose() const
        {
            return *this;
        }
        // does nothing. added just for Eigen portability
        CommonClassesINLINE Vector<T, X>& eval()
        {
            return *this;
        }
        #pragma endregion  

        #pragma region Convertions
        template <class TCast>
        Vector<TCast, X> cast()
        {
            Vector<TCast, X>  v;
            for (Index i = 0; i < size(); i++) v[i] = static_cast<TCast>(m_data[i]);
            return v;
        }
        #include "_IMemory_toString"
        friend std::ostream& operator<<(std::ostream& os, const Vector<T, X>& v)
        {
            os << v.toString();
            return os;
        }
        #pragma endregion  

        #pragma region Static Methods
        static Vector<T, X> Zero(Index size)
        {
            Vector<T, X>  m(size);
            m.setZero();
            return m;
        }
        static Vector<T, X> Constant(Index size, T value)
        {
            Vector<T, X>  m(size);
            m.setConstant(value);
            return m;
        }
        static Vector<T, X> Random(Index size)
        {
            Vector<T, X>  m(size);
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
    #pragma region Define   VectorX

    #define CommonClasses_MAKE_TYPEDEFS_DYNAMIC(Type, TypeSuffix)         \
    typedef Vector<Type, X> Vector##X##TypeSuffix;  

    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_DYNAMIC(Type, TypeSuffix) 

    CommonClasses_DEF_ALL_SIZES(bool, b)
        CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_MAKE_TYPEDEFS_DYNAMIC
        #undef CommonClasses_DEF_ALL_SIZES

        #pragma endregion 
}
#endif
