#include "stdafx.h"
#include "Matrix__FixedColumnSize.h"

//TEMPLATE CLASS MUST BE IMPLEMENTED IN HEADER
namespace CommonClasses
{
    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
template class  Matrix<Type, Dynamic, 2>; \
template class  Matrix<Type, Dynamic, 3>; \
template class  Matrix<Type, Dynamic, 4>; 


    //CommonClasses_DEF_ALL_SIZES(bool, b)
    //    CommonClasses_DEF_ALL_SIZES(long, l)
    //    CommonClasses_DEF_ALL_SIZES(int, i)
    //    CommonClasses_DEF_ALL_SIZES(unsigned int, ui)
    //    CommonClasses_DEF_ALL_SIZES(float, f)
    //    CommonClasses_DEF_ALL_SIZES(double, d)
    //    CommonClasses_DEF_ALL_SIZES(std::complex<float>, cf)
    //    CommonClasses_DEF_ALL_SIZES(std::complex<double>, cd)
    CommonClasses_DEF_ALL_SIZES__Execute


        #undef CommonClasses_DEF_ALL_SIZES
};