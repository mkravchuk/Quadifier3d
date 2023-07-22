#include "stdafx.h"
#include "Matrix__DynamicColumnSize.h"

//TEMPLATE CLASS MUST BE IMPLEMENTED IN HEADER
namespace CommonClasses
{
    #define CommonClasses_MAKE_TEMPLATE_CLASS_DYNAMIC(Type, TypeSuffix)         \
template class Matrix<Type, Dynamic, Dynamic>; 

    #define CommonClasses_MAKE_TEMPLATE_CLASS_FIXED_DYNAMIC(Type, TypeSuffix, Size)         \
template class  Matrix<Type, Size, Dynamic>;  


    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TEMPLATE_CLASS_DYNAMIC(Type, TypeSuffix) \
CommonClasses_MAKE_TEMPLATE_CLASS_FIXED_DYNAMIC(Type, TypeSuffix, 2) \
CommonClasses_MAKE_TEMPLATE_CLASS_FIXED_DYNAMIC(Type, TypeSuffix, 3) \
CommonClasses_MAKE_TEMPLATE_CLASS_FIXED_DYNAMIC(Type, TypeSuffix, 4) 


    //CommonClasses_DEF_ALL_SIZES(bool, b)
    //    CommonClasses_DEF_ALL_SIZES(long, l)
    //    CommonClasses_DEF_ALL_SIZES(int, i)
    //    CommonClasses_DEF_ALL_SIZES(unsigned int, ui)
    //    CommonClasses_DEF_ALL_SIZES(float, f)
    //    CommonClasses_DEF_ALL_SIZES(double, d)
    //    CommonClasses_DEF_ALL_SIZES(std::complex<float>, cf)
    //    CommonClasses_DEF_ALL_SIZES(std::complex<double>, cd)
    CommonClasses_DEF_ALL_SIZES__Execute


        #undef CommonClasses_MAKE_TEMPLATE_CLASS_DYNAMIC
        #undef CommonClasses_MAKE_TEMPLATE_CLASS_FIXED_DYNAMIC
        #undef CommonClasses_DEF_ALL_SIZES
};