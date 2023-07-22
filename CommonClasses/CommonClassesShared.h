#pragma once
#include <assert.h>

namespace CommonClasses
{ 
    typedef int Index;
    typedef float Decimal;
    const Index X = -1;
    #define CommonClassesINLINE inline

    #define CommonClasses_DEF_ALL_SIZES__Execute \
    CommonClasses_DEF_ALL_SIZES(int, i)\
    CommonClasses_DEF_ALL_SIZES(float, f)

    //#define CommonClasses_DEF_ALL_SIZES__Execute \
    //    CommonClasses_DEF_ALL_SIZES(long, l)\
    //    CommonClasses_DEF_ALL_SIZES(int, i)\
    //    CommonClasses_DEF_ALL_SIZES(unsigned int, ui)\
    //    CommonClasses_DEF_ALL_SIZES(float, f)\
    //    CommonClasses_DEF_ALL_SIZES(double, d)\
    //    CommonClasses_DEF_ALL_SIZES(std::complex<float>, cf)\
    //    CommonClasses_DEF_ALL_SIZES(std::complex<double>, cd)
};

namespace std
{
    //    std::string to_string(std::complex<double> c);
}