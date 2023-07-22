#include "stdafx.h"
#include "CommonClassesShared.h"
#include "Vector__Fixed.h"

namespace CommonClasses
{
    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
template class Vector<Type, 2>; \
template class Vector<Type, 3>; \
template class Vector<Type, 4>;

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_DEF_ALL_SIZES
};
