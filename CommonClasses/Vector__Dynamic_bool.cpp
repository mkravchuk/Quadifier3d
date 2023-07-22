#include "stdafx.h"
#include "CommonClassesShared.h"
#include "Vector__Dynamic_bool.h"

namespace CommonClasses
{
    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
template class Vector<Type, X>; 

    CommonClasses_DEF_ALL_SIZES(bool, b)

        #undef CommonClasses_DEF_ALL_SIZES
};

