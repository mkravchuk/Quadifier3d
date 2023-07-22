#include "stdafx.h"
#include "CommonClassesShared.h"
#include "Vector__Dynamic.h"

namespace CommonClasses
{
    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
template class Vector<Type, X>; 

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_DEF_ALL_SIZES

};

