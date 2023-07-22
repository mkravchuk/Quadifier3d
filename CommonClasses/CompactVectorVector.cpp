#include "stdafx.h"
#include "CommonClassesShared.h"
#include "CompactVectorVector.h"

namespace CommonClasses
{
    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
template class CompactVectorVector<Type>; 

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_DEF_ALL_SIZES

};
