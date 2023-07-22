#include "stdafx.h"
#include "CommonClassesShared.h"
#include "Table__DynamicDynamic.h"

namespace CommonClasses
{
    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
    template class Table<Type, X, X>;

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_DEF_ALL_SIZES

};

