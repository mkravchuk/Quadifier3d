#include "stdafx.h"
#include "CommonClassesShared.h"
#include "Table__FixedFixed.h"

namespace CommonClasses
{
    #define CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, Size) \
    template class Table<Type, Size, Size>;

    #define CommonClasses_DEF_ALL_SIZES(Type, TypeSuffix) \
CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, 2) \
CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, 3) \
CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED(Type, TypeSuffix, 4) 

    CommonClasses_DEF_ALL_SIZES__Execute

        #undef CommonClasses_MAKE_TYPEDEFS_FIXED_FIXED
        #undef CommonClasses_DEF_ALL_SIZES

};

