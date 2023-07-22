#include "stdafx.h"
#include "CommonClassesShared.h"
#include "Point34f.h"
#include "Vector34f.h"
#include "MatrixX34f.h"

namespace CommonClasses
{
template class MatrixX34f<Point34f>;
template class MatrixX34f<Vector34f>;
};

