#pragma once
#include "SolverTypes.h"

struct FaceConstrain;

void retFieldFromGeneralCoefficients(const VectorType &coeffs,
    VectorType &pv, bool ignore_x, const VectorXi &isFaceConstrained);
