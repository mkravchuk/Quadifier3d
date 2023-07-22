// stdafx.h : include file for standard system include files,
// or project specific include files that are used freComplexSchur_do_not_normalizequently, but
// are changed infrequently
//

#pragma once

#include "../CommonTypes/disabled_warnings.h"
#include "../CommonTypes/common_cpp_standard_headers.h"

//For our solver we dont need normalize vectors since values are always close to 1 - this will improve performance by 5% and quality (since) there will be no division by norm)
#define ComplexSchur_do_not_normalize

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>  // required for old solvers that was using Eigen sparce matrixes
//#include <Eigen/SparseCore>
//#include <Eigen/OrderingMethods>
//#include <Eigen/SparseCholesky>
//#include <Eigen/SparseLU>
//#include <Eigen/SparseQR>
//#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Eigenvalues>   // required for old solvers that was using Eigen sparce matrixes
//#include <Eigen/SVD>
//#include <Eigen/Dense>
//#include <Eigen/StdVector>

#include <igl/nchoosek.h>
#include <igl/slice.h>

#include "../CommonTypes/common_types.h"
using namespace std;

#include "_MeshLogicOptions.h"
#include "ViewerDrawObjects.h"
#include "Utils.h"

#include "polarvec.h"
#include "complexvec.h"

#include "SparceMatrixDirect.h"
#include <igl/angle_bound_frame_fields.h>
