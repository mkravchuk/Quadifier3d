#pragma once
#include "complexvec.h"

#define Real float
#define RealScalar Complex2f
#define Scalar Complex4f
#define Index int
#define SparceMatrixType SparseMatrix<Scalar, ColMajor>
#define CholSparceMatrixType SparseMatrix<Scalar, ColMajor, Index>
#define VectorType Matrix<Scalar, Dynamic, 1>
#define MatrixL SparseTriangularView<CholSparceMatrixType, UnitLower>  
#define MatrixU SparseTriangularView<typename CholSparceMatrixType::AdjointReturnType, UnitUpper> 
#define SolveMatrixType Matrix<Scalar, Dynamic, Dynamic, ColMajor>
#define UpLo UpLoType::Lower
#define Ordering AMDOrdering<Index>
#define MatrixX2d Matrix<RealScalar, Dynamic, 2>


namespace Eigen
{
    template<> 
    struct NumTraits<Complex4f > : NumTraits<Complex2f >
    {
        //typedef Complex2f Real;
        typedef Complex2f NonInteger;
        typedef Complex2f Nested;
        enum
        {
            IsComplex = 1,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3 * 3
        };


        static inline RealScalar dummy_precision()
        {
            // make sure to override this for floating-point types
            return RealScalar(1e-5f, 1e-5f);
        }
    };

}

namespace std
{
    static inline Complex4f conj(const Complex4f& x4)
    {
        Complex4f res = x4.getconjugated();
        return res;
    }

    static inline Complex2f real(const Complex4f& x4)
    {
        Complex2f res = x4.real();
        return res;
    }

}