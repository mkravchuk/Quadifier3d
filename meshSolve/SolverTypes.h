#pragma once
#include "complexvec.h"

#define RealScalar float
#define Scalar complex<RealScalar>
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
    struct NumTraits<Complex2f > : NumTraits<float >
    {
        //typedef Complex2f Real;
        typedef float NonInteger;
        typedef float Nested;
        enum
        {
            IsComplex = 1,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 0,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3 * 3
        };

        static inline RealScalar dummy_precision()
        {
            // make sure to override this for floating-point types
            return 1e-5f;
        }
    };


    namespace internal
    {
        template<>
        struct is_arithmetic<Complex2f>
        {
            enum
            {
                value = true
            };
        };
    }

}

namespace std
{
    static inline Complex2f conj(const Complex2f& x2)
    {
        Complex2f res = x2;
        res.conjugate();
        return res;
    }

    static inline float real(const Complex2f& x2)
    {
        float res = x2.real();
        return res;
    }
}