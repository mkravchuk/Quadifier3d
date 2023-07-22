#pragma once
#include "polarvec.h"
#include "complexvec.h"

#define Real float
#define RealScalar Complex2f
#define RealScalarComplex Complex2f
#define Scalar Complex4f 
#define ScalarComplex Complex4f 
#define Index int
#define SparceMatrixType SparseMatrix<Scalar, ColMajor>
#define SparceMatrixTypeComplex SparseMatrix<ScalarComplex, ColMajor>
#define CholSparceMatrixType SparseMatrix<Scalar, ColMajor, Index>
#define CholSparceMatrixTypeComplex SparseMatrix<ScalarComplex, ColMajor, Index>
#define VectorType Matrix<Scalar, Dynamic, 1>
#define VectorTypeComplex Matrix<ScalarComplex, Dynamic, 1>
#define MatrixL SparseTriangularView<CholSparceMatrixType, UnitLower>  
#define MatrixU SparseTriangularView<typename CholSparceMatrixType::AdjointReturnType, UnitUpper> 
#define SolveMatrixTypeComplex Matrix<ScalarComplex, Dynamic, Dynamic, ColMajor>
#define SolveMatrixType Matrix<Scalar, Dynamic, Dynamic, ColMajor>
#define UpLo UpLoType::Lower
//#define MatrixX2d Matrix<RealScalar, Dynamic, 2>


//#define Real float
//#define RealScalar Polar2f
//#define RealScalarComplex Complex2f
//#define Scalar Polar4f 
//#define ScalarComplex Complex4f 
//#define Index int
//#define SparceMatrixType SparseMatrix<Scalar, ColMajor>
//#define SparceMatrixTypeComplex SparseMatrix<ScalarComplex, ColMajor>
//#define CholSparceMatrixType SparseMatrix<Scalar, ColMajor, Index>
//#define CholSparceMatrixTypeComplex SparseMatrix<ScalarComplex, ColMajor, Index>
//#define VectorType Matrix<Scalar, Dynamic, 1>
//#define VectorTypeComplex Matrix<ScalarComplex, Dynamic, 1>
//#define MatrixL SparseTriangularView<CholSparceMatrixType, UnitLower>  
//#define MatrixU SparseTriangularView<typename CholSparceMatrixType::AdjointReturnType, UnitUpper> 
//#define SolveMatrixTypeComplex Matrix<ScalarComplex, Dynamic, Dynamic, ColMajor>
//#define SolveMatrixType Matrix<Scalar, Dynamic, Dynamic, ColMajor>
//#define UpLo UpLoType::Lower
//#define Ordering AMDOrdering<Index>
////#define MatrixX2d Matrix<RealScalar, Dynamic, 2>


namespace Eigen
{
    template<> 
    struct NumTraits<Complex4f> : NumTraits<Complex2f >
    {
        //typedef RealScalar Real;
        typedef RealScalar NonInteger;
        typedef RealScalar Nested;
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


        //static inline RealScalar dummy_precision()
        //{
        //    // make sure to override this for floating-point types
        //    return RealScalar(1e-5f, 1e-5f);
        //}
    };

    namespace internal
    {
        template<>
        struct is_arithmetic<Complex4f>
        {
            enum
            {
                value = true
            };
        };


        template<bool Align> inline Complex4f* conditional_aligned_new_auto(size_t size)
        {
            if (size == 0)
                return 0; 
            Complex4f *result = reinterpret_cast<Complex4f*>(conditional_aligned_malloc<Align>(sizeof(Complex4f)*size));
            return result;
        }
    }
}

namespace std
{
    static inline Scalar conj(const Scalar& x4)
    {
        Scalar res = x4.getconjugated();
        return res;
    }

    static inline RealScalar real(const Scalar& x4)
    {
        RealScalar res = x4.real();
        return res;
    }

}

void computeFacesLaplacianCoefficient_Polar_direct(int facesCount, const I3s& FE, const I2s& EF, const V3s& FN, D max_edge_length, const Ds& E_Length, const Ds& K, const Bs& isBorderEdge,
    const Bs& isFaceConstrained, int numConstrained, SparceMatrixType &Quu, SparceMatrixTypeComplex &Quk,
    Is& known_index_to_fid, Is& unknown_index_to_fid);