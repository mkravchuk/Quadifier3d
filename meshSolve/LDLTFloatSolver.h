#pragma once
#include "SolverTypes.h"
class ViewerDrawObjects;




class LDLTFloatSolver// : internal::noncopyable
{
public:


    LDLTFloatSolver()
        : m_info(Success), m_isInitialized(false), m_factorizationIsOk(false), m_analysisIsOk(false)
    {
    }

    LDLTFloatSolver(const SparceMatrixType& matrix)
        : m_info(Success), m_isInitialized(false)
    {
        compute(matrix);
    }

    inline Index cols() const
    {
        return L.cols();
    }
    inline Index rows() const
    {
        return L.rows();
    }

    /** \brief Reports whether previous computation was successful.
    *
    * \returns \c Success if computation was succesful,
    *          \c NumericalIssue if the matrix.appears to be negative.
    */
    ComputationInfo info() const
    {
        assert(m_isInitialized && "Decomposition is not initialized.");
        return m_info;
    }


    /** \returns a vector expression of the diagonal D */
    inline const VectorType vectorD() const
    {
        assert(m_factorizationIsOk && "Simplicial LDLT not factorized");
        return D;
    }
    /** \returns an expression of the factor L */
    inline const MatrixL matrixL() const
    {
        assert(m_factorizationIsOk && "Simplicial LDLT not factorized");
        return L;
    }

    /** \returns an expression of the factor U (= L^*) */
    inline const MatrixU matrixU() const
    {
        assert(m_factorizationIsOk && "Simplicial LDLT not factorized");
        return L.adjoint();
    }

    /** \returns the determinant of the underlying matrix from the current factorization */
    Scalar determinant() const
    {
        return D.prod();
    }


    /** \returns the permutation P
    * \sa permutationPinv() */
    const PermutationMatrix<Dynamic, Dynamic, Index>& permutationP() const
    {
        return m_P;
    }

    /** \returns the inverse P^-1 of the permutation P
    * \sa permutationP() */
    const PermutationMatrix<Dynamic, Dynamic, Index>& permutationPinv() const
    {
        return m_Pinv;
    }

    template<typename Stream>
    void dumpMemory(Stream& s)
    {
        int total = 0;
        s << "  L:        " << ((total += (L.cols() + 1) * sizeof(int) + L.nonZeros()*(sizeof(int) + sizeof(Scalar))) >> 20) << "Mb" << "\n";
        s << "  diag:     " << ((total += D.size() * sizeof(Scalar)) >> 20) << "Mb" << "\n";
        s << "  tree:     " << ((total += m_parent.size() * sizeof(int)) >> 20) << "Mb" << "\n";
        s << "  nonzeros: " << ((total += L_nonZerosPerCol.size() * sizeof(int)) >> 20) << "Mb" << "\n";
        s << "  perm:     " << ((total += m_P.size() * sizeof(int)) >> 20) << "Mb" << "\n";
        s << "  perm^-1:  " << ((total += m_Pinv.size() * sizeof(int)) >> 20) << "Mb" << "\n";
        s << "  TOTAL:    " << (total >> 20) << "Mb" << "\n";
    }



    /** Computes the sparse Cholesky decomposition of \a matrix */
    void compute(const SparceMatrixType& a); // compute L,D at first time (order and factorize)
    void recompute(const SparceMatrixType& a); // compute L,D at next times (a structure must remain same, but data allowed to change) (faster from calling 'compute' if

    void create_permuted_a(const SparceMatrixType& a, CholSparceMatrixType& ap, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo); // create ap based on m_P
    void analyzePattern(const SparceMatrixType& a, CholSparceMatrixType& ap); // creates L, m_parent, L_nonZerosPerCol, ap
    void analyzePattern_preordered(const CholSparceMatrixType& ap); // allocate matrix 'L', and create tree 'm_parent (not depends on data of 'a', just depends on relations between indexes)
    void ordering(const SparceMatrixType& a, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo); //  constructs permute matrices 'm_P', 'm_Pinv' (not depends on data of 'a', just depends on relations between indexes)

    void factorize(const SparceMatrixType& a); // 
    void factorize_preordered(const CholSparceMatrixType& ap);
    void factorize_preordered_origin(const CholSparceMatrixType& ap); //  creates matrices L, D for LDLT
    void factorize_preordered_fast(const CholSparceMatrixType& ap); //  creates matrices L, D for LDLT
    

    void solve(const SolveMatrixType &rhs, SolveMatrixType &res) const;

    mutable ComputationInfo m_info;
    bool m_isInitialized;
    bool m_factorizationIsOk;
    bool m_analysisIsOk;

    VectorType D;                               // the diagonal coefficients (D matrix of LDLT)
    CholSparceMatrixType L;                 // L matrix of LDLT
    Is m_parent;                        // elimination tree (not depends on data of 'a', just depends on relations between indexes)
    Is L_nonZerosPerCol;            // non zero values per column of L (not depends on data of 'a', just depends on relations between indexes)
    PermutationMatrix<Dynamic, Dynamic, Index> m_P;     // the permutation (not depends on data of 'a', just depends on relations between indexes)
    PermutationMatrix<Dynamic, Dynamic, Index> m_Pinv;  // the inverse permutation (not depends on data of 'a', just depends on relations between indexes)
};

