#pragma once
#include "SolverTypes_Polar2x.h"
#include "LinearEquationSolver_Polar2x.h"


class AngleBoundFFSolver_Polar
{
public:
    ViewerDrawObjects& draw;
    LinearEquationSolver_Polar2x solver;
    bool solver_reuse_ordering;
    const P3s& V;
    const I3s& F;
    const I2s& EV;
    const I3s& FE;
    const I2s& EF;
    const V3s& FX;
    const V3s& FY;
    const V3s& FN;
    const Ds& K;
    const Bs& E_isborder;
    D max_edge_length;
    const Ds& E_Length;
    const SparceMatrixType& borderconstrains__Qu;//laplacians
    const SparceMatrixTypeComplex& borderconstrains__Qk;//laplacians
    const Is& borderconstrains__known_index_to_fid;
    const Is& borderconstrains__unknown_index_to_fid;
    SparceMatrixType noneconstrains__Qu;//laplacians
    SparceMatrixType noneconstrains__Qk;//laplacians
    Is noneconstrains__known_index_to_fid;
    Is noneconstrains__unknown_index_to_fid;

    int numV;
    int numF;
    int numE;
    
    //SparseMatrix<complex<D>> DDA, DDB;//laplacians
    //Matrix<complex<D>, Dynamic, 1> Acoeff;//polyVF data
    //Matrix<complex<D>, Dynamic, 1> Bcoeff;//polyVF data
    //Matrix<D, Dynamic, 2> pvU, pvV;//polyVF data

    // v0 - original
    //SparceMatrixType dd;   //size = (numF,numF)
    //v1 - fast
    #define dd noneconstrains__Qu
    VectorType coeff;         //size = (numF,1)
    VectorType pv;            //size = (numF,1)

    D lambda;

    //parameters
    int maxIter;
    D thetaMin;
    D lambdaInit;
    D lambdaMultFactor;
    bool doHardConstraints;
    bool is_computeLaplacians_calculated;

    AngleBoundFFSolver_Polar(ViewerDrawObjects& draw, const P3s &V, const I3s& F, const I2s& EV, const I3s& FE, const I2s& EF,
        const V3s& FX, const V3s& FY, const V3s& FN, const Ds& K, const Bs& E_isborder, D max_edge_length, const Ds& E_Length,
        const Is& known_index_to_fid, const Is& unknown_index_to_fid, const SparceMatrixType& Quu, const SparceMatrixTypeComplex& Quk,
        int _maxIter = 50,
        D _thetaMin = 30,
        D _lambdaInit = 100,
        D _lambdaMultFactor = 1.01,
        bool _doHardConstraints = false);

    bool solve(const Bs& isConstrained,
        const vector<V3s> &initialSolution,
        vector<V3s> &output, bool logMessages, bool solver_reuse_ordering = false);

    D computeAngle(const complex<D> &u, const complex<D> &v);

    int getNumOutOfBounds();

    void rotateAroundBisector(const complex<D> &uin, const complex<D> &vin, D arg_u, D arg_v,
        const D theta, complex<D> &uout, complex<D> &vout);

    void localStep();

    void globalStep(const Bs& isConstrained, const VectorType& AB, bool logMessages);

    void minQuadWithKnownMini(const SparceMatrixType& Qu, const SparceMatrixType& Qk,
        const Bs& isConstrained, const Is& known_index_to_fid, const Is& unknown_index_to_fid,
        const VectorType& xknown,
        const VectorType& f,
        VectorType& x, 
        bool logMessages);


    static void setFieldFromCoefficients(const  VectorType &coeffs, VectorType &pv);
    void setCoefficientsFromField();

    static void compute_known_unknown_indexes(const Bs& isConstrained, Is& known, Is& unknown);
    //void computeLaplacians();
};
