#pragma once


class AngleBoundFFSolver_V3
{
public:
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
    int numV;
    int numF;
    int numE;


    //laplacians
    SparseMatrix<complex<D>> DDA, DDB;

    //polyVF data
    Matrix<complex<D>, Dynamic, 1> Acoeff, Bcoeff;
    Matrix<D, Dynamic, 2> pvU, pvV;
    D lambda;

    //parameters
    int maxIter;
    D thetaMin;
    D lambdaInit;
    D lambdaMultFactor;
    bool doHardConstraints;

    AngleBoundFFSolver_V3(const P3s &V, const I3s& F, const I2s& EV, const I3s& FE, const I2s& EF,
        const V3s& FX, const V3s& FY, const V3s& FN, const Ds& K, const Bs& E_isborder,
        int _maxIter = 50,
        D _thetaMin = 30,
        D _lambdaInit = 100,
        D _lambdaMultFactor = 1.01,
        bool _doHardConstraints = false);

    bool solve(const Bs& isConstrained,
        const vector<V3s> &initialSolution,
        vector<V3s> &output, bool logMessages);

    D computeAngle(const complex<D> &u, const complex<D> &v);

    int getNumOutOfBounds();

    void rotateAroundBisector(const complex<D> &uin,
        const complex<D> &vin,
        const D theta,
        complex<D> &uout,
        complex<D> &vout);

    void localStep();

    void globalStep(const Bs& isConstrained,
        const Matrix<complex<D>, Dynamic, 1>  &Ak,
        const Matrix<complex<D>, Dynamic, 1>  &Bk);

    void minQuadWithKnownMini(const SparseMatrix<complex<D> > &Q,
        const SparseMatrix<complex<D> > &f,
        const Bs& isConstrained,
        const Matrix<complex<D>, Dynamic, 1> &xknown,
        Matrix<complex<D>, Dynamic, 1> &x);

    void setFieldFromCoefficients();
    void setCoefficientsFromField();


    void computeLaplacians();
    void computeLaplacians_n(int n, SparseMatrix<complex<D> > &coeff);
};
