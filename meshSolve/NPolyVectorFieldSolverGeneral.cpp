#include "stdafx.h"
#include "NPolyVectorFieldSolverGeneral.h"
#include <igl/nchoosek.h>
#include <igl/polyroots.h>
//#include <igl/n_polyvector.h>
//#include <igl/n_polyvector_general.h>
#include "ViewerDrawObjects.h"
#include "FaceConstrain.h"
#include <igl/slice.h>
#include "Utils.h"


class NPolyVectorFieldSolverGeneral
{
public:
    NPolyVectorFieldSolverGeneral(const P3s& _V,
        const I3s& _F,
        const int &_n,
        const I2s& _EV,
        const I3s& _FE,
        const I2s& _EF,
        const Ds& _K,
        const V3s& _F_X,
        const V3s& _F_Y,
        const V3s& _FN
    );
    bool solve(const Is& isConstrained,
        const Matrix<D, Dynamic, Dynamic> &cfW,
        const Is&rootsIndex,
        vector<V3s>& Result_F);

private:
    const P3s &V;
    const I3s& F;
    int facesCount;
    const int n;

    const I3s& FE;
    const I2s& EV;
    const I2s& EF;
    int edgesCount;
    const Ds& K;

    Is isBorderEdge;
    int numInteriorEdges;
    I2s E2F_int;
    Is indInteriorToFull;
    Is indFullToInterior;

    const V3s& FX, FY, FN;//igl::local_basis

    void setFieldFromGeneralCoefficients(const  vector<Matrix<complex<D>, Dynamic, 1>> &coeffs,
        vector<Matrix<D, Dynamic, 2> > &pv);

    void computeFacesLaplacianCoefficient(int n, SparseMatrix<complex<D> > &S);

    void getGeneralCoeffConstraints(const Is& isConstrained,
        const Matrix<D, Dynamic, Dynamic> &cfW,
        int k,
        const Is& rootsIndex,
        Matrix<complex<D>, Dynamic, 1> &Ck);

    void precomputeInteriorEdges();


    void minQuadWithKnownMini(const SparseMatrix<complex<D> > &FacesLaplacianCoefficient,
        const SparseMatrix<complex<D> > &f,
        const Is isConstrained,
        const Matrix<complex<D>, Dynamic, 1> &xknown,
        Matrix<complex<D>, Dynamic, 1> &x);

};

NPolyVectorFieldSolverGeneral::NPolyVectorFieldSolverGeneral(const P3s& _V,
    const I3s& _F,
    const int &_n,
    const I2s& _EV,
    const I3s& _FE,
    const I2s& _EF,
    const Ds& _K,
    const V3s& _F_X,
    const V3s& _F_Y,
    const V3s& _FN) :
    V(_V),
    F(_F),
    facesCount(_F.rows()),
    n(_n),
    EV(_EV),
    FE(_FE),
    EF(_EF),
    K(_K),
    FX(_F_X),
    FY(_F_Y),
    FN(_FN)
{
    edgesCount = EV.rows();

    precomputeInteriorEdges();
};




//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//--- SOLVE -----------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------




void NPolyVectorFieldSolverGeneral::minQuadWithKnownMini(const SparseMatrix<complex<D> > &FacesLaplacianCoefficient,
    const SparseMatrix<complex<D> > &f,
    const Is isConstrained,
    const Matrix<complex<D>, Dynamic, 1> &xknown,
    Matrix<complex<D>, Dynamic, 1> &x)
{
    bool isSoft = false;
    int N = FacesLaplacianCoefficient.rows();

    int nc = xknown.rows();
    VectorXi known; known.setZero(nc, 1);
    VectorXi unknown; unknown.setZero(N - (isSoft ? 0 : nc), 1);

    int indk = 0;
    int indu = 0;
    for (int i = 0; i < N; ++i)
    {
        if (isConstrained[i])
        {
            known[indk] = i;
            indk++;
        }
        if (!isConstrained[i] || isSoft)
        {
            unknown[indu] = i;
            indu++;
        }
    }

    SparseMatrix<complex<D>> Quu, Quk;

    igl::slice(FacesLaplacianCoefficient, unknown, unknown, Quu);
    igl::slice(FacesLaplacianCoefficient, unknown, known, Quk);


    vector<typename Triplet<complex<D> > > tripletList;

    SparseMatrix<complex<D> > fu(N - (isSoft ? 0 : nc), 1);

    igl::slice(f, unknown, VectorXi::Zero(1, 1), fu);

    SparseMatrix<complex<D> > rhs = (Quk* xknown).sparseView() + .5*fu;

    SparseLU< SparseMatrix<complex<D>>> solver;
    solver.compute(-Quu);
    if (solver.info() != Success)
    {
        std::cerr << "Decomposition failed!" << std::endl;
        return;
    }
    SparseMatrix<complex<D>>  b = solver.solve(rhs);
    if (solver.info() != Success)
    {
        std::cerr << "Solving failed!" << std::endl;
        return;
    }

    indk = 0;
    indu = 0;
    x.setZero(N, 1);
    for (int i = 0; i < N; ++i)
    {
        if (isConstrained[i])
        {
            x[i] = xknown[indk++];
        }
        else
        {
            x[i] = b.coeff(indu++, 0);
        }
        if (isSoft)
        {
            if (isConstrained[i])
            {
                x[i] = x[i] + b.coeff(i, 0);
            }
            else
            {
                x[i] = b.coeff(i, 0);
            }
        }
    }
}






bool NPolyVectorFieldSolverGeneral::
solve(const Is& isConstrained,
    const Matrix<D, Dynamic, Dynamic> &cfW,
    const Is& rootsIndex,
    vector<V3s>& Result_F)
{
    auto time = utils::time::Now();

    // polynomial is of the form:
    // z^(2n) +
    // -c[0]z^(2n-1) +
    // c[1]z^(2n-2) +
    // -c[2]z^(2n-3) +
    // ... +
    // (-1)^n c[n-1]
    vector<Matrix<complex<D>, Dynamic, 1>> coeffs(n, Matrix<complex<D>, Dynamic, 1>::Zero(facesCount, 1));
    cout << "NPolyVectorFieldSolver::solve()  minQuadWithKnownMini  ...";
    extern bool IsOmpEnabled;
#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int i = 0; i < n; ++i)
    {
        int degree = i + 1;
        Matrix<complex<D>, Dynamic, 1> Ck;
        getGeneralCoeffConstraints(isConstrained,
            cfW,
            i,
            rootsIndex,
            Ck);

        SparseMatrix<complex<D> > FacesLaplacianCoefficient;
        computeFacesLaplacianCoefficient(degree, FacesLaplacianCoefficient);

        SparseMatrix<complex<D> > f;
        f.resize(facesCount, 1);

        if (isConstrained.sum() == facesCount)
            coeffs[i] = Ck;
        else
            minQuadWithKnownMini(FacesLaplacianCoefficient, f, isConstrained, Ck, coeffs[i]);
    }
    cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;

    //
    // setFieldFromGeneralCoefficients
    //
    cout << "NPolyVectorFieldSolver::solve()  setFieldFromGeneralCoefficients  ...";
    vector<Matrix<D, Dynamic, 2> > pv;
    setFieldFromGeneralCoefficients(coeffs, pv);
    cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;


    //
    // Result_F
    //
    cout << "NPolyVectorFieldSolver::solve()  generate output nrosy field  ...";
    Result_F.resize(n);
    for (int i = 0; i < n; i++)
    {
        Result_F[i].setZero(facesCount, 3);
    }
    extern bool IsOmpEnabled;
#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int fi = 0; fi < facesCount; ++fi)
    {
        const V3 &b1 = FX.row(fi);
        const V3 &b2 = FY.row(fi);
        for (int i = 0; i < n; i++)
        {
            Result_F[i].row(fi) = pv[i](fi, 0)*b1 + pv[i](fi, 1)*b2;
        }
    }
    cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    return true;
}



//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------




void NPolyVectorFieldSolverGeneral::setFieldFromGeneralCoefficients(const  vector<Matrix<complex<D>, Dynamic, 1>> &coeffs,
    vector<Matrix<D, Dynamic, 2>> &pv)
{
    pv.assign(n, Matrix<D, Dynamic, 2>::Zero(facesCount, 2));
    Matrix<complex<D>, 3, 3> I;
    I.setIdentity(3, 3);
    Matrix<complex<D>, 3, 1> z;
    z.setZero(3, 1);
    extern bool IsOmpEnabled;
#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int i = 0; i < facesCount; ++i)
    {
        //    poly coefficients: 1, 0, -Acoeff, 0, Bcoeff
        //    matlab code from roots (given there are no trailing zeros in the polynomial coefficients)
        Matrix<complex<D>, 4, 1> polyCoeff;
        int sign = 1;
        for (int k = 0; k < n; ++k)
        {
            polyCoeff[k] = (D)(1.*sign)*coeffs[k](i);
            sign = -sign;
        }

        Matrix<complex<D>, 4, 4> a;
        a << polyCoeff.transpose(), I, z;

        const Matrix<complex<D>, 4, 1>& roots = a.eigenvalues();

        for (int k = 0; k < n; ++k)
        {
            pv[k](i, 0) = real(roots[k]);
            pv[k](i, 1) = imag(roots[k]);
        }
    }

}


void NPolyVectorFieldSolverGeneral::computeFacesLaplacianCoefficient(int n, SparseMatrix<complex<D> > &S)
{
    vector<Triplet<complex<D> >> tripletList;

    // For every non-border edge
    for (int eid = 0; eid < edgesCount; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            tripletList.push_back(Triplet<complex<D> >(fid0,
                fid0,
                complex<D>(1.)));
            tripletList.push_back(Triplet<complex<D> >(fid1,
                fid1,
                complex<D>(1.)));
            tripletList.push_back(Triplet<complex<D> >(fid0,
                fid1,
                D(-1.)*std::polar(D(1.), D(-1.*n*K[eid]))));
            tripletList.push_back(Triplet<complex<D> >(fid1,
                fid0,
                D(-1.)*std::polar(D(1.), D(1.*n*K[eid]))));

        }
    }
    S.resize(facesCount, facesCount);
    S.setFromTriplets(tripletList.begin(), tripletList.end());


}

//this gives the coefficients without the (-1)^k that multiplies them
void NPolyVectorFieldSolverGeneral::getGeneralCoeffConstraints(const Is& isConstrained,
    const Matrix<D, Dynamic, Dynamic> &cfW,
    int k,
    const Is& rootsIndex,
    Matrix<complex<D>, Dynamic, 1> &Ck)
{
    int numConstrained = isConstrained.sum();
    Ck.resize(numConstrained, 1);
    // int n = rootsIndex.cols();

    MatrixXi allCombs;
    {
        VectorXi V = VectorXi::LinSpaced(n, 0, n - 1);
        igl::nchoosek(V, k + 1, allCombs);
    }

    int ind = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        const V3 &b1 = FX.row(fi);
        const V3 &b2 = FY.row(fi);
        if (isConstrained[fi])
        {
            complex<D> ck(0);

            for (int j = 0; j < allCombs.rows(); ++j)
            {
                complex<D> tk(1.);
                //collect products
                for (int i = 0; i < allCombs.cols(); ++i)
                {
                    int index = allCombs(j, i);

                    int ri = rootsIndex[index];
                    Matrix<D, 3, 1> w;
                    if (ri > 0)
                        w = cfW.block<1,3>(fi, 3 * (ri - 1)).transpose();
                    else
                        w = -cfW.block<1,3>(fi, 3 * (-ri - 1)).transpose();
                    V3 wv3 = convertEigenToV3(w);
                    D w0 = wv3.dot(b1);
                    D w1 = wv3.dot(b2);
                    complex<D> u(w0, w1);
                    tk *= u;
                }
                //collect sum
                ck += tk;
            }
            Ck(ind) = ck;
            ind++;
        }
    }


}


void NPolyVectorFieldSolverGeneral::precomputeInteriorEdges()
{
    // Flag border edges
    numInteriorEdges = 0;
    isBorderEdge.setZero(edgesCount);
    indFullToInterior = Is::Constant(edgesCount, -1);

    for (int i = 0; i < edgesCount; ++i)
    {
        if ((EF(i, 0) == -1) || ((EF(i, 1) == -1)))
            isBorderEdge[i] = 1;
        else
        {
            indFullToInterior[i] = numInteriorEdges;
            numInteriorEdges++;
        }
    }

    E2F_int.resize(numInteriorEdges, 2);
    indInteriorToFull.setZero(numInteriorEdges);
    int ii = 0;
    for (int k = 0; k < edgesCount; ++k)
    {
        if (isBorderEdge[k])
            continue;
        E2F_int.row(ii) = EF.row(k);
        indInteriorToFull[ii] = k;
        ii++;
    }

}

void n_polyvector_general_soft(
    ViewerDrawObjects& draw,
    const P3s &V,
    const I3s& F,
    const Ds &F_Areas,
    const I2s& EV,
    const I3s& FE,
    const I2s& EF,
    const Ds& K,
    const V3s& F_X,
    const V3s& F_Y,
    const V3s& F_Z,
    vector<FaceConstrain> Constrains,  // Constrained faces representative vector
    D softPercent, //ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
    vector<V3s>& Result_F, bool takeDirectionCorrected)
{
    Is rootsIndex(4);
    rootsIndex(0) = 1;
    rootsIndex(1) = 2;
    rootsIndex(2) = 3;
    rootsIndex(3) = 4;

    Matrix<D, Dynamic, Dynamic> bcSoft4;
    bcSoft4.resize(Constrains.size(), 3 * 4);
    for (int i = 0; i < Constrains.size(); ++i)
    {
        //TODO speed optimization - sort vectors to avoid sorting them later in method StreamLine::Init()::polyvector_field_matchings
        int fid = Constrains[i].FaceId;
        V3 constrDirection = takeDirectionCorrected 
                                            ? Constrains[i].DirectionY_Corrected 
                                            : Constrains[i].DirectionY;
        int eid = FE(fid, 0);
        V3 edgeDirectionV3 = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
        edgeDirectionV3.normalize();
        V3 directionV3= constrDirection;// do not normalize - let the contrain algorithm provide weights on contrains
        D m = softPercent;
        // i dont know why results are wrong when softPercent ==1 - in this case we have to avoid multiply opperaion - why i dont know
        Matrix<D, 1, 3> edgeDirection = convertV3ToEigen(edgeDirectionV3).transpose().cast<D>();
        Matrix<D, 1, 3> direction = convertV3ToEigen(directionV3).transpose().cast<D>();
        if (abs(1 - m) < 0.0000000000001)
        { 
            m = (D)0.999;
            bcSoft4.row(i) << edgeDirection, direction*m, -edgeDirection, -direction*m;
        }
        else
        {
            bcSoft4.row(i) << edgeDirection, direction*m, -edgeDirection, -direction*m;
        }
    }
    //DEBUG show bcSoft4
    //draw.ReservePoints(bSoft.size()); // reseve space - maximum of what we can add
    //draw.ReserveEdges(4 * bSoft.size()); // reseve space - maximum of what we can add
    //for (int i = 0; i < bSoft.size(); i++)
    //{
    //    //if (i != 0 && i != 10 && i != 20) continue;
    //    for (int ni = 0; ni < 4; ni++)
    //    {
    //        P3 v = F_Barycenters.row(bSoft(i));
    //        RowVector3d directoin = bcSoft4.block(i, 3 * ni, 1, 3);
    //        draw.AddEdge(v, v + directoin.transpose()*avg_edge_length/2, Color3d(1, 0, 0));
    //    }
    //    //show first face edge
    //    //int eid = FE(bSoft(i), 0);
    //    //P3 p = V.row(EV(eid, 0));
    //    //RowVector3d edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
    //    //edgeDirection.normalize();
    //    //draw.AddEdge(p, p + edgeDirection.transpose()*avg_edge_length, Color3d(1, 0, 0));
    //   // draw.AddPoint(p, Color3d(0, 0, 0));
    //}


    Is isConstrained = Is::Zero(F.rows());
    Matrix<D, Dynamic, Dynamic> cfW4 = Matrix<D, Dynamic, Dynamic>::Constant(F.rows(), bcSoft4.cols(), 0);

    for (int i = 0; i < Constrains.size(); ++i)
    {
        int fid = Constrains[i].FaceId;
        isConstrained(fid) = 1;
        cfW4.row(fid) << bcSoft4.row(i);
    }
    int n = rootsIndex.size();

    NPolyVectorFieldSolverGeneral pvff(V, F, n, EV, FE, EF, K, F_X, F_Y, F_Z);
    pvff.solve(isConstrained, cfW4, rootsIndex, Result_F);
}