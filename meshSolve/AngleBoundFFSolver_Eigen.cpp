#include "stdafx.h"
#include "AngleBoundFFSolver_Eigen.h"
#include <igl/angle_bound_frame_fields.h>
#include <igl/edge_topology.h>
#include <igl/local_basis.h>
#include <igl/sparse.h>
#include <igl/speye.h>
#include <igl/slice.h>
#include <igl/polyroots.h>
#include <igl/colon.h>

class AngleBoundFFSolverEigenData
{
public:
    const MatrixXd &V; int numV;
    const I3s& F; int numF;

    MatrixXi EV; int numE;
    MatrixXi F2E;
    MatrixXi E2F;
    VectorXd K;

    Is isBorderEdge;
    int numInteriorEdges;
    Matrix<int, Dynamic, 2> E2F_int;
    Is indInteriorToFull;
    Is indFullToInterior;

    MatrixXd B1, B2, FN;

    //laplacians
    SparseMatrix<complex<double>> DDA, DDB;

private:
    void computeLaplacians();
    void computeK();
    void computeCoefficientLaplacian(int n, SparseMatrix<complex<double> > &D);
    void precomputeInteriorEdges();

public:
    AngleBoundFFSolverEigenData(const MatrixXd &_V, const I3s& _F);
    AngleBoundFFSolverEigenData(const P3s &_V, const I3s& _F);
};


class AngleBoundFFSolverEigen
{
public:
    AngleBoundFFSolverEigen(const AngleBoundFFSolverEigenData &_data,
        int _maxIter = 50,
        const double &_thetaMin = 30,
        const double &_lambdaInit = 100,
        const double &_lambdaMultFactor = 1.01,
        const bool _doHardConstraints = false);

    AngleBoundFFSolverEigen(const P3s &_V, const I3s& _F,
        int _maxIter = 50,
        const double &_thetaMin = 30,
        const double &_lambdaInit = 100,
        const double &_lambdaMultFactor = 1.01,
        const bool _doHardConstraints = false);

    bool solve(const Is& isConstrained,
        const MatrixXd &initialSolution,
        MatrixXd &output,
        double *lambdaOut = nullptr);

    bool solve(const Bs& isConstrained,
        const vector<V3s>& initialSolution,
        vector<V3s>& output,
        double *lambdaOut = nullptr);

private:

    const AngleBoundFFSolverEigenData &data;

    //polyVF data
    Matrix<complex<double>, Dynamic, 1> Acoeff, Bcoeff;
    Matrix<double, Dynamic, 2> pvU, pvV;
    double lambda;

    //parameters
    double lambdaInit, lambdaMultFactor;
    int maxIter;
    double thetaMin;
    bool doHardConstraints;

    double computeAngle(const complex<double> &u, const complex<double> &v);
    //    void computeAngles(Matrix<double, Dynamic, 1> &angles);

    int getNumOutOfBounds();

    void rotateAroundBisector(const complex<double> &uin,
        const complex<double> &vin,
        const double theta,
        complex<double> &uout,
        complex<double> &vout);

    void localStep();

    void globalStep(const Is& isConstrained,
        const Matrix<complex<double>, Dynamic, 1>  &Ak,
        const Matrix<complex<double>, Dynamic, 1>  &Bk);

    void minQuadWithKnownMini(const SparseMatrix<complex<double> > &Q,
        const SparseMatrix<complex<double> > &f,
        const Is isConstrained,
        const Matrix<complex<double>, Dynamic, 1> &xknown,
        Matrix<complex<double>, Dynamic, 1> &x);

    void setFieldFromCoefficients();
    void setCoefficientsFromField();

};

//Implementation
/***************************** Data ***********************************/

AngleBoundFFSolverEigenData::AngleBoundFFSolverEigenData(const MatrixXd &_V,
    const I3s& _F) :
    V(_V),
    numV(_V.rows()),
    F(_F),
    numF(_F.rows())
{
    igl::edge_topology(V, convertI3sToEigenInt(F), EV, F2E, E2F);
    numE = EV.rows();

    precomputeInteriorEdges();

    igl::local_basis(V, convertI3sToEigenInt(F), B1, B2, FN);

    computeK();

    computeLaplacians();
    
}

AngleBoundFFSolverEigenData::AngleBoundFFSolverEigenData(const P3s &_V,
    const I3s& _F) : AngleBoundFFSolverEigenData(convertP3sToEigenDouble(_V), _F)
{
}

void AngleBoundFFSolverEigenData::computeLaplacians()
{
    computeCoefficientLaplacian(2, DDA);

    computeCoefficientLaplacian(4, DDB);
}

void AngleBoundFFSolverEigenData::precomputeInteriorEdges()
{
    // Flag border edges
    numInteriorEdges = 0;
    isBorderEdge.setZero(numE);
    indFullToInterior = Is::Constant(numE, -1);

    for (unsigned i = 0; i < numE; ++i)
    {
        if ((E2F(i, 0) == -1) || ((E2F(i, 1) == -1)))
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
    for (int k = 0; k < numE; ++k)
    {
        if (isBorderEdge[k])
            continue;
        E2F_int.row(ii) = E2F.row(k);
        indInteriorToFull[ii] = k;
        ii++;
    }

}



void AngleBoundFFSolverEigenData::computeCoefficientLaplacian(int n, SparseMatrix<complex<double> > &D)
{
    std::vector<Triplet<complex<double> >> tripletList;

    // For every non-border edge
    for (unsigned eid = 0; eid < numE; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = E2F(eid, 0);
            int fid1 = E2F(eid, 1);

            tripletList.push_back(Triplet<complex<double> >(fid0,
                fid0,
                complex<double>(1.)));
            tripletList.push_back(Triplet<complex<double> >(fid1,
                fid1,
                complex<double>(1.)));
            tripletList.push_back(Triplet<complex<double> >(fid0,
                fid1,
                -1.*std::polar(1., -1.*n*K[eid])));
            tripletList.push_back(Triplet<complex<double> >(fid1,
                fid0,
                -1.*std::polar(1., 1.*n*K[eid])));

        }
    }
    D.resize(numF, numF);
    D.setFromTriplets(tripletList.begin(), tripletList.end());


}

void AngleBoundFFSolverEigenData::computeK()
{
    K.setZero(numE);
    // For every non-border edge
    for (unsigned eid = 0; eid < numE; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = E2F(eid, 0);
            int fid1 = E2F(eid, 1);

            Matrix<double, 1, 3> N0 = FN.row(fid0);
            Matrix<double, 1, 3> N1 = FN.row(fid1);

            // find common edge on triangle 0 and 1
            int fid0_vc = -1;
            int fid1_vc = -1;
            for (unsigned i = 0; i < 3; ++i)
            {
                if (F2E(fid0, i) == eid)
                    fid0_vc = i;
                if (F2E(fid1, i) == eid)
                    fid1_vc = i;
            }
            assert(fid0_vc != -1);
            assert(fid1_vc != -1);

            Matrix<double, 1, 3> common_edge = V.row(F(fid0, (fid0_vc + 1) % 3)) - V.row(F(fid0, fid0_vc));
            common_edge.normalize();

            // Map the two triangles in a new space where the common edge is the x axis and the N0 the z axis
            Matrix<double, 3, 3> P;
            Matrix<double, 1, 3> o = V.row(F(fid0, fid0_vc));
            Matrix<double, 1, 3> tmp = -N0.cross(common_edge);
            P << common_edge, tmp, N0;
            //      P.transposeInPlace();


            Matrix<double, 3, 3> V0;
            V0.row(0) = V.row(F(fid0, 0)) - o;
            V0.row(1) = V.row(F(fid0, 1)) - o;
            V0.row(2) = V.row(F(fid0, 2)) - o;

            V0 = (P*V0.transpose()).transpose();

            Matrix<double, 3, 3> V1;
            V1.row(0) = V.row(F(fid1, 0)) - o;
            V1.row(1) = V.row(F(fid1, 1)) - o;
            V1.row(2) = V.row(F(fid1, 2)) - o;
            V1 = (P*V1.transpose()).transpose();

            // compute rotation R such that R * N1 = N0
            // i.e. map both triangles to the same plane
            double alpha = -atan2(V1((fid1_vc + 2) % 3, 2), V1((fid1_vc + 2) % 3, 1));

            Matrix<double, 3, 3> R;
            R << 1, 0, 0,
                0, cos(alpha), -sin(alpha),
                0, sin(alpha), cos(alpha);
            V1 = (R*V1.transpose()).transpose();

            // measure the angle between the reference frames
            // k_ij is the angle between the triangle on the left and the one on the right
            Matrix<double, 1, 3> ref0 = V0.row(1) - V0.row(0);
            Matrix<double, 1, 3> ref1 = V1.row(1) - V1.row(0);

            ref0.normalize();
            ref1.normalize();

            double ktemp = atan2(ref1(1), ref1(0)) - atan2(ref0(1), ref0(0));

            // just to be sure, rotate ref0 using angle ktemp...
            Matrix<double, 2, 2> R2;
            R2 << cos(ktemp), -sin(ktemp), sin(ktemp), cos(ktemp);

            Matrix<double, 1, 2> tmp1 = R2 * (ref0.head(2)).transpose();

            K[eid] = ktemp;
        }
    }

}


/***************************** Solver ***********************************/

AngleBoundFFSolverEigen::AngleBoundFFSolverEigen(const AngleBoundFFSolverEigenData &_data,
    int _maxIter,
    const double &_thetaMin,
    const double &_lambdaInit,
    const double &_lambdaMultFactor,
    const bool _doHardConstraints) :
    data(_data),
    lambdaInit(_lambdaInit),
    maxIter(_maxIter),
    lambdaMultFactor(_lambdaMultFactor),
    doHardConstraints(_doHardConstraints),
    thetaMin(_thetaMin)
{
    Acoeff.resize(data.numF, 1);
    Bcoeff.resize(data.numF, 1);
    pvU.setZero(data.numF, 2);
    pvV.setZero(data.numF, 2);
};


AngleBoundFFSolverEigen::AngleBoundFFSolverEigen(const P3s &_V, const I3s& _F,
    int _maxIter,
    const double &_thetaMin,
    const double &_lambdaInit,
    const double &_lambdaMultFactor,
    const bool _doHardConstraints)
    : AngleBoundFFSolverEigen(AngleBoundFFSolverEigenData(convertP3sToEigenDouble(_V), _F), _maxIter, _thetaMin, _lambdaInit, _lambdaMultFactor, _doHardConstraints)
{
}


void AngleBoundFFSolverEigen::rotateAroundBisector(const complex<double> &uin,
    const complex<double> &vin,
    const double diff,
    complex<double> &uout,
    complex<double> &vout)
{
    //rotate 2D complex vectors u and v around their bisector so that their
    //angle is at least theta

    uout = uin;
    vout = vin;
    double au = arg(uin);
    double av = arg(vin);
    if (au < av)
    {
        uout = std::polar(1.0, -.5*diff)*uin;
        vout = std::polar(1.0, .5*diff)*vin;
    }
    else
    {
        uout = std::polar(1.0, .5*diff)*uin;
        vout = std::polar(1.0, -.5*diff)*vin;
    }

}


void AngleBoundFFSolverEigen::localStep()
{
    for (int fid = 0; fid < data.numF; ++fid)
    {

        complex<double> u(pvU(fid, 0), pvU(fid, 1));
        complex<double> v(pvV(fid, 0), pvV(fid, 1));

        double current_angle = computeAngle(u, v);
        if (current_angle < thetaMin*M_PI / 180)
        {
            // bring all to 1st or 4th quarter plane
            if ((arg(u) >= 0.5*M_PI || arg(u) < -0.5*M_PI))
                u = -u;
            if ((arg(v) >= 0.5*M_PI || arg(v) < -0.5*M_PI))
                v = -v;
            assert(fabs(computeAngle(u, v) - current_angle) < 1e-5);

            if (fabs(arg(u) - arg(v)) > 0.5*M_PI)
                v = -v;
            assert(fabs(computeAngle(u, v) - current_angle) < 1e-5);

            complex<double> u1, v1;
            double diff = thetaMin * M_PI / 180 - current_angle + 1e-6;
            rotateAroundBisector(u, v, diff, u1, v1);

            //      if (computeAngle(u1, v1)<thetaMin*M_PI/180)
            //      {
            //        std::cerr<<"u = ["<<real(u)<<","<<imag(u)<< "]; v= ["<<real(v)<<","<<imag(v)<<"];"<<std::endl;
            //        std::cerr<<"u1 = ["<<real(u1)<<","<<imag(u1)<< "]; v1= ["<<real(v1)<<","<<imag(v1)<<"];"<<std::endl;
            //        std::cerr<<"current_angle = "<<current_angle<<std::endl;
            //        std::cerr<<"aout = "<<computeAngle(u1, v1)<< "; theta= "<<thetaMin*M_PI/180<<";"<<std::endl;
            //      }
            //      assert(computeAngle(u1, v1)>=thetaMin*M_PI/180);


            pvU.row(fid) << real(u1), imag(u1);
            pvV.row(fid) << real(v1), imag(v1);
        }
    }

}


//
//void AngleBoundFFSolverEigen::computeAngles(Matrix<double, Dynamic, 1> &angles)
//{
//  angles.resize(data.numF,1);
//  for (int i =0; i<data.numF; ++i)
//  {
//    complex<double> u(pvU(i,0),pvU(i,1));
//    complex<double> v(pvV(i,0),pvV(i,1));
//    angles[i] = fabs(arg(u) - arg(v));
//    if (angles[i]>M_PI)
//      angles[i] = 2*M_PI-angles[i];
//    if (angles[i]>.5*M_PI)
//      angles[i] = M_PI-angles[i];
//  }
//}

double AngleBoundFFSolverEigen::computeAngle(const complex<double> &u,
    const complex<double> &v)
{
    double angle = std::min(fabs(arg(u*conj(v))), fabs(arg(u*conj(-v))));

    //  double angle;
    //  double a1 = fabs(arg(u*conj(v)));
    //  double a2 = fabs(arg(u*conj(-v)));
    //  if (a1 < a2)
    //    angle = a1;
    //  else
    //  {
    //    angle = a2; v = -v;
    //  }

    //  double angle = fabs(arg(u) - arg(v));
    //  if (angle>M_PI)
    //  {
    //    u = -u;
    //    angle = fabs(arg(u) - arg(v));
    //  };
    //
    //  if (angle>.5*M_PI)
    //  {
    //    v = -v;
    //    angle = fabs(arg(u) - arg(v));
    //  };
    //
    //  assert(fabs(angle-angle1)<1e-6);

    //  if (angle>M_PI)
    //    angle = 2*M_PI-angle;
    //  if (angle>.5*M_PI)
    //    angle = M_PI-angle;

    //  double angle = fabs(arg(u) - arg(v));
    //    if (angle>M_PI)
    //      angle = 2*M_PI-angle;
    //    if (angle>.5*M_PI)
    //      angle = M_PI-angle;

    assert(angle <= .5*M_PI && angle > 0);

    return angle;
}


int AngleBoundFFSolverEigen::getNumOutOfBounds()
{
    Matrix<double, Dynamic, 1> angles;
    //  computeAngles(angles);
    int numOoB = 0;
    for (int i = 0; i < data.numF; ++i)
    {
        complex<double> u(pvU(i, 0), pvU(i, 1));
        complex<double> v(pvV(i, 0), pvV(i, 1));
        double angle = computeAngle(u, v);
        //    if (angles[i] <thetaMin*M_PI/180)
        if (angle < thetaMin*M_PI / 180)
            numOoB++;
    }
    return numOoB;
}

void AngleBoundFFSolverEigen::setCoefficientsFromField()
{
    for (int i = 0; i < data.numF; ++i)
    {
        complex<double> u(pvU(i, 0), pvU(i, 1));
        complex<double> v(pvV(i, 0), pvV(i, 1));
        Acoeff(i) = u * u + v * v;
        Bcoeff(i) = u * u*v*v;
    }
}


void AngleBoundFFSolverEigen::globalStep(const Is& isConstrained,
    const Matrix<complex<double>, Dynamic, 1>  &Ak,
    const Matrix<complex<double>, Dynamic, 1>  &Bk)
{
    setCoefficientsFromField();

    SparseMatrix<complex<double> > I;
    igl::speye(data.numF, data.numF, I);
    SparseMatrix<complex<double> > QA = data.DDA + lambda * I;
    SparseMatrix<complex<double> > fA = (-2 * lambda*Acoeff).sparseView();

    SparseMatrix<complex<double> > QB = data.DDB + lambda * I;
    SparseMatrix<complex<double> > fB = (-2 * lambda*I*Bcoeff).sparseView();

    if (doHardConstraints)
    {
        minQuadWithKnownMini(QA, fA, isConstrained, Ak, Acoeff);
        minQuadWithKnownMini(QB, fB, isConstrained, Bk, Bcoeff);
    }
    else
    {
        Is isknown_; 
        isknown_.setZero(data.numF);
        Matrix<complex<double>, Dynamic, 1> xknown_; xknown_.setZero(0, 1);
        minQuadWithKnownMini(QA, fA, isknown_, xknown_, Acoeff);
        minQuadWithKnownMini(QB, fB, isknown_, xknown_, Bcoeff);
    }
    setFieldFromCoefficients();

}


void AngleBoundFFSolverEigen::setFieldFromCoefficients()
{
    for (int i = 0; i < data.numF; ++i)
    {
        //    poly coefficients: 1, 0, -Acoeff, 0, Bcoeff
        //    matlab code from roots (given there are no trailing zeros in the polynomial coefficients)
        Matrix<complex<double>, Dynamic, 1> polyCoeff(5, 1);
        polyCoeff << 1., 0., -Acoeff(i), 0., Bcoeff(i);

        Matrix<complex<double>, Dynamic, 1> roots;
        igl::polyRoots<complex<double>>(polyCoeff, roots);

        complex<double> u = roots[0];
        int maxi = -1;
        float maxd = -1;
        for (int k = 1; k < 4; ++k)
        {
            float dist = abs(roots[k] + u);
            if (dist > maxd)
            {
                maxd = dist;
                maxi = k;
            }
        }
        complex<double> v = roots[maxi];
        pvU(i, 0) = real(u); pvU(i, 1) = imag(u);
        pvV(i, 0) = real(v); pvV(i, 1) = imag(v);
    }

}

void AngleBoundFFSolverEigen::minQuadWithKnownMini(const SparseMatrix<complex<double> > &Q,
    const SparseMatrix<complex<double> > &f,
    const Is isConstrained,
    const Matrix<complex<double>, Dynamic, 1> &xknown,
    Matrix<complex<double>, Dynamic, 1> &x)
{
    int N = Q.rows();

    int nc = xknown.rows();
    VectorXi known; known.setZero(nc, 1);
    VectorXi unknown; unknown.setZero(N - nc, 1);

    int indk = 0, indu = 0;
    for (int i = 0; i < N; ++i)
        if (isConstrained[i])
        {
            known[indk] = i;
            indk++;
        }
        else
        {
            unknown[indu] = i;
            indu++;
        }

    SparseMatrix<complex<double>> Quu, Quk;

    igl::slice(Q, unknown, unknown, Quu);
    igl::slice(Q, unknown, known, Quk);


    std::vector<typename Triplet<complex<double> > > tripletList;

    SparseMatrix<complex<double> > fu(N - nc, 1);

    igl::slice(f, unknown, VectorXi::Zero(1, 1), fu);

    SparseMatrix<complex<double> > rhs = (Quk*xknown).sparseView() + .5*fu;

    SparseLU< SparseMatrix<complex<double>>> solver;
    solver.compute(-Quu);
    if (solver.info() != Success)
    {
        std::cerr << "Decomposition failed!" << std::endl;
        return;
    }
    SparseMatrix<complex<double>>  b = solver.solve(rhs);
    if (solver.info() != Success)
    {
        std::cerr << "Solving failed!" << std::endl;
        return;
    }

    indk = 0, indu = 0;
    x.setZero(N, 1);
    for (int i = 0; i < N; ++i)
        if (isConstrained[i])
            x[i] = xknown[indk++];
        else
            x[i] = b.coeff(indu++, 0);

}


bool AngleBoundFFSolverEigen::solve(const Is& isConstrained,
    const MatrixXd &initialSolution,
    MatrixXd &output,
    double *lambdaOut)
{
    int numConstrained = isConstrained.sum();
    // coefficient values
    Matrix<complex<double>, Dynamic, 1> Ak, Bk;

    pvU.resize(data.numF, 2);
    pvV.resize(data.numF, 2);
    for (int fi = 0; fi < data.numF; ++fi)
    {
        const Matrix<double, 1, 3> &b1 = data.B1.row(fi);
        const Matrix<double, 1, 3> &b2 = data.B2.row(fi);
        const Matrix<double, 1, 3> &u3 = initialSolution.block(fi, 0, 1, 3);
        const Matrix<double, 1, 3> &v3 = initialSolution.block(fi, 3, 1, 3);
        pvU.row(fi) << u3.dot(b1), u3.dot(b2);
        pvV.row(fi) << v3.dot(b1), v3.dot(b2);
    }
    setCoefficientsFromField();
    Ak.resize(numConstrained, 1);
    Bk.resize(numConstrained, 1);
    int ind = 0;
    for (int i = 0; i < data.numF; ++i)
    {
        if (isConstrained[i])
        {
            Ak(ind) = Acoeff[i];
            Bk(ind) = Bcoeff[i];
            ind++;
        }
    }



    double smoothnessValue;
    int oob;

    smoothnessValue = (Acoeff.adjoint()*data.DDA*Acoeff + Bcoeff.adjoint()*data.DDB*Bcoeff).real()[0];
    printf("\n\nInitial smoothness: %.5g\n", smoothnessValue);
    oob = getNumOutOfBounds();
    printf("\n\nInitial out-of-bounds: %d\n", oob);
    printf(" %d %.5g %d\n", -1, smoothnessValue, oob);

    lambda = lambdaInit;
    for (int iter = 0; iter < maxIter; ++iter)
    {
        printf("\n\n--- Iteration %d ---\n", iter);

        localStep();
        globalStep(isConstrained, Ak, Bk);


        smoothnessValue = (Acoeff.adjoint()*data.DDA*Acoeff + Bcoeff.adjoint()*data.DDB*Bcoeff).real()[0];

        printf("Smoothness: %.5g\n", smoothnessValue);

        oob = getNumOutOfBounds();

        bool stoppingCriterion = (oob == 0);
        if (stoppingCriterion)
            break;
        lambda = lambda * lambdaMultFactor;
        //    printf(" %d %.5g %d\n",iter, smoothnessValue, oob);

    }

    output.setZero(data.numF, 6);
    for (int fi = 0; fi < data.numF; ++fi)
    {
        const Matrix<double, 1, 3> &b1 = data.B1.row(fi);
        const Matrix<double, 1, 3> &b2 = data.B2.row(fi);
        output.block(fi, 0, 1, 3) = pvU(fi, 0)*b1 + pvU(fi, 1)*b2;
        output.block(fi, 3, 1, 3) = pvV(fi, 0)*b1 + pvV(fi, 1)*b2;
    }

    if (lambdaOut)
        *lambdaOut = lambda;


    return (oob == 0);
}

bool AngleBoundFFSolverEigen::solve(const Bs& isConstrained,
    const vector<V3s>& initialSolution,
    vector<V3s>& output,
    double *lambdaOut)
{
    output.clear();
    int n = initialSolution.size();
    if (n == 0) return false;
    
    MatrixXd initialSolutionEigen;
    int numF = initialSolution[0].rows();
    initialSolutionEigen.setZero(numF, 3 * n);
    for (int fi = 0; fi < numF; ++fi)
    {
        for (int ni = 0; ni < n; ++ni)
        {
            initialSolutionEigen.block<1, 3>(fi, 3 * ni) = convertV3ToEigenDouble(initialSolution[ni].row(fi));
        }
    }
    Is isConstrainedEigen;
    isConstrained.ToIndexes(isConstrainedEigen);
    MatrixXd outputEigen;
    bool result = solve(isConstrainedEigen, initialSolutionEigen, outputEigen, lambdaOut);
    
    output.resize(n);
    for (int ni = 0; ni < n; ++ni) output[ni].resize(numF, 3);

    for(int fi = 0; fi < numF; ++fi)
    {
        Vector3d x_fixedAngleEigen = outputEigen.block<1, 3>(fi, 3 * 0).transpose();
        Vector3d y_fixedAngleEigen = outputEigen.block<1, 3>(fi, 3 * 1).transpose();
        V3 x_fixedAngle = convertEigenToV3(x_fixedAngleEigen);
        V3 y_fixedAngle = convertEigenToV3(y_fixedAngleEigen);

        if (!isConstrained(fi))
        {
            output[0].row(fi) = x_fixedAngle;
            output[1].row(fi) = y_fixedAngle;
            continue;
        }

        //  constrained faces must have same directions as initial
        V3 x = initialSolution[0].row(fi);
        V3 y = initialSolution[1].row(fi);
        if (abs(utils::vector::Dot(x.normalized(), x_fixedAngle.normalized())) < 0.5)
        {
            swap(x_fixedAngle, y_fixedAngle);// exchange x with y
        }
        if (!meshLogicOptions.SolverAngleBound.leave_x_original)
        {
            output[0].row(fi) = utils::vector::Dot(x, x_fixedAngle) > 0 ? x_fixedAngle : -x_fixedAngle;
        }
        output[1].row(fi) = utils::vector::Dot(y, y_fixedAngle) > 0 ? y_fixedAngle : -y_fixedAngle;
    }

    return result;
}


bool angle_bound_frame_fields_eigen(const MatrixXd &V, const I3s& F,
    const Is& isConstrained, const MatrixXd &initialSolution,
    int maxIter,
    double thetaMin,
    double lambdaInit,
    double lambdaMultFactor,
    bool doHardConstraints,
    MatrixXd& output)
{
    AngleBoundFFSolverEigenData csdata(V, F);
    AngleBoundFFSolverEigen cs(csdata, thetaMin, maxIter, lambdaInit, lambdaMultFactor, doHardConstraints);
    return cs.solve(isConstrained, initialSolution, output);
}

bool angle_bound_frame_fields_eigen(const P3s &V, const I3s& F,
    const Bs& isConstrained, const vector<V3s> &initialSolution,
    int maxIter,
    double thetaMin,
    double lambdaInit,
    double lambdaMultFactor,
    bool doHardConstraints,
    vector<V3s>& output)
{
    AngleBoundFFSolverEigenData csdata(V, F);
    AngleBoundFFSolverEigen cs(csdata, thetaMin, maxIter, lambdaInit, lambdaMultFactor, doHardConstraints);
    return cs.solve(isConstrained, initialSolution, output);
}