#include "stdafx.h"
#include "AngleBoundFFSolver_V3.h"
#include <igl/speye.h>
#include <igl/slice.h>
#include <igl/polyroots.h>
#include "SolverTypes_Polar2x.h"


#define options meshLogicOptions.SolverAngleBound
#define elapsed elapsedTimers.SolverAngleBound


void AngleBoundFFSolver_V3::computeLaplacians_n(int n, SparseMatrix<complex<D> > &coeff)
{
    std::vector<Triplet<complex<D> >> tripletList;

    // For every non-border edge
    for (int eid = 0; eid < numE; ++eid)
    {
        if (!E_isborder[eid]) 
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
                -1.*std::polar(1., -1.*n*K[eid])));
            tripletList.push_back(Triplet<complex<D> >(fid1,
                fid0,
                -1.*std::polar(1., 1.*n*K[eid])));

        }
    }
    coeff.resize(numF, numF);
    coeff.setFromTriplets(tripletList.begin(), tripletList.end());
}

void AngleBoundFFSolver_V3::computeLaplacians()
{
    computeLaplacians_n(2, DDA);
    computeLaplacians_n(4, DDB);
}



/***************************** Solver ***********************************/

AngleBoundFFSolver_V3::AngleBoundFFSolver_V3(const P3s &_V, const I3s& _F, const I2s& _EV, const I3s& _FE, const I2s& _EF,
    const V3s& _FX, const V3s& _FY, const V3s& _FN, const Ds& _K, const Bs& _E_isborder,
    int _maxIter,
    D _thetaMin,
    D _lambdaInit,
    D _lambdaMultFactor,
    bool _doHardConstraints) :
    V(_V),
    F(_F),
    EV(_EV),
    FE(_FE),
    EF(_EF),
    FX(_FX),
    FY(_FY),
    FN(_FN),
    K(_K),
    E_isborder(_E_isborder),
    numV(_V.rows()),
    numF(_F.rows()),
    numE(_EV.rows()),
    maxIter(_maxIter),
    thetaMin(_thetaMin),
    lambdaInit(_lambdaInit),
    lambdaMultFactor(_lambdaMultFactor),
    doHardConstraints(_doHardConstraints)
{
    computeLaplacians();

    Acoeff.resize(numF, 1);
    Bcoeff.resize(numF, 1);
    pvU.setZero(numF, 2);
    pvV.setZero(numF, 2);
};


void AngleBoundFFSolver_V3::rotateAroundBisector(const complex<D> &uin,
    const complex<D> &vin,
    const D diff,
    complex<D> &uout,
    complex<D> &vout)
{
    //rotate 2D complex vectors u and v around their bisector so that their
    //angle is at least theta

    uout = uin;
    vout = vin;
    D au = arg(uin);
    D av = arg(vin);
    if (au < av)
    {
        uout = std::polar((D)1.0, (D)-0.5*diff)*uin;
        vout = std::polar((D)1.0, (D)0.5*diff)*vin;
    }
    else
    {
        uout = std::polar((D)1.0, (D)0.5*diff)*uin;
        vout = std::polar((D)1.0, (D)-0.5*diff)*vin;
    }

}



void AngleBoundFFSolver_V3::localStep()
{
    for (int fid = 0; fid < numF; ++fid)
    {

        complex<D> u(pvU(fid, 0), pvU(fid, 1));
        complex<D> v(pvV(fid, 0), pvV(fid, 1));

        D current_angle = computeAngle(u, v); 


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

            complex<D> u1, v1;
            D diff = thetaMin * M_PI / 180 - current_angle + 1e-6;
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
//
//void AngleBoundFFSolver_V3::
//computeAngles(Matrix<D, Dynamic, 1> &angles)
//{
//  angles.resize(numF,1);
//  for (int i =0; i<numF; ++i)
//  {
//    complex<D> u(pvU(i,0),pvU(i,1));
//    complex<D> v(pvV(i,0),pvV(i,1));
//    angles[i] = fabs(arg(u) - arg(v));
//    if (angles[i]>M_PI)
//      angles[i] = 2*M_PI-angles[i];
//    if (angles[i]>.5*M_PI)
//      angles[i] = M_PI-angles[i];
//  }
//}


D AngleBoundFFSolver_V3::computeAngle(const complex<D> &u, const complex<D> &v)
{
    D angle = std::min(fabs(arg(u*conj(v))), fabs(arg(u*conj(-v))));

    //  D angle;
    //  D a1 = fabs(arg(u*conj(v)));
    //  D a2 = fabs(arg(u*conj(-v)));
    //  if (a1 < a2)
    //    angle = a1;
    //  else
    //  {
    //    angle = a2; v = -v;
    //  }

    //  D angle = fabs(arg(u) - arg(v));
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

    //  D angle = fabs(arg(u) - arg(v));
    //    if (angle>M_PI)
    //      angle = 2*M_PI-angle;
    //    if (angle>.5*M_PI)
    //      angle = M_PI-angle;

    constexpr D angleMax = (D)(0.5*M_PI);
    assert(angle <= angleMax && angle > 0);

    return angle;
}



int AngleBoundFFSolver_V3::getNumOutOfBounds()
{
    Matrix<D, Dynamic, 1> angles;
    //  computeAngles(angles);
    int numOoB = 0;
    for (int i = 0; i < numF; ++i)
    {
        complex<D> u(pvU(i, 0), pvU(i, 1));
        complex<D> v(pvV(i, 0), pvV(i, 1));
        D angle = computeAngle(u, v);
        //    if (angles[i] <thetaMin*M_PI/180)
        if (angle < thetaMin*M_PI / 180)
            numOoB++;
    }
    return numOoB;
}


void AngleBoundFFSolver_V3::setCoefficientsFromField()
{
    for (int i = 0; i < numF; ++i)
    {
        complex<D> u(pvU(i, 0), pvU(i, 1));
        complex<D> v(pvV(i, 0), pvV(i, 1));
        Acoeff(i) = u * u + v * v;
        Bcoeff(i) = u * u*v*v;
    }
}

void AngleBoundFFSolver_V3::globalStep(const Bs& isConstrained,
    const Matrix<complex<D>, Dynamic, 1>  &Ak,
    const Matrix<complex<D>, Dynamic, 1>  &Bk)
{
    setCoefficientsFromField();

    SparseMatrix<complex<D> > I;
    igl::speye(numF, numF, I);
    SparseMatrix<complex<D> > QA = DDA + lambda * I;
    SparseMatrix<complex<D> > fA = (-2 * lambda*Acoeff).sparseView();

    SparseMatrix<complex<D> > QB = DDB + lambda * I;
    SparseMatrix<complex<D> > fB = (-2 * lambda*I*Bcoeff).sparseView();

    if (doHardConstraints)
    {
        minQuadWithKnownMini(QA, fA, isConstrained, Ak, Acoeff);
        minQuadWithKnownMini(QB, fB, isConstrained, Bk, Bcoeff);
    }
    else
    {
        Bs isknown_;
        Matrix<complex<D>, Dynamic, 1> xknown_;
        isknown_.setZero(numF);
        xknown_.setZero(0, 1);
        minQuadWithKnownMini(QA, fA, isknown_, xknown_, Acoeff);
        minQuadWithKnownMini(QB, fB, isknown_, xknown_, Bcoeff);
    }
    setFieldFromCoefficients();

}



void AngleBoundFFSolver_V3::setFieldFromCoefficients()
{
    for (int i = 0; i < numF; ++i)
    {
        //    poly coefficients: 1, 0, -Acoeff, 0, Bcoeff
        //    matlab code from roots (given there are no trailing zeros in the polynomial coefficients)
        Matrix<complex<D>, Dynamic, 1> polyCoeff(5, 1);
        polyCoeff << 1., 0., -Acoeff(i), 0., Bcoeff(i);

        Matrix<complex<D>, Dynamic, 1> roots;
        igl::polyRoots<complex<D>>(polyCoeff, roots);

        complex<D> u = roots[0];
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
        complex<D> v = roots[maxi];
        pvU(i, 0) = real(u); 
        pvU(i, 1) = imag(u);
        pvV(i, 0) = real(v); 
        pvV(i, 1) = imag(v);
    }

}


void AngleBoundFFSolver_V3::minQuadWithKnownMini(const SparseMatrix<complex<D> > &Q,
    const SparseMatrix<complex<D> > &f,
    const Bs& isConstrained,
    const Matrix<complex<D>, Dynamic, 1> &xknown,
    Matrix<complex<D>, Dynamic, 1> &x)
{
    int count = Q.rows();

    int count_known = xknown.rows();
    VectorXi known; 
    VectorXi unknown; 
    known.setZero(count_known, 1);
    unknown.setZero(count - count_known, 1);

    int indk = 0, indu = 0;
    for (int i = 0; i < count; ++i)
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

    SparseMatrix<complex<D>> Quu, Quk;

    igl::slice(Q, unknown, unknown, Quu);
    igl::slice(Q, unknown, known, Quk);


    std::vector<typename Triplet<complex<D> > > tripletList;

    SparseMatrix<complex<D> > fu(count - count_known, 1);

    igl::slice(f, unknown, VectorXi::Zero(1, 1), fu);
     
    SparseMatrix<complex<D> > rhs = (Quk*xknown).sparseView() + .5*fu;

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

    indk = 0, indu = 0;
    x.setZero(count, 1);
    for (int i = 0; i < count; ++i)
        if (isConstrained[i])
            x[i] = xknown[indk++];
        else
            x[i] = b.coeff(indu++, 0);

}



bool AngleBoundFFSolver_V3::solve(const Bs& isConstrained,
    const vector<V3s> &initialSolution,
    vector<V3s> &output, bool logMessages)
{
    if (options.DebugEnabled && options.debug_logMessages)
    {
        logMessages = true;
    }
    int numConstrained = isConstrained.countOfValues(true);
    // coefficient values
    Matrix<complex<D>, Dynamic, 1> Ak, Bk;

    pvU.resize(numF, 2);
    pvV.resize(numF, 2);
    for (int fid = 0; fid < numF; fid++)
    {
        const V3 &b1 = FX.row(fid);
        const V3 &b2 = FY.row(fid);
        const V3 &u3 = initialSolution[0].row(fid);
        const V3 &v3 = initialSolution[1].row(fid);
        pvU.row(fid) << u3.dot(b1), u3.dot(b2);
        pvV.row(fid) << v3.dot(b1), v3.dot(b2);
    }
    setCoefficientsFromField();
    Ak.resize(numConstrained, 1);
    Bk.resize(numConstrained, 1);
    int ind = 0;
    for (int fid = 0; fid < numF; fid++)
    {
        if (isConstrained[fid])
        {
            Ak(ind) = Acoeff[fid];
            Bk(ind) = Bcoeff[fid];
            ind++;
        }
    }



    D smoothnessValue = (Acoeff.adjoint()*DDA*Acoeff + Bcoeff.adjoint()*DDB*Bcoeff).real()[0];
    if (logMessages) printf("\n\nInitial smoothness: %.5g\n", smoothnessValue);
    int oob = getNumOutOfBounds();
    if (logMessages) printf("Initial out-of-bounds: %d\n", oob);
    //printf(" %d %.5g %d\n", -1, smoothnessValue, oob);

    lambda = lambdaInit;
    for (int iter = 0; iter < maxIter; ++iter)
    {
        if (logMessages) printf("\n--- Iteration %d ---\n", iter);

        localStep();
        globalStep(isConstrained, Ak, Bk);


        smoothnessValue = (Acoeff.adjoint()*DDA*Acoeff + Bcoeff.adjoint()*DDB*Bcoeff).real()[0];

        if (logMessages) printf("  smoothness: %.5g\n", smoothnessValue);

        oob = getNumOutOfBounds();
        if (logMessages) printf("  out-of-bounds: %d\n", oob);

        bool stoppingCriterion = (oob == 0);
        if (stoppingCriterion)
        {
            break;
        }
        lambda = lambda * lambdaMultFactor;
        //    printf(" %d %.5g %d\n",iter, smoothnessValue, oob);

    }

    if (output.size() != 2) output.resize(2);
    output[0].resize(numF, 3);
    output[1].resize(numF, 3);
    for (int fi = 0; fi < numF; ++fi)
    {
        const V3 &b1 = FX.row(fi);
        const V3 &b2 = FY.row(fi);
        output[0].row(fi) = pvU(fi, 0)*b1 + pvU(fi, 1)*b2;
        output[1].row(fi) = pvV(fi, 0)*b1 + pvV(fi, 1)*b2;
    }

    return (oob == 0);
}
