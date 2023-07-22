#include "stdafx.h"
#include "AngleBoundFFSolver_Polar.h"
#include "SolverTypes_Polar2x.h"
#include "LinearEquationSolver_Polar2x.h"

#define options meshLogicOptions.SolverAngleBound
#define elapsed elapsedTimers.SolverAngleBound

AngleBoundFFSolver_Polar::AngleBoundFFSolver_Polar(ViewerDrawObjects& _draw, const P3s &_V, const I3s& _F, const I2s& _EV, const I3s& _FE, const I2s& _EF,
    const V3s& _FX, const V3s& _FY, const V3s& _FN, const Ds& _K, const Bs& _E_isborder, D _max_edge_length, const Ds& _E_Length,
    const Is& _known_index_to_fid, const Is& _unknown_index_to_fid, const SparceMatrixType& _Quu, const SparceMatrixTypeComplex& _Quk,
    int _maxIter,
    D _thetaMin,
    D _lambdaInit,
    D _lambdaMultFactor,
    bool _doHardConstraints) :
    draw(_draw),
    solver(draw),
    solver_reuse_ordering(false),
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
    max_edge_length(_max_edge_length),
    E_Length(_E_Length),
    borderconstrains__Qu(_Quu),
    borderconstrains__Qk(_Quk),
    borderconstrains__known_index_to_fid(_known_index_to_fid),
    borderconstrains__unknown_index_to_fid(_unknown_index_to_fid),
    numV(_V.rows()),
    numF(_F.rows()),
    numE(_EV.rows()),
    lambda(0),
    maxIter(_maxIter),
    thetaMin(_thetaMin),
    lambdaInit(_lambdaInit),
    lambdaMultFactor(_lambdaMultFactor),
    doHardConstraints(_doHardConstraints),
    is_computeLaplacians_calculated(false)
{
    coeff.resize(numF);
    pv.setZero(numF);
};


/*void AngleBoundFFSolver_Polar::computeLaplacians()
{
    std::vector<Triplet<Scalar>> tripletList;

    // For every non-border edge
    for (unsigned eid = 0; eid < numE; eid++)
    {
        if (!E_isborder[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            tripletList.push_back(Triplet<Scalar>(fid0, fid0, Scalar((D)1)));
            tripletList.push_back(Triplet<Scalar>(fid1, fid1, Scalar((D)1)));
            tripletList.push_back(Triplet<Scalar>(fid0, fid1, Scalar(-1.*std::polar(1., -1. * 2 * K[eid]), -1.*std::polar(1., -1. * 4 * K[eid]))));
            tripletList.push_back(Triplet<Scalar>(fid1, fid0, Scalar(-1.*std::polar(1., 1. * 2 * K[eid]), -1.*std::polar(1., 1. * 4 * K[eid]))));
        }
    }
    dd.resize(numF, numF);
    dd.setFromTriplets(tripletList.begin(), tripletList.end());
}*/

void AngleBoundFFSolver_Polar::rotateAroundBisector(const complex<D> &u, const complex<D> &v, D arg_u, D arg_v,
    const D diff, complex<D> &u_out, complex<D> &v_out)
{
    //rotate 2D complex vectors u and v around their bisector so that their
    //angle is at least theta

    u_out = u;
    v_out = v;
    if (arg_u < arg_v)
    {
        u_out = std::polar((D)1.0, (D)-0.5*diff)*u;
        v_out = std::polar((D)1.0, (D)0.5*diff)*v;
    }
    else
    {
        u_out = std::polar((D)1.0, (D)0.5*diff)*u;
        v_out = std::polar((D)1.0, (D)-0.5*diff)*v;
    }

}



void AngleBoundFFSolver_Polar::localStep()
{
    for (int fid = 0; fid < numF; fid++)
    {
        //complex<D> u(pvU(fid, 0), pvU(fid, 1));
        //complex<D> v(pvV(fid, 0), pvV(fid, 1));
        complex<D> u(pv(fid).get_low().toComplex());
        complex<D> v(pv(fid).get_high().toComplex());

        D current_angle = computeAngle(u, v);

        // TODO we can cache arg(u) and arg(v) to optimize speed by 8% 
        if (current_angle < thetaMin*M_PI / 180)
        {
            D arg_u = arg(u);
            D arg_v = arg(v);
            // bring all to 1st or 4th quarter plane
            if (arg_u >= 0.5*M_PI || arg_u < -0.5*M_PI)
            {
                u = -u;
                arg_u = arg(u);
            }
            if (arg_v >= 0.5*M_PI || arg_v < -0.5*M_PI)
            {
                v = -v;
                arg_v = arg(v);
            }
            assert(fabs(computeAngle(u, v) - current_angle) < 1e-5);

            if (fabs(arg_u - arg_v) > 0.5*M_PI)
            {
                v = -v;
                arg_v = arg(v);
            }
            assert(fabs(computeAngle(u, v) - current_angle) < 1e-5);

            complex<D> u1, v1;
            D diff = thetaMin * M_PI / 180 - current_angle + 1e-6;
            rotateAroundBisector(u, v, arg_u, arg_v, diff, u1, v1);

            //      if (computeAngle(u1, v1)<thetaMin*M_PI/180)
            //      {
            //        std::cerr<<"u = ["<<real(u)<<","<<imag(u)<< "]; v= ["<<real(v)<<","<<imag(v)<<"];"<<std::endl;
            //        std::cerr<<"u1 = ["<<real(u1)<<","<<imag(u1)<< "]; v1= ["<<real(v1)<<","<<imag(v1)<<"];"<<std::endl;
            //        std::cerr<<"current_angle = "<<current_angle<<std::endl;
            //        std::cerr<<"aout = "<<computeAngle(u1, v1)<< "; theta= "<<thetaMin*M_PI/180<<";"<<std::endl;
            //      }
            //      assert(computeAngle(u1, v1)>=thetaMin*M_PI/180);

            pv(fid) = Scalar(u1, v1);
        }
    }

}


//
//
//void AngleBoundFFSolver_Polar::
//computeAngles(Matrix<D, Dynamic, 1> &angles)
//{
//  angles.resize(numF,1);
//  for (int i =0; i<numF; i++)
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


D AngleBoundFFSolver_Polar::computeAngle(const complex<D> &u, const complex<D> &v)
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



int AngleBoundFFSolver_Polar::getNumOutOfBounds()
{
    Matrix<D, Dynamic, 1> angles;
    //  computeAngles(angles);
    int numOoB = 0;
    for (int fid = 0; fid < numF; fid++)
    {
        //complex<D> u(pvU(fid, 0), pvU(fid, 1));
        //complex<D> v(pvV(fid, 0), pvV(fid, 1));
        complex<D> u(pv(fid).get_low().toComplex());
        complex<D> v(pv(fid).get_high().toComplex());
        D angle = computeAngle(u, v);
        //    if (angles[i] <thetaMin*M_PI/180)
        if (angle < thetaMin*M_PI / 180)
            numOoB++;
    }
    return numOoB;
}


void AngleBoundFFSolver_Polar::setCoefficientsFromField()
{
    Timer timer_getGeneralCoeffConstraints;
    for (int fid = 0; fid < numF; fid++)
    {
        //complex<D> u(pvU(fid, 0), pvU(fid, 1));
        //complex<D> v(pvV(fid, 0), pvV(fid, 1));
        //Acoeff(fid) = u * u + v * v;
        //Bcoeff(fid) = u * u*v*v;
        auto u = pv(fid).get_low();
        auto v = pv(fid).get_high();
        auto uu = u * u;
        auto vv = v * v;
        coeff(fid) = Scalar(uu + vv, uu*vv);
    }
    timer_getGeneralCoeffConstraints.stop(elapsed.getGeneralCoeffConstraints);
}

void AngleBoundFFSolver_Polar::compute_known_unknown_indexes(const Bs& isConstrained, Is& known, Is& unknown)
{
    int faces_count = isConstrained.size();
    known.setZero(faces_count);
    unknown.setZero(faces_count);
    int indk = 0;
    int indu = 0;
    for (int fid = 0; fid < faces_count; fid++)
    {
        if (isConstrained[fid])
        {
            known[indk] = fid;
            indk++;
        }
        else
        {
            unknown[indu] = fid;
            indu++;
        }
    }
    known.conservativeResize(indk);
    unknown.conservativeResize(indu);
}


void speye(const int m, const int n, SparceMatrixType & I, D value)
{
    // size of diagonal
    int d = (m < n ? m : n);
    I = SparceMatrixType(m, n);
    I.reserve(d);
    for (int i = 0; i < d; i++)
    {
        I.insert(i, i) = value;
    }
    I.finalize();
}
SparceMatrixType sparseView(VectorType &values, D mult)
{
    // size of diagonal
    SparceMatrixType sparse = SparceMatrixType(values.rows(), 1);
    sparse.reserve(values.rows());
    for (int row = 0; row < values.rows(); row++)
    {
        sparse.insert(row, 0) = values[row] * mult;
    }
    sparse.finalize();
    return sparse;
}

void AngleBoundFFSolver_Polar::globalStep(const Bs& isConstrained, const VectorType& AB, bool logMessages)
{
    setCoefficientsFromField();

    D mult(-2 * lambda);
    //VectorType ff = coeff * Scalar(mult);
    VectorType f;
    f.resize(coeff.rows());
    for (int i = 0; i < coeff.rows(); i++) f[i] = coeff[i] * mult;


    if (doHardConstraints)
    {
        //TODO add lambda to Quu 
        minQuadWithKnownMini(borderconstrains__Qu, borderconstrains__Qk,
            isConstrained, borderconstrains__known_index_to_fid, borderconstrains__unknown_index_to_fid,
            AB, f, coeff, logMessages);
    }
    else
    {
        SparceMatrixType lambdaI;
        speye(numF, numF, lambdaI, lambda);
        SparceMatrixType Q = dd + lambdaI;

        Bs isConstrained_none = Bs::Zero(numF);
        VectorType xknown_none = VectorType::Zero(0, 1);
        minQuadWithKnownMini(Q, noneconstrains__Qk,
            isConstrained_none, noneconstrains__known_index_to_fid, noneconstrains__unknown_index_to_fid,
            xknown_none, f, coeff, logMessages);
    }

    Timer timer_setFieldFromGeneralCoefficients;
    setFieldFromCoefficients(coeff, pv);
    timer_setFieldFromGeneralCoefficients.stop(elapsed.setFieldFromGeneralCoefficients);
}

void AngleBoundFFSolver_Polar::setFieldFromCoefficients(const  VectorType &coeffs, VectorType &pv)
{
    const int facesCount = coeffs.rows();
    pv.resize(facesCount);
    bool sort = true;// options.sort_complex_vectors;
    for (int fid = 0; fid < facesCount; fid++)
    {
        RealScalar d1 = coeffs(fid).get_low();
        RealScalar d2 = -coeffs(fid).get_high();

        //
        // v3 - direct calculation of 2x2 matrix
        //                                                                      |x  y|
        // Eigen value of 2x2 matrix defined by equation      |1 0|*a=0
        // from this equation we will get quadric equation (a-x)*a - y = 0
        // solving this equation we will find solution for a = (x +- sqrt(x^2 + 4y))/2
        //
        //                                             |a-x  y|   |x1|
        // Solving equation of 2x2 matrix  |-1   a | * |x2| = 0   we can find a solutions: 
        // solution 1:   x1 = x2*a,  where   a = (x+sqrt(x^2 + 4y))/2
        // solution 2:   x1 = x2*a,  where   a = (x-sqrt(x^2 + 4y))/2
        // so, solutions differs only by '+' and '-' in 'a' calculus
        // we can assume that x2 == 1, then solution will be x1 = a
        // and since 'a' of 2x2 will be equal to sqrt('a' of 4x4), so final solution will be:
        // eigen values1: sqrt((x+sqrt(x^2 + 4y))/2)
        // eigen values2: sqrt((x-sqrt(x^2 + 4y))/2)
        //
        //Scalar sqrt_det = sqrt(d1*d1 + 4.0*d2);
        //Scalar a1_2x2 = (d1 + sqrt_det)*0.5;
        //Scalar a2_2x2 = (d1 - sqrt_det)*0.5;
        //Scalar r0fast = sqrt(a1_2x2); // convert 'a' of 2x2 to 'a' of 4x4
        //Scalar r1fast = sqrt(a2_2x2); // convert 'a' of 2x2 to 'a' of 4x4

        //
        // v4 - direct calculation of 2x2 matrix  with fast complex sqrt
        //  
        // we can simplify v3 by dividing x by 2
        // eigen values1: sqrt((x+sqrt(x^2 + 4y))/2)
        // eigen values1: sqrt(x/2+sqrt((x/2)^2 + y))
        d1 *= 0.5;
        //Scalar sqrt_det = csqrt(d1*d1 + d2);

        RealScalar sqrt_det = sqrt(d1*d1 + d2);
        RealScalar a1_2x2 = (d1 + sqrt_det);
        RealScalar a2_2x2 = (d1 - sqrt_det);

        Scalar res = sqrt(Scalar(a1_2x2, a2_2x2));

        //DEBUG - normalize r0fast and r1fast to test agains oldest method
        //RealScalar r0fast = res.get_low();
        //RealScalar r1fast = res.get_high();
        //Matrix<Scalar, 2, 1> roots2fast_norm;
        //roots2fast_norm(0) = r0fast;
        //roots2fast_norm(1) = r1fast;
        //roots2fast_norm.normalize();
        //Scalar r0fast_norm = roots2fast_norm(0);
        //Scalar r1fast_norm = roots2fast_norm(1);
        if (isnan(res[0]))
        {
            cout << "!  warning   setFieldFromGeneralCoefficientsPolar  isnan(res[0])" << endl;
        }

        // sort complex vectors - this is very important for detecting singularities
        if (sort)
        {
            //D x0 = r0fast.real();
            //D y0 = r0fast.imag();
            //D x1 = r1fast.real();
            //D y1 = r1fast.imag();

            //// ensure first vector is on right side (between [-90..90] degree), and second on left
            //if (x0 < 0)
            //{
            //    x0 = -x0;//rotate first vector by 180 degree
            //    y0 = -y0;//rotate first vector by 180 degree
            //    x1 = -x1;//rotate second vector by 180 degree
            //    y1 = -y1;//rotate second vector by 180 degree
            //}

            ////// ensure first vector in first sector (between [0..90] degree)
            ////if (y0 < 0)
            ////{
            ////    if (y1 < 0) // ensure second vector in first sector (between [0..90] degree)
            ////    {
            ////        x1 = -x1;//flip vector
            ////        y1 = -y1;//flip vector
            ////    }
            ////    swap(x0, x1);
            ////    swap(y0, y1);
            ////}

            ////// ensure vectors are sorted
            //if (y1 < 0)
            //{
            //    x1 = -x1;//flip vector
            //    y1 = -y1;//flip vector
            //}

            //if (x1 > x0)
            //{
            //    x1 = -x1;//flip vector
            //    y1 = -y1;//flip vector
            //}
            //

            //    r0fast = Scalar(x0, y0);
            //    r1fast = Scalar(x1, y1);
            //    //if (x0 < 0 || y0 < 0)
            //    //{
            //    //    cout << "r0fast = " << r0fast << "   r1fast = " << r1fast << endl;
            //    //}

            Scalar absPow2 = absPow2fast(res);
            Real r0fastabsPow2 = absPow2.extract(0);
            Real r1fastabsPow2 = absPow2.extract(2);
            if (r1fastabsPow2 < r0fastabsPow2)
            {
                res = Scalar(res.get_high(), res.get_low());
            }
        }

        // return results
        pv(fid) = res;
    }
}


void AngleBoundFFSolver_Polar::minQuadWithKnownMini(const SparceMatrixType& Qu, const SparceMatrixType& Qk,
    const Bs& isConstrained, const Is& known_index_to_fid, const Is& unknown_index_to_fid,
    const VectorType& xknown,
    const VectorType& f,
    VectorType& x,
    bool logMessages)
{
    //Is known_index_to_fid;
    //Is unknown_index_to_fid;
    //compute_known_unknown_indexes(isConstrained, known_index_to_fid, unknown_index_to_fid);
    solver.compute(Qu, logMessages, elapsed.minQuadWithKnownMini_compute, solver_reuse_ordering && options.reuse_ordering);
    solver_reuse_ordering = true; // remember ordering for next iterations
    solver.solve(isConstrained, Qk, xknown, f, x, known_index_to_fid, unknown_index_to_fid, logMessages, elapsed.minQuadWithKnownMini_solve);
}



bool AngleBoundFFSolver_Polar::solve(const Bs& isConstrained,
    const vector<V3s> &initialSolution,
    vector<V3s> &output, bool logMessages, bool solver_reuse_ordering)
{
    this->solver_reuse_ordering = solver_reuse_ordering;
    if (options.DebugEnabled && options.debug_logMessages)
    {
        logMessages = true;
    }
    if (logMessages) printf("reusing ordering from previous solver\n");

    if (!is_computeLaplacians_calculated)
    {
        Timer timer_computeFacesLaplacianCoefficient;
        is_computeLaplacians_calculated = true;
        //v0 - original
        //computeLaplacians();
        //v1 - use fast method
        Bs isConstrained_none = Bs::Zero(numF);
        //TODO we can take some values from  borderconstrains__Qu to optimize speed by 3% 
        computeFacesLaplacianCoefficient_Polar_direct(numF, FE, EF, FN, max_edge_length, E_Length, K, E_isborder, isConstrained_none, 0,
            noneconstrains__Qu, noneconstrains__Qk, noneconstrains__known_index_to_fid, noneconstrains__unknown_index_to_fid);
        timer_computeFacesLaplacianCoefficient.stop(elapsed.computeFacesLaplacianCoefficient);
    }

    int numConstrained = isConstrained.countOfValues(true);
    // coefficient values
    VectorType AB;

    pv.resize(numF);
    for (int fid = 0; fid < numF; fid++)
    {
        const V3 &x = FX.row(fid);
        const V3 &y = FY.row(fid);
        const V3 &u = initialSolution[0].row(fid);
        const V3 &v = initialSolution[1].row(fid);
        pv(fid) = Scalar(u.dot(x), u.dot(y), v.dot(x), v.dot(y));
    }
    setCoefficientsFromField();
    AB.resize(numConstrained);
    int ind = 0;
    for (int fid = 0; fid < numF; fid++)
    {
        if (isConstrained[fid])
        {
            AB(ind) = coeff[fid];
            ind++;
        }
    }


    auto get_smoothnessValue = [&]()
    {
        //D smoothnessValue = (Acoeff.adjoint()*DDA*Acoeff + Bcoeff.adjoint()*DDB*Bcoeff).real()[0];
        Scalar m = (coeff.adjoint()*dd*coeff)[0];
        D summ = m[0] + m[2];
        return summ;
    };
    D smoothnessValue = get_smoothnessValue();
    if (logMessages) printf("\n\nInitial smoothness: %.5g\n", smoothnessValue);
    int oob = getNumOutOfBounds();
    if (logMessages) printf("Initial out-of-bounds: %d\n", oob);
    //if (logMessages) printf(" %d %.5g %d\n", -1, smoothnessValue, oob);

    lambda = lambdaInit;
    for (int iter = 0; iter < maxIter; iter++)
    {
        if (logMessages) printf("\n--- Iteration %d ---\n", iter);

        localStep();
        globalStep(isConstrained, AB, logMessages);


        smoothnessValue = get_smoothnessValue();

        if (logMessages) printf("\n  smoothness: %.5g\n", smoothnessValue);
        oob = getNumOutOfBounds();
        if (logMessages) printf("  out-of-bounds: %d\n", oob);

        bool stoppingCriterion = (oob == 0);
        if (stoppingCriterion)
        {
            break;
        }
        lambda = lambda * lambdaMultFactor;
        //    if (logMessages) printf(" %d %.5g %d\n",iter, smoothnessValue, oob);

    }

    if (output.size() != 2) output.resize(2);
    output[0].resize(numF, 3);
    output[1].resize(numF, 3);
    for (int fid = 0; fid < numF; fid++)
    {
        const V3 &x = FX.row(fid);
        const V3 &y = FY.row(fid);
        float uv[4];
        pv(fid).store(uv);
        output[0].row(fid) = uv[0] * x + uv[1] * y;
        output[1].row(fid) = uv[2] * x + uv[3] * y;
    }

    return (oob == 0);
}
