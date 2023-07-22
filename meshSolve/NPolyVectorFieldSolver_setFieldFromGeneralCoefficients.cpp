#include "stdafx.h"
#include "NPolyVectorFieldSolver_setFieldFromGeneralCoefficients.h"
#include "FaceConstrain.h"

const MeshLogicOptions_Solver& options2 = meshLogicOptions.Solver;

void setFieldFromGeneralCoefficients_origin(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    Matrix<Scalar, 3, 3> Identity3x3;
    Identity3x3.setIdentity(3, 3);
    Matrix<Scalar, 3, 1> z;
    z.setZero(3, 1);

    //Matrix<Scalar, 1, 1> I1;
    //I1.setIdentity(1, 1);
    //Matrix<Scalar, 1, 1> z1;
    //z1.setZero(1, 1);

    //Matrix<Scalar, 2, 2> I2;
    //I2.setIdentity(2, 2);
    //Matrix<Scalar, 2, 1> z2;
    //z2.setZero(2, 1);

    Scalar polyCoeff1_prev = Scalar(0, 0);
    Scalar polyCoeff2_prev = Scalar(0, 0);

    for (int i = 0; i < facesCount; ++i)
    {
        //poly coefficients: 1, 0, -Acoeff, 0, Bcoeff
        //matlab code from roots (given there are no trailing zeros in the polynomial coefficients)
        ////////////// v1 origin
        /*VectorType polyCoeff;
        polyCoeff.setZero(2 * n + 1, 1);
        polyCoeff[0] = 1.;
        int sign = 1;
        for (int k = 0; k < n; ++k)
        {
        sign = -sign;
        int degree = 2 * (k + 1);
        polyCoeff[degree] = (1.*sign)*coeffs[k](i);
        }

        VectorType roots;
        igl::polyRoots<Scalar, double >(polyCoeff, roots);*/

        ////////////// v2 fast
        Matrix<Scalar, 1, 4> polyCoeff;
        polyCoeff.setZero(1, 4);
        int sign = 1;
        for (int k = 0; k < n; ++k)
        {
            int degree = 2 * (k + 1) - 1;
            polyCoeff[degree] = (static_cast<float>(1.)*sign)*coeffs[k](i);
            sign = -sign;
        }


        //cache - for speed up 1%
        if (polyCoeff1_prev == polyCoeff(1)
            && polyCoeff2_prev == polyCoeff(3))
        {
            for (int k = 0; k < n; ++k)
            {
                pv[k](i, 0) = pv[k](i - 1, 0);
                pv[k](i, 1) = pv[k](i - 1, 1);
            }
            continue;
        }
        polyCoeff1_prev = polyCoeff(1);
        polyCoeff2_prev = polyCoeff(3);

        Matrix<Scalar, 4, 4> a;
        a << polyCoeff, Identity3x3, z;
        const Matrix<Scalar, 4, 1> roots = a.eigenvalues();

        ////////////// v3 test
        /*Matrix<Scalar, 1, 3> polyCoeff2;
        polyCoeff2.setZero(1, 3);
        int sign2 = 1;
        for (int k = 0; k < n; ++k)
        {
        polyCoeff2[k+1] = (1.*sign2)*coeffs[k](i);
        sign2 = -sign2;
        }

        Matrix<Scalar, 3, 3> a2;
        a2 << polyCoeff2, I2, z2;

        Matrix<Scalar, 3, 1> roots2 = a2.eigenvalues();*/

        ////////////// v4 test
        /*Matrix<Scalar, 1, 2> polyCoeff1;
        polyCoeff1.setZero(1, 2);
        int sign2 = 1;
        for (int k = 0; k < n; ++k)
        {
        polyCoeff1[1] = (1.*sign2)*coeffs[k](i);
        Matrix<Scalar, 2, 2> a1;
        a1 << polyCoeff1, I1, z1;

        Matrix<Scalar, 2, 1> roots1 = a1.eigenvalues();

        sign2 = -sign2;
        }*/
        //////////////

        Eigen::Vector4i done;
        done.setZero(4, 1);

        Eigen::Matrix<Scalar, 2, 1> u(n, 1);
        int ind = 0;
        for (int k = 0; k < 2 * n; ++k)
        {
            if (done[k])
                continue;
            u[ind] = roots[k];
            done[k] = 1;

            int mini = -1;
            double mind = 1e10;
            for (int l = k + 1; l < 2 * n; ++l)
            {
                double dist = abs(roots[l] + u[ind]);
                if (dist < mind)
                {
                    mind = dist;
                    mini = l;
                }
            }
            done[mini] = 1;
            ind++;
        }
        for (int k = 0; k < n; ++k)
        {
            pv[k](i, 0) = real(u[k]);
            pv[k](i, 1) = imag(u[k]);
        }
    }


}

void setFieldFromGeneralCoefficients2_fast(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    Scalar d1;
    Scalar d2;
    Matrix<Scalar, 2, 2> a2;
    a2(1, 0) = 1;
    a2(1, 1) = 0;
    
    for (int i = 0; i < facesCount; ++i)
    {
        d1 = coeffs[0](i);
        d2 = -coeffs[1](i);

        // Matrix 4x4 can be simplified to matrix 2x2
        //   0  x  0  y          x  y
        //   1  0  0  0   ==>  1  0
        //   0  1  0  0
        //   0  0  1  0

        a2(0, 0) = d1;
        a2(0, 1) = d2;
        Matrix<Scalar, 2, 1> roots2 = a2.eigenvalues();
        Scalar r0 = sqrt(roots2(0));
        Scalar r1 = sqrt(roots2(1));

        // skip sorting - it is not important

        //return result
        pv[0](i, 0) = r0.real();
        pv[0](i, 1) = r0.imag();
        pv[1](i, 0) = r1.real();
        pv[1](i, 1) = r1.imag();
    }


}


void setFieldFromGeneralCoefficients3_fast_fast(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv, bool ignore_x, const VectorXi &isFaceConstrained)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    for (int i = 0; i < facesCount; ++i)
    {
        Scalar d1;
        if (ignore_x)
            d1 = 0.;
        else
            d1 = coeffs[0](i);
        Scalar d2 = -coeffs[1](i);

        double temp = 0;
        ////DEBUG - calc solution using matrix to test agains new solution
        ////        
        //// v2 - 2x2 matrix
        ////
        //// Matrix 4x4 can be simplified to matrix 2x2, with small correction: 'a' of 2x2 will be equal to sqrt('a' of 4x4), so final eigen values should be 'sqrt'
        ////   0  x  0  y          x  y
        ////   1  0  0  0   ==>  1  0
        ////   0  1  0  0
        ////   0  0  1  0
        //Matrix<Scalar, 2, 2> m2;
        //m2(1, 0) = 1;
        //m2(1, 1) = 0;
        //m2(0, 0) = d1;
        //m2(0, 1) = d2;
        //Matrix<Scalar, 2, 1> roots2 = m2.eigenvalues();
        //roots2(0) = sqrt(roots2(0)); // convert 'a' of 2x2 to 'a' of 4x4
        //roots2(1) = sqrt(roots2(1)); // convert 'a' of 2x2 to 'a' of 4x4
        //Scalar r0 = roots2(0);
        //Scalar r1 = roots2(1);

        ////DEBUG - normalize r0 and r1 to test agains newest method
        //Matrix<Scalar, 2, 1> roots2_norm;
        //roots2_norm(0) = r0;
        //roots2_norm(1) = r1;
        //roots2_norm.normalize();
        //Scalar r0_norm = roots2_norm(0);
        //Scalar r1_norm = roots2_norm(1);

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
        Scalar sqrt_det = sqrt(d1*d1 + (float)4.0*d2);
        Scalar a1_2x2 = (d1 + sqrt_det)*(float)0.5;
        Scalar a2_2x2 = (d1 - sqrt_det)*(float)0.5;
        Scalar r0fast = sqrt(a1_2x2); // convert 'a' of 2x2 to 'a' of 4x4
        Scalar r1fast = sqrt(a2_2x2); // convert 'a' of 2x2 to 'a' of 4x4

                                               //DEBUG - normalize r0fast and r1fast to test agains oldest method
                                               //Matrix<Scalar, 2, 1> roots2fast_norm;
                                               //roots2fast_norm(0) = r0fast;
                                               //roots2fast_norm(1) = r1fast;
                                               //roots2fast_norm.normalize();
                                               //Scalar r0fast_norm = roots2fast_norm(0);
                                               //Scalar r1fast_norm = roots2fast_norm(1);


                                               // sort complex vectors - this is very important for detecting singularities
        if (options2.sort_complex_vectors_fast && !isFaceConstrained[i])
        {
            //if (r1fast.real() > r0fast.real())
            //{
            //    swap(r0fast, r1fast);
            //}

            double r0fastabsPow2 = utils::complex::absPow2(r0fast);
            double r1fastabsPow2 = utils::complex::absPow2(r1fast);
            if (r1fastabsPow2 < r0fastabsPow2)
            {
                swap(r0fast, r1fast);
            }
        }
        else if (options2.sort_complex_vectors
            && !isFaceConstrained[i])// skip sorting constrained faces
        {
            double r0fastabs = abs(r0fast);
            double r1fastabs = abs(r1fast);
            if (r1fastabs < r0fastabs)
            {
                swap(r0fast, r1fast);
                //cout << endl << "sorted!!!   NPolyVectorFieldSolver::setFieldFromGeneralCoefficients3_fast_fast" << endl << endl;
            }
            /*if ((r1fastabsPow2 < r0fastabsPow2) != (r1fastabs < r0fastabs))
            {
            cout << "(r1fastabsPow2 < r0fastabsPow2) != (r1fastabs < r0fastabs):    "<< (r1fastabsPow2 < r0fastabsPow2) <<" != "<< (r1fastabs < r0fastabs) <<    "r1fastabsPow2="<< r1fastabsPow2<<", r0fastabsPow2="<< r0fastabsPow2<<"      r1fastabs="<< r1fastabs<<", r0fastabs=" << r0fastabs<<endl;
            }*/
            //else if (abs(-r1fast) < r0fastabs)
            //{
            //    swap(r0fastabs, r1fastabs);
            //    swap(r0fast, r1fast);
            //    r0fast = -r0fast;
            //    //cout << endl << "sorted!!!   NPolyVectorFieldSolver::setFieldFromGeneralCoefficients3_fast_fast" << endl << endl;
            //}
            //r0fast = -r0fast;
            //r1fast = -r1fast;
        }

        // return results
        pv[0](i, 0) = r0fast.real();
        pv[0](i, 1) = r0fast.imag();
        pv[1](i, 0) = r1fast.real();
        pv[1](i, 1) = r1fast.imag();
    }


}




void setFieldFromGeneralCoefficients4_fast_fast_fast(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv, bool ignore_x, const VectorXi &isFaceConstrained)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    //if (ignore_x) // special case where always d1==0, it happend when second directionis not changed
    //{
    //    for (int fid = 0; fid < facesCount; ++fid)
    //    {
    //        // Scalar d1 = 0  - always for ignore_x, so we can simplify our solution
    //        Scalar d2 = -1.0*coeffs[1](fid);
    //        Scalar sqrt_det = utils::complex::sqrtFast(d2); // d1 eqaul to 0, so sqrt is simple of d2
    //        Scalar r0fast = utils::complex::sqrtFast(sqrt_det); // convert 'a' of 2x2 to 'a' of 4x4 (here we calculate 'sqrt(d1+sqrt_det)')
    //        Scalar r1fast = Scalar(r0fast.imag(), -r0fast.real()); // r1fast is miror of r0fast, since 'utils::complex::sqrtFast(-sqrt_det)' is miror of 'utils::complex::sqrtFast(sqrt_det)'
    //                                                                                // here we dont have to sort, since both r0fast and r1fast are of same length
    //        pv[0](fid, 0) = r0fast.real();
    //        pv[0](fid, 1) = r0fast.imag();
    //        pv[1](fid, 0) = r1fast.real();
    //        pv[1](fid, 1) = r1fast.imag();
    //    }
    //}
    //else
    {

        for (int fid = 0; fid < facesCount; ++fid)
        {
            Scalar d1 = coeffs[0](fid);
            Scalar d2 = -coeffs[1](fid);

            double temp = 0;
            ////DEBUG - calc solution using matrix to test agains new solution
            ////        
            //// v2 - 2x2 matrix
            ////
            //// Matrix 4x4 can be simplified to matrix 2x2, with small correction: 'a' of 2x2 will be equal to sqrt('a' of 4x4), so final eigen values should be 'sqrt'
            ////   0  x  0  y          x  y
            ////   1  0  0  0   ==>  1  0
            ////   0  1  0  0
            ////   0  0  1  0
            //Matrix<Scalar, 2, 2> m2;
            //m2(1, 0) = 1;
            //m2(1, 1) = 0;
            //m2(0, 0) = d1;
            //m2(0, 1) = d2;
            //Matrix<Scalar, 2, 1> roots2 = m2.eigenvalues();
            //roots2(0) = sqrt(roots2(0)); // convert 'a' of 2x2 to 'a' of 4x4
            //roots2(1) = sqrt(roots2(1)); // convert 'a' of 2x2 to 'a' of 4x4
            //Scalar r0 = roots2(0);
            //Scalar r1 = roots2(1);

            ////DEBUG - normalize r0 and r1 to test agains newest method
            //Matrix<Scalar, 2, 1> roots2_norm;
            //roots2_norm(0) = r0;
            //roots2_norm(1) = r1;
            //roots2_norm.normalize();
            //Scalar r0_norm = roots2_norm(0);
            //Scalar r1_norm = roots2_norm(1);

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
            Scalar sqrt_det = utils::complex::sqrtFast(d1*d1 + d2);
            if (!options2.take_sqrt_a_2x2_1)
            {
                sqrt_det = d1 * d1 + d2;
            }
            Scalar a1_2x2 = (d1 + sqrt_det); 
            Scalar a2_2x2 = (d1 - sqrt_det);
            //v0
            //Scalar r0fast = utils::complex::sqrtFast(a1_2x2); // convert 'a' of 2x2 to 'a' of 4x4
            //Scalar r1fast = utils::complex::sqrtFast(a2_2x2); // convert 'a' of 2x2 to 'a' of 4x4
            //v1
            Scalar r0fast;
            Scalar r1fast;
            utils::complex::sqrtFast2(a1_2x2, a2_2x2, r0fast, r1fast);

            if (!options2.take_sqrt_a_2x2_2)
            {
                r0fast = a1_2x2;
                r1fast = a2_2x2;
            }
            //DEBUG - normalize r0fast and r1fast to test agains oldest method
            //Matrix<Scalar, 2, 1> roots2fast_norm;
            //roots2fast_norm(0) = r0fast;
            //roots2fast_norm(1) = r1fast;
            //roots2fast_norm.normalize();
            //Scalar r0fast_norm = roots2fast_norm(0);
            //Scalar r1fast_norm = roots2fast_norm(1);


            // sort complex vectors - this is very important for detecting singularities
            if (options2.sort_complex_vectors_fast && !isFaceConstrained[fid])
            {
                //double x0 = r0fast.real();
                //double y0 = r0fast.imag();
                //double x1 = r1fast.real();
                //double y1 = r1fast.imag();

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

                RealScalar r0fastabsPow2 = utils::complex::absPow2(r0fast);
                RealScalar r1fastabsPow2 = utils::complex::absPow2(r1fast);
                if (r1fastabsPow2 < r0fastabsPow2)
                {
                    swap(r0fast, r1fast);
                }
            }
            else if (options2.sort_complex_vectors
                && !isFaceConstrained[fid])// skip sorting constrained faces
            {
                RealScalar r0fastabs = abs(r0fast);
                RealScalar r1fastabs = abs(r1fast);
                if (r1fastabs < r0fastabs)
                {
                    swap(r0fast, r1fast);
                    //cout << endl << "sorted!!!   NPolyVectorFieldSolver::setFieldFromGeneralCoefficients3_fast_fast" << endl << endl;
                }
                /*if ((r1fastabsPow2 < r0fastabsPow2) != (r1fastabs < r0fastabs))
                {
                cout << "(r1fastabsPow2 < r0fastabsPow2) != (r1fastabs < r0fastabs):    "<< (r1fastabsPow2 < r0fastabsPow2) <<" != "<< (r1fastabs < r0fastabs) <<    "r1fastabsPow2="<< r1fastabsPow2<<", r0fastabsPow2="<< r0fastabsPow2<<"      r1fastabs="<< r1fastabs<<", r0fastabs=" << r0fastabs<<endl;
                }*/
                //else if (abs(-r1fast) < r0fastabs)
                //{
                //    swap(r0fastabs, r1fastabs);
                //    swap(r0fast, r1fast);
                //    r0fast = -r0fast;
                //    //cout << endl << "sorted!!!   NPolyVectorFieldSolver::setFieldFromGeneralCoefficients3_fast_fast" << endl << endl;
                //}
                //r0fast = -r0fast;
                //r1fast = -r1fast;
            }

            // return results
            pv[0](fid, 0) = r0fast.real();
            pv[0](fid, 1) = r0fast.imag();
            pv[1](fid, 0) = r1fast.real();
            pv[1](fid, 1) = r1fast.imag();
        }
    }

}