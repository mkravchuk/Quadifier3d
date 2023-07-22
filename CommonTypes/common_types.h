#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//#define USE_EIGEN

#ifdef USE_EIGEN

//*****************************************
// *** Eigen
//*****************************************

// it is possible to speed a bit app, if processor will support AVX2
//#define __AVX2__
#define __SSE4_2__
#define EIGEN_DEFAULT_TO_ROW_MAJOR 1

#include <Eigen/Core>
#include <Eigen/Geometry>  //  Quaterniond, AngleAxis
//#include <Eigen/Sparse> 
//#include <Eigen/Eigenvalues> 
//#include <Eigen/SVD>
//#include <Eigen/Dense>
//#include <Eigen/StdVector>
using namespace Eigen;

using B = bool;  //same as 'typedef float D' but using C++14 style
using D = double;  //same as 'typedef double D' but using C++14 style
using DD = double;  //same as 'typedef double D' but using C++14 style
using I = int;
using II = long long;
using P3 = Eigen::Point3d;
using V3 = Eigen::Vector3d;
using D33 = Eigen::Matrix3d;
using C3 = Eigen::Color3d;
using C4 = Eigen::Color4d;
using Ds = Eigen::VectorXd;
using Bs = Eigen::VectorXb;
using Is = Eigen::VectorXi;
using P3s = Eigen::MatrixXd;
using V3s = Eigen::MatrixXd;
using I2s = Eigen::MatrixXi;
using I3s = Eigen::MatrixXi;
using I4s = Eigen::MatrixXi;
using I2 = Eigen::Vector2i;
using I3 = Eigen::Vector3i;
using I4 = Eigen::Vector4i;
using S = std::string;

inline P3 convertEigenToP3(const Eigen::Point3d& p3)
{
    return p3;
}
inline V3 convertEigenToV3(const Eigen::Vector3d& v3)
{
    return v3;
}
inline Eigen::Point3d convertP3ToEigen(const P3& p3)
{
    return p3;
}
inline Eigen::Vector3d convertV3ToEigen(const V3& v3)
{
    return v3;
}

inline Eigen::MatrixXd convertP3sToEigen(const P3s& V)
{
    return V;
}
inline Eigen::MatrixXd convertP3sToEigen(const V3s& V)
{
    return V;
}
inline P3s convertEigenToP3s(const Eigen::MatrixXd& V)
{
    return V;
}
inline V3s convertEigenToV3s(const Eigen::MatrixXd& V)
{
    return V;
}


#else




//*****************************************
// *** CommonClasses
//*****************************************

#define __SSE4_2__
#define EIGEN_DEFAULT_TO_ROW_MAJOR 1

#include <Eigen/Core>
#include <Eigen/Geometry>  //  Quaterniond, AngleAxis
#include "CommonClasses.h"

using namespace Eigen;
using namespace CommonClasses;
using B = bool;  //same as 'typedef float D' but using C++14 style
using D = float;  //same as 'typedef float D' but using C++14 style
using DD = double;  //same as 'typedef float D' but using C++14 style
using I = int;
using II = long long;
using P3 = CommonClasses::Point34f;
using V3 = CommonClasses::Vector34f;
using D33 = CommonClasses::Matrix34f;
using C3 = CommonClasses::Vector<D, 3>;
using C4 = CommonClasses::Vector<D, 4>;
using Ds = CommonClasses::Vector<D>;
using Bs = CommonClasses::Vector<B>;
using Is = CommonClasses::Vector<I>;
using P3s = CommonClasses::MatrixX34f<P3>;
using V3s = CommonClasses::MatrixX34f<V3>;
using I2s = CommonClasses::Table<I, X, 2>;
using I3s = CommonClasses::Table<I, X, 3>;
using I4s = CommonClasses::Table<I, X, 4>;
using I2 = CommonClasses::Vector<I, 2>;
using I3 = CommonClasses::Vector<I, 3>;
using I4 = CommonClasses::Vector<I, 4>;
using S = std::string;

//using namespace CommonClasses;
inline P3 convertEigenToP3(const Eigen::Point3d& p3)
{
    return P3(p3(0), p3(1), p3(2));
}
inline V3 convertEigenToV3(const Eigen::Vector3f& v3)
{
    return V3(v3(0), v3(1), v3(2));
}
inline V3 convertEigenToV3(const Eigen::Vector3d& v3)
{
    return V3(v3(0), v3(1), v3(2));
}
inline Eigen::Point3f convertP3ToEigen(const P3& p3)
{
    return Eigen::Point3f(p3(0), p3(1), p3(2));
}
inline Eigen::Point3d convertP3ToEigenDouble(const P3& p3)
{
    return Eigen::Point3d(p3(0), p3(1), p3(2));
}
inline Eigen::Vector3f convertV3ToEigen(const V3& v3)
{
    return Eigen::Vector3f(v3(0), v3(1), v3(2));
}
inline Eigen::Vector3d convertV3ToEigenDouble(const V3& v3)
{
    return Eigen::Vector3d(v3(0), v3(1), v3(2));
}
inline Eigen::VectorXd convertD3sToEigenDouble(const Ds& d)
{
    Eigen::VectorXd res(d.size(), 1);
    for (int i = 0; i < d.size(); i++)
    {
        res(i,0) = d(i);
    }
    return res;
}

inline Eigen::MatrixXd convertP3sToEigenDouble(const P3s& V)
{
    Eigen::MatrixXd res(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++)
    {
        P3 row = V.row(i);
        res.row(i) = Eigen::Vector3d(row(0), row(1), row(2));
    }
    return res;
}
inline Eigen::MatrixXd convertV3sToEigenDouble(const V3s& V)
{
    Eigen::MatrixXd res(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++)
    {
        V3 row = V.row(i);
        res.row(i) = Eigen::Vector3d(row(0), row(1), row(2));
    }
    return res;
}
inline Ds convertEigenToDs(const Eigen::VectorXd& d)
{
    Ds res(d.size());
    for (int i = 0; i < d.size(); i++)
    {
        res(i) = d(i);
    }
    return res;
}
inline P3s convertEigenToP3s(const Eigen::MatrixXd& V)
{
    P3s res(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++)
    {
        Eigen::Point3d row = V.row(i);
        res.row(i) = P3(row(0), row(1), row(2));
    }
    return res;
}
inline V3s convertEigenToV3s(const Eigen::MatrixXd& V)
{
    V3s res(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++)
    {
        Eigen::Vector3d row = V.row(i);
        res.row(i) = V3(row(0), row(1), row(2));
    }
    return res;
}

inline Eigen::MatrixXi convertI2sToEigenInt(const I2s& m)
{
    Eigen::MatrixXi res(m.rows(), 2);
    for (int i = 0; i < m.rows(); i++)
    {
        auto row = m.row(i);
        res.row(i) = Eigen::Vector2i(row(0), row(1));
    }
    return res;
}
inline Eigen::MatrixXi convertI3sToEigenInt(const I3s& m)
{
    Eigen::MatrixXi res(m.rows(), 3);
    for (int i = 0; i < m.rows(); i++)
    {
        I3 row = m.row(i);
        res.row(i) = Eigen::Vector3i(row(0), row(1), row(2));
    }
    return res;
}
inline I2s convertEigenToI2s(const Eigen::MatrixXi& m)
{
    I2s res(m.rows(), 2);
    for (int i = 0; i < m.size(); i++)
    {
        Eigen::Vector2i row = m.row(i);
        res.row(i) = I2(row(0), row(1));
    }
    return res;
}
inline I3s convertEigenToI3s(const Eigen::MatrixXi& m)
{
    I3s res(m.rows(), 3);
    for (int i = 0; i < m.size(); i++)
    {
        Eigen::Vector3i row = m.row(i);
        res.row(i) = I3(row(0), row(1), row(2));
    }
    return res;
}
#endif

//*****************************************
// *** define std methods for mixed use of float and double
//*****************************************

namespace std
{
    inline float min(const float& a, const double& b)
    {
        float bd = static_cast<float>(b);
        return a < bd ? a : bd;
    }
    inline float min(const double& a, const float& b)
    {
        float ad = static_cast<float>(a);
        return ad < b ? ad : b;
    }
    inline float max(const float& a, const double& b)
    {
        float bd = static_cast<float>(b);
        return a > bd ? a : bd;
    }
    inline float max(const double& a, const float& b)
    {
        float ad = static_cast<float>(a);
        return ad > b ? ad : b;
    }
}


//*****************************************
// *** to_string
//*****************************************


namespace std
{
    inline string to_string(P3 p)
    {
        return "" + to_string(p(0)) + ", " + to_string(p(1)) + ", " + to_string(p(2));
    }
    inline string to_string(V3 v)
    {
        return "" + to_string(v(0)) + ", " + to_string(v(1)) + ", " + to_string(v(2));
    }
    inline string to_string(C3 c)
    {
        return "" + to_string(c(0)) + ", " + to_string(c(1)) + ", " + to_string(c(2));
    }
    inline string to_string(C4 c)
    {
        return "" + to_string(c(0)) + ", " + to_string(c(1)) + ", " + to_string(c(2)) + ", " + to_string(c(3));
    }
    inline string to_string(Vector3f v)
    {
        return "{" + to_string(v(0)) + ", " + to_string(v(1)) + ", " + to_string(v(2)) + "}";
    }
    inline string to_string(Vector2i v)
    {
        return "{" + to_string(v(0)) + ", " + to_string(v(1))  + "}";
    }
    inline string to_string(Vector3i v)
    {
        return "{" + to_string(v(0)) + ", " + to_string(v(1)) + ", " + to_string(v(2)) + "}";
    }
    inline string to_string(I2 v)
    {
        return "{" + to_string(v(0)) + ", " + to_string(v(1)) + "}";
    }
    inline string to_string(I3 v)
    {
        return "{" + to_string(v(0)) + ", " + to_string(v(1)) + ", " + to_string(v(2)) + "}";
    }
    inline string to_string(I4 v)
    {
        return "{" + to_string(v(0)) + ", " + to_string(v(1)) + ", " + to_string(v(2)) + ", " + to_string(v(3)) + "}";
    }
}

//*****************************************
// *** CACHE
//*****************************************



template <class T>
void log(T a, std::string caption)
{
    std::cout << caption << "  " << a << std::endl;
};

inline void CommonTypes__Cache()
{
    using namespace std;
    V3 v1, v2;

    V3 vSum = v1 + v2; log(vSum, "vSum");
    V3 vMin = v1 - v2; log(vMin, "vMin");
    D vDot = v1.dot(v2); log(vDot, "vDot");
    V3 vCross = v1.cross(v2); log(vCross, "vMult");
    P3 p3(0, 0, 0); log(p3, "p3");
    C3 c3(0, 0, 0); log(c3, "c3");
    C4 c4(0, 0, 0, 0); log(c4, "c4");
    P3s V(1, 3);
    P3 v = V.row(0); log(v, "v");
    I3s F(1, 3); log(F(0, 1), "F(0, 1)");
    I3 f = F.row(0); log(f, "f");
    I2s EF(1, 2);
    I2 ef = EF.row(0);  log(ef, "ef");
    I4s TTT(1, 4);
    I4 ttt = TTT.row(0); log(ttt, "ttt");
    D33 m3;
}

