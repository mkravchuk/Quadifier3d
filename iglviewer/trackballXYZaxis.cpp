#include "stdafx.h"
#include "trackballXYZaxis.h"
#include "igl/trackball.h" 

#include <igl/EPS.h>
#include <igl/dot.h>
#include <igl/cross.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/quat_mult.h>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <algorithm>
#include <iostream>
#include <igl/PI.h>

//
// Help for quaternions
// https://eater.net/quaternions
//

template <typename type>
double quatnorm(const type* quat)
{
    return sqrt(quat[0] * quat[0] +
        quat[1] * quat[1] +
        quat[2] * quat[2] +
        quat[3] * quat[3]);
}

void quatmult(double* quat, const double* quatMult)
{
    double qres[4];
    igl::quat_mult<double>(quat, quatMult, qres);
    quat[0] = qres[0];
    quat[1] = qres[1];
    quat[2] = qres[2];
    quat[3] = qres[3];
}

// Utility IGL_INLINE functions
template <typename Q_type>
static IGL_INLINE Q_type _QuatD(double w, double h)
{
    using namespace std;
    return (Q_type)(std::abs(w) < std::abs(h) ? std::abs(w) : std::abs(h)) - 4;
}
template <typename Q_type>
static IGL_INLINE Q_type _QuatIX(double x, double w, double h)
{
    return (2.0f*(Q_type)x - (Q_type)w - 1.0f) / _QuatD<Q_type>(w, h);
}
template <typename Q_type>
static IGL_INLINE Q_type _QuatIY(double y, double w, double h)
{
    return (-2.0f*(Q_type)y + (Q_type)h - 1.0f) / _QuatD<Q_type>(w, h);
}


// This is largely the trackball as implemented in AntTweakbar. Much of the
// code is straight from its source in TwMgr.cpp
// http://www.antisphere.com/Wiki/tools:anttweakbar
template <typename Q_type>
IGL_INLINE void igl::trackballXYZAxis(
    double w,
    double h,
    Q_type speed_factor,
    double down_mouse_x,
    double down_mouse_y,
    double mouse_x,
    double mouse_y,
    bool rotateX, bool rotateY, bool rotateZ,
    Q_type * quat)
{

    /*assert(speed_factor > 0);

    double original_x =
        _QuatIX<Q_type>(speed_factor*(down_mouse_x - w / 2) + w / 2, w, h);
    double original_y =
        _QuatIY<Q_type>(speed_factor*(down_mouse_y - h / 2) + h / 2, w, h);

    double x = _QuatIX<Q_type>(speed_factor*(mouse_x - w / 2) + w / 2, w, h);
    double y = _QuatIY<Q_type>(speed_factor*(mouse_y - h / 2) + h / 2, w, h);

    double z = 1;
    double n0 = sqrt(original_x*original_x + original_y * original_y + z * z);
    double n1 = sqrt(x*x + y * y + z * z);
    if (n0 > igl::DOUBLE_EPS && n1 > igl::DOUBLE_EPS)
    {
        double v0[] = { original_x / n0, original_y / n0, z / n0 };
        double v1[] = { x / n1, y / n1, z / n1 };
        double axis[3];
        cross(v0, v1, axis);
        double sa = sqrt(dot(axis, axis));
        double ca = dot(v0, v1);
        double angle = atan2(sa, ca);
        if (x*x + y * y > 1.0)
        {
            angle *= 1.0 + 0.2f*(sqrt(x*x + y * y) - 1.0);
        }
        double qrot[4];
        axis_angle_to_quat(axis, angle, qrot);
        quat[0] = qrot[0];
        quat[1] = qrot[1];
        quat[2] = qrot[2];
        quat[3] = qrot[3];
    }
    return;*/


    assert(speed_factor > 0);
    
    double res_quat[4];
    int res_quat__applyedRotationCount = 0;
    auto applyRotation = [&res_quat, &res_quat__applyedRotationCount](const double* quat)
    {
        if (res_quat__applyedRotationCount == 0)
        {
            res_quat[0] = quat[0];
            res_quat[1] = quat[1];
            res_quat[2] = quat[2];
            res_quat[3] = quat[3];
        }
        else
        {
            quatmult(res_quat, quat);
        }
        res_quat__applyedRotationCount++;
    };


    speed_factor /= 4; // TEMP decrease rotate ratio from PI to P/4

    if (rotateX)
    {
        double angleX = -speed_factor * igl::PI *(down_mouse_y - mouse_y) / (h / 2);
        //cout << "angleX=" << angleX << endl;
        double axisX[3] = { 1,0,0 }; 
        double quatX[4];
        axis_angle_to_quat(axisX, angleX, quatX);
        applyRotation(quatX);
    }

    if (rotateY)
    {
        double angleY = -speed_factor * igl::PI *(down_mouse_x - mouse_x) / (w / 2);
        //cout << "angleY=" << angleY << endl;
        double axisY[3] = { 0,1,0 };
        double quatY[4];
        axis_angle_to_quat(axisY, angleY, quatY);
        applyRotation(quatY);
    }

    if (rotateZ)
    {
        double xDiff = (down_mouse_x - mouse_x) / (w / 2);
        double yDiff = -(down_mouse_y - mouse_y) / (h / 2);
        yDiff = 0; //  ignore rotation by 'y' axis
        complex<double> angleX = { 1, speed_factor * igl::PI *xDiff };
        complex<double> angleY = { 1, speed_factor * igl::PI *yDiff };
        complex<double> angleXY = angleX * angleY; //  summ rotation
        double angleZ = -angleXY.imag();

        double axisZ[3] = { 0,0,1 };
        double quatZ[4];
        axis_angle_to_quat(axisZ, angleZ, quatZ);
        applyRotation(quatZ);
    }

    quat[0] = res_quat[0];
    quat[1] = res_quat[1];
    quat[2] = res_quat[2];
    quat[3] = res_quat[3];
}


template <typename Q_type>
IGL_INLINE void igl::trackballXYZAxis(
    double w,
    double h,
    const Q_type speed_factor,
    const Q_type * down_quat,
    double down_mouse_x,
    double down_mouse_y,
    double mouse_x,
    double mouse_y,
    bool rotateX, bool rotateY, bool rotateZ,
    Q_type * quat)
{
    double rot_quat[4], res_quat[4], down_quat_normalized[4];
    igl::trackballXYZAxis<double>(
        w, h,
        speed_factor,
        down_mouse_x, down_mouse_y,
        mouse_x, mouse_y,
        rotateX, rotateY, rotateZ,
        rot_quat);
    double down_quat_norm = quatnorm(down_quat);
    if (fabs(down_quat_norm) > igl::DOUBLE_EPS_SQ)
    {
        down_quat_normalized[0] = down_quat[0] / down_quat_norm;
        down_quat_normalized[1] = down_quat[1] / down_quat_norm;
        down_quat_normalized[2] = down_quat[2] / down_quat_norm;
        down_quat_normalized[3] = down_quat[3] / down_quat_norm;
        igl::quat_mult<double>(rot_quat, down_quat_normalized, res_quat);
        quat[0] = static_cast<Q_type>(res_quat[0]);
        quat[1] = static_cast<Q_type>(res_quat[1]);
        quat[2] = static_cast<Q_type>(res_quat[2]);
        quat[3] = static_cast<Q_type>(res_quat[3]);
    }
    else
    {
        quat[0] = static_cast<Q_type>(rot_quat[0]);
        quat[1] = static_cast<Q_type>(rot_quat[1]);
        quat[2] = static_cast<Q_type>(rot_quat[2]);
        quat[3] = static_cast<Q_type>(rot_quat[3]);
    }
}

template <typename Scalardown_quat, typename Scalarquat>
IGL_INLINE void igl::trackballXYZAxis(
    double w,
    double h,
    double speed_factor,
    const Quaternion<Scalardown_quat> & down_quat,
    double down_mouse_x,
    double down_mouse_y,
    double mouse_x,
    double mouse_y,
    bool rotateX, bool rotateY, bool rotateZ,
    Quaternion<Scalarquat> & quat)
{
    return trackballXYZAxis(
        w,
        h,
        static_cast<Scalarquat>(speed_factor),
        down_quat.coeffs().data(),
        down_mouse_x,
        down_mouse_y,
        mouse_x,
        mouse_y,
        rotateX, rotateY, rotateZ,
        quat.coeffs().data());
}



template void igl::trackballXYZAxis<float>(double, double, float, float const*, double, double, double, double, bool, bool, bool, float*);
template void igl::trackballXYZAxis<float, float>(double, double, double, Quaternion<float, 0> const&, double, double, double, double, bool, bool, bool, Quaternion<float, 0>&);
template void igl::trackballXYZAxis<double>(double, double, double, double const*, double, double, double, double, bool, bool, bool, double*);
template void igl::trackballXYZAxis<double, double>(double, double, double, Quaternion<double, 0> const&, double, double, double, double, bool, bool, bool, Quaternion<double, 0>&);
