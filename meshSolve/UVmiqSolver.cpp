#include "stdafx.h"
#include "UVmiqSolver.h"
#include "Utils.h"
#include <igl/compute_frame_field_bisectors.h>
#include <igl/comb_cross_field.h>
#include <igl/cross_field_missmatch.h>
#include <igl/find_cross_field_singularities.h>
#include <igl/cut_mesh_from_singularities.h>
#include <igl/comb_frame_field.h>
#include <igl/copyleft/comiso/miq.h>
#include <igl/parula.h>
#include "../iglcomiso/nrosy.h"
#include <igl/local_basis.h>
#include <igl/rotate_vectors.h>
#include <igl/PI.h>


UVmiqSolver::UVmiqSolver(ViewerDrawObjects& _draw, const P3s& _V, const I3s& _F)
    : draw(_draw), V(_V), F(_F)
{
}

void UVmiqSolver::Solve(const vector<V3s>& nrosyField, MatrixXf& UV)
{
    if (nrosyField.size() == 0) return;

    V3 basedirection = nrosyField[0].row(0);
    vector<V3s> fields = nrosyField;
    /*for (int fid = 0; fid < fields[0].rows(); fid++)
    {
        V3 direction = fields[0].row(fid);
        V3 direction2 = fields[1].row(fid);
        V3 normal = direction.cross(direction2);
        normal.normalize();
        D angle = utils::vector::AngleFull(direction, basedirection, normal);
        if (angle < 0) angle = 360 - angle;
        for(int k = 0; k < 4; k++)
        if (angle > 90)
        {
            direction = utils::vector::Rotate(direction, normal, -90, true);
            angle = utils::vector::AngleFull(direction, basedirection, normal);
        }
        fields[0].row(fid) = direction;
        V3 directionRotated = utils::vector::Rotate(direction, normal, 90, true);
        fields[1].row(fid) = directionRotated;
    }*/
    for (int fid = 0; fid < fields[0].rows(); fid++)
    {
        V3 direction = fields[0].row(fid);
        direction.normalize();
        V3 direction2 = fields[1].row(fid);
        V3 normal = direction.normalized().cross(direction2.normalized());
        normal.normalize();
        D angle = utils::vector::Angle(direction.normalized(), basedirection.normalized());// utils::vector::AngleFull(direction, basedirection, normal);
        if (angle < -45) angle = 360 + angle;
        for (int k = 0; k <= 4; k++)
            if (angle > 45 || angle < -45)
            {
                direction = utils::vector::Rotate(direction, normal, 90, true);
                //direction = normal.cross(direction);
                angle = utils::vector::Angle(direction.normalized(), basedirection.normalized());
            }
        //cout << "angle=" << angle << endl;
        fields[0].row(fid) = direction;
        V3 directionRotated = utils::vector::Rotate(direction, normal, 90, true);
        fields[1].row(fid) = directionRotated;
    }

    MatrixXd V_ = convertP3sToEigenDouble(V);
    MatrixXd X1 = convertV3sToEigenDouble(fields[0]); // Cross field
    MatrixXd X2 = convertV3sToEigenDouble(fields[1]); // Cross field
    MatrixXd V_uv; // Global parametrization
    MatrixXi F_uv;  // Global parametrization

    //
    // v0 
    //
    igl::copyleft::comiso::miq(V_,
        convertI3sToEigenInt(F),
        X1,
        X2,
        V_uv,
        F_uv,
        4.0,      //   scale                global scaling for the gradient (controls the quads resolution)
        0.5,       //   stiffness           weight for the stiffness iterations
        true,     //   direct_round      greedily round all integer variables at once (greatly improves optimization speed but lowers quality)
        0);        //   iter                  stiffness iterations(0 = no stiffness)


    //
    // v1
    //
    //MatrixXd B; // Face barycenters
    //MatrixXd BIS1, BIS2; // Bisector field
    //MatrixXd BIS1_combed, BIS2_combed; // Combed bisector
    //MatrixXi MMatch; // Per-corner, integer mismatches
    //VectorXi isSingularity, singularityIndex; // Field singularities
    //MatrixXi Seams; // Per corner seams
    //MatrixXd X1_combed, X2_combed; // Combed field
    //MatrixXd UV_seams; // Global parametrization (with seams)
    //MatrixXi FUV_seams; // Global parametrization (with seams)

    //// Always work on the bisectors, it is more general
    //igl::compute_frame_field_bisectors(V_, F, X1, X2, BIS1, BIS2);

    //// Comb the field, implicitly defining the seams
    //igl::comb_cross_field(V_, F, BIS1, BIS2, BIS1_combed, BIS2_combed);

    //// Find the integer mismatches
    //igl::cross_field_missmatch(V_, F, BIS1_combed, BIS2_combed, true, MMatch);

    //// Find the singularities
    //igl::find_cross_field_singularities(V_, F, MMatch, isSingularity, singularityIndex);

    //// Cut the mesh, duplicating all vertices on the seams
    //igl::cut_mesh_from_singularities(V_, F, MMatch, Seams);

    //// Comb the frame-field accordingly
    //igl::comb_frame_field(V_, F, X1, X2, BIS1_combed, BIS2_combed, X1_combed, X2_combed);

    //igl::copyleft::comiso::miq(V_,
    //    F,
    //    X1_combed,
    //    X2_combed,
    //    MMatch,
    //    isSingularity,
    //    Seams,
    //    V_uv,
    //    F_uv
    //);

    //
    // convert results - v0
    //
    //double* uv = V_uv.data();
    //double umax = V_uv(0, 0);
    //double vmax = V_uv(0, 1);
    //double umin = umax;
    //double vmin = umin;
    //for (int i = 0; i < V_uv.rows(); i++)
    //{
    //    if (*uv > umax) umax = *uv;
    //    if (*uv < umin) umin = *uv;
    //    uv++;
    //    if (*uv > vmax) vmax = *uv;
    //    if (*uv < vmin) vmin = *uv;
    //    uv++;
    //}
    //uv = V_uv.data();
    //double ulength = umax - umin;
    //double vlength = vmax - vmin;
    //for (int i = 0; i < V_uv.rows(); i++)
    //{
    //    *uv = ((*uv )- umin)/ ulength;
    //    if (isnan(*uv)) *uv = 0;
    //    uv++;
    //    *uv = ((*uv) - vmin) / vlength;
    //    if (isnan(*uv)) *uv = 0;
    //    uv++;
    //}
    //UV = V_uv.cast<float>();

    //
    // convert results - v1
    //
    VectorXd U = V_uv.col(0);
    VectorXd V = V_uv.col(1);
    double umax = U.maxCoeff();
    double vmax = V.maxCoeff();
    double umin = U.minCoeff();
    double vmin = V.minCoeff();
    double ulength = umax - umin;
    double vlength = vmax - vmin;
    double* u = U.data();
    double* v = V.data();
    for (int i = 0; i < U.rows(); i++)
    {
        *u = ((*u) - umin) / ulength;
        u++;
        *v = ((*v) - vmin) / vlength;
        v++;
    }
    UV.resize(U.rows(), 2);
    D *uv = UV.data();
    u = U.data();
    v = V.data();
    for (int i = 0; i < U.rows(); i++)
    {
        *uv = *u;
        uv++;
        *uv = *v;
        uv++;
        u++;
        v++;
    }
}

void UVmiqSolver::Solve(const VectorXi& b, const MatrixXd& bc, MatrixXf& UV)
{
    MatrixXd V_ = convertP3sToEigenDouble(V);
    MatrixXd X1;
    VectorXd S;
    igl::copyleft::comiso::nrosy(V_, convertI3sToEigenInt(F), b, bc, VectorXi(), VectorXd(), MatrixXd(), 4, 0.5, X1, S);

    // Find the orthogonal vector
    MatrixXd B1, B2, B3;
    igl::local_basis(V_, convertI3sToEigenInt(F), B1, B2, B3);
    MatrixXd X2 = igl::rotate_vectors(X1, VectorXd::Constant(1, igl::PI / 2), B1, B2);

    MatrixXd V_uv; // Global parametrization
    MatrixXi F_uv;  // Global parametrization
    igl::copyleft::comiso::miq(V_,
        convertI3sToEigenInt(F),
        X1,
        X2,
        V_uv,
        F_uv,
        4.0,      //   scale                global scaling for the gradient (controls the quads resolution)
        0.5,       //   stiffness           weight for the stiffness iterations
        false,     //   direct_round      greedily round all integer variables at once (greatly improves optimization speed but lowers quality)
        2);        //   iter                  stiffness iterations(0 = no stiffness)

    //
    // convert results - v1
    //
    VectorXd U = V_uv.col(0);
    VectorXd V = V_uv.col(1);
    double umax = U.maxCoeff();
    double vmax = V.maxCoeff();
    double umin = U.minCoeff();
    double vmin = V.minCoeff();
    double ulength = umax - umin;
    double vlength = vmax - vmin;
    double* u = U.data();
    double* v = V.data();
    for (int i = 0; i < U.rows(); i++)
    {
        *u = ((*u) - umin) / ulength;
        u++;
        *v = ((*v) - vmin) / vlength;
        v++;
    }
    UV.resize(U.rows(), 2);
    D *uv = UV.data();
    u = U.data();
    v = V.data();
    for (int i = 0; i < U.rows(); i++)
    {
        *uv = *u;
        uv++;
        *uv = *v;
        uv++;
        u++;
        v++;
    }
}

