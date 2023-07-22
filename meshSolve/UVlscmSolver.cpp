#include "stdafx.h"
#include "UVlscmSolver.h"
#include "Utils.h"
//#include <igl/boundary_loop.h>
#include <igl/lscm.h>

UVlscmSolver::UVlscmSolver(ViewerDrawObjects& _draw, const P3s& _V, const I3s& _F)
    : draw(_draw), V(_V), F(_F)
{
}

void UVlscmSolver::Solve(const vector<UVlscmSolverConstrain>& contrains, MatrixXf& UV)
{
    // v0 - example
    // Fix two points on the boundary
    //VectorXi bnd, b(2, 1);
    //MatrixXd bc(2, 2);
    //igl::boundary_loop(F, bnd);
    //b(0) = bnd(0);
    //b(1) = bnd(round(bnd.size() / 2));
    //bc.row(0) = Vector2d(0, 0);
    //bc.row(1) = Vector2d(1, 0);

    //v1
    VectorXi b;
    MatrixXd bc;
    b.resize(contrains.size(), 1);
    bc.resize(contrains.size(), 2);
    for (int i = 0; i < contrains.size(); i++)
    {
        b(i) = contrains[i].vid;
        D u = contrains[i].u;
        D v = contrains[i].v;
        bc.row(i) = Vector2d(v, u); // yes, here we swap u and v just to fix an issue in 'igl::lscm' method where resulted matrix is swaped by column
    }

    Eigen::MatrixXd V_uv;
    igl::lscm(convertP3sToEigenDouble(V), convertI3sToEigenInt(F), b, bc, V_uv);
    UV = V_uv.cast<float>();
}

