#pragma once
class ViewerDrawObjects;


struct UVlscmSolverConstrain
{
    int vid;
    D u;
    D v;
};

class UVlscmSolver
{
private:
public:
    ViewerDrawObjects& draw;
    const P3s& V;
    const I3s& F;

    UVlscmSolver(ViewerDrawObjects& draw, const P3s& V, const I3s& F);
    void Solve(const vector<UVlscmSolverConstrain>& contrains, MatrixXf& UV);
};
