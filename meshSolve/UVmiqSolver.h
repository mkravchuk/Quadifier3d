#pragma once
class MeshConstrains;
class ViewerDrawObjects;


class UVmiqSolver
{
private:
public:
    ViewerDrawObjects& draw;
    const P3s& V;
    const I3s& F;

    UVmiqSolver(ViewerDrawObjects& draw, const P3s& V, const I3s& F);
    void Solve(const vector<V3s>& nrosyField, MatrixXf& UV);
    void Solve(const VectorXi& b, const MatrixXd& bc, MatrixXf& UV);
};
