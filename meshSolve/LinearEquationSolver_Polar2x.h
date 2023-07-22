#pragma once
#include "SolverTypes_Polar2x.h"
class AnySolver_Polar2x;

class LinearEquationSolver_Polar2x
{
private:
    ViewerDrawObjects& draw;
    ComputationInfo compute_info;
    ComputationInfo solve_info;
public:
    AnySolver_Polar2x * solver;
    LinearEquationSolver_Polar2x(ViewerDrawObjects& draw);
    ~LinearEquationSolver_Polar2x();

    bool IsAlreadyComputed();
    bool compute(const SparceMatrixType &Quu, bool logMessages, TimeElapsed& elapsed_compute, bool reuse_ordering = false);
    bool solve(const Bs& isFaceConstrained, const SparceMatrixTypeComplex &Quk,
        const VectorTypeComplex &xknown, const VectorTypeComplex& f,
        VectorTypeComplex &x,
        const Is& known_index_to_fid, const Is& unknown_index_to_fid,
        bool logMessages, TimeElapsed& elapsed_solve);
    vector<int> Get_compute_unknown_indexes();
    void copy_ordering_from_another_solver(const LinearEquationSolver_Polar2x& another_solver);
    void copy_ordering_from_another_solver__from_limited_to_full(const LinearEquationSolver_Polar2x& another_solver, const Is& known_index_to_fid, const Is& unknown_index_to_fid);
};