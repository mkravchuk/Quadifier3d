#pragma once
#include "SolverTypes_Complex2x.h"
class AnySolver_Complex2x;

class LinearEquationSolver2x
{
private:
    AnySolver_Complex2x * solver;
    ComputationInfo compute_info;
    ComputationInfo solve_info;
public:
    LinearEquationSolver2x();
    ~LinearEquationSolver2x();

    bool IsAlreadyComputed();
    bool compute(const SparceMatrixType &Quu, bool logMessages);
    bool solve(const Is& isFaceConstrained, const SparceMatrixType &Quk,
        const VectorType &xknown,
        VectorType &x,
        bool logMessages);
private:
};