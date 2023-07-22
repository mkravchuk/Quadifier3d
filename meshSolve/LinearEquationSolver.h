#pragma once
#include "SolverTypes.h"
class AnySolver;

class LinearEquationSolver
{
private:
    AnySolver * solver;
    ComputationInfo compute_info;
    ComputationInfo solve_info;
public:
    LinearEquationSolver();
    ~LinearEquationSolver();

    bool IsAlreadyComputed();
    bool compute(const SparceMatrixType &Quu,  bool logMessages);
    bool solve(const Is& isFaceConstrained, const SparceMatrixType &Quk,
        const VectorType &xknown,
        VectorType &x,
        bool logMessages);
private:
};
