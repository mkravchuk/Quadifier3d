#include "stdafx.h"
#include "LinearEquationSolver_Complex2x.h"
#include "SolverTypes_Complex2x.h"
//#include "MeshLinearSolver.h"
#include "LDLTFloatSolver_Complex2x.h"


#if Intel_MKL_SUPPORTED
#include <Eigen/PardisoSupport>
#endif

//const MeshLogicOptions_LinearEquationSolver& options = meshLogicOptions.LinearEquationSolver;
//Elapsed_LinearEquationSolver& elapsed = elapsedTimers.LinearEquationSolver;
//Elapsed_Solver& elapsedS = elapsedTimers.Solver;
#define options meshLogicOptions.LinearEquationSolver
#define elapsed elapsedTimers.LinearEquationSolver
#define elapsedS elapsedTimers.Solver


// holds many of linear sparce equation solvers
// allows other classes to use any solver dynamicaly using common methods 'compute' and 'solve'
class AnySolver_Complex2x
{
private:
    LDLTFloatSolver_Complex2x solver_LDLTFloat;
    #if alternative_solvers_SUPPORTED
    SimplicialLLT    < SparceMatrixType> solver_SimplicialLLT;
    SimplicialLDLT    < SparceMatrixType> solver_SimplicialLDLT;
    SparseLU< SparceMatrixType> solver_SparseLU;
    BiCGSTAB  < SparceMatrixType> solver_BiCGSTAB;
    ConjugateGradient   < SparceMatrixType> solver_ConjugateGradient;
    BiCGSTAB  < SparceMatrixType, IncompleteLUT<Scalar>> solver_IncompleteLUT;
    #endif
    #if Intel_MKL_SUPPORTED
    PardisoLDLT < SparceMatrixType> solver_PardisoLDLT;
    PardisoLU< SparceMatrixType> solver_PardisoLU;
    #endif
public:
    AnySolver_Complex2x(MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType type);
    MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType Type;
    ComputationInfo compute(const SparceMatrixType &Quu);
    ComputationInfo solve(const Matrix<Scalar, Dynamic, Dynamic, ColMajor>& rhsM, Matrix<Scalar, Dynamic, Dynamic, ColMajor>& bM);
};

AnySolver_Complex2x::AnySolver_Complex2x(MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType type)
    :Type(type)
{
}

ComputationInfo AnySolver_Complex2x::compute(const SparceMatrixType &Quu)
{
    ComputationInfo compute_info;
    switch (Type)
    {
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
        default:
            solver_LDLTFloat.compute(Quu); 
            compute_info = solver_LDLTFloat.info();
            break;
        //    #if alternative_solvers_SUPPORTED
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::SimplicialLLT:
        //    solver_SimplicialLLT.compute(Quu);
        //    compute_info = solver_SimplicialLLT.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::SimplicialLDLT:
        //    solver_SimplicialLDLT.compute(Quu);
        //    compute_info = solver_SimplicialLDLT.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::SparseLU:
        //    solver_SparseLU.setPivotThreshold(meshLogicOptions.Solver.Tolerance);
        //    solver_SparseLU.compute(Quu);
        //    compute_info = solver_SparseLU.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::BiCGSTAB:
        //    solver_BiCGSTAB.setTolerance(meshLogicOptions.Solver.Tolerance);
        //    solver_BiCGSTAB.compute(Quu);
        //    compute_info = solver_BiCGSTAB.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::ConjugateGradient:
        //    solver_ConjugateGradient.setTolerance(meshLogicOptions.Solver.Tolerance);
        //    solver_ConjugateGradient.compute(Quu);
        //    compute_info = solver_ConjugateGradient.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::IncompleteLUT:
        //    solver_IncompleteLUT.setTolerance(meshLogicOptions.Solver.Tolerance);
        //    solver_IncompleteLUT.preconditioner().setDroptol(meshLogicOptions.Solver.Tolerance);
        //    solver_IncompleteLUT.compute(Quu);
        //    compute_info = solver_IncompleteLUT.info();
        //    break;
        //    #endif
            #if Intel_MKL_SUPPORTED
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::PardisoLDLT:
            solver_PardisoLDLT.compute(Quu);
            compute_info = solver_PardisoLDLT.info();
            break;
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::PardisoLU:
            solver_PardisoLU.compute(Quu);
            compute_info = solver_PardisoLU.info();
            break;
            #endif
    }
    return compute_info;
}
ComputationInfo AnySolver_Complex2x::solve(const Matrix<Scalar, Dynamic, Dynamic, ColMajor>& rhsM, Matrix<Scalar, Dynamic, Dynamic, ColMajor>& bM)
{
    SparceMatrixType rhs;
    SparceMatrixType  b;

    //#if alternative_solvers_SUPPORTED
    //switch (Type)
    //{
    //    case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
    //    case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
    //        //no need in convertion to sparce view
    //        break;
    //    default:
    //        rhs = rhsM.sparseView();
    //        break;
    //}
    //#endif

    ComputationInfo solve_info;
    switch (Type)
    {
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
        default:
            solver_LDLTFloat.solve(rhsM, bM);
            solve_info = solver_LDLTFloat.info();
            break;
        //    #if alternative_solvers_SUPPORTED
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::SimplicialLLT:
        //    b = solver_SimplicialLLT.solve(rhs);
        //    solve_info = solver_SimplicialLLT.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::SimplicialLDLT:
        //    b = solver_SimplicialLDLT.solve(rhs);
        //    solve_info = solver_SimplicialLDLT.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::SparseLU:
        //    b = solver_SparseLU.solve(rhs);
        //    solve_info = solver_SparseLU.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::BiCGSTAB:
        //    b = solver_BiCGSTAB.solve(rhs);
        //    solve_info = solver_BiCGSTAB.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::ConjugateGradient:
        //    b = solver_ConjugateGradient.solve(rhs);
        //    solve_info = solver_ConjugateGradient.info();
        //    break;
        //case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::IncompleteLUT:
        //    b = solver_IncompleteLUT.solve(rhs);
        //    solve_info = solver_IncompleteLUT.info();
        //    break;
        //    #endif
            #if Intel_MKL_SUPPORTED
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::PardisoLDLT:
            b = solver_PardisoLDLT.solve(rhs);
            solve_info = solver_PardisoLDLT.info();
            break;
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::PardisoLU:
            b = solver_PardisoLU.solve(rhs);
            solve_info = solver_PardisoLU.info();
            break;
            #endif
    }

    //#if alternative_solvers_SUPPORTED
    //if (solve_info == Success)
    //{
    //    switch (Type)
    //    {
    //        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
    //        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
    //            //no need in convertion from sparce to matrix view
    //            break;
    //        default:
    //            Matrix<Scalar, Dynamic, Dynamic, ColMajor> bMM(b);
    //            bM = bMM;
    //            break;
    //    }
    //}
    //#endif

    return solve_info;
}




LinearEquationSolver2x::LinearEquationSolver2x()
    : compute_info(ComputationInfo::InvalidInput), solve_info(ComputationInfo::InvalidInput)
{
    solver = new AnySolver_Complex2x(options.Solver);
}

LinearEquationSolver2x::~LinearEquationSolver2x()
{
    delete solver;
}

bool LinearEquationSolver2x::IsAlreadyComputed()
{
    return compute_info != ComputationInfo::InvalidInput;
}

bool LinearEquationSolver2x::compute(const SparceMatrixType &Quu, bool logMessages)
{
    if (logMessages) cout << "  (compute ";

    //
    // compute
    //
    Timer timer_compute;
    compute_info = solver->compute(Quu);
    //timer_compute.stop(elapsed.compute, elapsedS.minQuadWithKnownMini_compute);
    timer_compute.stop(elapsedS.minQuadWithKnownMini_compute);

    //
    // return results
    //
    if (logMessages) cout << "" << timer_compute.ElapsedSecondsStr() << ")";
    if (compute_info != Success)
    {
        std::cerr << "!!!   LinearEquationSolver - compute failed" << std::endl;
        return false;
    }
    return true;
}

bool LinearEquationSolver2x::solve(const Is& isFaceConstrained, const SparceMatrixType &Quk,
    const VectorType &xknown,
    VectorType &x,
    bool logMessages)
{
    if (compute_info != Success)
    {
        return false;
    }

    if (logMessages) cout << "  (solve ";


    //
    // solve
    //
    Timer timer_solve;
    //const SparceMatrixType &f;
    //igl::slice(f, unknown, Is::Zero(1, 1), fu);
    //SparceMatrixType rhs = (Quk* xknown).sparseView() + 0.5*fu;
    Matrix<Scalar, Dynamic, Dynamic, ColMajor> rhsM = (Quk* xknown);
    //for (int i = 0; i < rhsM.rows(); i++)
    //{
    //    Scalar zero = 0;
    //    cout << "rhsM#" << i << ";  " << (int)rhsM(i, 0).real()<<"," << (int)rhsM(i, 0).imag() << endl;
    //}
    Matrix<Scalar, Dynamic, Dynamic, ColMajor> bM;
    solve_info = solver->solve(-rhsM, bM);
    //timer_solve.stop(elapsed.solve, elapsedS.minQuadWithKnownMini_solve);
    timer_solve.stop(elapsedS.minQuadWithKnownMini_solve);


    //
    // return results
    //

    if (logMessages) cout << "" << timer_solve << ")";
    if (solve_info != Success)
    {
        std::cerr << "!!!   LinearEquationSolver - solve failed" << std::endl;
        return false;
    }



    x.resize(isFaceConstrained.size());
    // v0
    //int indk = 0;
    //int indu = 0;
    //for (int i = 0; i < isFaceConstrained.size(); ++i)
    //{
    //    if (isFaceConstrained[i])
    //    {
    //        x[i] = xknown[indk++];
    //    }
    //    else
    //    {
    //        x[i] = bM(indu++);
    //    }
    //}

    // v1 - faster version using pointers
    Scalar* px = x.data();
    const int* pisFaceConstrained = isFaceConstrained.data();
    const int* pisFaceConstrainedEnd = pisFaceConstrained + isFaceConstrained.size();
    const Scalar* pk = xknown.data();
    const Scalar* pu = bM.data();
    if (pisFaceConstrained < pisFaceConstrainedEnd)
    {
        do
        {
            if (*pisFaceConstrained)
            {
                *px = *pk;
                pk++;
            }
            else
            {
                *px = *pu;
                pu++;
            }
            px++;
            pisFaceConstrained++;
        } while (pisFaceConstrained < pisFaceConstrainedEnd);
    }

    return true;
}

