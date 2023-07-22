#include "stdafx.h"
#include "LinearEquationSolver_Polar2x.h"
#include "SolverTypes_Polar2x.h"
//#include "MeshLinearSolver.h"
#include "LDLTFloatSolver_Polar2x.h"


#if Intel_MKL_SUPPORTED
#include <Eigen/PardisoSupport>
#endif

//const MeshLogicOptions_LinearEquationSolver& options = meshLogicOptions.LinearEquationSolver;
//Elapsed_LinearEquationSolver& elapsed = elapsedTimers.LinearEquationSolver;
//Elapsed_Solver& elapsedS = elapsedTimers.Solver;
#define options meshLogicOptions.LinearEquationSolver
#define elapsed elapsedTimers.LinearEquationSolver


// holds many of linear sparce equation solvers
// allows other classes to use any solver dynamicaly using common methods 'compute' and 'solve'
class AnySolver_Polar2x
{
public:
    LDLTFloatSolver_Polar2x solver_LDLTFloat;
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
    AnySolver_Polar2x(MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType type);
    MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType Type;
    ComputationInfo compute(const SparceMatrixType &Quu, bool reuse_ordering = false);
    ComputationInfo solve(const SolveMatrixTypeComplex& rhsM, SolveMatrixTypeComplex& bM);
};

AnySolver_Polar2x::AnySolver_Polar2x(MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType type)
    :Type(type)
{
}

ComputationInfo AnySolver_Polar2x::compute(const SparceMatrixType &Quu, bool reuse_ordering)
{
    ComputationInfo compute_info;
    switch (Type)
    {
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
        default:
            solver_LDLTFloat.compute(Quu, reuse_ordering);
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
ComputationInfo AnySolver_Polar2x::solve(const SolveMatrixTypeComplex& rhsM, SolveMatrixTypeComplex& bM)
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
    //            SolveMatrixType bMM(b);
    //            bM = bMM;
    //            break;
    //    }
    //}
    //#endif

    return solve_info;
}




LinearEquationSolver_Polar2x::LinearEquationSolver_Polar2x(ViewerDrawObjects& _draw)
    : draw(_draw), compute_info(ComputationInfo::InvalidInput), solve_info(ComputationInfo::InvalidInput)
{
    solver = new AnySolver_Polar2x(options.Solver);
}

LinearEquationSolver_Polar2x::~LinearEquationSolver_Polar2x()
{
    delete solver;
}

bool LinearEquationSolver_Polar2x::IsAlreadyComputed()
{
    return compute_info != ComputationInfo::InvalidInput;
}

bool LinearEquationSolver_Polar2x::compute(const SparceMatrixType &Quu, bool logMessages, TimeElapsed& elapsed_compute, bool reuse_ordering)
{
    if (logMessages) cout << "  (compute ";

    //
    // compute
    //
    Timer timer_compute;
    compute_info = solver->compute(Quu, reuse_ordering);
    timer_compute.stop(elapsed_compute);

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

bool LinearEquationSolver_Polar2x::solve(const Bs& isFaceConstrained, const SparceMatrixTypeComplex &Quk,
    const VectorTypeComplex& xknown, const VectorTypeComplex& f,
    VectorTypeComplex& x,
    const Is& known_index_to_fid, const Is& unknown_index_to_fid,
    bool logMessages, TimeElapsed& elapsed_solve)
{
    if (compute_info != Success)
    {
        return false;
    }

    if (logMessages) cout << "  (solve ";

    //int count = isFaceConstrained.rows();
    //int count_known = xknown.rows();

    //
    // solve
    //
    Timer timer_solve;
    SolveMatrixTypeComplex rhsM;
    if (f.rows() == 0)
    {
        rhsM = Quk * xknown;
    }
    else
    {
        VectorTypeComplex fu(unknown_index_to_fid.size(), 1);
        //igl::slice(f, unknown_index_to_fid, VectorXi::Zero(1, 1), fu);
        for (int i = 0; i < fu.rows(); i++)
        {
            fu[i] = f[unknown_index_to_fid[i]] * 0.5;//  0.5 multiplayer keeps vectors at same length - very visible for 'AngleBoundFFSolver_Polar' 
        }
        if (xknown.rows() == 0)
            rhsM = fu;
        else
            rhsM = Quk * xknown + fu;
    }
    //for (int i = 0; i < rhsM.rows(); i++)
    //{
    //    Scalar zero = 0;
    //    cout << "rhsM#" << i << ";  " << (int)rhsM(i, 0).real()<<"," << (int)rhsM(i, 0).imag() << endl;
    //}
    SolveMatrixTypeComplex bM;
    solve_info = solver->solve(rhsM, bM); // instead of sending '-rhsM' we will negate results by using '-*pu' - speed optimization
    timer_solve.stop(elapsed_solve);


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
    //        x[i] = -bM(indu++);// use '-bM' instead of sending '-rhsM' to the solver - speed optimization
    //    }
    //}

    // v1 - faster version using pointers
    const ScalarComplex* pk = xknown.data();
    const ScalarComplex* pu = bM.data();
    int known_index = 0;
    int unknown_index = 0;
    for(int i = 0; i < isFaceConstrained.size() ;i++)
    {
            if (isFaceConstrained[i])
            {
                int fid = known_index_to_fid[known_index];
                x[fid] = *pk;
                known_index++;
                pk++;
            }
            else
            {
                int fid = unknown_index_to_fid[unknown_index];
                x[fid] = -(*pu); // use '-(*pu)' instead of sending '-rhsM' to the solver - speed optimization
                unknown_index++;
                pu++;
            }
    }

    return true;
}

vector<int> LinearEquationSolver_Polar2x::Get_compute_unknown_indexes()
{
    vector<int> unknown_indexes;
    for (auto ap_index : solver->solver_LDLTFloat.compute_ap_indexes)
    {
        int unknown_index = solver->solver_LDLTFloat.m_Pinv.indices()[ap_index];
        unknown_indexes.push_back(unknown_index);
    }
    return unknown_indexes;
}

void LinearEquationSolver_Polar2x::copy_ordering_from_another_solver(const LinearEquationSolver_Polar2x& another_solver)
{
    switch (options.Solver)
    {
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
        default:
            solver->solver_LDLTFloat.m_P = another_solver.solver->solver_LDLTFloat.m_P;
            solver->solver_LDLTFloat.m_Pinv = another_solver.solver->solver_LDLTFloat.m_Pinv;
            break;
    }
}

void LinearEquationSolver_Polar2x::copy_ordering_from_another_solver__from_limited_to_full(const LinearEquationSolver_Polar2x& another_solver, const Is& known_index_to_fid, const Is& unknown_index_to_fid)
{
    if (known_index_to_fid.size() == 0)
    {
        copy_ordering_from_another_solver(another_solver);
        return;
    }

    switch (options.Solver)
    {
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::Default:
        case MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat:
        default:
            // with this way of createion m_P method 'compute' works slower then just rebuilding m_P from scratch - so this idea fail (maybe it is possible to srink m_P, but extending it is not a good idea)

            //auto& m_P = solver->solver_LDLTFloat.m_P;
            //auto& m_P_another = another_solver.solver->solver_LDLTFloat.m_P;
            //auto& m_Pinv_another = another_solver.solver->solver_LDLTFloat.m_Pinv;
            //if (m_P_another.indices().size() != unknown_index_to_fid.size())
            //{
            //    assert(m_P_another.indices().size() == a.size() && "size must be same");
            //    cout << "! warning   LinearEquationSolver_Polar2x::copy_ordering_from_another_solver__from_limited_to_full   size must be same" << endl;
            //    return;
            //}
            //m_P.resize(unknown_index_to_fid.size() + known_index_to_fid.size());
            //int* pindexes = m_P.indices().data();
            //int* pindexes_another = m_P_another.indices().data();
            //for (int i = 0; i < known_index_to_fid.size(); i++)
            //{
            //    *pindexes = known_index_to_fid[i];
            //    pindexes++;
            //}
            //for (int i = 0; i < unknown_index_to_fid.size(); i++)
            //{
            //    *pindexes = unknown_index_to_fid[*pindexes_another];
            //    pindexes++;
            //    pindexes_another++;
            //}

            //solver->solver_LDLTFloat.m_Pinv = m_P.inverse();
            break;
    }
}

