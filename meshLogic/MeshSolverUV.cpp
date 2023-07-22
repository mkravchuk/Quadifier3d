#include "stdafx.h"
#include "MeshSolverNrosy.h"
#include "MeshSolverUV.h"
#include "ViewerDrawObjects.h"
#include "Mesh.h"
#include "UVlscmSolver.h"
#include "UVmiqSolver.h"


MeshLogicOptions_SolverUV& options = meshLogicOptions.SolverUV;

MeshSolverUV::MeshSolverUV(const Mesh& _mesh, ViewerDrawObjects& _draw)
    : mesh(_mesh), draw(_draw), Constrains(_mesh, _draw), Result(_mesh)
{
}


void MeshSolverUV::Solve(bool logMessages)
{
    MeshLogicOptions_SolverUV::AlgorithmType solverType = options.Algorithm;

    //
    // init constains
    //
    if (Constrains.Constrains.size() == 0)
    {
        //auto time = utils::time::Now();
        //cout << "detectBorderConstraines ...";
        Timer t;
        Constrains.InitFromMeshLoop();
        t.stop(elapsedTimers.SolverUV.ConstrainsInit);
        //if (meshLogicOptions.showTimeElapsed) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl; else cout << endl;
    }

    Result.SolverType = solverType;
    Result.Clear();
    if (Constrains.Constrains.size() == 0)
    {
        cout << "!warning   MeshSolverUV::Solve  failed to generate constrains for mesh " << mesh.GetMeshIdStr() << endl;
        return;
    }

    //
    // solve
    //
    MatrixXf resultUV;
    if (solverType == MeshLogicOptions_SolverUV::AlgorithmType::lscm)
    {
        UVlscmSolver lscm(draw, mesh.V, mesh.F);
        vector<UVlscmSolverConstrain> constrains;
        constrains.reserve(Constrains.Constrains.size());
        for (const auto& c : Constrains.Constrains)
        {
            if (c.Type == MeshPointType::onVertex)
            {
                constrains.push_back({ c.vid_eid_fid, c[0], c[1] });
            }
            else
            {
                cout << "!warning   MeshSolverUV::Solve  lscm solver doesnt support constrains on edge" << endl;
            }
        }
        lscm.Solve(constrains, resultUV);

    }
    #if UVmiqSolver_SUPPORTED
    else if (solverType == MeshLogicOptions_SolverUV::AlgorithmType::miq)
    {
        // v0S
        //MeshSolverNrosy solver(mesh, draw);
        //solver.Solve(logMessages);
        //if (!solver.Result.isEmpty())
        //{
        //    UVmiqSolver miq(draw, mesh.V, mesh.F);
        //    miq.Solve(solver.Result.Field, resultUV);
        //}

        //v1
        MeshConstrains Constrains(mesh, draw);
        Constrains.Init(meshLogicOptions.Constrains.Type);
        if (Constrains.Constrains.size() != 0)
        {
            Is b;
            MatrixXd bc;
            b.resize(Constrains.Constrains.size());
            bc.resize(Constrains.Constrains.size(), 3);
            for (int i = 0; i < Constrains.Constrains.size(); ++i)
            {
                int fid = Constrains.Constrains[i].FaceId;
                V3 constrDirection = Constrains.Constrains[i].DirectionY_Corrected;
                b(i) = fid;
                bc.row(i) = convertV3ToEigenDouble(constrDirection);
            }
            UVmiqSolver miq(draw, mesh.V, mesh.F);
            miq.Solve(b, bc, resultUV);
        }
    }
    #endif
    else
    {
        cout << "!warning   MeshSolverUV::Solve  unknown algorithm type" << endl;
    }

    //
    // copy results from solver
    //
    if (resultUV.rows() == mesh.V.rows())
    {
        Result.Field.clear();
        Result.Field.reserve(mesh.V.rows());
        for (int vid = 0; vid < resultUV.rows(); vid++)
        {
            Vector2f uv = resultUV.row(vid);
            Result.Field.push_back({ uv(0), uv(1) });
        }
    }

    //
    // DEBUG add constrains uv if solver failed or is not implemented yet
    //
    if (options.DebugEnabled && Result.Field.size() != mesh.V.rows())
    {
        cout << "!warning   MeshSolverUV::Solver  failed to generate results - adding only constrains uv just for debug information" << endl;
        Result.Field.resize(mesh.V.rows());
        for (const auto& c : Constrains.Constrains)
        {
            if (c.Type == MeshPointType::onVertex)
            {
                Result.Field[c.vid_eid_fid] = c.uv;
            }
        }
    }
}


void MeshSolverUV::Clear()
{
    Constrains.Clear();
    Result.Clear();
}

II MeshSolverUV::SizeOF() const
{
    II r = sizeof(MeshSolverUV);

    r += Constrains.SizeOF();
    r += Result.Field.size() * sizeof(UV);

    return r;
}