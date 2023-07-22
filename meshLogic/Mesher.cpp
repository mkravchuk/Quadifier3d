#include "stdafx.h"
#include "Mesher.h"
#include "ModelFile.h"
#include "ModelObject.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshSurface.h"
#include "MeshTopology.h"
#include "_MeshLogicDraw.h"
#include "MeshStreamsJoiner.h"
#include "DividerIterator.h"
#include "DividerOptimalConnector.h"
#include "DividerLogicConnector.h"


const MeshLogicOptions_Mesher& options = meshLogicOptions.Mesher;

  


Mesher::Mesher(const ModelFile& _file)
    :isDividedIntoQuadMeshes(false), file(_file), meshSize(0), meshGenerator(_file, draw)
{
}

II Mesher::SizeOF() const
{
    II r = sizeof(Mesher);
    r += draw.SizeOF();
    for (const auto& d : dividers) r += d.SizeOF();
    for (const auto& md : meshCutters) r += md.SizeOF();
    r += meshGenerator.SizeOF();
    return r;
}


D Mesher::GetMeshSize(const vector<ModelObject*>& objects)
{
    if (options.is_meshSize_RelativeToTrianglesSize && objects.size() > 0)
    {
        D sum_avg_edge_length = 0;
        for (ModelObject* obj : objects)
        {
            sum_avg_edge_length += obj->srf.mesh.avg_edge_length;
        }
        D avg_edge_length = sum_avg_edge_length / objects.size();
        return avg_edge_length * options.meshSize_RelativeToTrianglesSize;
    }
    return options.meshSize_absolute;
}


void Mesher::ClearCache()
{
    isDividedIntoQuadMeshes = false;
    meshSize = 0;
    dividers.clear();
    meshCutters.clear();
    meshGenerator.ClearCache();
}





vector<const Mesh*> Mesher::GetCuttedMeshes()
{
    vector<const Mesh*> cutedMeshes;
    int cutedMeshesTotal = 0;
    for (const auto& md : meshCutters) cutedMeshesTotal += md.cutedMeshes.size();
    cutedMeshes.reserve(cutedMeshesTotal);
    for (const auto& md : meshCutters)
    {
        for (int i = 0; i < md.cutedMeshes.size(); i++)
        {
            if (options.DebugEnabled && options.debug_debug_mesh_id != -1 && options.debug_debug_mesh_id != md.cutedMeshes[i].id) continue;
            if (md.cutedMeshes[i].V.rows() < 3) continue; // dont add empty or invalid meshes
            cutedMeshes.push_back(&md.cutedMeshes[i]);
        }
    }
    return cutedMeshes;
}



// *******************************************************************************
// *******************************************************************************
//  DivideComplexMeshes
// *******************************************************************************
// *******************************************************************************

void Mesher::DivideComplexMeshesIntoQuadMeshes(vector<ModelObject*> objects)
{
    // clear mesher (makes possible use 'meshSize' property, and removes previous cached information)
    ClearCache();

    //  recalculate mesh size since 'file.ActiveObjects' could be changed
    meshSize = GetMeshSize(objects);
    cout << "meshSize = " << meshSize << endl;

    extern bool IsOmpEnabled;

    //
    // Divide
    //
    if (meshLogicOptions.Divider.Enabled)
    {
        // get all sorvers
        vector<const MeshSolverNrosy*> solvers;
        solvers.reserve(objects.size());
        for (int i = 0; i < objects.size(); i++)
        {
            auto obj = objects[i];
            solvers.push_back(&obj->srf.solver);
        }

        cout << "Dividing... ";
        Timer timer_Dividing;

        // form list of dividers to support multithreading
        for (int i = 0; i < objects.size(); i++)
        {
            auto obj = objects[i];
            dividers.push_back(Divider(file.Topology, obj->srf));
        }

        // create map to easy access divider by meshid
        map<int, Divider*> map_meshid_divider;
        for (int i = 0; i < dividers.size(); i++)
        {
            map_meshid_divider[dividers[i].mesh.id] = &dividers[i];
        }

        // get all vids involved in MeshTopologyLoopCurveSegment as start and end points and update each divide, so when dividing those points will be used (so this logic must go before dividers[i].Divide();)
        for (const auto& c : file.Topology.Connections) // for all connections
        {
            if (c.segments.size() < 2) continue; //segment should have a friend to be able define pseudo sharp point
            for (const auto& s : c.segments)
            {
                if (s.EdgesCount == s.curve.EdgesCount && s.curve.isCyclic)
                {
                    cout << "!!!!!!!!!!!!            s.EdgesCount == s.curve.EdgesCount && s.curve.isCyclic  for meshid=" << s.meshid << endl;
                    continue; // dont add cyclic point as pseudo sharp point
                }
                vector<int>& vids = map_meshid_divider[s.meshid]->vertexesIdInvolvedInTopologySegments;
                vids.push_back(s.pointStart_vertexId); // we can add duplicates, since they will be ignored by Divider
                vids.push_back(s.pointEnd_vertexId); // we can add duplicates, since they will be ignored by Divider
            }
        }

        // divide
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < dividers.size(); i++)
        {
            cout << ".";
            dividers[i].Divide(true);
        }
        timer_Dividing.stop(elapsedTimers.Mesher_DivideComplexMeshes.Dividing);
        //cout << "Dividing time:   " << timer_Dividing << std::endl;
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Dividing << endl; else cout << endl;

        //
        // Joiner
        //
        if (meshLogicOptions.Joiner.Enabled)
        {
            cout << "Joining streams... ";
            Timer timer_Joining;
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < dividers.size(); i++)
            {
                cout << ".";
                MeshStreamsJoiner joiner(dividers[i], meshSize);
                joiner.JoinStreams();
            }
            timer_Joining.stop(elapsedTimers.Mesher_DivideComplexMeshes.JoiningStreams);
            if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Joining << endl; else cout << endl;
        }

        //
        // MeshStreamsTopology
        //
        SSMeshes ssmeshes(file.Topology, draw, dividers, solvers, meshSize);
        if (meshLogicOptions.MeshStreamsTopology.Enabled
            //&& (meshLogicOptions.DividerIterator.Enabled
            //|| meshLogicOptions.DividerLogicConnector.Enabled
            //|| meshLogicOptions.DividerOptimalConnector.Enabled)
            )
        {
            cout << "Constructing streams topology... ";
            Timer timer_StreamsTopology;
            ssmeshes.Build();
            timer_StreamsTopology.stop(elapsedTimers.Mesher_DivideComplexMeshes.StreamTopology);
            if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_StreamsTopology << endl; else cout << endl;
        }

        //
        // DividerIterator
        //
        if (meshLogicOptions.DividerIterator.Enabled)
        {
            cout << "Iterating dividing and joining streams... ";
            Timer timer_DividerIterator;
            DividerIterator iterator(file.Topology, draw, dividers, solvers, meshSize);
            iterator.Divide();
            timer_DividerIterator.stop(elapsedTimers.Mesher_DivideComplexMeshes.DividingIterator);
            if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DividerIterator << endl; else cout << endl;
        }

        //
        // DividerLogicConnector
        //
        if (meshLogicOptions.DividerLogicConnector.Enabled)
        {
            cout << "Solving logic connections ... ";
            Timer timer_LogicSolver;
            DividerLogicConnector logicSolver(file.Topology, draw, dividers);
            logicSolver.Solve();
            logicSolver.DrawDebug();

            timer_LogicSolver.stop(elapsedTimers.Mesher_DivideComplexMeshes.DividerLogicConnector);
            if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_LogicSolver << endl; else cout << endl;
        }

        //
        // DividerOptimalConnector
        //
        if (meshLogicOptions.DividerOptimalConnector.Enabled)
        {
            //cout << endl << "----- Iterating dividing and joining streams -----" << endl << endl;
            cout << "Solving dividing positions ... ";
            Timer timer_DividerSolver;
            DividerOptimalConnector optimalSolver(draw, ssmeshes);
            optimalSolver.Solve();

            timer_DividerSolver.stop(elapsedTimers.Mesher_DivideComplexMeshes.DividerOptimalConnector);
            if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DividerSolver << endl; else cout << endl;
        }


        ssmeshes.DrawDebug();

        //
        // MeshCutter
        //
        if (meshLogicOptions.MeshCutter.Enabled || meshLogicOptions.Mesher.Enabled)
        {
            //cout << endl << "----- Cutting mesh -----" << endl << endl;
            cout << "Cutting mesh... ";
            Timer timer_CuttingMesh;
            meshCutters.reserve(dividers.size());
            for (int i = 0; i < dividers.size(); i++)
            {
                auto obj = objects[i];
                meshCutters.push_back(MeshCutter(obj->srf.draw, dividers[i].mesh, true, true, true));
            }
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < dividers.size(); i++)
            {
                cout << ".";
                meshCutters[i].Cut(dividers[i].streams.streams);
            }
            timer_CuttingMesh.stop(elapsedTimers.Mesher_DivideComplexMeshes.CuttingMesh);
            //cout << "Cutting mesh time:   " << timer_CuttingMesh << std::endl;
            if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_CuttingMesh << endl; else cout << endl;
        }

    }

    isDividedIntoQuadMeshes = true;
}



// *******************************************************************************
// *******************************************************************************
//  GenerateMesh
// *******************************************************************************
// *******************************************************************************

void Mesher::GenerateMesh()
{
    if (!isDividedIntoQuadMeshes)
    {
        cout << "!!! Mesher::GenerateMesh - cannot generate mesh since complex meshes are not divided into quad meshes" << endl;
        meshGenerator.ClearCache();
        return;
    }
    Timer timer_Meshing;
    meshGenerator.GenerateMesh(GetCuttedMeshes(), meshSize);
    timer_Meshing.stop(elapsedTimers.Mesher_GenerateMesh.Total);
}

PolygonMeshes Mesher::GetPolygonMeshes()
{
    if (!isDividedIntoQuadMeshes || !meshGenerator.isMeshGenerated)
    {
        cout << "!!! Mesher::GetPolygonMeshes - cannot get polygon meshes since meshes are generated" << endl;
        meshGenerator.ClearCache();
        return PolygonMeshes();
    }
    return meshGenerator.GetPolygonMeshes();
}


// *******************************************************************************
// *******************************************************************************
//  Draw
// *******************************************************************************
// *******************************************************************************




void Mesher::ClearDrawObjects()
{
    draw.Clear();
    meshGenerator.topology.draw.Clear();
}


void Mesher::DrawAll()
{
    if (!options.Enabled) return;

    if (options.DebugEnabled)
    {
        if (options.debug_show_meshsize_on_borders)
        {
            const auto& connections = file.Topology.Connections;
            for (const auto& con : file.Topology.Connections)
            {
                if (con.segments.size() == 0)
                {
                    assert(con.segments.size() != 0 && "segments array must be populated");
                    continue;
                }
                const auto& seg = con.segments[0];
                D atLength = meshSize;
                while (atLength < seg.Length3d)
                {
                    P3 p = seg.GetPointAtLength(atLength);
                    draw.AddPoint(p, Color3d(0, 1, 0));
                    atLength += meshSize;
                }
            }

        }
    }

    for (auto& md : meshCutters)
    {
        for (int i = 0; i < md.cutedMeshes.size(); i++)
        {
            md.cutedMeshes[i].V_Info.InitMesh_VcenterProjectedIndside_IfRequiredByDraw(md.cutedMeshes[i]);
        }
    }
    meshGenerator.DrawAll();
}




