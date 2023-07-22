#include "stdafx.h"
#include "MeshGenerator.h"
#include "ModelFile.h"
#include "ModelObject.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshSurface.h"
#include "MeshTopology.h"
#include "_MeshLogicDraw.h"
#include "MeshStreamsJoiner.h"
#include "DividerIterator.h"



MesherLoopConnection::MesherLoopConnection(int index, TopologyConnection* _connection)
    : Index(index), connection(_connection), Length(_connection->segments[0].Length3d)
{
}

int MesherLoopConnection::GetDivisioncount(D meshSize)
{
    return static_cast<int>(round(max(Length / meshSize, 1.0)));
}

P3 MesherLoopConnection::MiddlePoint()
{
    return connection->segments[0].MiddlePoint();
}

MesherTopologyLoop::MesherTopologyLoop(int index, const Topology& _topology)
    : Index(index), topology(_topology), isCyclic(false)
{

}

int MesherTopologyLoop::GetDivisioncount(D meshSize)
{
    int maxDiv = 1;
    for (auto& c : connections)
    {
        int div = c.GetDivisioncount(meshSize);
        //if (Index == 25)
        //{
        //    cout << "Loop " << Index << "    c.Length=" << c.Length << "  meshSize=" << meshSize << "  div=" << div << endl;
        //}
        if (div > maxDiv)
        {
            maxDiv = div;
        }
    }

    return maxDiv;
}

II MesherTopologyLoop::SizeOF() const
{
    II r = sizeof(MesherTopologyLoop);
    r += connections.size() * sizeof(MesherLoopConnection);
    return r;
}

bool TwoOppositeSegmentsPair::Sort()
{
    isSorted = false;

    // validate input
    if (X.s0 == nullptr || Y.s0 == nullptr)
    {
        cout << "!!! incorrect sorting in TwoOppositeSegmentsPair:  X.s0 == nullptr || Y.s0 == nullptr" << endl;
        assert((X.s0 != nullptr && Y.s0 != nullptr) && "incorrect sorting in TwoOppositeSegmentsPair:  X.s0 == nullptr || Y.s0 == nullptr");
        return false;
    }

    // X will have longest lines, Y will have shortest lines
    if (X.sumLength3d < Y.sumLength3d)
    {
        swap(X.sumLength3d, Y.sumLength3d);
        swap(X.divisioncount, Y.divisioncount);
        swap(X.s0, Y.s0);
        swap(X.s1, Y.s1);
    }

    // ensure Y1 will be on right hand from X0
    if (X.s0->PrevSegmentInLoop == Y.s1)
    {
        swap(Y.s0, Y.s1);
    }
    if (X.s0->PrevSegmentInLoop == X.s1)
    {
        swap(X.s0, X.s1);
    }

    // validate sorting
    if (Y.s0 == nullptr)
    {
        cout << "!!! incorrect sorting in TwoOppositeSegmentsPair:  Y.s0 == nullptr" << endl;
        assert(Y.s0 != nullptr && "incorrect sorting in TwoOppositeSegmentsPair:  Y.s0 == nullptr");
        return false;
    }
    if (Y.s0->PrevSegmentInLoop != X.s1)
    {
        cout << "!!! incorrect sorting in TwoOppositeSegmentsPair:  Y.s0->PrevSegmentInLoop != X.s1" << endl;
        assert(Y.s0->PrevSegmentInLoop == X.s1 && "incorrect sorting in TwoOppositeSegmentsPair:  Y.s0->PrevSegmentInLoop != X.s1");
        return false;
    }
    if (Y.s0->NextSegmentInLoop != X.s0)
    {
        cout << "!!! incorrect sorting in TwoOppositeSegmentsPair:  Y.s0->NextSegmentInLoop != X.s0" << endl;
        assert(Y.s0->NextSegmentInLoop == X.s0 && "incorrect sorting in TwoOppositeSegmentsPair:  Y.s0->NextSegmentInLoop != X.s0");
        return false;
    }

    // rearange X segments so longest segment from Y will be on the left hand from first segment from X
    D Y0Len = Y.s0->Length3d;
    D Y1Len = Y.s1 != nullptr ? Y.s1->Length3d : 0;
    if (Y0Len < Y1Len)
    {
        swap(X.s0, X.s1);
        swap(Y.s0, Y.s1);
    }
    isSorted = true;
    return true;
}

// ***************************************************
// MeshGenerator
// ***************************************************


MeshGenerator::MeshGenerator(const ModelFile& _file, ViewerDrawObjects& _draw)
    : options(meshLogicOptions.Mesher), file(_file), draw(_draw), isMeshGenerated(false), meshSize(0)
{
}

II MeshGenerator::SizeOF() const
{
    II r = sizeof(MeshGenerator);
    r += topology.SizeOF();

    for (const auto& s : solvers) r += s.SizeOF();
    for (const auto& s : solversUV) r += s.SizeOF();
    for (const auto& d : dividers) r += d.SizeOF();
    for (const auto& p : polygonMeshes) r += p.SizeOF();
    for (const auto& l : Loops) r += l.SizeOF();

    return r;
}

void MeshGenerator::ClearCache()
{
    isMeshGenerated = false;
    meshes.clear();
    meshSize = 0;

    topology.Clear();
    solvers.clear();
    solversUV.clear();
    dividers.clear();
    polygonMeshes.clear();
    Loops.clear();
}

void MeshGenerator::DrawAll()
{
    draw.ReserveEdges(meshes.size() * 1000);
    draw.ReserveLabels(meshes.size() * 1000);
    draw.ReservePoints(meshes.size() * 1000);

    //
    // draw mesh
    //
    if (options.DebugEnabled && options.debug_draw_cuttedMesh)
    {
        topology.DrawAll();
        for (const auto& m : meshes)
        {
            //m->V_Info.Init_VcenterProjectedIndside(m->V);
            MeshLogicDraw drawer(*m, draw);
            drawer.DrawAll();
        }
    }

    //
    // draw loop debug info
    //
    if (options.DebugEnabled && isMeshGenerated)
    {
        int reserveEdges = 0;
        if (options.debug_show_meshing_process)
        {
            for (auto& d : dividers) reserveEdges += d.DebugShowStreams_GetAddedEdgesCount();
        }
        if (options.debug_show_produced_mesh)
        {
            GetPolygonMeshes().Draw_GetAddedEdgesCount();
        }
        draw.ReserveEdges(reserveEdges);


        for (auto& l : Loops)
        {
            if (options.debug_debug_meshingLoop_id != -1 && options.debug_debug_meshingLoop_id != l.Index) continue;
            for (auto& c : l.connections)
            {
                string text = "";
                if (options.debug_show_meshingLoops_ids)
                {
                    if (!empty(text)) text += "  ";
                    text += "#" + to_string(l.Index) + "." + to_string(c.Index);
                }
                if (options.debug_show_divisioncount_before_adjustment)
                {
                    if (!empty(text)) text += "  ";
                    int divisioncount = c.GetDivisioncount(meshSize);
                    text += ":" + to_string(divisioncount);
                }
                if (options.debug_show_divisioncount_after_adjustment)
                {
                    if (!empty(text)) text += "  ";
                    int divisioncount = l.GetDivisioncount(meshSize);
                    text += "::" + to_string(divisioncount);
                }
                draw.AddLabel(c.MiddlePoint(), text, Color3d(0, 0, 1), 4);
            }
        }

        if (options.debug_show_meshing_process)
        {
            for (auto& d : dividers)
            {
                d.DebugShowStreams();
            }
        }

        if (options.debug_show_produced_mesh)
        {
            //for (auto& p : polygonMeshes)
            //{
            //    p.Draw(draw);
            //}
            GetPolygonMeshes().Draw(draw);
        }
    }
}


// *******************************************************************************
// *******************************************************************************
//  GenerateMesh
// *******************************************************************************
// *******************************************************************************

const MeshTopologyLoopCurveSegment* MeshGenerator::GetOppositeSegment(const MeshTopologyLoopCurveSegment* segment)
{
    //debug properties
    //const auto& con = *segment->topologyConnection;
    //auto& segmentOpposite = *segment->ConnectedTo;
    //int meshid = segment->curve.mesh.id;

    auto fail = [&]()
    {
        #if DEBUG
        cout << "! warning:  sOpposite == nullptr      mesh.id=" << segment->meshid << "   segment.id=" << segment->id << endl;
        #endif
    };

    // only 1 loop allowed
    if (segment->curve.mesh.Loops.size() != 1)
    {
        fail();
        return nullptr;
    }

    // create list of all segments in mesh
    const MeshTopologyLoopCurveSegment*  segments[4];
    segments[0] = segment;
    int count = 1;
    MeshTopologyLoopCurveSegment* next = segments[0]->NextSegmentInLoop;
    while (next != segments[0])
    {
        if (count == 4)
        {
            fail();
            return nullptr;// fail for more than 4 segments
        }
        segments[count] = next;
        next = next->NextSegmentInLoop;
        count++;
    }

    // get lengths
    D lengths[4];
    int minLength_Index = -1;
    D minLength = 0;
    for (int i = 0; i < count; i++)
    {
        lengths[i] = segments[i]->Length3d;
        if (lengths[i] < minLength || minLength_Index == -1)
        {
            minLength = lengths[i];
            minLength_Index = i;
        }
    }

    // only meshes with 3 and 4 segments allowed
    if (count == 3)
    {
        if (minLength_Index == 0) return nullptr; // fail if current segment is shortest - shortest segment from 3 segments wont have opposite 
        if (minLength_Index == 1)
            return segments[2]; // return not shortest segment
        else
            return segments[1]; // return not shortest segment
    }
    if (count == 4)
    {
        return segments[2];
    }

    fail();
    return nullptr;
}


PolygonMeshes MeshGenerator::GetPolygonMeshes()
{
    cout << "GetPolygonMeshes... ";
    Timer timerTopology;
    PolygonMeshes res;
    vector<const PolygonMesh*>& polyMeshes = res.meshes;
    polyMeshes.reserve(polygonMeshes.size());// reserve space for better performance
    for (const auto& pm : polygonMeshes)
    {
        if (options.DebugEnabled && options.debug_debug_mesh_id != -1 && options.debug_debug_mesh_id != pm.groupid) continue;
        polyMeshes.push_back(&pm);
    }
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timerTopology << endl; else cout << endl;
    return res;
}

void MeshGenerator::GenerateMesh_UpdateTopology()
{
    cout << "Updating topology for cutted meshes... ";
    Timer timerTopology;
    topology.Update(meshes);
    timerTopology.stop(elapsedTimers.Mesher_GenerateMesh.Topology);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timerTopology << endl; else cout << endl;
}

void MeshGenerator::GenerateMesh_BuildLoops()
{
    cout << "Building loops for cutted meshes... ";
    Timer timer_BuildLoops;

    Loops.clear();

    vector<bool> connectionIsUsed(topology.Connections.size(), false);
    auto findUnusedConnection = [&]()
    {
        for (int i = 0; i < topology.Connections.size(); i++)
        {
            if (connectionIsUsed[i]) continue;
            return i;
        }
        return -1;
    };

    auto addConnections = [&](MesherTopologyLoop& loop, int startConnectionIndex, MeshTopologyLoopCurveSegment* s)
    {
        while (const MeshTopologyLoopCurveSegment* sOpposite = GetOppositeSegment(s))
        {
            if (sOpposite == nullptr) break; // stop if this is triangle mesh and segment has no friend
            if (sOpposite->ConnectedTo == nullptr) // if we reach border of the object (single connection)
            {
                loop.connections.push_back(MesherLoopConnection(loop.connections.size(), sOpposite->topologyConnection)); // add border connection
                break; // stop since there are no more friends
            }
            if (sOpposite->ConnectedTo->topologyConnection->Index == startConnectionIndex)// stop if this is cyclic loop
            {
                loop.isCyclic = true;
                break;
            }
            loop.connections.push_back(MesherLoopConnection(loop.connections.size(), sOpposite->topologyConnection)); // add next connection
            s = sOpposite->ConnectedTo;
        }
    };

    // until there are unused connection - repeat
    int connectionIndex = findUnusedConnection();
    while (connectionIndex != -1)
    {
        auto& startConnection = topology.Connections[connectionIndex];
        if (startConnection.segments.size() == 0) continue;
        if (startConnection.segments.size() > 2) continue;

        // start a new mesher loop
        Loops.push_back(MesherTopologyLoop(Loops.size(), topology));
        MesherTopologyLoop& loop = Loops.back();

        // add current connection to loop
        loop.connections.push_back(MesherLoopConnection(loop.connections.size(), &startConnection));

        //cout << "loop#"<< loop.Index<<"   startConnection=" << startConnection.id << endl;


        // add all friend connections to segment#0
        addConnections(loop, connectionIndex, &startConnection.segments[0]);
        if (!loop.isCyclic && startConnection.segments.size() > 1)
        {
            // revert loop to keep connection indexes consistent
            std::reverse(loop.connections.begin(), loop.connections.end());
            addConnections(loop, connectionIndex, startConnection.segments[0].ConnectedTo);
        }

        // mark all conection from this loop used
        for (int i = 0; i < loop.connections.size(); i++)
        {
            auto& cc = loop.connections[i];
            cc.Index = i;
            connectionIsUsed[cc.connection->Index] = true;
        }
        // find next unused connection
        connectionIndex = findUnusedConnection();
    }

    timer_BuildLoops.stop(elapsedTimers.Mesher_GenerateMesh.BuildLoops);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_BuildLoops << endl; else cout << endl;
}


void MeshGenerator::GenerateMesh_Solve_Streams()
{
    //
    // solve
    //
    cout << "Solving cutted meshes using 'Streams' algorithm... ";
    Timer timer_SolveTotal;
    solvers.clear();
    solvers.reserve(meshes.size());
    for (int i = 0; i < meshes.size(); i++)
    {
        solvers.push_back(MeshSolverNrosy(*meshes[i], draw));
    }
    //bool save_straightTo90 = meshLogicOptions.Constrains.CorrectInAngles_force_4side_to_straightTo90; // save options
    //meshLogicOptions.Constrains.CorrectInAngles_force_4side_to_straightTo90 = true;  // force
    extern bool IsOmpEnabled;
    bool logSolverMessages = false;// !IsOmpEnabled && (meshes.size() <= 10 && meshLogicOptions.Solver.DebugEnabled && meshLogicOptions.Solver.debug_trace_solver_performance_per_mesh);
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        if (!logSolverMessages && options.DebugEnabled) cout << ".";
        solvers[i].Solve(logSolverMessages);
    }
    //meshLogicOptions.Constrains.CorrectInAngles_force_4side_to_straightTo90 = save_straightTo90;// restore options
    timer_SolveTotal.stop(elapsedTimers.Mesher_GenerateMesh.Solver);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_SolveTotal << endl; else cout << endl;

    //
    // draw solver
    //
    if (options.DebugEnabled && options.debug_draw_cuttedMesh)
    {
        cout << "Draw solvers for cutted meshes... ";
        Timer timer_DrawSolver;
        for (int i = 0; i < meshes.size(); i++)
        {
            //if (!logSolverMessages && options.DebugEnabled) cout << ".";
            MeshLogicDraw drawer(*meshes[i], draw);
            drawer.draw_mesh_nrosy(solvers[i], logSolverMessages);
        }
        timer_DrawSolver.stop(elapsedTimers.Mesher_GenerateMesh.DrawSolver);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DrawSolver << endl; else cout << endl;
    }
}

void MeshGenerator::GenerateMesh_Solve_LSCM()
{
    //
    // solve
    //
    cout << "Solving cutted meshes using 'LSCM' algorithm... ";
    Timer timer_SolveTotal;
    solversUV.clear();
    solversUV.reserve(meshes.size());
    for (int i = 0; i < meshes.size(); i++)
    {
        solversUV.push_back(MeshSolverUV(*meshes[i], draw));
    }
    //bool save_straightTo90 = meshLogicOptions.Constrains.CorrectInAngles_force_4side_to_straightTo90; // save options
    //meshLogicOptions.Constrains.CorrectInAngles_force_4side_to_straightTo90 = true;  // force
    extern bool IsOmpEnabled;
    bool logSolverMessages = false;// !IsOmpEnabled && (meshes.size() <= 10 && meshLogicOptions.Solver.DebugEnabled && meshLogicOptions.Solver.debug_trace_solver_performance_per_mesh);
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        if (!logSolverMessages && options.DebugEnabled) cout << ".";
        solversUV[i].Solve(logSolverMessages);
    }
    //meshLogicOptions.Constrains.CorrectInAngles_force_4side_to_straightTo90 = save_straightTo90;// restore options
    timer_SolveTotal.stop(elapsedTimers.Mesher_GenerateMesh.Solver);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_SolveTotal << endl; else cout << endl;

    //
    // draw solver
    //
    if (options.DebugEnabled && options.debug_draw_cuttedMesh)
    {
        cout << "Draw solvers UV for cutted meshes... ";
        Timer timer_DrawSolver;
        for (int i = 0; i < meshes.size(); i++)
        {
            //if (!logSolverMessages && options.DebugEnabled) cout << ".";
            MeshLogicDraw drawer(*meshes[i], draw);
            drawer.draw_mesh_UV(solversUV[i], logSolverMessages);
        }
        timer_DrawSolver.stop(elapsedTimers.Mesher_GenerateMesh.DrawSolver);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DrawSolver << endl; else cout << endl;
    }

}

void MeshGenerator::GenerateMesh_Solve_Laplacian()
{
}

void MeshGenerator::GenerateMesh_Solve()
{
    switch (options.Algorithm)
    {
        case MeshLogicOptions_Mesher::AlgorithmType::StreamsOnCuttedMeshes:
            GenerateMesh_Solve_Streams();
            break;
        case MeshLogicOptions_Mesher::AlgorithmType::UVOnCuttedMeshes:
            GenerateMesh_Solve_LSCM();
            break;
        case MeshLogicOptions_Mesher::AlgorithmType::LaplacianOnLoopStreams:
            GenerateMesh_Solve_Laplacian();
            break;
        default:
            cout << "!!!error in MeshGenerator::GenerateMesh_Solve - undefined algorithm" << endl;
            break;
    }
}

void MeshGenerator::GenerateMesh_Divide_Streams(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
    cout << "Dividing cutted meshes... ";
    Timer timer_Dividing;

    bool debug_show_meshing_process = options.DebugEnabled && options.debug_show_meshing_process &&
        (meshLogicOptions.MeshStream.show_dividing_streams || meshLogicOptions.MeshStream.show_streamIndexes_atStart || meshLogicOptions.MeshStream.show_streamIndexes_InAllPoints || meshLogicOptions.MeshStream.show_streamIndexes_Every10Points || meshLogicOptions.MeshStream.show_stream_iterations);
    //
    // create dividers
    //
    dividers.clear();
    dividers.reserve(meshes.size());
    map<int, int> map_meshid_divederindex;
    for (int i = 0; i < meshes.size(); i++)
    {
        dividers.push_back(Divider(topology, *meshes[i], solvers[i], draw));
        map_meshid_divederindex[meshes[i]->id] = i;
    }


    //
    // show warnings
    //
    int cout_warning_count = 0;
    #if DEBUG
    for (int i = 0; i < solvers.size(); i++)
    {
        const Mesh* mesh = meshes[i];
        const MeshSolverNrosy& solver = solvers[i];
        if (solver.Result.Field.size() == 0)
        {
            if (cout_warning_count == 0) cout << endl;
            cout_warning_count++;
            cout << "! warning:  MeshGenerator::GenerateMesh_Divide()    solver.Result.Field is empty   mesh.id=" << mesh->id << endl;
        }
    }
    #endif

    //
    // addAnchors method
    //
    auto addAnchors = [&](const MeshTopologyLoopCurveSegment* s, const MeshTopologyLoopCurveSegment* sOpposite, int divisioncount)
    {
        if (s == nullptr) return;
        if (divisioncount <= 1) return;
        int index = map_meshid_divederindex[s->meshid];
        const Mesh* mesh = meshes[index];
        const MeshSolverNrosy& solver = solvers[index];
        Divider& divider = dividers[index];
        if (solver.Result.Field.size() == 0) return;
        D edgeLength = s->Length3d / divisioncount;
        for (int i = 1; i < divisioncount; i++)
        {
            D newPos = i * edgeLength;
            if (DividerIterator::addAnchorAndStartPoints(newPos, true, s, 0, *mesh, solver, divider.startPoints, divider.anchorPoints))
            {
                // set contract to force specific streams joing together
                if (sOpposite != nullptr)  // if segment has opposite segment, then streams from current segment must be connected to streams of their opposite segment
                {
                    auto& startStreamPoint = divider.startPoints.back();// get reference to currently added startStreamPoint
                    startStreamPoint.haveContract = true;
                    startStreamPoint.contract = { s->id, i };

                    auto& anchorPoint = divider.anchorPoints.back(); // get reference to currently added anchorPoint
                    anchorPoint.haveContract = true;
                    anchorPoint.contract = { sOpposite->id, divisioncount - i };
                }
                //draw.AddLabel(divider.startPoints.back().point, to_string(i));
            }
            else
            {
                #if DEBUG
                if (cout_warning_count == 0) cout << endl;
                cout_warning_count++;
                cout << "! warning:  failed add anchor   mesh.id=" << mesh->id << "   segment.id=" << s->id << endl; // << "    loop.Index=" << loop.Index
                #endif
            }
        }
    };

    //
    // add start and anchor points (old stream for X and Y)
    //
    //for (auto& loop : Loops)
    //{
    //    int divisioncount = loop.GetDivisioncount(meshSize);
    //    if (divisioncount <= 1) continue;
    //    for (auto& c : loop.connections)
    //    {
    //        for (auto& s : c.connection->segments)
    //        {
    //            const MeshTopologyLoopCurveSegment* sOpposite = GetOppositeSegment(&s);
    //            addAnchors(&s, sOpposite, divisioncount);
    //        }
    //    }
    //}

    //
    // add start and anchor points (new stream only for X)
    //
    auto addSS = [&](const TwoOppositeSegments& ss)
    {
        const MeshTopologyLoopCurveSegment* s = ss.s0;
        const MeshTopologyLoopCurveSegment* sOpposite = ss.s1;
        addAnchors(s, sOpposite, ss.divisioncount);
        addAnchors(sOpposite, s, ss.divisioncount);
    };
    for (auto& o : oppositeSegments)
    {
        addSS(o.X);
        if (debug_show_meshing_process && options.debug_show_meshing_process_Y)
        {
            addSS(o.Y);
        }
    }


    //
    // divide
    //
    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < dividers.size(); i++)
    {
        if (options.DebugEnabled) cout << ".";
        dividers[i].Divide(false);
    }
    timer_Dividing.stop(elapsedTimers.Mesher_GenerateMesh.Dividing);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Dividing << endl; else cout << endl;

    //
    // join
    //
    cout << "Joining streams... ";
    Timer timer_Joining;
    atomic_int  joiner_StreamsAdjuster_callsCount_directMethod = 0;
    atomic_int  joiner_StreamsAdjuster_callsCount_iterativeMethod = 0;

    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < dividers.size(); i++)
    {
        if (options.DebugEnabled) cout << ".";
        MeshStreamsJoiner joiner(dividers[i], meshSize);
        joiner.JoinStreams();
        joiner_StreamsAdjuster_callsCount_directMethod += joiner.StreamsAdjuster_callsCount_directMethod;
        joiner_StreamsAdjuster_callsCount_iterativeMethod += joiner.StreamsAdjuster_callsCount_iterativeMethod;
    }
    timer_Joining.stop(elapsedTimers.Mesher_GenerateMesh.JoiningStreams);
    //cout << "Joining time:   " << timer_Joining << std::endl;
    if (meshLogicOptions.showTimeElapsed) cout << " (direct " << joiner_StreamsAdjuster_callsCount_directMethod << ",   iterative " << joiner_StreamsAdjuster_callsCount_iterativeMethod << ")";
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Joining << endl; else cout << endl;

    //
    // DEBUG show dividing streams - alredy done in "mesher.DrawAll();"
    //    
    //if (debug_show_meshing_process)
    //{
    //    cout << "Showing dividing streams... ";
    //    Timer timer_ShowingDividingStreams;
    //    for (auto& d : dividers)
    //    {
    //        d.DebugShowStreams();
    //    }
    //    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_ShowingDividingStreams << endl; else cout << endl;
    //}
}

void MeshGenerator::GenerateMesh_Divide_LSCM(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
    cout << "Dividing cutted meshes... ";
    Timer timer_Dividing;

    bool debug_show_meshing_process = options.DebugEnabled && options.debug_show_meshing_process &&
        (meshLogicOptions.MeshStream.show_dividing_streams || meshLogicOptions.MeshStream.show_streamIndexes_atStart || meshLogicOptions.MeshStream.show_streamIndexes_InAllPoints || meshLogicOptions.MeshStream.show_streamIndexes_Every10Points || meshLogicOptions.MeshStream.show_stream_iterations);

    //
    // create dividers
    //
    dividers.clear();
    dividers.reserve(meshes.size());
    map<int, int> map_meshid_divederindex;
    for (int i = 0; i < meshes.size(); i++)
    {
        dividers.push_back(Divider(topology, *meshes[i], solversUV[i], draw)); //needed for backward capability
        map_meshid_divederindex[meshes[i]->id] = i;
    }


    //
    // show warnings
    //
    int cout_warning_count = 0;
    #if DEBUG
    for (int i = 0; i < solversUV.size(); i++)
    {
        const Mesh* mesh = meshes[i];
        const MeshSolverUV& solver = solversUV[i];
        if (solver.Result.Field.size() == 0)
        {
            if (cout_warning_count == 0) cout << endl;
            cout_warning_count++;
            cout << "! warning:  MeshGenerator::GenerateMesh_Divide()    solver.Result.Field is empty   mesh.id=" << mesh->id << endl;
        }
    }
    #endif

    //
    // addAnchors method
    //
    auto StreamAnchors = [&](const MeshTopologyLoopCurveSegment* s, const MeshTopologyLoopCurveSegment* sOpposite, int divisioncount)
    {
        if (s == nullptr) return;
        if (divisioncount <= 1) return;
        int index = map_meshid_divederindex[s->meshid];
        const Mesh* mesh = meshes[index];
        const MeshSolverUV& solver = solversUV[index];
        Divider& divider = dividers[index];
        if (solver.Result.Field.size() == 0) return;
        D edgeLength = s->Length3d / divisioncount;
        vector<StreamPoint> points;
        s->GetPoints(points);
        int points_size = points.size();
        int start_from_point_index = 0;
        for (int i = 1; i < divisioncount; i++)
        {
            D atLength3d = i * edgeLength;
            MeshUV uvStart;
            for (int point_index = start_from_point_index; point_index < points_size-1; point_index++)
            {
                D currentEdgeLength = points[point_index].lengthToNextPoint;
                D Length3dUntilThisPoint = points[point_index].Length3dUntilThisPoint;
                if (Length3dUntilThisPoint <= atLength3d && atLength3d <= Length3dUntilThisPoint + currentEdgeLength)
                {
                    D shift = (atLength3d - Length3dUntilThisPoint);
                    D shiftPercent = (atLength3d - Length3dUntilThisPoint) / currentEdgeLength;
                    int vid0 = points[point_index].vid_eid_fid;
                    int vid1 = points[point_index+1].vid_eid_fid;
                    P3 point0 = mesh->V.row(vid0);
                    V3 edgeDir = points[point_index].dirToNextPointNormalized;
                    int eid = mesh->CommonEdgeId(points[point_index], points[point_index + 1]);
                    UV uv0 = solver.Result.Field[vid0];
                    UV uv1 = solver.Result.Field[vid1];
                    //D uvLength = (uv1.toP3() - uv0.toP3()).norm();
                    UV uv = uv0 + (uv1 - uv0)* shiftPercent;
                    P3 point = point0 + edgeDir * shift;
                    uvStart = MeshUV(MeshPointType::onEdge, eid, point, uv);
                    break;
                }
            }
            if (uvStart.isUndefined())
            {
                #if DEBUG
                if (cout_warning_count == 0) cout << endl;
                cout_warning_count++;
                cout << "! warning:  MeshGenerator::GenerateMesh_Divide_LSCM    didnt found uvStart   mesh.id=" << mesh->id << "   segment.id=" << s->id << endl; // << "    loop.Index=" << loop.Index
                #endif
                continue;            
            }

            UV uvEnd = uvStart.uv.getUVEnd();

            divider.streams.Add(-1, -1, uvStart, uvEnd, Color3d(0, 0, 0), 0);

            // add additional needed for backward capability for method 'MeshGenerator::DividedMeshToPolyMesh'
            divider.streams.streams.back().IsMerged = true;
            StreamStartPoint p(uvStart.Type, uvStart.vid_eid_fid, uvStart.point);
            p.Index = divider.startPoints.size();
            p.StreamAnchorPointId = i;
            p.dir = V3(0,0,0);
            p.maxAngleChangeForDir = 20;
            p.fid = -1;
            p.dir_ri = 0;
            p.dividesBy = 1;
            p.isBorderPoint = true;
            p.isSingularPoint = false;
            p.canBeJoined = false;
            p.joinedAtIndex = 0;
            p.dividingIteration = 0;
            p.haveContract = true;
            p.contract = { s->id, i };
            divider.startPoints.push_back(p);
        }
    };

    //
    // add start and anchor points (old stream for X and Y)
    //
    //for (auto& loop : Loops)
    //{
    //    int divisioncount = loop.GetDivisioncount(meshSize);
    //    if (divisioncount <= 1) continue;
    //    for (auto& c : loop.connections)
    //    {
    //        for (auto& s : c.connection->segments)
    //        {
    //            const MeshTopologyLoopCurveSegment* sOpposite = GetOppositeSegment(&s);
    //            addAnchors(&s, sOpposite, divisioncount);
    //        }
    //    }
    //}

    //
    // add start and anchor points (new stream only for X)
    //
    auto addSS = [&](const TwoOppositeSegments& ss)
    {
        const MeshTopologyLoopCurveSegment* s = ss.s0;
        const MeshTopologyLoopCurveSegment* sOpposite = ss.s1;
        StreamAnchors(s, sOpposite, ss.divisioncount);
    };
    for (auto& o : oppositeSegments)
    {
        addSS(o.X);
        if (debug_show_meshing_process && options.debug_show_meshing_process_Y)
        {
            addSS(o.Y);
        }
    }


    //
    // divide
    //
    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < dividers.size(); i++)
    {
        if (options.DebugEnabled) cout << ".";
        int isoLinesExtensionsCount = min(meshLogicOptions.Divider.SingularityExtendCount, dividers[i].mesh.FacesCount);
        dividers[i].streams.ExtendStreams(isoLinesExtensionsCount);
    }
    timer_Dividing.stop(elapsedTimers.Mesher_GenerateMesh.Dividing);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Dividing << endl; else cout << endl;

    //
    // join (no need for LSCM algorithm)
    //
    //cout << "Joining streams... ";
    //Timer timer_Joining;
    //atomic_int  joiner_StreamsAdjuster_callsCount_directMethod = 0;
    //atomic_int  joiner_StreamsAdjuster_callsCount_iterativeMethod = 0;

    //#pragma omp parallel for  if(IsOmpEnabled)
    //for (int i = 0; i < dividers.size(); i++)
    //{
    //    if (options.DebugEnabled) cout << ".";
    //    MeshStreamsJoiner joiner(dividers[i], meshSize);
    //    joiner.JoinStreams();
    //    joiner_StreamsAdjuster_callsCount_directMethod += joiner.StreamsAdjuster_callsCount_directMethod;
    //    joiner_StreamsAdjuster_callsCount_iterativeMethod += joiner.StreamsAdjuster_callsCount_iterativeMethod;
    //}
    //timer_Joining.stop(elapsedTimers.Mesher_GenerateMesh.JoiningStreams);
    ////cout << "Joining time:   " << timer_Joining << std::endl;
    //if (meshLogicOptions.showTimeElapsed) cout << " (direct " << joiner_StreamsAdjuster_callsCount_directMethod << ",   iterative " << joiner_StreamsAdjuster_callsCount_iterativeMethod << ")";
    //if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Joining << endl; else cout << endl;

    //
    // DEBUG show dividing streams - alredy done in "mesher.DrawAll();"
    //    
    //if (debug_show_meshing_process)
    //{
    //    cout << "Showing dividing streams... ";
    //    Timer timer_ShowingDividingStreams;
    //    for (auto& d : dividers)
    //    {
    //        d.DebugShowStreams();
    //    }
    //    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_ShowingDividingStreams << endl; else cout << endl;
    //}
}

void MeshGenerator::GenerateMesh_Divide_Laplacian(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
}

void MeshGenerator::GenerateMesh_Divide(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
    switch (options.Algorithm)
    {
        case MeshLogicOptions_Mesher::AlgorithmType::StreamsOnCuttedMeshes:
            GenerateMesh_Divide_Streams(oppositeSegments);
            break;
        case MeshLogicOptions_Mesher::AlgorithmType::UVOnCuttedMeshes:
            GenerateMesh_Divide_LSCM(oppositeSegments);
            break;
        case MeshLogicOptions_Mesher::AlgorithmType::LaplacianOnLoopStreams:
            GenerateMesh_Divide_Laplacian(oppositeSegments);
            break;
        default:
            cout << "!!!error in MeshGenerator::GenerateMesh_Divide - undefined algorithm" << endl;
            break;
    }
}

void MeshGenerator::DividedMeshToPolyMesh(const Mesh* mesh, const Divider& divider, PolygonMesh& polymesh, const TwoOppositeSegmentsPair& oppositeSegments)
{
    //if (mesh->id != 6) return;
    int segmentsCountInLoop = oppositeSegments.getSegmentsCountInLoop();
    if (oppositeSegments.X.s1 == nullptr || segmentsCountInLoop > 4 || segmentsCountInLoop < 3)
    {
        if (options.DebugEnabled)
        {
            draw.AddLabel(mesh->V_Info.Vcenter, "!!! not supported mesh with " + to_string(segmentsCountInLoop) + " segments for making quad mesh", Color3d(1, 0, 0));
        }
        cout << "!!!  not supported mesh with " << segmentsCountInLoop << " segments - only 3 and 4   at Mesher::DividedMeshToPolyMesh  " << endl;
        return;
    }
    if (!oppositeSegments.isSorted) return;
    int divisioncountX = oppositeSegments.X.divisioncount;
    int divisioncountY = oppositeSegments.Y.divisioncount;
    int pointscountY = divisioncountY + 1;

    //DEBUG draw oppositeSegments X, Y
    //draw.AddLabel(oppositeSegments.X.s0->MiddlePoint(), "X0");
    //draw.AddLabel(oppositeSegments.X.s1->MiddlePoint(), "X1");
    //draw.AddLabel(oppositeSegments.Y.s0->MiddlePoint(), "Y0");
    //if (oppositeSegments.Y.s1 != nullptr) draw.AddLabel(oppositeSegments.Y.s1->MiddlePoint(), "Y1");


    struct StreamOnSegmentX
    {
        int n;
        const MeshStream* stream;
        bool isStreamReversed;
        StreamOnSegmentX(int _n, const MeshStream* _stream, bool _isStreamReversed)
            : n(_n), stream(_stream), isStreamReversed(_isStreamReversed)
        {
        }
    };

    //
    // get streams for X segments
    //
    vector<StreamOnSegmentX> streamsOnSegmentX;
    for (const auto& s : divider.streams.streams)
    {
        if (s.IsDeleted) continue;
        if (!s.IsMerged) continue;

        int segmentid = divider.startPoints[s.Index].contract.first; // segment id from which stream starts
        int n = divider.startPoints[s.Index].contract.second; // stream number for this segment (started from 1 and ended at divisioncount-1)
        if (segmentid == oppositeSegments.X.s0->id)
        {
            streamsOnSegmentX.push_back(StreamOnSegmentX(n, &s, false)); // 'false' means not reverted
        }
        if (segmentid == oppositeSegments.X.s1->id)
        {
            n = divisioncountX - n;
            streamsOnSegmentX.push_back(StreamOnSegmentX(n, &s, true));// 'true' means not reverted
        }
    }


    // DEBUG  show points on streams
    //for (auto& s : streamsOnSegmentX)
    //{
    //    for (auto&p : s.stream->Points)
    //    {
    //        draw.AddPoint(p.point);
    //    }
    //}


    //
    // Get Points for X segments
    //
    P3s pointsOnX0;
    P3s pointsOnX1;
    oppositeSegments.X.s0->GetPointsDividedByCount(divisioncountX, pointsOnX0, false); // get Points for X0 (later we will use them to correct streamStartPoint and streamAndPoint)
    oppositeSegments.X.s1->GetPointsDividedByCount(divisioncountX, pointsOnX1, true); // get Points for X1 (later we will use them to correct streamStartPoint and streamAndPoint)

    //
    // Get Points for each stream and segment
    //
    vector<P3s> pointsOnStreams;
    pointsOnStreams.resize(divisioncountX + 1);
    oppositeSegments.Y.s0->GetPointsDividedByCount(divisioncountY, pointsOnStreams[0], true); // get points for segment Y.s0
    for (int i = 0; i < streamsOnSegmentX.size(); i++)
    {
        const StreamOnSegmentX& s = streamsOnSegmentX[i];
        P3s& streamPoints = pointsOnStreams[s.n];
        s.stream->GetPointsDividedByCount(divisioncountY, streamPoints, s.isStreamReversed);
        // fix streamStartPoint and streamAndPoint for each stream (since streamStartPoint can be shifted to vetex)
        streamPoints.row(0) = pointsOnX0.row(s.n);  // correct start point (to be sure that this point is same as point from opposite mesh)
        streamPoints.row(streamPoints.rows() - 1) = pointsOnX1.row(s.n); // correct end point (to be sure that this point is same as point from opposite mesh)
    }
    bool isLastStreamSingularity = false;
    if (oppositeSegments.Y.s1 != nullptr)
    {
        oppositeSegments.Y.s1->GetPointsDividedByCount(divisioncountY, pointsOnStreams.back(), false); // get points for segment Y.s1
    }
    else
    {
        isLastStreamSingularity = true;
        pointsOnStreams.back().resize(1, 3);
        pointsOnStreams.back().row(0) = oppositeSegments.X.s0->pointEnd; // get singular point for segment Y.s1 that is actually a point
    }
    // construct approximated points for failed streams 
    for (int n = 1; n < pointsOnStreams.size() - 1; n++)
    {
        P3s& streamPoints = pointsOnStreams[n];
        if (streamPoints.rows() == 0)
        {
            streamPoints.resize(divisioncountY + 1, 3);
            streamPoints.row(0) = pointsOnX0.row(n); // set start point
            streamPoints.row(streamPoints.rows() - 1) = pointsOnX1.row(n); // set end point
            V3 fromX0toX1 = pointsOnX1.row(n) - pointsOnX0.row(n);
            //D len = utils::vector::Length(fromX0toX1);
            //len = max(len, 0.00000000001); // protection from division by 0
            //V3 fromX0toX1normalized = fromX0toX1 / len;
            for (int i = 1; i < divisioncountY; i++)
            {
                P3 pointI = pointsOnX0.row(n).transpose() + fromX0toX1 * ((1.0*i) / (1.0*divisioncountY));
                streamPoints.row(i) = mesh->ProjectPoint(pointI, true);
            }
        }
    }


    // DEBUG show vertex points of quads
    //for (int pi = 0; pi < pointsOnStreams.size(); pi++)
    //{
    //    auto& ps = pointsOnStreams[pi];
    //    for (int i = 0; i < ps.rows(); i++)
    //    {
    //        P3 point = ps.row(i);
    //        //draw.AddPoint(point, Color3d(0, 0, 1));
    //        draw.AddPoint(point, Color3d(0, 0, 1), " X=" + to_string(pi)+", Y="+to_string(i));
    //    }
    //}


    //
    // setup PolygonMesh 
    //
    int totalPoints = 0;
    for (auto& ps : pointsOnStreams)  totalPoints += ps.rows();
    polymesh.V.resize(totalPoints, 3);
    int index = 0;
    for (auto& ps : pointsOnStreams)
    {
        for (int i = 0; i < ps.rows(); i++)
        {
            polymesh.V.row(index) = ps.row(i);
            index++;
        }
    }
    polymesh.ngons.resizeBegin(divisioncountX*divisioncountY, true);
    for (int i = 0; i < (divisioncountX - 1)*divisioncountY; i++)
    {
        polymesh.ngons.size(i) = 4;
    }
    for (int i = (divisioncountX - 1)*divisioncountY; i < divisioncountX*divisioncountY; i++)
    {
        polymesh.ngons.size(i) = !isLastStreamSingularity ? 4 : 3;
    }
    polymesh.ngons.resizeEnd();

    int vid = 0;
    int fid = 0;
    for (int x = 0; x < divisioncountX; x++)
    {
        for (int y = 0; y < divisioncountY; y++)
        {
            polymesh.ngons.add(fid, vid + y);
            int vid20 = vid + y + divisioncountY + 1;
            int vid21 = vid + y + divisioncountY + 2;
            if (isLastStreamSingularity && x == divisioncountX - 1)
            {
                polymesh.ngons.add(fid, polymesh.V.rows() - 1);
            }
            else
            {
                polymesh.ngons.add(fid, vid20);
                polymesh.ngons.add(fid, vid21);
            }
            polymesh.ngons.add(fid, vid + y + 1);
            fid++;
        }
        vid += divisioncountY + 1;
    }
}

void MeshGenerator::getOppositeSegments(vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
    // create map meshid->meshIndex
    map<int, int> map_meshid_meshIndex;
    for (int i = 0; i < meshes.size(); i++)
    {
        map_meshid_meshIndex[meshes[i]->id] = i;
    }

    // create 'TwoOppositeSegmentsPair' for each mesh
    oppositeSegments.clear();
    oppositeSegments.resize(meshes.size());
    for (auto& loop : Loops)
    {
        int divisioncount = loop.GetDivisioncount(meshSize);
        if (divisioncount == 0) continue;
        for (auto& c : loop.connections)
        {
            for (auto& s : c.connection->segments)
            {
                const MeshTopologyLoopCurveSegment* sOpposite = GetOppositeSegment(&s);
                int meshIndex = map_meshid_meshIndex[s.meshid];
                TwoOppositeSegmentsPair& pair = oppositeSegments[meshIndex];
                TwoOppositeSegments& o = (pair.X.s0 == nullptr) ? pair.X : pair.Y;
                o.divisioncount = divisioncount;
                o.s0 = &s;
                o.s1 = sOpposite;
                o.sumLength3d = s.Length3d;
                if (sOpposite != nullptr) o.sumLength3d += sOpposite->Length3d;

            }
        }
    }
    for (int i = 0; i < meshes.size(); i++)
    {
        auto& o = oppositeSegments[i];
        int segmentsCountInLoop = o.getSegmentsCountInLoop();
        if (segmentsCountInLoop == 3 || segmentsCountInLoop == 4)
        {
            if (!o.Sort())
            {
                cout << "!!!  failed to sort segments for meshid= " << meshes[i]->id << endl;
            }
        }
    }
}
void MeshGenerator::GenerateMesh_GeneratePolygonMeshes_Streams(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{

    cout << "generating polygon meshes... ";
    Timer timer;

    // create list of empty polygonMesh
    polygonMeshes.clear();
    polygonMeshes.reserve(meshes.size());
    for (const Mesh* m : meshes)
    {
        polygonMeshes.push_back(PolygonMesh(m->id));
    }

    // in parallel construct polygonMesh for each mesh
    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        if (options.DebugEnabled) cout << ".";
        for (auto& s : dividers[i].streams.streams)
        {
            s.Init_lengthToNextPoint();
        }
        DividedMeshToPolyMesh(meshes[i], dividers[i], polygonMeshes[i], oppositeSegments[i]);
    }
    timer.stop(elapsedTimers.Mesher_GenerateMesh.GeneratePolygonMeshes);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer << endl; else cout << endl;
}

void MeshGenerator::GenerateMesh_GeneratePolygonMeshes_LSCM(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
    GenerateMesh_GeneratePolygonMeshes_Streams(oppositeSegments);
}

void MeshGenerator::GenerateMesh_GeneratePolygonMeshes_Laplacian(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
}

void MeshGenerator::GenerateMesh_GeneratePolygonMeshes(const vector<TwoOppositeSegmentsPair>& oppositeSegments)
{
    switch (options.Algorithm)
    {
        case MeshLogicOptions_Mesher::AlgorithmType::StreamsOnCuttedMeshes:
            GenerateMesh_GeneratePolygonMeshes_Streams(oppositeSegments);
            break;
        case MeshLogicOptions_Mesher::AlgorithmType::UVOnCuttedMeshes:
            GenerateMesh_GeneratePolygonMeshes_LSCM(oppositeSegments);
            break;
        case MeshLogicOptions_Mesher::AlgorithmType::LaplacianOnLoopStreams:
            GenerateMesh_GeneratePolygonMeshes_Laplacian(oppositeSegments);
            break;
        default:
            cout << "!!!error in MeshGenerator::GenerateMesh_GeneratePolygonMeshes - undefined algorithm" << endl;
            break;
    }
}

void MeshGenerator::GenerateMesh(const vector<const Mesh*>& _meshes, D _meshSize)
{
    ClearCache();
    isMeshGenerated = false;
    meshes = _meshes;
    meshSize = _meshSize;

    //
    // update topology
    //
    GenerateMesh_UpdateTopology();

    //
    // Build loops
    //
    GenerateMesh_BuildLoops();

    //
    // get opposite segments
    //
    vector<TwoOppositeSegmentsPair> oppositeSegments;
    getOppositeSegments(oppositeSegments);

    //
    // solve
    //
    GenerateMesh_Solve();

    //
    // stream lines for each mesh 
    //
    GenerateMesh_Divide(oppositeSegments);

    //
    // generate polygon meshes
    //
    GenerateMesh_GeneratePolygonMeshes(oppositeSegments);


    isMeshGenerated = true;
}