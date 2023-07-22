#pragma once
#include "Mesh.h"
#include "MeshTopology.h"
#include "Divider.h"
#include "MeshCutter.h"
#include "PolygonMesh.h"
#include "MeshSolverUV.h"

class ModelFile;
class Mesh;
class MeshSurface;
class ViewerDrawObjects;


struct MesherLoopConnection
{
    int Index; // index in vector<MesherLoopConnection> 
    TopologyConnection* connection;
    D Length;

    MesherLoopConnection(int index, TopologyConnection* connection);
    int GetDivisioncount(D meshSize);
    P3 MiddlePoint();
};

struct MesherTopologyLoop
{
public:
    int Index; // index in vector<MesherLoops> 
    const Topology& topology;
    vector<MesherLoopConnection> connections;
    bool isCyclic;
    MesherTopologyLoop(int index, const Topology& _topology);
    int GetDivisioncount(D meshSize);
    II SizeOF() const;
private:
};

struct TwoOppositeSegments
{
    D sumLength3d;
    int divisioncount;
    const MeshTopologyLoopCurveSegment* s0;
    const MeshTopologyLoopCurveSegment* s1;
    TwoOppositeSegments()
        : sumLength3d(0), divisioncount(0), s0(nullptr), s1(nullptr)
    {

    }
};

struct TwoOppositeSegmentsPair
{
    TwoOppositeSegments X; // X will have longest lines, Y will have shortest lines
    TwoOppositeSegments Y; // X will have longest lines, Y will have shortest lines
    bool isSorted;
    bool Sort(); // X will have longest lines, Y will have shortest lines
    TwoOppositeSegmentsPair()
        : isSorted(false)
    {
    }
    int getSegmentsCountInLoop() const
    {
        return X.s0 != nullptr ? X.s0->GetSegmentsCountInLoop() : 0;
    }
};


class MeshGenerator
{
private:
    const MeshLogicOptions_Mesher& options;
public:
    const ModelFile& file;
    ViewerDrawObjects& draw;

    bool isMeshGenerated;
    vector<const Mesh*> meshes;
    D meshSize;

    Topology topology;
    vector<MeshSolverNrosy> solvers; //  caches solvers for cutted meshes
    vector<MeshSolverUV> solversUV; //  caches solvers for cutted meshes
    vector<Divider> dividers; //  caches dividers for cutted meshes
    vector<PolygonMesh> polygonMeshes; // quad meshes for each cutted mesh 
    vector<MesherTopologyLoop> Loops;

    MeshGenerator(const ModelFile& file, ViewerDrawObjects& draw);
    II SizeOF() const;
    void ClearCache();
    void GenerateMesh(const vector<const Mesh*>& meshes, D meshSize);
    PolygonMeshes GetPolygonMeshes();
    void DrawAll();
private:
    const MeshTopologyLoopCurveSegment * GetOppositeSegment(const MeshTopologyLoopCurveSegment* segment); // return null if not found - possible if mesh has only 3 segments, or more than 4
    void GenerateMesh_UpdateTopology();
    void GenerateMesh_BuildLoops();
    void GenerateMesh_Solve_Streams();
    void GenerateMesh_Solve_LSCM();
    void GenerateMesh_Solve_Laplacian();
    void GenerateMesh_Solve();
    void getOppositeSegments(vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_Divide_Streams(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_Divide_LSCM(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_Divide_Laplacian(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_Divide(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void DividedMeshToPolyMesh(const Mesh* mesh, const Divider& divider, PolygonMesh& polymesh, const TwoOppositeSegmentsPair& oppositeSegments);
    void GenerateMesh_GeneratePolygonMeshes_Streams(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_GeneratePolygonMeshes_LSCM(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_GeneratePolygonMeshes_Laplacian(const vector<TwoOppositeSegmentsPair>& oppositeSegments);
    void GenerateMesh_GeneratePolygonMeshes(const vector<TwoOppositeSegmentsPair>& oppositeSegments);

};