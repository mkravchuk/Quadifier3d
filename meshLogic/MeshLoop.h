#pragma once


class Mesh;
class MeshLoop;

struct MeshLoopEdge
{
    int EdgeId;
    int FaceId;
    int StartVertexId;
    int EndVertexId;
    V3 directionAlongEdge; //Edge vector
    D Length;
    V3 directionToFace; //Edge normal to face centroid, normalized

    int Index;// index in loop that holds this struct
    int IndexPrev; // previuos loop edge index
    int IndexNext; // next loop edge index
    MeshLoopEdge(const MeshLoop& _loop, int _edgeId, int _faceId, int _startVertexId, int _endVertexId);
};

struct MeshLoopPoint
{
    int VertexId;
    int EdgeIdForward;
    int EdgeIdBackward;
    D AngleBetweenEdges;
    D AngleBetweenEdgesFull;
    bool IsSharp;
    bool probably_IsSharp;

    int Index;// index of loop point in a loop that holds this point
};

enum class MeshLoopType
{
    Outher, Inner
};

class MeshLoop 
{
    static atomic_int nextMeshLoopID;
    MeshLoop(const Mesh& _mesh);
    void Init() ; // init points and other information base on edges
    bool angleSuddenlyHasChanged(const int DEEP, const int prevsnexts[], const vector<D>& loopAngles)const;
    void GetSharpPoints_Indexes(vector<int>& edgeIndexes) const;
public:
    int id; // unique id
    int Length; // length of loop - same as edges.size()
    const Mesh& mesh;
    vector<MeshLoopEdge> edges; // sorted edges in loop
    vector<MeshLoopPoint> points; // points between edges in loop (start poins of each edge)
    MeshLoopType Type;
    D Loop3dLength; // 3d length of loop - sum of all edges length
    P3 Centroid; // centroid of vertexes from list 'points'
    D DistFromCentroidToZero;

    II SizeOF() const;
    static void GetLoops(const Mesh& mesh, vector<MeshLoop>& sortedloops, bool initLoops);
    static void GetLoops_slow(const Mesh& mesh, vector<MeshLoop>& sortedloops, bool initLoops);
    static void GetLoops_fast(const Mesh& mesh, vector<MeshLoop>& sortedloops, bool initLoops);
    void GetSharpPoints(vector<int>& vertexes, vector<D>& angleChange, vector<int>& pointIndexes) const;
    void GetAngleFull(V3 directionToFace, V3 directionToFacePrev, const MeshLoopEdge& e, const MeshLoopEdge& ePrev,
        D& AngleBetweenEdges, D& AngleBetweenEdgesFull) const;
    void GetLoopFaceIds(vector<int>& faceids) const;
};

