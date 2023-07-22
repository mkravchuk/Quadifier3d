#pragma once
#include "Divider.h"
#include "MeshTopology.h"
class Mesh;
class MeshSolverNrosy;
class MeshSurface;
class ViewerDrawObjects;


struct DividerDynamicIteration
{
public:
    int index_InOriginalDividersList;
    Divider& divider;
    const MeshSolverNrosy* solver;
    int meshId;
    bool isNeedToRedivide;
    vector<int> StopVertexIds; // vertexes where iterator will stop - those vertexes are singularities or sharp points or involved in topology
    DividerDynamicIteration(int index_InOriginalDividersList, Divider& divider, const MeshSolverNrosy* solver);
};

struct EndStreamPoint
{
    DividerDynamicIteration* divider; // (must be a pointer to be able use condition 'map.find() != map.end()')
    MeshPoint point;
    MeshStream* stream;
    int connectionIndex; // index in Topology.Connection 
    int segmentIndex; // index in Topology.Connection[].segments
    EndStreamPoint(DividerDynamicIteration* _divider, const MeshPoint& _point, MeshStream* _stream)
        : divider(_divider), point(_point), stream(_stream), connectionIndex(-1), segmentIndex(-1)
    {
    }
};


class EndStreamPointOnSegment
{
public:
    int globalStreamIndex;
    int iterationNum;
    int connectionIndex; // index in Topology.Connection 
    int segmentIndex; // index in Topology.Connection[].segments

    const Topology * topology;  // pointer to ModelFile.Topology (must be a pointer to be able use condition 'map.find() != map.end()')
    DividerDynamicIteration* divider; // (must be a pointer to be able use condition 'map.find() != map.end()')
    MeshPoint point;
    V3 point_direction;
    V3 point_directionNormal;

    bool isTaken;
    bool isCyclicStream;
    int meshId;
    const MeshTopologyLoopCurveSegment* segment;// (must be a pointer to be able use condition 'map.find() != map.end()')
    int segmentId; // global id of segment - same as Topology.Connection[connectionIndex].segments[].id
    D atLengthOriginal;
    D atLengthRevertedToFirstSegment;
    bool isSegmentReverted;
    EndStreamPointOnSegment(const Topology* topology, int _connectionIndex, int segmentIndex, DividerDynamicIteration* divider, const MeshPoint& point, V3 point_direction, V3 point_directionNormal, int streamIndex);

    MeshStream* stream() // must be here as dynamic method, since we cannot use static pointers since adding new streams to divider can reallocate vector and pointer will be outdated - so we use index to get stream from vector
    {
        return &divider->divider.streams[stream_Index];
    }
private:
    int stream_Index;
};


class DividerIterator
{
public:
    int dividerIterationNum;
    const Topology& topology; // topology for current file
    ViewerDrawObjects& draw;
    D meshSize;
    vector<DividerDynamicIteration> dividersAll; // all dividers
    vector<DividerDynamicIteration*> dividersNeedsToRedivide; // dividers that needs to be redivided since some startPoint or Anchor is added into 'additional_startStreamPoints' or 'additional_anchorPoints'
    map<int, DividerDynamicIteration*> map_meshid_DividerDynamicIteration;
    DividerIterator(const Topology& topology, ViewerDrawObjects& draw, vector<Divider>& dividers, vector<const MeshSolverNrosy*> solvers, D meshSize);
    static bool addAnchorAndStartPoints(D newPos, bool allowPointOnVertex, const MeshTopologyLoopCurveSegment* segment, DividerIteratorStreamInfo dividingIteration, const Mesh& mesh, const MeshSolverNrosy& solver, vector<StreamStartPoint>& startStreamPoints, vector<StreamAnchorPoint>& anchorPoints, V3* direction = nullptr, V3* directionNormal = nullptr);

    void Divide();
private:
    void GetEndStreamPoints(vector<EndStreamPoint>& points, int selectOnlyPointsWithIterationNum = -1);
    bool ProceedCurrentIteration_and_start_NextIteration(vector<EndStreamPointOnSegment>& newPoints, vector<vector<int>>& streams_visited_meshids); // find merge points between 2 meshes, or place end of stream from 1 mesh to its friend as start point of stream
    void DefineJoinPoints(vector<vector<EndStreamPointOnSegment>>& newSegmentPoints, vector<vector<int>>& streams_visited_meshids);
};




