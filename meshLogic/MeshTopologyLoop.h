#pragma once

struct MeshPoint;
struct StreamPoint;
class Mesh;
class MeshLoop;
class MeshSurface;
class ViewerDrawObjects;
class MeshTopologyLoopCurve;


class MeshTopologyLoopPoint
{
public:
    const Mesh& mesh;
    const MeshLoop& loop;
    P3 point;
    const MeshLoopPoint& loopPoint;
    MeshTopologyLoopCurve* CurvePrev; // each point has prev  and next curve, since each loop is circular. is defined as pointer since impossilbe to define as reference
    MeshTopologyLoopCurve* CurveNext; // each point has prev  and next curve, since each loop is circular. is defined as pointer since impossilbe to define as reference
    MeshTopologyLoopPoint(const Mesh& mesh, const MeshLoop& loop, const MeshLoopPoint& loopPoint);
    II SizeOF() const;
};

class MeshTopologyLoopCurve
{
    static atomic_int nextTopologyLoopCurveID;
public:
    int id; // unique id
    const Mesh& mesh;
    const MeshLoop& loop;
    const MeshTopologyLoopPoint& pointStart; //  start point of curve
    const MeshTopologyLoopPoint& pointEnd;  // end point of curve
    bool isCyclic; // if there are no sharp points, means cuve is cyclic, but since we must define start and end - the only possible way to keep this knowledge is to use such property as 'isCyclic'
    P3s points; // points of curve
    Is loopPointIndexes; // indexes in MeshLoop for points
    int PointsCount; //  count of points in curve
    int EdgesCount; //  count of edges in curve
    D Length3d; // summ of 3d length of all edges
    D AvgEdgeLength3d; // avarage edge length 
    P3 middlePoint; // middle of curve - in place of Length3d/2
    D middlePoint_DistToZeroPow2;
    MeshTopologyLoopCurve(const Mesh& mesh, const MeshLoop& loop, MeshTopologyLoopPoint& pointStart, MeshTopologyLoopPoint& _pointEnd);
    __forceinline int CorrectCyclicIndex(int cyclicIndex) const // allows to use cyclic index
    {
        if (isCyclic)
        {
            if (cyclicIndex >= EdgesCount)
            {
                cyclicIndex -= EdgesCount;
            }
            if (cyclicIndex < 0)
            {
                cyclicIndex = EdgesCount + cyclicIndex;
            }
        }
        else
        {
            if (cyclicIndex >= PointsCount)
            {
                assert(isCyclic && "correction is valid only for cyclic curves, otherwise this is out of array range exception");
                cyclicIndex -= PointsCount;
            }
            if (cyclicIndex < 0)
            {
                assert(isCyclic && "correction is valid only for cyclic curves, otherwise this is out of array range exception");
                cyclicIndex = PointsCount + cyclicIndex;
            }
        }
        return cyclicIndex;
    }
    MeshPoint GetMeshPointAtLength(D atLength3d, int fromIndex, int count, bool allowPointOnVertex) const;
    MeshPoint GetMeshPointAtLength(D atLength3d) const;
    P3 GetPointAtLength(D atLength3d, int fromIndex, int count) const;
    P3 GetPointAtLength(D atLength3d) const;
    P3 GetPointAtLengthPercent(D percent) const;
    P3 GetPointAtLengthPercent(D percent, int fromIndex, int count) const;
    D GetAtLengthFromPoint(const MeshPoint& point, int fromIndex, int count) const;
    void GetPointsDividedByCount(int fromIndex, int count, int divisioncount, P3s& points, bool isPointsStoredInRevertedOrder, D segmentLength3d =-1) const;
    II SizeOF() const;
};


class TopologyPoint;
class TopologyConnection;
class MeshTopologyLoopCurveSegment
{
    static atomic_int nextTopologyLoopCurveSegmentID;
    void Init(int fromIndex, int toIndex);
public:
    int id; // unique id
    const MeshTopologyLoopCurve& curve; //  original curve from which we take segment
    int meshid;
    int fromIndex;// index in curve points
    int toIndex;// index in curve points
    bool isCyclic; // if there are no sharp points, means cuve is cyclic, and segment cover complete curve (defined only one segment for curve)
    int PointsCount; //  count of points in this segment curve
    int EdgesCount; //  count of edges in this segment curve
    D Length3d; // summ of 3d length of edges (partial ammount of)
    D AvgEdgeLength3d; // avarage edge length 
    D positionInPseudo3dStart2; // x+y+z of second point from 'start'
    D positionInPseudo3dEnd2; // x+y+z of second point from 'end' 
    D positionInPseudo3dMid; // x+y+z of middle edge or point
    D edgeLengthStart;// first edge length 
    D edgeLengthEnd;// last edge length
    P3 pointStart; // point at start of segment
    P3 pointEnd;// point at end of segment
    int pointStart_vertexId; // vertex id at start of segment
    int pointEnd_vertexId;// vertex id at end of segment

    int IndexInTopologyConnections; // index in Topology.Connections (vector<TopologyConnection> Connections)
    MeshTopologyLoopCurveSegment* PrevSegmentInLoop; // (updated outside this class)
    MeshTopologyLoopCurveSegment* NextSegmentInLoop; // (updated outside this class)
    MeshTopologyLoopCurveSegment* ConnectedTo;// every segment can be connected to another segment from another mesh (updated outside this class)
    TopologyPoint* topologyPointStart; // topology point at start - topology point is common point for few segments - two segmnets from same mesh and mb connected segments and connected-connected (updated outside this class)
    TopologyPoint* topologyPointEnd; //  topology point at end - topology point is common point for few segments - two segmnets from same mesh and mb connected segments and connected-connected (updated outside this class)
    TopologyConnection* topologyConnection;// connection to which belong this segment. every connection can hold 1 or 2 segments. (updated outside this class)

    MeshTopologyLoopCurveSegment(const MeshTopologyLoopCurve& curve);
    MeshTopologyLoopCurveSegment(const MeshTopologyLoopCurve& curve, vector<int> edgeIndexes);
    MeshTopologyLoopCurveSegment(const MeshTopologyLoopCurve& curve, int fromIndex, int toIndex);
    II SizeOF() const;
    __forceinline int CorrectCyclicIndex(int cyclicIndex) const // allows to use cyclic index
    {
        return curve.CorrectCyclicIndex(cyclicIndex);
    }
    __forceinline P3 Point(int index) const // index is local: from 0 to Count-1
    {
        P3 point = curve.points.row(CorrectCyclicIndex(fromIndex + index));
        return point;
    }
    __forceinline int VertexId(int index) const // index is local: from 0 to Count (since vertexesCount are more by 1 and equal edgesCount+1)
    {
        int loopIndex = curve.loopPointIndexes(CorrectCyclicIndex(fromIndex + index));
        return curve.loop.points[loopIndex].VertexId;
    }
    __forceinline D EdgeLength(int index) const // index is local: from 0 to Count-1
    {
        int loopindex = curve.loopPointIndexes(CorrectCyclicIndex(fromIndex + index)); // translate segment index to loop index
        return curve.loop.edges[loopindex].Length;
    }
    bool PointIsSharp(int index) const; // index is local: from 0 to Count-1
    P3 MiddlePoint() const;
    P3s GetPointsForDraw(bool& isPointsOptimizedToLine) const;
    P3s GetPoints() const;
    void GetPoints(vector<StreamPoint>& points) const;
    P3 GetPointAtLength(D atLength3d) const;
    MeshPoint GetMeshPointAtLength(D atLength3d, bool allowPointOnVertex) const;
    P3 GetPointAtLengthPercent(D percent) const;
    D GetAtLengthFromPoint(const MeshPoint& point) const;
    bool IsOppositeDirections(const MeshTopologyLoopCurveSegment& anotherSegment) const ;
    void GetPointsDividedByCount(int divisioncount, P3s& points, bool isPointsStoredInRevertedOrder) const;
    int GetSegmentsCountInLoop() const;
};

class MeshTopologyLoop
{
    static atomic_int next_updateId;
public:
    int updateId; // unique id of update - can be used to detect if topologies are same (after each call to 'Update' this id will be changed)
    vector<MeshTopologyLoopPoint> points;
    vector<MeshTopologyLoopCurve> curves;
    MeshTopologyLoop();
    II SizeOF() const;
    void Update(const Mesh& mesh);
};
