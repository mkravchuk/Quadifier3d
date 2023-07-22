#pragma once
#include "Divider.h"
#include "MeshTopology.h"
struct SSMesh;
struct SSTopologyConnection;
class Mesh;
class MeshSolverNrosy;
class MeshSurface;
class ViewerDrawObjects;


struct SSTopologySegment;
struct StreamSegment
{
    int Index; // index in 'streamsSegments' (each stream can have few segments)
    int intersectionIds[2];
    D Length3d;
    const MeshStream& stream;
    int fromPointIndex; // from point index in stream
    int toPointIndex; // to point index in stream (including)
    const SSTopologySegment* ssSegment; // will be populated(not null) if stream is on topology segment
    const MeshTopologyLoopCurveSegment* curveSegment; // will be populated(not null) if stream is on topology segment
    ViewerDrawObjects& draw;
    StreamSegment(int index, const MeshStream& stream, int from, int to, const SSTopologySegment* ssSegment, ViewerDrawObjects& draw);
    P3 PointAt(D posAtLength, bool shiftPointToFaceIfOnSegment) const;
    P3 PointMid(bool shiftIfOnSegment) const;
};

// StreamSegmentIntersection
struct SSIntersection
{
    int IntersectionId;             // some id from StreamSegment.intersectionIds[2]
    int IntersectionIdOpposite; // another id in StreamSegment.intersectionIds[2]
    StreamSegment* segment;
    V3 dirNormalized; // direction from IntersectionId to IntersectionIdOpposite
    V3 dirNormalizedOnIntersectionNormal;
    D AngleFullToFirstSegment;
    D AngleFullToNextSegment;
    SSIntersection();
    SSIntersection(int intersectionId, StreamSegment* segment);
};

struct SSQuad
{
    int Index; // index in 'quads'
    int intersectionIds[4];
    StreamSegment* segments[4];
    bool isTriangle; // true if only 3 segments defined instead of 4
    SSQuad(int index, int intersectionIds[4], map<pair<int, int>, StreamSegment*>& map_intersectionIds_to_streamSegment);
    SSQuad(int index, SSIntersection* segments[4]);
    bool isValid();
    P3 PointMid();
};

struct SSQuad_InLoop
{
    const SSQuad* quad;
    int index0;
    int index1;
    const StreamSegment* segment0;
    const StreamSegment* segment1; // if quad is Triangle and [index0=1,index1=3] then segment1==segment0
    bool segment1_isTriangleStop; // this quad doesnt have opponent edge, since it is triangle and segment1==segment0

    SSQuad_InLoop(const SSQuad* quad, int index0);
};

struct SSQuadsLoop_Side;
struct SSQuadsLoop_Side_ConflictJoin
{
    int id; // unique id
    SSQuadsLoop_Side& side;
    int divIndex; // index in 'vector<SSQuadsLoop_Side_ConflictJoin> conflictJoins'
    bool isTaken;

    SSQuadsLoop_Side_ConflictJoin(SSQuadsLoop_Side& side, int divIndex);
    SSQuadsLoop_Side_ConflictJoin* opposite(); //  can be null if we cant provide with opposite side because of different 'divby'
    void Clear();
    int GetDivIndexInSegment(); // return -1 if not found
    P3 GetMidPoint();
    int meshid();
};

struct SSQuadsLoop_Side_Cycl
{
    int id; // unique id
    SSQuadsLoop_Side& side;
    int Index; // index in 'vector<SSQuadsLoop_Side_Cycl> cycles'

    SSQuadsLoop_Side_Cycl(SSQuadsLoop_Side& side, int index);
    void Clear();
    int GetDivIndexInSegment(); // return -1 if not found
    P3 GetMidPoint();
    int meshid();
};


struct SSQuadsLoop;
struct SSQuadsLoop_Side
{
    SSQuadsLoop* loop;
    int sideIndex; // side index 0 or 1
    bool isTriangleStop; // flag indicates if this side is triangle (and so cannot have other loops connected to it)
    vector<SSQuadsLoop_Side_ConflictJoin> conflictJoins; // populated in method 'BuildConflictedLoops()'
    vector<SSQuadsLoop_Side_Cycl> cycles; // populated in class 'DvividerLogicConnector'

    SSQuadsLoop_Side();
    SSQuadsLoop_Side(SSQuadsLoop* loop, int sideIndex);
    int& div() const; // div01 for current side
    const StreamSegment* segment() const; // segment01 for current sideIndex
    SSQuadsLoop_Side& oppositeSide(); // opposite side to current in loop
    void ReinitConflictJoins();
};


struct SSQuadsLoop
{
    int id; // unique id
    int meshId;
    vector<SSQuad_InLoop> quads;
    const StreamSegment* segment01[2]; // fist and last segments in loop (border segments)
    int div01[2]; // for how many pieces divide first and last quad
    bool isTriangleStop[2]; // flag indicates if first or last quad is triangle
    SSQuadsLoop_Side sides[2];
    int reserved_div; // how many loops can come thourght this loop in addition to div01
    D reserved_length3d; // how many length3d is reserved for reserved loops
    vector<int> reserved_for_loops_ids; // ids of loops for which space was reserved in this loop

    SSQuadsLoop(int meshId);
    void UpdateAfterAdd();
};




struct SSTopologySegment
{
    const MeshTopologyLoopCurveSegment* curveSegment; // will be populated in 'SSTopologyConnection::SSTopologyConnection'
    const SSTopologyConnection* connection;            // will be populated in 'SSTopologyConnection::SSTopologyConnection
    SSMesh* mesh;// will be populated in 'SSMesh.BuildQuadLoops__SetRelationBetweenLoopsAndSegments()'
    vector<SSQuadsLoop_Side*> loopSides; // sorted, will be populated in 'SSMesh.BuildQuadLoops__SetRelationBetweenLoopsAndSegments()'
    SSTopologySegment* Friend; // friend segment in connection. can be null if this is a border of model (if mesh doesnt have friend on this curve)
    SSTopologySegment();
    int GetSumDivBy() const;
    void SetDivBySumm(int new_divBySumm, bool updateLoopEnd, int iterationNum, bool debug_iteration, ViewerDrawObjects& draw);
    SSQuadsLoop_Side_ConflictJoin* GetJoinByIndex(int joinIndex, bool getFromBack); // if getFromBack then search is in opposite way
};


struct SSTopologySegment_DividingPoint
{
    const SSTopologySegment& topologySegment;
    D pos;
    const StreamSegment* segmentOrtogonal; // can be nullptr if no StreamSegment is connected to this dividing point on topologySegment

    SSTopologySegment_DividingPoint(const SSTopologySegment& _topology_segment, D _pos, const StreamSegment* _segment_ortogonal)
        : topologySegment(_topology_segment), pos(_pos), segmentOrtogonal(_segment_ortogonal)
    {
    }
};


struct SSTopologyConnection_Conflict
{
    enum class Status
    {
        None, NotFound, Found, Resolved
    };
    Status status;
    int max_divBy; //  max divBy for 2 segments in connection
    int divBy0;
    int divBy1;
    const SSTopologyConnection* connection; // owner connection
    int resolvesTimesCount; // how many times conflict was resolved - protection from forever loop
    SSTopologyConnection_Conflict();
    void Calculate();
    void ResolveConflict(int max_conflicts, int iterationNum, bool debug_iteration, ViewerDrawObjects& draw);
};

struct SSTopologyConnection
{
    int Index; // index in 'vector<SSTopologyConnection> connections'
    const TopologyConnection& connection;
    SSTopologySegment segments[2]; // variable declared as array to avoid memory allocation - speed optimization
    int segmentsCount;
    SSTopologyConnection_Conflict conflict;
    SSTopologyConnection(int index, const TopologyConnection& connection);
    void UpdateReferences();
    int GetMaxDiv() const;
    string GetDivByText() const;
};





struct SSMesh
{
public:
    int Index; // index in 'dividers'
    int meshId;
    const Mesh& mesh;
    Divider& divider;
    const MeshSolverNrosy* solver;
    const Topology& topology; // topology for current file
    ViewerDrawObjects& draw;
    D meshSize;

    vector<MeshStream> streamsWithIntersections; // populated in 'BuildQuadLoops__FindIntersections_OnStreams'
    vector<const SSTopologySegment*> streamsWithIntersections_segments; // populated in 'BuildQuadLoops__FindIntersections_OnStreams'
    vector<MeshPoint> Intersections;    // populated in 'BuildQuadLoops__FindIntersections_OnStreams'

    vector<StreamSegment> streamsSegments;  // populated in 'BuildQuadLoops__CreateStreamSegments_FromStreams'

    map<pair<int, int>, StreamSegment*> map_intersectionIds_to_streamSegment; // populated in 'BuildQuadLoops__CreateRelations_BetweenIntersectionsAndSegments'
    CompactVectorVector<SSIntersection> intersectionsId_streamSegments; // populated in 'BuildQuadLoops__CreateRelations_BetweenIntersectionsAndSegments'

    vector<SSQuad> quads; // populated in 'BuildQuadLoops__CreateQuads_FromStreamSegments'
    vector<SSQuadsLoop> quadLoops; // populated in 'BuildQuadLoops__CreateQuadLoops'

    vector<SSTopologyConnection*> meshConnections; // what connections are related to our meshes, populated in 'BuildQuadLoops__SetRelationBetweenLoopsAndSegments'

    SSMesh(int index, Divider& divider, const MeshSolverNrosy* solver, const Topology& topology, ViewerDrawObjects& draw, D meshSize);
    void BuildQuadLoops(vector<SSTopologyConnection>& allConnections);
    void JoinStreamsWithDividingPoints();
    vector<SSTopologySegment_DividingPoint> GetDividingPointsForJoins(const SSTopologySegment& segment, bool addNormalDividingPoints = true, bool addStreamDividingPoints = true);
private:
    void Init_lengthToNextPoint();
    void CutStreamsAtCross(vector<MeshStream>& streams, const vector<const SSTopologySegment*>& segments, int& IntersectionsCount); // cut streams at points of their intersections, cutted stream will be that that is longer at that intersection point
    void RemoveRedundantIntersections(vector<MeshStream>& streams, const vector<const SSTopologySegment*>& segments, int& IntersectionsCount);  // check if pre-last intersection point is not duplicating last intersection
    void BuildQuadLoops__FindIntersections_OnStreams(const vector<SSTopologyConnection>& allConnections); // creates 'streamsWithIntersections', 'streamsWithIntersections_segments', 'Intersections'
    void BuildQuadLoops__CreateStreamSegments_FromStreams();// creates 'streamsSegments'
    void BuildQuadLoops__CreateRelations_BetweenIntersectionsAndSegments(); // creates 'map_intersectionIds_to_streamSegment', 'intersectionsId_streamSegments'
    void BuildQuadLoops__CreateQuads_FromStreamSegments(); // creates 'quads'
    void BuildQuadLoops__CreateQuadLoops(); // creates 'quadLoops'
    void BuildQuadLoops__SetRelationBetweenLoopsAndSegments(vector<SSTopologyConnection>& allConnections); //  populates 'connections', and  'connection.segmens[].loopSides', and ,  'connection.segmens[].mesh'
    void ConvertMeshBorderSegmentsToStreams(const vector<SSTopologyConnection>& allConnections, vector<MeshStream>& streams, vector<const SSTopologySegment*>& segments);
    void CreateQuads_FromStreamSegments_recursive(); // creates 'quads'
    void CreateQuads_FromStreamSegments_rotation(); // creates 'quads'
    bool addAnchorAndStartPoints(D newPos, const MeshTopologyLoopCurveSegment* segment, DividerIteratorStreamInfo dividingIteration);
};

struct SSMeshConflictedLoop
{
    int Index; // index in conflictedLoops
    bool isCyclic; // if loop is cyclic
    int debugUpdatedReserveSpaceAtIterationNum;
    vector<SSQuadsLoop_Side_ConflictJoin*> joins;
    SSMeshConflictedLoop(int _index)
        : Index(_index), isCyclic(false), debugUpdatedReserveSpaceAtIterationNum(-1)
    {
    };
};


struct SSMeshes
{
    const Topology& topology; // topology for current file
    ViewerDrawObjects& draw;
    D meshSize;
    vector<SSMesh> meshes; // all dividers
    map<int, SSMesh*> map_meshid_ssmesh;
    SSMeshes(const Topology& topology, ViewerDrawObjects& draw, vector<Divider>& dividers, vector<const MeshSolverNrosy*> solvers, D meshSize);
    void Build();
    void DrawDebug();
    vector<SSTopologyConnection> allConnections; // wrapper around 'Topology.Connections'
};