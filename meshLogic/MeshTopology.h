#pragma once
#include "Mesh.h"
#include "MeshTopologyLoop.h"

class Mesh;
class ModelObject;
struct MeshLoopPoint;
class MeshSurface;
class ViewerDrawObjects;
class MeshTopologyLoopCurve;







class TopologyPoint
{
private:
    static atomic_int nextTopologyPointID;
public:
    int id; // unique id
    int Index; // index in Topology.Points (vector<TopologyPoint> Points)
    P3 point; // avarage point for all segments that share this topology point
    Color3d color;
    vector<MeshTopologyLoopCurveSegment*> segments; // two segmnets from same mesh and mb connected segments and connected-connected 
    TopologyPoint(int index);
    II SizeOF() const;
    void Update();  // updates 'segments' classes and 'color' after changing 'segments'
private:
    Color3d Color();
};


class TopologyConnection
{
private:
    static atomic_int nextTopologyConnectionID;
public:
    int id; // unique id
    int Index; // index in Topology.Connections (vector<TopologyConnection> Connections)
    P3s points;// points of connection - just copy of smallest segment of connected curves
    bool isPointsOptimizedToLine;
    Color3d color;
    P3 middlePoint; // middle of curve - in place of Length3d/2
    vector<MeshTopologyLoopCurveSegment> segments; // 1 or 2 curves, could be more but for solid objects very rare
    TopologyConnection(int index);
    bool IsSegmentReverted(int segmentIndex) const;
    II SizeOF() const;
    void Update(); // updates 'points' and 'color' and 'middlePoint' after changing 'curves'
    const MeshTopologyLoopCurveSegment& getOppositeSegment(const MeshTopologyLoopCurveSegment& segment) const; // return nullptr if no opposite segment
    int findSegmentIndex(const MeshTopologyLoopCurveSegment& segment) const; //  get index in 'segments', return -1 if not found
private:
    Color3d Color();
};


class Topology
{
private:
    vector<int> last_Update_for_Objects_Ids; // object for which last update was done (sorted array)
public:
    //const Model& model;
    vector<TopologyPoint> Points;
    vector<TopologyConnection> Connections;
    ViewerDrawObjects draw;
    map<int, map<int, int>> map_meshid_vid_connectionIndex;
    map<int, map<int, int>> map_meshid_eid_connectionIndex;

    II SizeOF() const;

    void Update(const vector<ModelObject*>& activeObjects, const vector<int>& activeObjects_SerialNumbers, bool updateOnlyIfObjectsAreChanged);
    void Update(const vector<const Mesh*>& meshes);
    void Clear();
    bool IsUpdateNeeded(const vector<int>& activeObjects_Ids); // check if objects changed so we need to call 'Update' method
    void DrawAll();
private:
    void AddPerfectConnections(vector<MeshTopologyLoopCurveSegment>& freeCurves);
    void AddPartialConnections(vector<MeshTopologyLoopCurveSegment>& freeCurves);
    void AddInsideConnections(vector<MeshTopologyLoopCurveSegment>& freeCurves);
    void AddConnections(const vector<const Mesh*>& meshes);
    void SetPrevNext_SegmentInLoop();
    void AddPoints();
};

