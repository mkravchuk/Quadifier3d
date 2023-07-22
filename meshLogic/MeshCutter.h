#pragma once
#include "MeshStreams.h"
class Divider;
class Mesh;
class MeshSolverNrosy;
class MeshSurface;
class ViewerDrawObjects;

class MeshCutter
{
public:
    const Mesh& mesh_original;
    ViewerDrawObjects& draw;
    D avg_edge_length;
    vector<Mesh> cutedMeshes;
    bool splitNakedTriangleWith2NakedEdge; // best of all set to true before NrosySolver. check Mesh class to undestand this parameter
    bool correctFacesEdgeIndexes; // best of all set to true before NrosySolver. check Mesh class to undestand this parameter
    bool optimizeVertexIndexesForGPU;

    //MeshCutter(MeshSurface& srf, const Divider& divider, bool splitNakedTriangleWith2NakedEdge, bool correctFacesEdgeIndexes);
    MeshCutter(ViewerDrawObjects& draw, const Mesh& mesh, bool splitNakedTriangleWith2NakedEdge, bool correctFacesEdgeIndexes, bool optimizeVertexIndexesForGPU);
    II SizeOF() const;
    void Cut(const vector<MeshStream>& streams);
    int FindStreamIntersections(vector<MeshStream>& streams, bool debug_show_intersections = false); // returns count of intersections
    int FindStreamIntersections(vector<MeshStream>& streams, bool improveStreamLines, bool mergeClosePointsOnEdges, bool improveMeshToConnectStreamLines, bool debug_show_intersections); // returns count of intersections
private:
    void checkStreamsForDuplicatedPoints(vector<MeshStream>& streams, string callerName);
    void Debug_ShowWorkMesh(const Mesh& mi, bool onlyborders);
    int MergeClosePointsOnEdgesOnce(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections);
    void MergeClosePointsOnEdges(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections);
    void ImproveStreamLines(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams);
    void ImproveMeshToConnectStreamLines(const Mesh& m, P3s& m_V, vector<MeshStream>& streams, bool allow_improvements_on_borders, bool dissalow_move_other_streams);
    void ImproveMeshToConnectStreamLines_removeReduntantPoints(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams);
    void FindStreamIntersections_Edge(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections);
    void FindStreamIntersections_Vertex(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections);
    void FindStreamIntersections_Face(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections);
    void FindStreamEdgeIntersection(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections);
    void CutMesh(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, vector<Mesh>& ms);
    void ValidateIntersectionsIds(const vector<MeshStream>& streams, int intersectoinInfo_currentId);
};