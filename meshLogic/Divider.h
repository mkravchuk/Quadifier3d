#pragma once
#include "MeshStreams.h"
#include "Mesh.h"
#include "MeshTopology.h"
class Mesh;
class MeshSolverNrosy;
class MeshSurface;
class ViewerDrawObjects;



class Divider
{
public:
    const Topology& topology;
    const Mesh& mesh;
    const MeshSolverNrosy& solver;
    const MeshSolverUV& solverUV;
    ViewerDrawObjects& draw;

    vector<int> vertexesIdInvolvedInTopologySegments; // is populated outside this class - contains all vids that are start or end of MeshTopologyLoopCurveSegment
    vector<StreamStartPoint> startPoints;
    vector<StreamAnchorPoint> anchorPoints;
    int startStreamPoints_originalsize; // size of 'startStreamPoints' when first called 'GetStartStreamDirections' - used fro caching and to allow DividerIterator to change property 'contract'
    int anchorPoints_originalsize; // size of 'anchorPoints' when first called 'GetStartStreamDirections' - used fro caching and to allow DividerIterator to change property 'contract'

    MeshStreams streams;

    Divider(const Topology& topology, MeshSurface& srf);
    Divider(const Topology& topology, const Mesh& mesh, const MeshSolverNrosy& solver, ViewerDrawObjects& draw);
    Divider(const Topology& topology, const Mesh& mesh, const MeshSolverUV& solver, ViewerDrawObjects& draw);
    bool Divide(vector<StreamStartPoint> additional_startStreamPoints, vector<StreamAnchorPoint> additional_anchorPoints, bool autoDetectSharpPointsAndSingularPoints = true, int preserveStreamsUntilDividingIteration = -1);
    bool Divide(bool autoDetectSharpPointsAndSingularPoints = true, int preserveStreamsUntilDividingIteration = -1);
    int DebugShowStreams_GetAddedEdgesCount() const;
    void DebugShowStreams();
    static int getFacesIndex(const vector<VertexToFacesSortedInfo>& faces, vector<D>& facesAngleSegments, D rotationDegree);
    static V3 getAngleFacePlaneVector(const vector<VertexToFacesSortedInfo>& faces, vector<D>& facesAngleSegments, int vertexIndex, D angleSumm360, const Mesh& mesh, D rotationDegree);
    II SizeOF() const;
    StreamStartPoint* FindStartPoint(int id);
    StreamAnchorPoint* FindAnchorPoint(int id);
    void SignContract(StreamStartPoint& sp0, StreamAnchorPoint& ap0, StreamStartPoint& sp1, StreamAnchorPoint& ap1);
    void GetDividingPoints(vector<MeshPoint>& points, vector<int>& dividesBy, vector<bool>& isSingularPoints);
private:
    static void GetPointDirection_RetriveResults(const Mesh& mesh, ViewerDrawObjects& draw, int vertexIndex, int dividesBy,
        const vector<VertexToFacesSortedInfo>& faces, const vector<D>& fieldWeight, Ds& avarageFieldWeight, bool isDEBUG_show_results,
        vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees, bool mergeDirection_whenCountIsHigherFrom_dividesBy = true);
    void GetPointDirection(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees);
    void GetPointDirection_AvgDir(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees);
    void GetPointDirection_BestDir(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees);
    void GetPointDirection_BestDir_fast(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees);
    void GetPointDirection_AvgStream(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees);
    void GetPointDirection_StreamAngles(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees);
    void GetStartStreamDirections(vector<StreamStartPoint>& startStreamPoints, vector<StreamAnchorPoint>& anchorPoints, int dividerIterator_iterationNum);
};
