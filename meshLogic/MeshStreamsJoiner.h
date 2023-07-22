#pragma once
#include "Divider.h"
class Mesh;
class Divider;
class MeshSolverNrosy;
class MeshSurface;
class ViewerDrawObjects;
class MeshStreams;


class MeshStreamsJoiner
{
public:
    const Mesh& mesh;
    const MeshSolverNrosy& solver;
    ViewerDrawObjects& draw;
    
    vector<StreamStartPoint>& startStreamPoints;
    vector<StreamAnchorPoint>& anchorPoints;
    MeshStreams& streams;
    D meshSize;


    MeshStreamsJoiner(Divider& divider, D meshSize);
    void JoinStreams();
    int StreamsAdjuster_callsCount_directMethod;
    int StreamsAdjuster_callsCount_iterativeMethod;
private:
    void JoinStreams1time();
    void JoinStreams1000times();
    struct ClosestPointFromStreamToAnchor
    {
        bool isCyclic; // is cyclic - if stream dont ends at border and was terminated becuase of cyclic condition
         //D dists_to_anchors_from_start_point;
        int index; // index of closest edge in stream 'Points'
        D length; // total length to closest index plus length to closest point from closest index
        D dist; // closest dist from anchor to stream
        P3 pointOnStream; // closest point on stream
        bool isDistAcceptable; // if dist from stream to anchor is acceptable for connecting them
    };
    struct StartPointToAnchorPossibleConnection
    {
        const MeshStreams& streams;
        const MeshStream& stream;
        int Index; // index in 'vector<StreamPointToAnchorPossibleConnection> possibleConnections'
        StreamStartPoint& sp;
        const StreamAnchorPoint& ap;
        ClosestPointFromStreamToAnchor closestPoint;
        StartPointToAnchorPossibleConnection(const MeshStreams& _streams, int index, StreamStartPoint& _sp, const StreamAnchorPoint& _ap, const ClosestPointFromStreamToAnchor& _closestPoint)
            : streams(_streams), stream(streams[_sp.Index]), Index(index), sp(_sp), ap(_ap), closestPoint(_closestPoint)
        {
        }
        P3 MidPoint() const
        {
            return stream.GetPointAtLength(closestPoint.length / 2);
        }
    };

    struct StreamPointsConnection
    {
        int Index;// index in vector<StreamPointsConnection> connections
        pair<int, int> sindexes_min_max; // = {min(pc1.sp.StreamId , pc2.sp.StreamId),  max(pc1.sp.StreamId , pc2.sp.StreamId) };
        //pair<int, int> streamIndexes;
        //pair<int, int> sid_min_max; // = {min(pc1.sp.StreamId , pc2.sp.StreamId),  max(pc1.sp.StreamId , pc2.sp.StreamId) };
        pair<int, int> aid_min_max; // = {min(pc1.sp.AnchorId , pc2.sp.AnchorId),  max(pc1.sp.AnchorId , pc2.sp.AnchorId) };
        const StartPointToAnchorPossibleConnection& pc1;
        const StartPointToAnchorPossibleConnection& pc2;
        D travel_length; // avarage connection length of 2 streams from startStreamPoint to anchorPoint
        D connectionDistAtMid; // connection distance between 2 streams in connection point at mid of shortest path of 2 streams
        int connection_pointsIndex1; 
        int connection_pointsIndex2;
        P3 connectionPoint;
        bool hasCyclicStream;
        bool haveContract;             // pc1.sp.haveContract || pc2.sp.haveContract
        D sortWeight;
        StreamPointsConnection(int index, const StartPointToAnchorPossibleConnection& _pc1, const StartPointToAnchorPossibleConnection& _pc2, D _connectionDistAtMid,int _connection_pointsIndex1, int _connection_pointsIndex2)
            : Index(index), pc1(_pc1), pc2(_pc2), travel_length((_pc1.closestPoint.length + _pc2.closestPoint.length) / 2), connectionDistAtMid(_connectionDistAtMid), connection_pointsIndex1(_connection_pointsIndex1), connection_pointsIndex2(_connection_pointsIndex2), connectionPoint(P3(0, 0, 0))
        {
            hasCyclicStream = (pc1.closestPoint.isCyclic || pc2.closestPoint.isCyclic);
            
            int sindex1 = (pc1.sp.dividingIteration.globalStreamIndex != -1) ? pc1.sp.dividingIteration.globalStreamIndex : pc1.sp.Index;
            int sindex2 = (pc2.sp.dividingIteration.globalStreamIndex != -1) ? pc2.sp.dividingIteration.globalStreamIndex : pc2.sp.Index;
            sindexes_min_max = { min(sindex1 , sindex2),  max(sindex1 , sindex2) };

            //streamIndexes = { _pc1.sp.Index, _pc2.sp.Index };

            //int sid1 = (pc1.sp.dividingIteration.streamStartPoint_id__at_first_iteration != -1) ? pc1.sp.dividingIteration.streamStartPoint_id__at_first_iteration : pc1.sp.id;
            //int sid2 = (pc2.sp.dividingIteration.streamStartPoint_id__at_first_iteration != -1) ? pc2.sp.dividingIteration.streamStartPoint_id__at_first_iteration : pc2.sp.id;
            //sid_min_max = { min(sid1 , sid2),  max(sid1 , sid2) };

            int aid1 = pc1.sp.StreamAnchorPointId;
            int aid2 = pc2.sp.StreamAnchorPointId;
            aid_min_max = { min(aid1 , aid2),  max(aid1 , aid2) };


            haveContract = pc1.sp.haveContract || pc2.sp.haveContract;
            sortWeight = 0;
        }
    };

    struct AlternativePath
    {
        int Index; // index in vector<AlternativePath> alternativePaths
        StreamPointsConnection& originConnection;

        vector<StreamPointsConnection*> alternativeConnection;
        vector<StreamPointsConnection*> alternativeConnection_nonExpanded;
        //vector<int> aids;
        //int start_aid;
        //int end_aid;
        //pair<int, int> aid_min_max; // = min(start_aid, end_aid), max(start_aid, end_aid)
        D maxDistFromAidsToStream;
        bool isRemovedBecauseItIsUseless;

        AlternativePath(int _index, StreamPointsConnection& _c)
            : Index(_index), originConnection(_c)/*, start_aid(0), end_aid(0), aid_min_max({ 0,0 })*/, maxDistFromAidsToStream(0), isRemovedBecauseItIsUseless(false)
        {
        }


        D GetMinSortWeight();
        D GetMaxSortWeight();
        bool ExpandAlternativeConnection(vector<AlternativePath*> connection_to_alternativePath);
        bool Exists(int connectionIndex, bool serachInNonExpandable = false);
    };
    bool FindAlternativePath(D maxDist, const StreamPointsConnection& c, map<int, vector<StreamPointsConnection*>>& map_anchorid_connection, bool debug, int debugStreamId, vector<StreamPointsConnection*>& alternativePath, vector<int>& aids, D& maxDistFromAidsToStream);
    void FindAlternativePaths(D maxDist, const vector<StartPointToAnchorPossibleConnection>& possibleConnections, vector<StreamPointsConnection>& connections_all, vector<AlternativePath>& alternativePaths);
    void ExpandAlternativePaths(const vector<StreamPointsConnection>& connections_all, vector<AlternativePath>& alternativePaths);

    bool FindPossibleConnections(const MeshStreams& streams, StreamStartPoint& sp, const vector<StreamAnchorPoint>& anchors, vector<StartPointToAnchorPossibleConnection>& possibleConnections);
    void FindAllConnections(D maxDist, const vector<StreamStartPoint>& startStreamPoints, const vector<StartPointToAnchorPossibleConnection>& possibleConnections, vector<StreamPointsConnection>& connections_all);
    void SortConnections(const vector<StreamPointsConnection>& connections_all, vector<unsigned int>& sorted_indexes);

    void FindConnections(const vector<StreamStartPoint>& startStreamPoints, const vector<StartPointToAnchorPossibleConnection>& possibleConnections, vector<StreamPointsConnection>& connections);
    void SetConnectionPoint(MeshStreams& streams, StreamPointsConnection& c);
    void AdjustConnectedStreams(MeshStreams& s, StreamPointsConnection& connection);
    bool MergeConnectedStreams(MeshStreams& s, StreamPointsConnection& connection);
};