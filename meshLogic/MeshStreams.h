#pragma once
#include "Mesh.h"
#include "MeshConstrainsUV.h"

class MeshSolverUV;
class MeshSolverNrosy;
class ViewerDrawObjects;

struct StreamPoint : MeshPoint
{
    int fid; // faceid of point
    int intersectionID; // intersection between two streams - intersection can be on edge or in face or in vertex    
    D lengthToNextPoint;
    V3 dirToNextPointNormalized;
    D Length3dUntilThisPoint;
    StreamPoint(MeshPointType _Type, int _vid_eid_fid, const P3& _point, int _fid)
        : MeshPoint(_Type, _vid_eid_fid, _point), fid(_fid), intersectionID(-1), lengthToNextPoint(0), dirToNextPointNormalized(V3(0,0,0)), Length3dUntilThisPoint(0)
    {
    }    
};


struct DividerIteratorStreamInfo
{
    int iterationNum;
    int meshid__at_first_iteration;  //meshid of stream point - for dividerIterator this meshID will copied for next streams (next iterations)
    int streamStartPoint_id__at_first_iteration;
    int globalStreamIndex; // global index of stream for all meshes
    D Length3dUntilThisIteration;
    DividerIteratorStreamInfo()
        : iterationNum(-1), meshid__at_first_iteration(-1), streamStartPoint_id__at_first_iteration(-1), globalStreamIndex(-1), Length3dUntilThisIteration(0)
    {
    }
    DividerIteratorStreamInfo(int _iterationNum)
        : iterationNum(_iterationNum), meshid__at_first_iteration(-1), streamStartPoint_id__at_first_iteration(-1), globalStreamIndex(-1), Length3dUntilThisIteration(0)
    {
    }
};

struct StreamAnchorPoint : MeshPoint
{
private:
    static atomic_int nextStreamAnchorPointID;
public:
    int id; // unique id
    bool isBorderPoint;
    bool isSingularPoint;
    int producedStartStreamDirectionsCount;
    int dividerIterator_iterationNum; // used in DividerIterator
    bool haveContract;  // flag that indicated that this startStreamPoint is signed with anchorPoint by contract which stored in 'contract' property
    pair<int, int> contract; // { segment.id, streamIndex } set anchorPoint and startStreamPoint to same contract to force them to be connected
    StreamAnchorPoint(MeshPointType _Type, int _vid_eid_fid, const P3& _point, bool _isBorderPoint, bool _isSingularPoint, int _producedStartStreamDirectionsCount, int _dividerIterator_iterationNum)
        : MeshPoint(_Type, _vid_eid_fid, _point), id(nextStreamAnchorPointID++), isBorderPoint(_isBorderPoint), isSingularPoint(_isSingularPoint), producedStartStreamDirectionsCount(_producedStartStreamDirectionsCount), dividerIterator_iterationNum(_dividerIterator_iterationNum),
        haveContract(false), contract(pair<int, int>(0, 0))
    {
    }
    void ClearModifications(bool clearContract)
    {
        if (clearContract)
        {
            haveContract = false;
            contract = { 0,0 };
        }
    }
};

struct StreamStartPoint : MeshPoint
{
private:
    static atomic_int nextStreamStartDirectionID;
public:
    int id; // unique id
    int Index; // index in streams
    int StreamAnchorPointId; // unique id of StreamAnchorPoint that creates this startStreamDirection
    V3 dir;
    bool dir_isCustomDefined;
    D maxAngleChangeForDir; // maximum angle change for direction
    int fid; // face index of start direction
    D dir_ri;//dir_angleChangeFromStartFace
    int dividesBy; // how many directions starts from this point
    bool isBorderPoint; // is start point on edge
    bool isSingularPoint; // is start point in singularity
    bool canBeJoined;
    int joinedAtIndex;
    DividerIteratorStreamInfo dividingIteration; // used in DividerIterator    
    int dividingIteration_zero_meshid; // meshid of stream point - for dividerIterator this meshID will copied for next streams (next iterations)
    bool haveContract; // flag that indicated that this startStreamPoint is signed with anchorPoint by contract which stored in 'contract' property
    pair<int, int> contract; // { segment.id, streamIndex }, where 'streamIndex' started from '1' and ended at 'divisionCount-1', set anchorPoint and startStreamPoint to same contract to force them to be connected
    int rootStreamStartPointId; // dividerIterator continues stream lines in next meshes, and those next startPoints will reference to root startPoint from which everething begins
    vector<int> rootStartPoint_meshesids; // which meshes
    StreamStartPoint(MeshPointType _Type, int _vid_eid_fid, const P3& _point)
        : MeshPoint(_Type, _vid_eid_fid, _point), id(nextStreamStartDirectionID++), dir_isCustomDefined(false),
        dividingIteration(DividerIteratorStreamInfo()), haveContract(false), contract({ 0, 0 }), rootStreamStartPointId(-1)
    {
    }
    void SignContract(StreamAnchorPoint& a) //  default contract template
    {
        a.haveContract = true;
        a.contract = { a.id, 0 }; //  here 0 because there can be only 1 AnchorPoint for few StartPoint, so to avoid conflicts lets make this contract always unique for specific AnchorPoint
        haveContract = true;
        contract = a.contract;
    }
    void ClearModifications(bool clearJoin, bool clearContract)
    {
        if (clearJoin)
        {
            canBeJoined = false;
            joinedAtIndex = 0;
        }
        if (clearContract)
        {
            haveContract = false;
            contract = { 0,0 };
            rootStreamStartPointId = -1;
            rootStartPoint_meshesids.clear();
        }
    }
};



class MeshStream
{
public:
    int Index; // index in class MeshStreams
    int StreamStartPointId; // unique id of StreamStartPoint that creates this startStreamDirection
    int StreamAnchorPointId; // unique id of StreamAnchorPoint that creates this startStreamDirection
    ViewerDrawObjects & draw;
    const Mesh& mesh;
    vector<StreamPoint> Points;
    Color3d color;

    bool IsMerged;
    int IsMerged_with_streamIndex;
    bool IsDeleted;
    DividerIteratorStreamInfo dividingIteration; // used in DividerIterator
    D Length3d;
    bool isCyclic;


    StreamPoint& operator[](int index)
    {
        return Points[index];
    }
    const StreamPoint& operator[](int index) const
    {
        return Points[index];
    }
    MeshStream(int index, int streamStartPointId, int streamAnchorPointId, ViewerDrawObjects& draw, const Mesh& mesh, const Color3d& color, DividerIteratorStreamInfo dividingIteration);

    void Add(const StreamPoint& p);
    void Remove(int index, int count = 1);
    bool DuplicatePointsExists();
    int RemoveDuplicatePoints(bool tracedebuginfo = false);
    void Insert(int index, const StreamPoint& point);
    void SetNewPoints(const vector<StreamPoint>& newPoints, bool isInited_lengthToNextPoint_in_newPoints = false);
    void Set_IsInited_lengthToNextPoint(bool isInited);
    bool IsInited_lengthToNextPoint() const
    {
        return isInited_lengthToNextPoint;
    }
    void Init_lengthToNextPoint();
    bool isEndsAtBorder() const;
    P3 GetPointAtLength(D atLength) const;
    P3 GetMidPoint() const;
    DD GetDistToLineIndexDD(const Point3d& point, int lineIndex) const;
    D GetClosestDist_Linear(int points_startFromIndex, const P3& point, P3& pointOnPoints, int& pointIndex, bool breakAtFirstContact, bool returnFirstLastIndexIfFail, D maxDistRatioForNextContact = 8)  const;
    D GetClosestDist_Quick(const P3& point, P3& pointOnPoints, int& pointIndex, int points_startIndex = 0, int points_endIndex = -1) const;
    D GetTotalLength(int indexStart = 0, int indexEnd = -1) const;
    void GetPointsDividedByCount(int divisioncount, P3s& points, bool isPointsStoredInRevertedOrder) const;
    II SizeOF() const;
    static bool GetConnection(ViewerDrawObjects& draw, const Mesh& mesh, MeshStream& stream1, MeshStream& stream2, D& connectionDist, int& connectionIndex_points1, int& connectionIndex_points2, int points1_maxIndex=-1, int points2_maxIndex=-1, D maxConnectionAngle_Directions = 40, D maxConnectionAngle_Normals = 120, bool debug = false, bool debugEveryPoint = false, D maxLenghtToTest = -1);
    bool Remove_PrelastPoint_If_LastIsVertex_And3LastsHaveCommonFace();
    bool static FindBestCommonEdgeOrVertexBetween2Streams_InScope(const MeshStream& stream1, const MeshStream& stream2, int stream1_connectionIndex, int stream2_connectionIndex, int& stream1_commonPointIndex, int& stream2_commonPointIndex, int maxSeekInterval = 7);
    bool static FindBestCommonEdgeOrVertexBetween2Streams_AllPoints(const MeshStream& stream1, const MeshStream& stream2, int& stream1_commonPointIndex, int& stream2_commonPointIndex); // search everething - slower version but tryis harder
    bool static FindBestCommonFaceBetween2Streams(const MeshStream& stream1, const MeshStream& stream2, int& commonPointIndex1, int& commonPointIndex2); // search everething, return most middle face point 
    void DebugShowStreamPoints(const Color3d& color, int maxIndex = -1) const;
    void clear()
    { 
        isInited_lengthToNextPoint = false;
        Points.clear();
        Length3d = 0;
        isCyclic = false;
    }
    int size() 
    {
        return Points.size();
    }
    int size() const
    {
        return Points.size();
    }
    int capacity() const
    {
        return Points.capacity();
    }
    void shrink_to_fit() 
    {
        Points.shrink_to_fit();
    }
    void reserve(int count)
    {
        Points.reserve(count);
    }
private:
    bool isInited_lengthToNextPoint;
};

struct MatchABBA
{
    __int16 match_ab_ba;
    MatchABBA()        
    {
    }
    MatchABBA(int _match_ab_ba)
        :match_ab_ba((__int16)_match_ab_ba)
    {
    }


    // constructor for field with N=4
    MatchABBA(int ab0, int ab1, int ab2, int ab3, int ba0, int ba1, int ba2, int ba3)
    {
        match_ab_ba = (__int16)((ab0 << (2 * 0)) | (ab1 << (2 * 1)) | (ab2 << (2 * 2)) | (ab3 << (2 * 3))
            | (ba0 << (8 + 2 * 0)) | (ba1 << (8 + 2 * 1)) | (ba2 << (8 + 2 * 2)) | (ba3 << (8 + 2 * 3)));
    }
   
    static __forceinline MatchABBA Create(int rotation_degree)  // degree=0..3 (0,90,180,270)
    {
        switch (rotation_degree)
        {
            case 0:
                return  MatchABBA(0, 1, 2, 3, 0, 1, 2, 3);
            case 1:
                return MatchABBA(1, 2, 3, 0, 3, 0, 1, 2);
            case 2:
                return MatchABBA(2, 3, 0, 1, 2, 3, 0, 1);
            case 3:
                return MatchABBA(3, 0, 1, 2, 1, 2, 3, 0);
            default:
                assert(0<= rotation_degree && rotation_degree <= 3);
                return MatchABBA();
        }
    }
    int ab(int ni) const // ni=0..3 for nrosy field with N=4
    {
        return (match_ab_ba >> (2 * ni)) & 3;
    }
    int ba(int ni) const // ni=0..3 for nrosy field with N=4
    {
        return (match_ab_ba >> (8 + 2 * ni)) & 3;
    }
    void IncDegree(int inc) // value can be positive or negative
    {
        if (inc == 0) return;
        int a[4] = { ab(0), ab(1), ab(2), ab(3) };
        int b[4] = { ba(0), ba(1), ba(2), ba(3) };
        for (int i = 0; i < 3; i++)
        {
            a[i] += inc;
            while (a[i] < 0) a[i] += 4;
            while (a[i] > 3) a[i] -= 4;
            b[i] -= inc;
            while (b[i] < 0) b[i] += 4;
            while (b[i] > 3) b[i] -= 4;
        }
        *this = MatchABBA(a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]);
    }
};

using MatchABBAs = Vector<MatchABBA>;



class MeshStreamer
{
public:
    ViewerDrawObjects & draw;
    const Mesh& mesh;
    const vector<V3s>& nrosyField;
    bool IsNrosyFieldSymetric;  // symetric field can be stored with half of fields, others half will be negative
    const MatchABBAs& match_ab_ba;   //  #E by N matrix, describing for each edge the matching a->b, where a and b are the current_faces adjacent to the edge (i.e. vector #i of the vector set in a is matched to vector #mab[i] in b),    #E by N matrix, describing the inverse relation to match_ab

    V3 start_direction;
    V3 start_normal;
    P3 current_start_point;
    P3 current_end_point;
    int current_fid;
    int current_direction;
    int current_start_edgeid;
    MeshPointType current_end_pointType;
    int current_end_vid_eid_fid;
    bool finished;

    MeshStreamer(ViewerDrawObjects& draw, const Mesh& mesh, const vector<V3s>& nrosyField, bool IsNrosyFieldSymetric, const MatchABBAs& match_ab_ba);
    void RestartStream(MeshStream& Stream, int fid, MeshPointType pointType, int vid_eid_fid, const P3& point, const V3& direction);
    int ExtendStream(MeshStream& Stream, int extendCount, bool storeNewPointInPointsVector = true);
    II SizeOF() const;
private:
    bool ExtendStreamDirect(MeshStream& Stream);
    int GetClosestNrosyFieldDirectionNi(MeshStream& Stream, int faceId, const V3& direction);
    bool ManuallyGetNextFaceFromStartPoint(MeshStream& Stream, int fid, int start_vid, const V3& direction, P3& endPoint, int& nextfid, V3& nextdirection);
    bool ManuallyGetNextFaceFromStartPoint(MeshStream& Stream, int fid, const P3& startPoint, const V3& direction, P3& endPoint, int& nextfid, V3& nextdirection, int& vid);
    bool ManuallyGetNextFaceFromVertex(MeshStream& Stream, int fid, int vid, const V3& direction, const P3& endPoint, bool TEST, int& nextfid, V3& nextdirection);
    V3 getnrosyField(int ni, int fid) const
    {
        assert(ni < 4);
        if (ni >= 2 && IsNrosyFieldSymetric)
        {
            return -nrosyField[ni % 2].row(fid);
        }
        else
        {
            return nrosyField[ni].row(fid);
        }
    }

};

class MeshStreamerUV
{
public:
    ViewerDrawObjects & draw;
    const Mesh& mesh;
    const P3* V;          // pointer to mesh.V.data()
    const I3* F;  // pointer to mesh.F.data()
    const I3* FE; // pointer to mesh.FE.data()
    const I2* EF; // pointer to mesh.EF.data()
    const vector<UV>& uvField;

    MeshUV start_direction;
    UV end_direction;
    int current_fid; // help improves perfromance by remembering from what face we are come from, so we can ignore it at test
    MeshUV current_start_direction;
    MeshUV current_end_direction;
    bool finished;

    MeshStreamerUV(ViewerDrawObjects& draw, const Mesh& mesh, const vector<UV>& uvField);
    void RestartStream(MeshStream& Stream, const MeshUV& uvBegin, const UV& uvEnd);
    int ExtendStream(MeshStream& Stream, int extendCount, bool storeNewPointInPointsVector = true);
    II SizeOF() const;
private:
    bool ExtendStreamDirect(MeshStream& Stream);
    //bool ManuallyGetNextFaceFromStartPoint(MeshStream& Stream, int fid, int start_vid, const V3& direction, P3& endPoint, int& nextfid, V3& nextdirection);
    //bool ManuallyGetNextFaceFromStartPoint(MeshStream& Stream, int fid, const P3& startPoint, const V3& direction, P3& endPoint, int& nextfid, V3& nextdirection, int& vid);
    //bool ManuallyGetNextFaceFromVertex(MeshStream& Stream, int fid, int vid, const V3& direction, const P3& endPoint, bool TEST, int& nextfid, V3& nextdirection);
};


class MeshStreams
{
public:
    vector<MeshStream> streams;// declared on top for easyier debuging
    vector<MeshStreamer> streamers;// declared on top for easyier debuging
    vector<MeshStreamerUV> streamersUV;// declared on top for easyier debuging

    ViewerDrawObjects& draw;
    const Mesh& mesh;
    
    // nrosy solver data
    const vector<V3s>& nrosyField;
    bool IsNrosyFieldSymetric;  // symetric field can be stored with half of fields, others half will be negative
    const MatchABBAs& match_ab_ba;   //  #E by N matrix, describing for each edge the matching a->b, where a and b are the current_faces adjacent to the edge (i.e. vector #i of the vector set in a is matched to vector #mab[i] in b), #E by N matrix, describing the inverse relation to match_ab

    // uv solver data
    const vector<UV>& uvs;

    
    MeshStream& operator[](int index)
    {
        return streams[index];
    }
    const MeshStream& operator[](int index) const
    {
        assert(index < streams.size());
        return streams[index];
    }

    MeshStreams(ViewerDrawObjects& draw, const Mesh& mesh, const MeshSolverNrosy& solver);
    MeshStreams(ViewerDrawObjects& draw, const Mesh& mesh, const MeshSolverUV& solver);

    void Reserve(int count);
    void ClearStreams();
    int ExtendStreams(int extendCount, bool storeNewPointInPointsVector = true, int preserveStreamsUntilDividingIteration = -1); // this populates StreamPoints property and works multiprocessor
    void Add(int streamStartPointId, int streamAnchorPointId, int fid, MeshPointType pointType, int vid_eid_fid, const P3& point, const V3& direction, const Color3d& color, DividerIteratorStreamInfo dividingIteration);
    void Add(int streamStartPointId, int streamAnchorPointId,const MeshUV& uvStart, const UV& uvEnd, const Color3d& color, DividerIteratorStreamInfo dividingIteration);
    int Count() const;
    II SizeOF() const;
private:
    V3 getnrosyField(int ni, int fid) const
    {
        assert(ni < 4);
        if (ni >= 2 && IsNrosyFieldSymetric)
        {
            return -nrosyField[ni % 2].row(fid);
        }
        else
        {
            return nrosyField[ni].row(fid);
        }
    }
};


