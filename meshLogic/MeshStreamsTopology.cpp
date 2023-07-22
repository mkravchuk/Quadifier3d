#include "stdafx.h"
#include "MeshStreamsTopology.h"
#include "Divider.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshStreams.h"
#include "MeshSolverNrosy.h"
#include "MeshSurface.h"
#include "MeshCutter.h"
#include "DividerIterator.h"
#include "MeshStreamsJoiner.h"

const MeshLogicOptions_MeshStreamsTopology& options = meshLogicOptions.MeshStreamsTopology;



StreamSegment::StreamSegment(int _index, const MeshStream& _stream, int from, int to, const SSTopologySegment* _ssSegment, ViewerDrawObjects& _draw)
    : Index(_index), Length3d(0), stream(_stream), fromPointIndex(from), toPointIndex(to),
    ssSegment(_ssSegment), curveSegment(_ssSegment == nullptr ? nullptr : _ssSegment->curveSegment),
    draw(_draw)
{
    Length3d = 0;
    for (int i = fromPointIndex; i < toPointIndex; i++)
    {
        Length3d += stream[i].lengthToNextPoint;
    }
    intersectionIds[0] = stream.Points[fromPointIndex].intersectionID;
    intersectionIds[1] = stream.Points[toPointIndex].intersectionID;
}

P3 StreamSegment::PointAt(D posAtLength, bool shiftIfOnSegment) const
{
    D currentLength3d = 0;
    for (int i = fromPointIndex; i < toPointIndex; i++)
    {
        D edgeLength = stream[i].lengthToNextPoint;
        //draw.AddPoint(stream[i].point, Color3d(1, 0, 0));
        if (currentLength3d <= posAtLength && posAtLength <= currentLength3d + edgeLength)
        {
            D shift = posAtLength - currentLength3d;
            //cout << stream[i].dirToNextPointNormalized.norm() << endl;
            assert(i < stream.size() && "dirToNextPointNormalized is not correct to use in last point");
            P3 point = stream[i].point + stream[i].dirToNextPointNormalized*shift;
            if (shiftIfOnSegment && curveSegment != nullptr)
            {
                int commonFaceId1;
                int commonFaceId2;
                const Mesh& mesh = stream.mesh;
                if (mesh.CommonFaceIds(stream[i], stream[i + 1], commonFaceId1, commonFaceId2))
                {
                    int fid = commonFaceId1;
                    if (mesh.F_isborder[fid])
                    {
                        int eid = mesh.FE(fid, 0);
                        if (!mesh.E_isborder[eid]) eid = mesh.FE(fid, 1);
                        if (!mesh.E_isborder[eid]) eid = mesh.FE(fid, 2);
                        point += mesh.EdgeNormalToFace(eid, fid) * mesh.avg_edge_length / 2;
                    }
                }
            }
            return point;
        }
        currentLength3d += edgeLength;
    }
    assert(false && "StreamSegment::PointMid()   didnt found midpoint");
    cout << "!!! wrong:  StreamSegment::PointMid()   didnt found midpoint" << endl;
    return P3(0, 0, 0);
}

P3 StreamSegment::PointMid(bool shiftIfOnSegment) const
{
    D midLength3d = Length3d / 2;
    return PointAt(midLength3d, shiftIfOnSegment);
}


SSIntersection::SSIntersection()
    : IntersectionId(0), IntersectionIdOpposite(0), segment(nullptr), dirNormalized(V3(0, 0, 0)), dirNormalizedOnIntersectionNormal(V3(0, 0, 0)), AngleFullToFirstSegment(0), AngleFullToNextSegment(0)
{
}

SSIntersection::SSIntersection(int _intersectionId, StreamSegment* _segment)
    : IntersectionId(_intersectionId), IntersectionIdOpposite(0), segment(_segment), dirNormalized(V3(0, 0, 0)), dirNormalizedOnIntersectionNormal(V3(0, 0, 0)), AngleFullToFirstSegment(0), AngleFullToNextSegment(0)
{
    if (_segment->intersectionIds[0] == IntersectionId)
    {
        IntersectionIdOpposite = _segment->intersectionIds[1];
        dirNormalized = _segment->stream[_segment->fromPointIndex + 1] - _segment->stream[_segment->fromPointIndex];
    }
    else
    {
        IntersectionIdOpposite = _segment->intersectionIds[0];
        dirNormalized = _segment->stream[_segment->toPointIndex - 1] - _segment->stream[_segment->toPointIndex];
    }
    dirNormalized.normalize();
    dirNormalizedOnIntersectionNormal = dirNormalized;
}

SSQuad::SSQuad(int _index, int _intersectionIds[4], map<pair<int, int>, StreamSegment*>& map_intersectionIds_to_streamSegment)
    : Index(_index), isTriangle(false)
{
    for (int i = 0; i < 4; i++)
    {
        intersectionIds[i] = _intersectionIds[i];
        int i0 = _intersectionIds[i];
        int i1 = _intersectionIds[(i + 1) % 4];
        if (i1 < i0) swap(i0, i1); // make sure first id is lower
        auto find = map_intersectionIds_to_streamSegment.find({ i0, i1 });
        if (find == map_intersectionIds_to_streamSegment.end())
        {
            assert(find != map_intersectionIds_to_streamSegment.end() && "cannot find segment from intersection ids");
            cout << "!!!wrong   DividerOptimalConnector:  cannot find segment from intersection ids" << endl;
            segments[i] = nullptr;
        }
        else
        {
            segments[i] = find->second;
        }
    }
}

SSQuad::SSQuad(int _index, SSIntersection* _segments[4])
    : Index(_index), isTriangle(false)
{
    isTriangle = (_segments[3] == nullptr);

    // rotate sides,that smallest will at second position, and longest at first and thirth
    if (isTriangle)
    {
        auto smallestSegmentIndex = [&]()
        {
            int index = -1;
            D min_length = 0;
            for (int i = 0; i < 3; i++)
            {
                D len = _segments[i]->segment->Length3d;
                if (index == -1 || len < min_length)
                {
                    min_length = len;
                    index = i;
                }
            }
            return index;
        };
        // rotate segments until smallest one will be on second place
        for (int i = 0; i < 2; i++)
        {
            if (smallestSegmentIndex() == 1) break; // stop when smallest is on second place
            swap(_segments[2], _segments[0]); // was 0,1,2 became 2,1,0
            swap(_segments[2], _segments[1]); // was 2,1,0 became 2,0,1
        }
    }

    // init arrays
    for (int i = 0; i < 4; i++)
    {
        if (isTriangle && i == 3) // triangle doesnt have 4-th side
        {
            intersectionIds[i] = -1;
            segments[i] = nullptr;
            break;
        }
        intersectionIds[i] = _segments[i]->IntersectionId;
        segments[i] = _segments[i]->segment;
    }
}

bool SSQuad::isValid()
{
    // validate quad - if all segments are found in 'mesh.map_intersectionIds_to_streamSegment'
    if (segments[0] == nullptr
        || segments[1] == nullptr
        || segments[2] == nullptr
        || (!isTriangle && segments[3] == nullptr))
    {
        return false;
    }

    // validate quad - avoid adding quads that goes around holes using same segment                          
    if (segments[0]->curveSegment != nullptr
        && segments[1]->curveSegment != nullptr
        && segments[2]->curveSegment != nullptr
        && (isTriangle || segments[3]->curveSegment != nullptr)) // if all quad-segment on curve-segments
    {
        if (segments[0]->curveSegment->curve.loop.Type == MeshLoopType::Inner)
        {
            return false;
        }
    }

    // no issues - quad is valid
    return true;
}

P3 SSQuad::PointMid()
{
    P3 middle = segments[0]->PointMid(false);
    middle += segments[1]->PointMid(false);
    middle += segments[2]->PointMid(false);
    if (isTriangle)
    {
        return middle / 3;
    }
    else
    {
        middle += segments[3]->PointMid(false);
        return middle / 4;
    }
}

SSQuad_InLoop::SSQuad_InLoop(const SSQuad* _quad, int _index0)
    : quad(_quad), index0(_index0), index1((_index0 + 2) % 4), segment0(_quad->segments[_index0]), segment1(_quad->segments[(_index0 + 2) % 4])
{
    segment1_isTriangleStop = false;
    if (quad->isTriangle && index1 == 3)
    {
        segment1 = segment0;
        segment1_isTriangleStop = true;
    }
}


atomic_int SSQuadsLoop_id = 0;
SSQuadsLoop::SSQuadsLoop(int _meshId)
    : id(SSQuadsLoop_id++), meshId(_meshId), reserved_div(0), reserved_length3d(0)
{
    segment01[0] = segment01[1] = nullptr;
    div01[0] = div01[1] = 0;
    isTriangleStop[0] = isTriangleStop[1] = false;
    sides[0] = SSQuadsLoop_Side();
    sides[1] = SSQuadsLoop_Side();
}

void SSQuadsLoop::UpdateAfterAdd()
{
    segment01[0] = quads.front().segment0;
    segment01[1] = quads.back().segment1; // can be a triangleStop, means segment1 == segment0

    isTriangleStop[0] = false; // first segment never is triangle
    isTriangleStop[1] = quads.back().segment1_isTriangleStop; // 'isTriangle[1]' corresponds to 'segment01[1] = quads.back().segment1', so this side can be traingleStop

    sides[0] = SSQuadsLoop_Side(this, 0); // must be called after populating 'isTriangle'
    sides[1] = SSQuadsLoop_Side(this, 1); // must be called after populating 'isTriangle'
}

atomic_int SSQuadsLoop_Side_ConflictJoin_id = 0;
SSQuadsLoop_Side_ConflictJoin::SSQuadsLoop_Side_ConflictJoin(SSQuadsLoop_Side& _side, int _divIndex)
    : id(SSQuadsLoop_Side_ConflictJoin_id++), side(_side), divIndex(_divIndex), isTaken(false)
{
}

SSQuadsLoop_Side_ConflictJoin* SSQuadsLoop_Side_ConflictJoin::opposite()
{
    auto& sideOpposite = side.oppositeSide();
    if (side.div() != sideOpposite.div())
    {
        return nullptr;
    }

    // rare case for loop with one quad-triangle, where one side have joins and other doesnt
    if (sideOpposite.isTriangleStop && sideOpposite.conflictJoins.size() == 0)
    {
        return nullptr;
    }

    int divIndexOpposite = side.div() - (divIndex + 1);
    assert(divIndexOpposite < sideOpposite.conflictJoins.size());
    return &sideOpposite.conflictJoins[divIndexOpposite];
}

void SSQuadsLoop_Side_ConflictJoin::Clear()
{
    isTaken = false;
}

int SSQuadsLoop_Side_ConflictJoin::GetDivIndexInSegment()
{
    auto segment = side.segment()->ssSegment;
    if (segment == nullptr)
    {
        cout << "! wrong:   SSQuadsLoop_Side_ConflictJoin::GetDivIndexInSegment()   segment == nullptr" << endl;
        return -1;
    }
    int index = -1;
    for (auto& sideI : segment->loopSides)
    {
        for (auto& join : sideI->conflictJoins)
        {
            index++;
            if (join.id == id)
            {
                return index;
            }
        }
    }

    cout << "! wrong:   SSQuadsLoop_Side_ConflictJoin::GetDivIndexInSegment()   join not found in segment" << endl;
    return -1;
}

P3 SSQuadsLoop_Side_ConflictJoin::GetMidPoint()
{
    int divIndex = GetDivIndexInSegment();
    int total = side.segment()->ssSegment->GetSumDivBy();
    D Length3d = side.segment()->curveSegment->Length3d;
    D len1div = Length3d / total;
    D pos0 = divIndex * len1div;
    D pos1 = (divIndex + 1) * len1div;
    D posMid = (pos0 + pos1) / 2;
    //cout << "divIndex=" << divIndex << "  total=" << total << "   Length3d="<< Length3d << "  pos0="<< pos0 <<"  pos1="<< pos1<< endl;
    return side.segment()->curveSegment->GetPointAtLength(posMid);
}

int SSQuadsLoop_Side_ConflictJoin::meshid()
{
    return side.segment()->stream.mesh.id;
}


atomic_int SSQuadsLoop_Side_Cycl_id = 0;
SSQuadsLoop_Side_Cycl::SSQuadsLoop_Side_Cycl(SSQuadsLoop_Side& _side, int _Index)
    : id(SSQuadsLoop_Side_Cycl_id++), side(_side), Index(_Index)
{
}

void SSQuadsLoop_Side_Cycl::Clear()
{

}

int SSQuadsLoop_Side_Cycl::GetDivIndexInSegment()
{
    auto segment = side.segment()->ssSegment;
    if (segment == nullptr)
    {
        cout << "! wrong:   SSQuadsLoop_Side_Cycl::GetDivIndexInSegment()   segment == nullptr" << endl;
        return -1;
    }
    int index = -1;
    for (auto& sideI : segment->loopSides)
    {
        for (auto& cycl : sideI->cycles)
        {
            index++;
            if (cycl.id == id)
            {
                return index;
            }
        }
    }

    cout << "! wrong:   SSQuadsLoop_Side_Cycl::GetDivIndexInSegment()   join not found in segment" << endl;
    return -1;
}

P3 SSQuadsLoop_Side_Cycl::GetMidPoint()
{
    int index = GetDivIndexInSegment();
    int total = side.segment()->ssSegment->GetSumDivBy();
    D Length3d = side.segment()->curveSegment->Length3d;
    D len1div = Length3d / total;
    D pos0 = index * len1div;
    D pos1 = (index + 1) * len1div;
    D posMid = (pos0 + pos1) / 2;
    //cout << "divIndex=" << divIndex << "  total=" << total << "   Length3d="<< Length3d << "  pos0="<< pos0 <<"  pos1="<< pos1<< endl;
    return side.segment()->curveSegment->GetPointAtLength(posMid);
}

int SSQuadsLoop_Side_Cycl::meshid()
{
    return side.segment()->stream.mesh.id;
}

SSQuadsLoop_Side::SSQuadsLoop_Side()
    : loop(nullptr), sideIndex(0), isTriangleStop(false)
{
}

SSQuadsLoop_Side::SSQuadsLoop_Side(SSQuadsLoop* _loop, int _sideIndex)
    : loop(_loop), sideIndex(_sideIndex)
{
    isTriangleStop = loop->isTriangleStop[sideIndex];
}

int& SSQuadsLoop_Side::div() const
{
    return loop->div01[sideIndex];
}

const StreamSegment* SSQuadsLoop_Side::segment() const
{
    return loop->segment01[sideIndex];
}

SSQuadsLoop_Side& SSQuadsLoop_Side::oppositeSide()
{
    return loop->sides[(sideIndex + 1) % 2];
}

void SSQuadsLoop_Side::ReinitConflictJoins()
{
    int divby = div();
    if (divby == 0)
    {
        conflictJoins.clear();
        return;
    }
    while (conflictJoins.size() > divby)
    {
        conflictJoins.pop_back();
    }
    if (conflictJoins.size() < divby)
    {
        conflictJoins.reserve(divby);
        while (conflictJoins.size() < divby)
        {
            conflictJoins.push_back(SSQuadsLoop_Side_ConflictJoin(*this, conflictJoins.size()));
        }
    }
    for (int i = 0; i < divby; i++)
    {
        conflictJoins[i].Clear();
    }
}

SSTopologySegment::SSTopologySegment()
    : curveSegment(nullptr), connection(nullptr), mesh(nullptr), Friend(nullptr)
{
    loopSides.clear();
}

int SSTopologySegment::GetSumDivBy() const
{
    int divBy_summ = 0;
    for (auto loopSide : loopSides)
    {
        int divBy = loopSide->div();
        divBy_summ += divBy;
    }
    return divBy_summ;
}

void SSTopologySegment::SetDivBySumm(int new_divBySumm, bool updateLoopEnd, int iterationNum, bool debug_iteration, ViewerDrawObjects& draw)
{
    bool trace = options.debug_trace_iterations_in_console;
    if (trace) cout << "    segment#" << curveSegment->id << "  has " << GetSumDivBy() << " and have to change to " << new_divBySumm << endl;

    int inc_divBy = new_divBySumm - GetSumDivBy();
    while (inc_divBy > 0)
    {
        //
        // find best loop to increase size
        //
        int best_index = -1;
        D best_maxsize = 0;
        bool best_isLoopConnectedToSegment2Times = false;
        for (int i = 0; i < loopSides.size(); i++)
        {
            SSQuadsLoop_Side& side = *loopSides[i];
            bool isLoopConnectedToSegment2Times = side.loop->segment01[0] != side.loop->segment01[1]
                && side.loop->segment01[0]->curveSegment != nullptr
                && side.loop->segment01[1]->curveSegment != nullptr
                && side.loop->segment01[0]->curveSegment->id == side.loop->segment01[1]->curveSegment->id; // if loop connected to current segment 2 times - then we have to increase div by 1+1==2
            if (isLoopConnectedToSegment2Times && inc_divBy == 1) continue;

            D length = side.segment()->Length3d;
            int divby = side.div();
            D maxSize = length / (divby + 1);
            if (isLoopConnectedToSegment2Times)
            {
                D length2 = side.oppositeSide().segment()->Length3d;
                int divby2 = side.oppositeSide().div();
                D maxSize2 = length2 / (divby2 + 1);
                maxSize = (maxSize + maxSize2) / 2; // take avarage
            }

            if (best_index == -1 || maxSize > best_maxsize)
            {
                best_index = i;
                best_maxsize = maxSize;
                best_isLoopConnectedToSegment2Times = isLoopConnectedToSegment2Times;
            }
        }


        //
        // increase size
        //
        if (best_index != -1)
        {
            SSQuadsLoop_Side& side = *loopSides[best_index];
            if (side.div() < new_divBySumm) side.div()++;
            if (best_isLoopConnectedToSegment2Times || updateLoopEnd)
            {
                if (side.oppositeSide().div() < new_divBySumm) side.oppositeSide().div()++;
            }

            if (debug_iteration)
            {
                auto segment = side.segment();
                draw.AddLabel(segment->PointMid(true), "iteration#" + to_string(iterationNum), Color3d(1, 0, 0), 5);
            }
        }


        // decrement
        inc_divBy -= 1;
        if (best_isLoopConnectedToSegment2Times) inc_divBy -= 1;
    }
}

SSQuadsLoop_Side_ConflictJoin* SSTopologySegment::GetJoinByIndex(int joinIndex, bool getFromBack)
{
    int joinIndexNormalized = joinIndex;
    if (getFromBack)
    {
        joinIndexNormalized = GetSumDivBy() - (joinIndex + 1);
    }
    int index = -1;
    for (auto& sideI : loopSides)
    {
        for (auto& join : sideI->conflictJoins)
        {
            index++;
            if (index == joinIndexNormalized)
            {
                return &join;
            }
        }
    }
    assert(false && "SSTopologySegment::GetJoinByIndex   join not found");
    cout << "! wrong  SSTopologySegment::GetJoinByIndex   join not found for index " << joinIndex << endl;
    return nullptr;
}

SSTopologyConnection::SSTopologyConnection(int _index, const TopologyConnection& _connection)
    : Index(_index), connection(_connection), segmentsCount(connection.segments.size())
{
    for (int i = 0; i < segmentsCount; i++)
    {
        segments[i].curveSegment = &connection.segments[i];
    }
}

void SSTopologyConnection::UpdateReferences()
{
    conflict.connection = this;
    for (int i = 0; i < segmentsCount; i++)
    {
        segments[i].connection = this;
    }
    if (segmentsCount == 1)
    {
        segments[0].Friend = nullptr;
        segments[1].Friend = nullptr;
    }
    else
    {
        segments[0].Friend = &segments[1];
        segments[1].Friend = &segments[0];
    }
}

int SSTopologyConnection::GetMaxDiv() const
{
    int maxDivBy = 0;
    for (int i = 0; i < segmentsCount; i++)
    {
        auto& segment = segments[i];
        int divBy_summ = segment.GetSumDivBy();
        if (divBy_summ > maxDivBy) maxDivBy = divBy_summ;
    }
    return maxDivBy;
}

string SSTopologyConnection::GetDivByText() const
{
    string text = "divBy=max(";
    for (int i = 0; i < segmentsCount; i++)
    {
        auto& segment = segments[i];
        text += "[";
        int divBy_summ = 0;
        for (int il = 0; il < segment.loopSides.size(); il++)
        {
            auto& loopSide = *segment.loopSides[il];
            int divBy = loopSide.div();
            divBy_summ += divBy;
            if (il != 0) text += "+";
            text += to_string(divBy);
        }
        text += "]";
        if (i != segmentsCount - 1)text += ", ";
    }
    text += ")=" + to_string(GetMaxDiv());
    return text;
}

SSTopologyConnection_Conflict::SSTopologyConnection_Conflict()
    : status(Status::None), max_divBy(0), divBy0(0), divBy1(0), connection(nullptr), resolvesTimesCount(0)
{

}

void SSTopologyConnection_Conflict::Calculate()
{
    status = Status::NotFound;
    divBy0 = connection->segments[0].GetSumDivBy();
    divBy1 = 0;
    max_divBy = divBy0;

    if (connection->segmentsCount == 2)
    {
        divBy1 = connection->segments[1].GetSumDivBy();
        if (divBy0 != divBy1)
        {
            status = Status::Found;
            max_divBy = max(divBy0, divBy1);
        }
    }
}

void SSTopologyConnection_Conflict::ResolveConflict(int max_conflicts, int iterationNum, bool debug_iteration, ViewerDrawObjects& draw)
{
    resolvesTimesCount++;

    bool trace = options.debug_trace_iterations_in_console;
    if (trace) cout << "resolving conflict#" << iterationNum << ":  connection# " << connection->Index << "   resolvesTimesCount=" << resolvesTimesCount << endl;

    if (debug_iteration)
    {
        string text = connection->GetDivByText();
        draw.AddLabel(connection->connection.middlePoint, text, Color3d(0.4, 0.4, 0.4), 5);
    }

    bool updateLoopEnd = (resolvesTimesCount != max_conflicts); // update end of the loop only if this is first time, otherwise dont update to stop cyclic conflict
    SSTopologySegment segmentToUpdate = (divBy0 != max_divBy) ? connection->segments[0] : connection->segments[1];
    segmentToUpdate.SetDivBySumm(max_divBy, updateLoopEnd, iterationNum, debug_iteration, draw);
}


SSMesh::SSMesh(int _index, Divider& _divider, const MeshSolverNrosy* _solver, const Topology& _topology, ViewerDrawObjects& _draw, D _meshSize)
    : Index(_index), meshId(_divider.mesh.id), mesh(_divider.mesh), divider(_divider), solver(_solver), topology(_topology), draw(_draw), meshSize(_meshSize)
{
}

void SSMesh::BuildQuadLoops(vector<SSTopologyConnection>& allConnections)
{
    BuildQuadLoops__FindIntersections_OnStreams(allConnections);
    BuildQuadLoops__CreateStreamSegments_FromStreams();
    BuildQuadLoops__CreateRelations_BetweenIntersectionsAndSegments();
    BuildQuadLoops__CreateQuads_FromStreamSegments();
    BuildQuadLoops__CreateQuadLoops();
    BuildQuadLoops__SetRelationBetweenLoopsAndSegments(allConnections);
}

void SSMesh::ConvertMeshBorderSegmentsToStreams(const vector<SSTopologyConnection>& allConnections, vector<MeshStream>& streams, vector<const SSTopologySegment*>& segments)
{
    for (const auto& con : allConnections)
    {
        for (int si = 0; si < con.segmentsCount; si++)
        {
            const auto& seg = con.segments[si];
            if (seg.curveSegment->meshid == meshId)
            {
                // add
                streams.push_back(MeshStream(streams.size(), -1, -1, draw, mesh, Color3d(0, 0, 0), 0));
                segments.push_back(&seg);

                // set stream points from segment points
                MeshStream& stream = streams.back();
                seg.curveSegment->GetPoints(stream.Points);
                stream.Set_IsInited_lengthToNextPoint(true); // tell stream that 'lengthToNextPoint' and 'dirToNextPointNormalized' is already populated, and that we need update Length3d

                //DEBUG show segment Start and End points
                //int index = streams.size();
                //MeshPoint& pointStart = stream.Points[0];
                //MeshPoint& pointEnd = stream.Points.back();
                //draw.AddPoint(pointStart.point, Color3d(1, 0, 0), "Start vid=" + to_string(pointStart.vid_eid_fid)+ "  stream#" + to_string(index));
                //draw.AddPoint(pointEnd.point, Color3d(1, 0, 0), "End vid=" + to_string(pointEnd.vid_eid_fid) + "  stream#" + to_string(index));
            }
        }
    }
}

void SSMesh::Init_lengthToNextPoint()
{
    for (auto& stream : streamsWithIntersections)
    {
        stream.Init_lengthToNextPoint();
    }
}

void SSMesh::CutStreamsAtCross(vector<MeshStream>& streams, const vector<const SSTopologySegment*>& segments, int& IntersectionsCount)
{
    struct StreamPointIntersection
    {
        int intersectoinID;
        int streamIndex;
        int pointIndex;
        D pointLength3d; //  same as Length3dUntilThisPoint
        StreamPointIntersection(int _intersectoinID, int _streamIndex, int _pointIndex, D _pointLength3d)
            : intersectoinID(_intersectoinID),
            streamIndex(_streamIndex),
            pointIndex(_pointIndex),
            pointLength3d(_pointLength3d)
        {
        }

    };


    // find intersections
    vector<vector<StreamPointIntersection>> intersections;
    intersections.resize(IntersectionsCount);
    for (int is = 0; is < streams.size(); is++)
    {
        auto& s = streams[is];
        for (int ip = 0; ip < s.Points.size(); ip++)
        {
            auto& p = s.Points[ip];
            if (p.intersectionID != -1)
            {
                intersections[p.intersectionID].push_back(StreamPointIntersection(p.intersectionID, is, ip, p.Length3dUntilThisPoint));
            }
        }
    }

    // sort intersections
    for (int ii = 0; ii < IntersectionsCount; ii++)
    {
        auto& is = intersections[ii];
        std::sort(is.begin(), is.end(), [](const StreamPointIntersection &a, const StreamPointIntersection &b)
        {
            return a.pointLength3d < b.pointLength3d; // sort horizontally
        });
    }
    std::sort(intersections.begin(), intersections.end(), [](const vector<StreamPointIntersection> &a, const vector<StreamPointIntersection> &b)
    {
        return a[0].pointLength3d < b[0].pointLength3d; // sort vertically
    });


    // find cut index
    auto isSegmentStreamIndex = [&](int index)
    {
        return segments[index] != nullptr;
    };
    vector<int> cutIndexes(streams.size(), -1); // populate with -1 that means that no need to cut stream
    for (auto& intersec : intersections)
    {
        if (intersec.size() == 0) continue;
        if (isSegmentStreamIndex(intersec[0].streamIndex)) continue; // skip streams on segments
        if (cutIndexes[intersec[0].streamIndex] != -1 // if stream that is initializing the cut process is itself being cutted already
            && intersec[0].pointIndex > cutIndexes[intersec[0].streamIndex] // and cut index is bigger from the new stream size
            )
        {
            continue; // then skipp cutting, since our stream cannt cat at length bigger than a new length
        }
        for (int i = 1; i < intersec.size(); i++) // cut others streams, excluding first one
        {
            auto streamIndex = intersec[i].streamIndex;
            if (isSegmentStreamIndex(streamIndex)) continue; // skip streams on segments
            if (intersec[i].pointIndex == 0) continue; // skip start of the stream
            if (cutIndexes[streamIndex] == -1 || intersec[i].pointIndex < cutIndexes[streamIndex])
            {
                cutIndexes[streamIndex] = intersec[i].pointIndex;
            }
        }
    }


    // cut streams
    for (int i = 0; i < cutIndexes.size(); i++)
    {
        int cutIndex = cutIndexes[i];
        if (cutIndex == -1) continue;
        int newsize = cutIndex + 1;
        utils::stdvector::remove_at(streams[i].Points, newsize, streams[i].Points.size() - newsize);
    }


    // detect redundand intersection ids that now usselles because we cut the streams
    vector<int> intersectionsUsages(IntersectionsCount,0); // how many times intersectoinID found
    vector<P3> intersectionsPoints(IntersectionsCount, P3(0,0,0)); // avarage point of intersections - since for MeshpointType==Face it could be some approximation, and points can be very close to eachother but not exatlye same - here we will set them same avarage 3d value
    for (int is = 0; is < streams.size(); is++)
    {
        auto& s = streams[is];
        for (int ip = 0; ip < s.Points.size(); ip++)
        {
            auto& p = s.Points[ip];
            if (p.intersectionID == -1) continue;
            intersectionsUsages[p.intersectionID]++;
            intersectionsPoints[p.intersectionID] += p.point;
        }
    }
    for (int i = 0; i < intersectionsPoints.size(); i++)
    {
        if (intersectionsUsages[i] > 0)
        {
            intersectionsPoints[i] /= intersectionsUsages[i];
        }
    }


    // create new ids for old intersectionID
    vector<int> newIntersectionsIds;
    newIntersectionsIds.resize(IntersectionsCount);
    int newIntersectionsCount = 0;
    for (int i = 0; i < intersectionsUsages.size(); i++)
    {
        if (intersectionsUsages[i] < 2)
        {
            newIntersectionsIds[i] = -1;
        }
        else
        {
            newIntersectionsIds[i] = newIntersectionsCount;
            newIntersectionsCount++;
        }
    }


    // remove redundand points in redundand intersections
    auto isPointOnEdge = [&](const MeshPoint& p, int eid)
    {
        if (p.Type == MeshPointType::onEdge)
        {
            return (p.vid_eid_fid == eid);
        }
        if (p.Type == MeshPointType::onVertex)
        {
            return mesh.VE.exists(p.vid_eid_fid, eid);
        }
        return false;
    };
    for (int is = 0; is < streams.size(); is++)
    {
        auto& s = streams[is];
        int copyPointIndex = 0;
        int ipMax = s.Points.size() -1;
        for (int ip = 0; ip < s.Points.size(); ip++)
        {
            auto point = s.Points[ip];
            if (point.intersectionID != -1 && intersectionsUsages[point.intersectionID] < 2)
            {
                if (point.Type == MeshPointType::onFace)
                {
                    continue; // skip copy point
                }
                else  if (point.Type == MeshPointType::onEdge &&  0 <= ip - 1 && ip + 1 <= ipMax)
                {
                    if (isPointOnEdge(s.Points[ip - 1], point.vid_eid_fid) && isPointOnEdge(s.Points[ip + 1], point.vid_eid_fid))
                    {
                        continue; // skip copy point
                    }
                }
            }

            if (point.intersectionID != -1)
            {
                point.point = intersectionsPoints[point.intersectionID]; // must be before changing 'point.intersectionID'
                point.intersectionID = newIntersectionsIds[point.intersectionID];
            }
            s.Points[copyPointIndex] = point;
            copyPointIndex++;
        }
        // remove uncopied points
        if (copyPointIndex != s.Points.size())
        {
            utils::stdvector::remove_at(s.Points, copyPointIndex, s.Points.size() - copyPointIndex);
        }
        // reinit lengths
        s.Set_IsInited_lengthToNextPoint(false);
        s.Init_lengthToNextPoint();
    }

    // update IntersectionsCount
    IntersectionsCount = newIntersectionsCount;
}


void SSMesh::RemoveRedundantIntersections(vector<MeshStream>& streams, const vector<const SSTopologySegment*>& segments, int& IntersectionsCount)
{    
    // get all streams that are connected to intersection points
    CompactVectorVector<MeshStream*> intersectionsStreams;
    intersectionsStreams.resizeBegin(IntersectionsCount);
    // set capacity
    for (int is = 0; is < streams.size(); is++)
    {
        if (segments[is] != nullptr)continue;  //skip streams on curves
        const MeshStream& stream = streams[is];
        if (stream.Points.size() < 2) continue; //skip invalid streams
        for (const auto& point : stream.Points)
        {
            if (point.intersectionID == -1)  continue;
            intersectionsStreams.size(point.intersectionID)++;
        }
    }
    intersectionsStreams.resizeEnd();
    // add
    for (int is = 0; is < streams.size(); is++)
    {
        if (segments[is] != nullptr)continue;  //skip streams on curves
        MeshStream& stream = streams[is];
        if (stream.Points.size() < 2) continue; //skip invalid streams
        for (const auto& point : stream.Points)
        {
            if (point.intersectionID == -1)  continue;
            intersectionsStreams.add(point.intersectionID, &stream);
        }
    }


    // find redundant intersections (that are created near to triangle sharp point)
    vector<int> redundantIntersectoinIds;
    for (int is = 0; is < streams.size(); is++)
    {
        if (segments[is] != nullptr)continue;  //skip streams on segments
        MeshStream& stream = streams[is];
        if (stream.Points.size() < 2) continue; //skip invalid streams
        auto lastPoint = stream.Points.back();
        if (lastPoint.intersectionID == -1)  continue; // work only with streams for which last point belongs to some intersectionID

        for (int i = stream.Points.size() - 2; i > 0; i--) // iterate backward all points skiping last and first one
        {
            auto& point = stream.Points[i];
            if (point.intersectionID == -1)  continue; // check only points with intersections            
            MeshStream* testStream = &stream;
            for (int n = 0; n < intersectionsStreams.size(point.intersectionID); n++) // iterate all streams that belongs to this pre-last intersection
            {
                MeshStream* friendStream = intersectionsStreams(point.intersectionID, n); // get stream that belong to this pre-last intersection
                if (friendStream == testStream) continue;  // skip our stream
                if (intersectionsStreams.exists(lastPoint.intersectionID, friendStream) // if two streams have 2 intersection in a row
                    && point.intersectionID != friendStream->Points.back().intersectionID) // and this is not a last point in stream
                {                    
                    if (!utils::stdvector::exists(redundantIntersectoinIds, point.intersectionID)) // dont add twice
                    {
                        redundantIntersectoinIds.push_back(point.intersectionID); // then we can safely remove this intersection
                        if (options.debug_trace_iterations_in_console)
                        {
                            cout << "found redundant intersection point #" << redundantIntersectoinIds.back() << endl;
                        }
                    }
                    break; // check only if pre-last intersection point is not duplicating last intersection - so others we dont care, just break the loop
                }
            }
            break; // check only pre-last intersection on stream
        }
    }

    // remove redundant intersections
    if (redundantIntersectoinIds.size() > 0)
    {
        //utils::strings::printArray("meshId=" + to_string(meshId) + "  redundantIntersectoinIds", redundantIntersectoinIds, redundantIntersectoinIds.size());
        vector<int> new_intersectoin_ids(IntersectionsCount);
        iota(new_intersectoin_ids.begin(), new_intersectoin_ids.end(), 0);
        utils::stdvector::sort(redundantIntersectoinIds, false);
        for (int i : redundantIntersectoinIds)
        {
            utils::stdvector::remove_at(Intersections, i);
            new_intersectoin_ids[i] = -1; // remove intersection Id
            for (int i2 = i + 1; i2 < IntersectionsCount; i2++)
            {
                if (new_intersectoin_ids[i2] != -1) // if not removed
                {
                    new_intersectoin_ids[i2]--; // decrease intersectoinId
                }
            }
        }
        IntersectionsCount -= redundantIntersectoinIds.size();
        // reindex stream intersection points
        for (MeshStream& stream : streams)
        {
            if (stream.Points.size() < 2) continue; //skip invalid streams
            for (auto& point : stream.Points)
            {
                if (point.intersectionID == -1)  continue;
                point.intersectionID = new_intersectoin_ids[point.intersectionID];
            }
        }
    }
}

void SSMesh::BuildQuadLoops__FindIntersections_OnStreams(const vector<SSTopologyConnection>& allConnections)
{
    auto& streams = streamsWithIntersections;
    streams.clear();
    streams.reserve(divider.streams.streams.size() + 10);

    auto& segments = streamsWithIntersections_segments;
    segments.clear();
    segments.reserve(streams.size());

    // add divider streams 
    for (MeshStream& s : divider.streams.streams)
    {
        if (s.size() == 0) continue; // skip empty streams (that was removed when merging streams)        
        streams.push_back(s);
        segments.push_back(nullptr);
    }


    // add segments as streams
    ConvertMeshBorderSegmentsToStreams(allConnections, streams, segments); //  some elements in 'streamsWithIntersections_segments' will be populated

    // find intersections
    MeshCutter meshCutter(draw, mesh, false, false, false);
    int IntersectionsCount = meshCutter.FindStreamIntersections(streams, false, false, false, false);
    Init_lengthToNextPoint();


    // cut streams at cross
    if (options.cut_streams_at_cross)
    {
        CutStreamsAtCross(streams, segments, IntersectionsCount);
        // DEBUG - replace devider streams with cutted to view them in viewport
        if (options.cut_streams_at_cross__show)
        {
            for (int i = 0; i < divider.streams.streams.size(); i++)
            {
                auto&  divider_stream = divider.streams.streams[i];
                if (divider_stream.size() == 0) continue;
                if (i >= streams.size()) break;
                auto&  cutted_stream = streams[i];
                divider_stream.Points = cutted_stream.Points;
                divider_stream.Length3d = cutted_stream.Length3d;
                divider_stream.isCyclic = cutted_stream.isCyclic;
            }
        }
    }


    // populate 'Intersections'
    Intersections.resize(IntersectionsCount);
    for (MeshStream& stream : streams)
    {
        if (stream.Points.size() < 2) continue; //skip invalid streams
        for (const auto& point : stream.Points)
        {
            if (point.intersectionID == -1)  continue;
            Intersections[point.intersectionID] = MeshPoint(point.Type, point.vid_eid_fid, point.point);
        }
    }


    // find redundant intersections (that are created near to triangle sharp point - two or more streams are joined into one just before ending in triangle sharp point)
    RemoveRedundantIntersections(streams, segments, IntersectionsCount);
}

void SSMesh::BuildQuadLoops__CreateStreamSegments_FromStreams()
{
    auto& streams = streamsWithIntersections;

    // add segments to  'streamsSegments'
    auto& segments = streamsSegments;
    segments.clear();
    segments.reserve(Intersections.size() * 5);
    for (int is = 0; is < streams.size(); is++)
    {
        auto& stream = streams[is];
        auto seg = streamsWithIntersections_segments[is];
        if (stream.Points.size() < 2) continue; //skip invalid streams

        // if stream if formed from cyclic segment, then we have to make modification to stream to catch lost pieces - Start and End, since no intersection points will be defined in cylcic segments at Start and End
        if (stream.Points[0].intersectionID == -1)
        {
            int interFirstIndex = 0;
            int interLastIndex = 0;
            for (int i = 1; i < stream.Points.size(); i++)
            {
                const auto& point = stream.Points[i];
                if (point.intersectionID != -1)
                {
                    if (interFirstIndex == 0) interFirstIndex = i;
                    interLastIndex = i;
                }
            }
            if (interFirstIndex != 0) // if there are some intersections on the stream - move first points to the end of the stream
            {
                for (int i = 1; i <= interFirstIndex; i++) // add points [1,2,3,...] to the end of the stream (skipping first points, since first and last points are same - we dont want to have duplicate points
                {
                    stream.Add(stream.Points[i]);
                }
                stream.Remove(0, interFirstIndex); // remove points [1,2,3,...] 
                stream.Init_lengthToNextPoint();
            }
        }

        // add segments
        int from = 0;
        for (int i = 1; i < stream.Points.size(); i++)
        {
            const auto& point = stream.Points[i];
            if (point.intersectionID != -1)
            {
                if (stream.Points[from].intersectionID != -1) // skipp adding segments without intersectionID defined
                {
                    segments.push_back(StreamSegment(segments.size(), stream, from, i, streamsWithIntersections_segments[is], draw));
                }
                from = i;
            }
        }
    }



}

void SSMesh::BuildQuadLoops__CreateRelations_BetweenIntersectionsAndSegments()
{
    //
    // Create map 'map_intersectionIds_to_streamSegment'
    //
    map_intersectionIds_to_streamSegment.clear();
    for (auto& seg : streamsSegments)
    {
        int i0 = seg.intersectionIds[0];
        int i1 = seg.intersectionIds[1];
        if (i1 < i0) swap(i0, i1); // make sure first id is lower
        map_intersectionIds_to_streamSegment[{i0, i1}] = &seg;
    }

    //
    // Create CompactVectorVector 'intersectionsId_streamSegments'
    //
    intersectionsId_streamSegments.clear();

    // resize 'intersectionsId_streamSegments'
    int maxIntersectionId = -1;
    for (auto& seg : streamsSegments)
    {
        maxIntersectionId = max(maxIntersectionId, seg.intersectionIds[0]);
        maxIntersectionId = max(maxIntersectionId, seg.intersectionIds[1]);
    }
    // set capacities
    intersectionsId_streamSegments.resizeBegin(maxIntersectionId + 1);
    for (auto& seg : streamsSegments)
    {
        intersectionsId_streamSegments.size(seg.intersectionIds[0])++;
        intersectionsId_streamSegments.size(seg.intersectionIds[1])++;
    }
    intersectionsId_streamSegments.resizeEnd();

    // check for empty items
    #if DEBUG
    for (int i = 0; i < intersectionsId_streamSegments.size(); i++)
    {
        if (intersectionsId_streamSegments.capacity(i) == 0)
        {
            assert(intersectionsId_streamSegments.capacity(i) != 0 && "empty intersectionsId not allowed");
            draw.AddPoint(Intersections[i].point, Color3d(1, 0, 0), "empty intersectionsId = " + to_string(i));
        }
    }
    #endif

    //if (meshId == 83)
    //{
    //    for (int i = 0; i < Intersections.size(); i++)
    //    {
    //        draw.AddPoint(Intersections[i].point, Color3d(0, 1, 0), to_string(i));
    //    }
    //}

    // add
    for (auto& seg : streamsSegments)
    {
        for (int i = 0; i < 2; i++)
        {
            int id = seg.intersectionIds[i];
            //int index = intersectionsId_streamSegments.size(id);
            intersectionsId_streamSegments.add(id, SSIntersection(id, &seg));
        }
    }

    // sort segments for each intersection
    for (int inid = 0; inid < intersectionsId_streamSegments.size(); inid++)
    {
        MeshPoint& intersectionPoint = Intersections[inid];
        V3 intersectionNormal = mesh.NormalAtPoint(intersectionPoint);
        int imax = intersectionsId_streamSegments.size(inid);
        for (int i = 0; i < imax; i++)
        {
            SSIntersection& ss = intersectionsId_streamSegments(inid, i);
            ss.dirNormalizedOnIntersectionNormal = utils::vector::Translate(ss.dirNormalized, intersectionNormal, false);
            ss.dirNormalizedOnIntersectionNormal.normalize();
            if (i == 0)
            {
                ss.AngleFullToFirstSegment = 0;
            }
            else
            {
                SSIntersection& ss0 = intersectionsId_streamSegments(inid, 0);
                ss.AngleFullToFirstSegment = utils::vector::AngleFull(ss0.dirNormalizedOnIntersectionNormal, ss.dirNormalizedOnIntersectionNormal, intersectionNormal);
            }
        }
        for (int i = 0; i < imax; i++)
        {
            SSIntersection& ss = intersectionsId_streamSegments(inid, i);
            int iNext = i + 1;
            if (iNext == imax)
            {
                ss.AngleFullToNextSegment = 360 - ss.AngleFullToFirstSegment;
            }
            else
            {
                SSIntersection& ssNext = intersectionsId_streamSegments(inid, iNext);
                ss.AngleFullToNextSegment = ssNext.AngleFullToFirstSegment - ss.AngleFullToFirstSegment;
            }

        }
        // sort segments by angleFull
        intersectionsId_streamSegments.sort(inid, [](const SSIntersection& a, const SSIntersection& b)
        {
            return a.AngleFullToFirstSegment < b.AngleFullToFirstSegment;
        });
        //DEBUG show connected intersectoin Ids
        //{
        //    cout << "Intersection#" << inid << "   connected to ";
        //    for (int i = 0; i < intersectionsId_streamSegments.size(inid); i++)
        //    {
        //        IntersectionsId_StreamSegment& ss = intersectionsId_streamSegments(inid, i);
        //        cout << "  " << ss.IntersectionIdOpposite;
        //    }
        //    cout << "   angles  ";
        //    for (int i = 0; i < intersectionsId_streamSegments.size(inid); i++)
        //    {
        //        IntersectionsId_StreamSegment& ss = intersectionsId_streamSegments(inid, i);
        //        cout << "  " << ss.AngleFullToFirstSegment;
        //    }
        //    cout << endl;
        //}
    }

}

void SSMesh::CreateQuads_FromStreamSegments_recursive()
{
    auto& segments = streamsSegments;
    if (segments.size() < 4)  return; // skip malformed mesh

    // form relation between intersections
    CompactVectorVector<pair<D, int>> intersectionsConnections; // pair<segment length, segment id>
    int maxIntersectionId = -1;
    for (auto& seg : segments)
    {
        if (maxIntersectionId < seg.intersectionIds[0]) maxIntersectionId = seg.intersectionIds[0];
        if (maxIntersectionId < seg.intersectionIds[1]) maxIntersectionId = seg.intersectionIds[1];
    }
    intersectionsConnections.resizeBegin(maxIntersectionId + 1);
    for (auto& seg : segments)
    {
        intersectionsConnections.size(seg.intersectionIds[0])++;
        intersectionsConnections.size(seg.intersectionIds[1])++;
    }
    intersectionsConnections.resizeEnd();
    for (auto& seg : segments)
    {
        intersectionsConnections.add(seg.intersectionIds[0], { seg.Length3d, seg.intersectionIds[1] });
        intersectionsConnections.add(seg.intersectionIds[1], { seg.Length3d, seg.intersectionIds[0] });
    }

    // sort - to choise the first shortest path, this is guqrantee that we want get cyclic path that is formed around holes
    for (int i = 0; i < intersectionsConnections.size(); i++)
    {
        intersectionsConnections.sort(i, [](const pair<D, int>& a, const pair<D, int>& b)
        {
            return a.first < b.first;
        });
    }

    //cout << endl << "--- meshid    "<<mesh.meshId << endl;
    // create quads from relation between intersections
    quads.reserve(segments.size() * 4);
    int foundsCount = 0;
    for (int i0 = 0; i0 < intersectionsConnections.size(); i0++)
    {
        int size0 = intersectionsConnections.size(i0);
        assert(size0 >= 2 && "intersection must have at least 2 stream segments connected to it");
        pair<D, int>* pi1Start = intersectionsConnections.pointer(i0);
        pair<D, int>* pi1End = pi1Start + size0;
        pair<D, int>* pi1a = pi1Start;
        do
        {
            int i1a = pi1a->second;
            if (i1a > i0) // take only path where i0 < i1  (skip reduntant validation, since we do cycles for each intersectionId)
            {
                pair<D, int>* pi1b = pi1a + 1;
                do
                {
                    if (pi1b == pi1a) continue;
                    int i1b = pi1b->second;
                    if (i1b > i0)
                    {
                        bool foundQuad = false;
                        int iLastCount = intersectionsConnections.size(i1a);
                        for (int i1 = 0; i1 < iLastCount; i1++)
                        {
                            int iLast = intersectionsConnections(i1a, i1).second;
                            if (iLast > i0)  //avoid using already used intersections
                            {
                                //intersectionsConnections.exists(i1b, iLast)
                                const pair<D, int>* p = intersectionsConnections.pointer(i1b);
                                const pair<D, int>* pEnd = p + intersectionsConnections.size(i1b);
                                while (p < pEnd)
                                {
                                    if (p->second == iLast)
                                    {
                                        foundQuad = true;
                                        break;
                                    }
                                    ++p;
                                }

                                if (foundQuad)
                                {
                                    foundsCount++;
                                    int ids[4] = { i0, i1a, iLast, i1b };
                                    quads.push_back(SSQuad(quads.size(), ids, map_intersectionIds_to_streamSegment));
                                    // valide quad
                                    auto& quad = quads.back();
                                    if (!quad.isValid())
                                    {
                                        foundQuad = false;
                                        quads.pop_back();
                                        continue;
                                    }

                                    cout << "    found quad #" << foundsCount << ": " << ids[0] << "," << ids[1] << "," << ids[2] << "," << ids[3] << endl;
                                    break;
                                }
                            }
                        }
                    }
                    pi1b++;
                } while (pi1b < pi1End);
            }
            pi1a++;
        } while (pi1a < pi1End - 1);
    }
}

void SSMesh::CreateQuads_FromStreamSegments_rotation()
{
    auto& segments = streamsSegments;
    if (segments.size() < 3)  return; // skip malformed mesh
    int streamsSegmentsCount = streamsSegments.size();

    auto nextSegment = [&](SSIntersection* seg)
    {
        int inid = seg->IntersectionId;
        int search_segment_index = seg->segment->Index;
        int inid_next = seg->IntersectionIdOpposite;
        int size_next = intersectionsId_streamSegments.size(inid_next);
        SSIntersection* pStart = intersectionsId_streamSegments.pointer(inid_next);
        SSIntersection* pEnd = pStart + size_next;
        SSIntersection* p = pStart;
        SSIntersection* next = nullptr;
        do
        {
            //if (p->IntersectionIdOpposite == inid) - doesn't works for cyclic curves
            if (p->segment->Index == search_segment_index)
            {
                p++;
                if (p == pEnd) p = pStart;
                next = p;
                break;
            }
            p++;
        } while (p < pEnd);
        if (next == nullptr)
        {
            cout << "!!!wrong    DividerOptimalConnector:   not found next segment for segment: [" << seg->IntersectionId << "-->" << seg->IntersectionIdOpposite << "]" << endl;
            assert(next != nullptr && "not found next segment");
        }
        return next;
    };

    quads.clear();
    quads.reserve(segments.size() * 4);
    int foundsCount = 0;
    for (int i0 = 0; i0 < intersectionsId_streamSegments.size(); i0++)
    {
        int size0 = intersectionsId_streamSegments.size(i0);
        assert(size0 >= 2 && "intersection must have at least 2 stream segments connected to it");
        if (size0 == 0) continue;
        SSIntersection* pi0Start = intersectionsId_streamSegments.pointer(i0);
        SSIntersection* pi0End = pi0Start + size0;
        SSIntersection* pi0 = pi0Start;
        do
        {
            // get 4 sides
            SSIntersection* s4[4] = { pi0, nullptr , nullptr , nullptr };
            D angleSumm = pi0->AngleFullToNextSegment;
            for (int i = 1; i < 4; i++)
            {
                int inid_next = s4[i - 1]->IntersectionIdOpposite;
                if (inid_next <= i0) break; // take only path where i0 < i1  (skip reduntant validation, since we do cycles for each intersectionId)
                s4[i] = nextSegment(s4[i - 1]);
                if (s4[i] == nullptr) break;
                angleSumm += s4[i]->AngleFullToNextSegment;
            }

            // form quad from sides if they valid
            if ((s4[3] != nullptr  // found 4 sides
                && s4[3]->IntersectionIdOpposite == s4[0]->IntersectionId // first and last points mutch
                && (streamsSegmentsCount != 4 || angleSumm < 4 * 150) // for quad - avoid adding opposite path - valid quad should have 360 degree in total - using this check we avoid adding 2 times same quad using backward path
                )
                ||
                (s4[3] == nullptr && s4[2] != nullptr  // found 3 sides
                    && s4[2]->IntersectionIdOpposite == s4[0]->IntersectionId // first and last points mutch
                    && (streamsSegmentsCount != 3 || angleSumm < 3 * 150) // for quad - avoid adding opposite path - valid quad should have 360 degree in total - using this check we avoid adding 2 times same quad using backward path
                    )
                )
            {
                foundsCount++;
                quads.push_back(SSQuad(quads.size(), s4));
                // valide quad
                auto& quad = quads.back();
                if (!quad.isValid())
                {
                    quads.pop_back();
                }
                else
                {
                    //cout << "    found quad #" << foundsCount << ": " << s4[0]->IntersectionId << "," << s4[1]->IntersectionId << "," << s4[2]->IntersectionId << "," << s4[3]->IntersectionId << endl;
                }
            }

            pi0++;
        } while (pi0 < pi0End);
    }
}

void SSMesh::BuildQuadLoops__CreateQuads_FromStreamSegments()
{
    //CreateQuads_FromStreamSegments_recursive();
    CreateQuads_FromStreamSegments_rotation();
}

void SSMesh::BuildQuadLoops__CreateQuadLoops()
{
    quadLoops.clear();

    //
    // create 'map_segment_quads'
    //
    struct QuadSide
    {
        const SSQuad* quad;
        int segmentIndex;
    };
    CompactVectorVector<QuadSide> map_segment_quads;
    map_segment_quads.resizeBegin(streamsSegments.size());
    for (const auto& quad : quads)
    {
        for (int i = 0; i < 4; i++)
        {
            if (quad.isTriangle && i == 3) break;// 4-th side not exists for triangle
            auto& segment = quad.segments[i];
            map_segment_quads.size(segment->Index)++;
        }
    }
    map_segment_quads.resizeEnd();
    for (const auto& quad : quads)
    {
        for (int i = 0; i < 4; i++)
        {
            if (quad.isTriangle && i == 3) break;// 4-th side not exists for triangle
            auto& segment = quad.segments[i];
            map_segment_quads.add(segment->Index, { &quad, i });
        }
    }

    //
    // create 'quadLoops'
    //
    auto quadEdgeEndsLoop = [](const SSQuad& quad, int segmentIndex)
    {
        return (quad.isTriangle && segmentIndex == 1)  // if this edge doent have opposite edge
            || quad.segments[segmentIndex]->curveSegment != nullptr; // if quad segment is on curve
    };

    vector<pair<int, int>> takenQuadSegments;
    takenQuadSegments.reserve(quads.size());
    for (const auto& quad : quads)
    {
        for (int i = 0; i < 4; i++)
        {
            if (quad.isTriangle && i == 3) break;// 4-th side not exists for triangles

            if (quadEdgeEndsLoop(quad, i)// if this streamSegment is on curveSegment
                && !utils::stdvector::exists(takenQuadSegments, { quad.Index, i }) // side wasnt used in loop creation (protection from creating duplicate loop)
                )
            {
                //  create loop
                quadLoops.push_back(SSQuadsLoop(meshId));
                auto& loop = quadLoops.back();
                // add first quad
                QuadSide first = { &quad, i };
                loop.quads.push_back(SSQuad_InLoop(first.quad, first.segmentIndex)); // add first quad
                // search in all connected quads
                QuadSide last = first;
                bool foundEndOfTheLoop = false;
                do
                {
                    int oppositeSegmentIndex = (quad.isTriangle && i == 1)
                        ? last.segmentIndex
                        : (last.segmentIndex + 2) % 4;

                    // if opposite segment is on curve - then we found end of the loop
                    if (last.quad->segments[oppositeSegmentIndex]->curveSegment != nullptr)
                    {
                        foundEndOfTheLoop = true;
                        break;
                    }

                    // find next quad
                    int nextSegmentIndex = last.quad->segments[oppositeSegmentIndex]->Index;
                    QuadSide* nextSide = nullptr;
                    for (int qi = 0; qi < map_segment_quads.size(nextSegmentIndex); qi++)
                    {
                        if (map_segment_quads(nextSegmentIndex, qi).quad->Index != last.quad->Index)
                        {
                            nextSide = &map_segment_quads(nextSegmentIndex, qi);
                            break;
                        }
                    }

                    // validate if we found next quad
                    if (nextSide == nullptr)
                    {
                        //assert(nextSide != nullptr && "nextSide not found");
                        //cout << "!!! warning   SSMesh::BuildQuadLoops__CreateQuadLoops:   nextSide not found   " << mesh.GetMeshIdStr() << endl;
                        break;
                    }

                    // add next quad
                    loop.quads.push_back(SSQuad_InLoop(nextSide->quad, nextSide->segmentIndex)); // add next quad
                    last = *nextSide;

                    // break when we reach triangle with edge#1 (triagle quad doesnt have opposite side for edge#1, so this is end of the loop)
                    if (last.quad->isTriangle && last.segmentIndex == 1)
                    {
                        foundEndOfTheLoop = true;
                        break;
                    }
                } while (!foundEndOfTheLoop);

                if (foundEndOfTheLoop)
                {
                    int oppositeSegmentIndex = (last.segmentIndex + 2) % 4; // add opposite index
                    if (last.quad->isTriangle && last.segmentIndex == 1) // triangle quad doesnt have opposite side for edge#1
                    {
                        oppositeSegmentIndex = last.segmentIndex; // add same index, since triagle quad doesnt have opposite side for edge#1
                    }
                    takenQuadSegments.push_back({ last.quad->Index,  oppositeSegmentIndex });
                }
                else
                {
                    quadLoops.pop_back(); // remove loop if we failed to find end of the loop
                }

            }
        }
    }

    for (auto& loop : quadLoops)
    {
        loop.UpdateAfterAdd();
    }
}

void SSMesh::BuildQuadLoops__SetRelationBetweenLoopsAndSegments(vector<SSTopologyConnection>& allConnections)
{
    // populate 'connections',  'connection.segmens[].loopSides',  'connection.segmens[].mesh'
    meshConnections.clear();
    for (auto& loop : quadLoops)
    {
        for (int sideIndex = 0; sideIndex < 2; sideIndex++)
        {
            if (sideIndex == 1 && loop.segment01[0]->Index == loop.segment01[1]->Index) continue; // ignore same segment for triangle quads - only 1 side can be registered, because segment0==segment1 for triangle quad
            const MeshTopologyLoopCurveSegment* segment = loop.segment01[sideIndex]->curveSegment;
            if (segment == nullptr) continue; //triangle quads ends the loop without need to be on curve - edge#1 end the loop anyway
            int conIndex = segment->topologyConnection->Index;
            SSTopologyConnection& con = allConnections[conIndex];
            int segmentIndex01 = con.connection.findSegmentIndex(*segment);
            assert(segmentIndex01 != -1);
            if (segmentIndex01 == -1) continue; // unexpected error protection
            con.segments[segmentIndex01].loopSides.push_back(&loop.sides[sideIndex]);
            con.segments[segmentIndex01].mesh = this;
            if (!utils::stdvector::exists(meshConnections, &con))
            {
                meshConnections.push_back(&con);
            }
        }
    }


    // sort vector 'connection.segmens[].loopSides'
    for (int i = 0; i < meshConnections.size(); i++)
    {
        SSTopologyConnection* connection = meshConnections[i];
        for (int k = 0; k < connection->segmentsCount; k++)
        {
            SSTopologySegment& segment = connection->segments[k];
            if (segment.mesh != this) continue; // proceed only segments from this mesh
            if (segment.loopSides.size() == 0) continue;
            // sort in order to start them from begining of segment
            sort(segment.loopSides.begin(), segment.loopSides.end(), [](const SSQuadsLoop_Side* a, const SSQuadsLoop_Side* b)
            {
                int a_fromIndex = a->segment()->fromPointIndex;
                int b_fromIndex = b->segment()->fromPointIndex;
                return a_fromIndex < b_fromIndex;
            });
        }
    }
}

bool SSMesh::addAnchorAndStartPoints(D newPos, const MeshTopologyLoopCurveSegment* segment, DividerIteratorStreamInfo dividingIteration)
{
    return DividerIterator::addAnchorAndStartPoints(newPos, true, segment, dividingIteration, mesh, *solver, divider.startPoints, divider.anchorPoints);
}


void SSMesh::JoinStreamsWithDividingPoints()
{
    for (auto connection : meshConnections) // proceed all connection realted to this mesh
    {
        for (int k = 0; k < connection->segmentsCount; k++)
        {
            SSTopologySegment& segment = connection->segments[k];
            if (segment.mesh != this) continue; // proceed only segments from this mesh
            if (segment.loopSides.size() == 0) continue;

            auto points = GetDividingPointsForJoins(segment, false, true);
            for (int i = 0; i < points.size(); i++)
            {
                D newPos = points[i].pos;
                const StreamSegment* segmentOrtogonal = points[i].segmentOrtogonal;
                if (segmentOrtogonal == nullptr)
                {
                    cout << "!!! GetDividingPointsForJoins(segment, false, true) must return only points for segments" << endl;
                    assert(segmentOrtogonal != nullptr &&  "GetDividingPointsForJoins(segment, false, true) must return only points for segments");
                    continue;
                }

                int dividingIteration = 1;
                if (addAnchorAndStartPoints(newPos, segment.curveSegment, dividingIteration))
                {
                    auto& new_startStreamPoint = divider.startPoints.back();// get reference to currently added startStreamPoint
                    auto& new_anchorPoint = divider.anchorPoints.back(); // get reference to currently added anchorPoint
                    StreamStartPoint* connectingTo_startStreamPoint = divider.FindStartPoint(segmentOrtogonal->stream.StreamStartPointId);
                    StreamAnchorPoint* connectingTo_anchorPoint = divider.FindAnchorPoint(segmentOrtogonal->stream.StreamAnchorPointId);

                    // validate 
                    if (connectingTo_startStreamPoint == nullptr || connectingTo_anchorPoint == nullptr)  // should be never happend
                    {
                        divider.startPoints.pop_back(); // if we fail to establish connection - remove startStreamPoint and anchorPoint to avoid redundant connections
                        divider.anchorPoints.pop_back(); // if we fail to establish connection - remove startStreamPoint and anchorPoint to avoid redundant connections
                        assert(connectingTo_startStreamPoint != nullptr && connectingTo_anchorPoint != nullptr);
                        continue;
                    }

                    // set contract to force specific streams joing together
                    connectingTo_startStreamPoint->SignContract(new_anchorPoint);
                    new_startStreamPoint.SignContract(*connectingTo_anchorPoint);

                    // DEBUG trace added points
                    if (options.debug_trace_iterations_in_console)
                    {
                        cout << "JoinStreamsWithDividingPoints  added:   startPoint#" << new_startStreamPoint.id << "  {" << new_startStreamPoint.contract.first << "," << new_startStreamPoint.contract.second << "}     anchor#" << new_anchorPoint.id << "  {" << new_anchorPoint.contract.first << "," << new_anchorPoint.contract.second << "}   stream#" << segmentOrtogonal->stream.Index << "  connectingTo_startStreamPoint#" << connectingTo_startStreamPoint->id << "   connectingTo_anchorPoint#" << connectingTo_anchorPoint->id << endl;
                    }

                }
            }
        }
    }


}


vector<SSTopologySegment_DividingPoint> SSMesh::GetDividingPointsForJoins(const SSTopologySegment& segment, bool addNormalDividingPoints, bool addStreamDividingPoints)
{
    D Length3d = segment.curveSegment->Length3d;
    int divBySumm = 0;
    for (auto s : segment.loopSides) divBySumm += s->div();

    vector<SSTopologySegment_DividingPoint> res;
    res.reserve(divBySumm + 1);

    if (addNormalDividingPoints)
    {
        for (int k = 0; k <= divBySumm; k++)
        {
            res.push_back(SSTopologySegment_DividingPoint(segment, Length3d * k / divBySumm, nullptr));
        }
    }

    if (addStreamDividingPoints)
    {
        int current_divBySumm = 0;
        for (auto side : segment.loopSides)
        {
            current_divBySumm += side->div();
            if (current_divBySumm == divBySumm) continue; // dont add last point, because it is not a stream-point but curve-point

            D newPos = Length3d / divBySumm * current_divBySumm;
            const StreamSegment* segmentHorizontal = side->segment();
            int inid = segmentHorizontal->intersectionIds[1];
            auto& quad = (side->sideIndex == 0) ? side->loop->quads.front() : side->loop->quads.back();
            int index0 = (side->sideIndex == 0) ? quad.index0 : quad.index1;
            // try each side 
            int ortogonalIndex = -1;
            for (int i = 1; i < 4; i++)
            {
                int index = (index0 + i) % 4;
                if (quad.quad->isTriangle && index == 3) continue; // triangle doesnt have 4-th side
                if (quad.quad->segments[index]->intersectionIds[0] == inid || quad.quad->segments[index]->intersectionIds[1] == inid)
                {
                    ortogonalIndex = index;
                    break;
                }
            }
            if (ortogonalIndex == -1)
            {
                cout << "!!! wrong   SSMesh::GetDividingPointsForJoins    ortogonalIndex == -1" << endl;
                assert(ortogonalIndex != -1 && "!!! wrong   SSMesh::GetDividingPointsForJoins    ortogonalIndex == -1");
                continue;
            }

            const StreamSegment* segmentOrtogonal = quad.quad->segments[ortogonalIndex];
            if (segmentOrtogonal->curveSegment != nullptr) continue; // if ortogonal streamSegment is on topologySegment then ignore it - can happend on triangle surfaces

            if (addNormalDividingPoints)
            {
                res[current_divBySumm].pos = newPos;                                           // replace normal point with stream point
                res[current_divBySumm].segmentOrtogonal = segmentOrtogonal;       // replace normal point with stream point
            }
            else
            {
                res.push_back(SSTopologySegment_DividingPoint(segment, newPos, segmentOrtogonal));
            }
        }
    }

    return res;
}


SSMeshes::SSMeshes(const Topology& _topology, ViewerDrawObjects& _draw, vector<Divider>& _dividers, vector<const MeshSolverNrosy*> _solvers, D _meshSize)
    : topology(_topology), draw(_draw), meshSize(_meshSize)
{
    // convert divider into wrapper
    meshes.clear();
    meshes.reserve(_dividers.size());
    for (int i = 0; i < _dividers.size(); i++)
    {
        meshes.push_back(SSMesh(i, _dividers[i], _solvers[i], topology, draw, meshSize)); // lets use global mesher draw instead of _dividers[i].draw
    }

    // create map <meshid, ssmesh>
    for (int i = 0; i < meshes.size(); i++)
    {
        map_meshid_ssmesh[meshes[i].meshId] = &meshes[i];
    }
}

void SSMeshes::Build()
{
    extern bool IsOmpEnabled;

    //
    // Create wrapper around 'Topology.Connections'
    //
    allConnections.clear();
    allConnections.reserve(topology.Connections.size()); //we must reserve space to avoid memory realocation and outdated references
    for (auto& connection : topology.Connections)
    {
        allConnections.push_back(SSTopologyConnection(allConnections.size(), connection));
    }
    for (auto& connection : allConnections)
    {
        connection.UpdateReferences();
    }

    //
    // Proceed every mesh
    //
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        meshes[i].BuildQuadLoops(allConnections);
    }
}

void SSMeshes::DrawDebug()
{
    //
   // DEBUG - show stream intersections
   //
    if (options.debug_show_streams_intersections)
    {
        int addcount = 0;
        for (auto& m : meshes) addcount += m.Intersections.size();
        draw.ReservePoints(addcount);
        draw.ReserveLabels(addcount);
        for (auto& mesh : meshes)
        {
            for (int inid = 0; inid < mesh.Intersections.size(); inid++)
            {
                mesh.draw.AddPoint(mesh.Intersections[inid].point, Color3d(1, 0, 0), "I#" + to_string(inid));
            }
        }
    }

    //
    // DEBUG - show stream segments
    //
    if (options.debug_show_streams_segments)
    {
        int addcount = 0;
        for (auto& m : meshes) addcount += m.streamsSegments.size();
        draw.ReserveLabels(addcount);
        for (auto& mesh : meshes)
        {
            for (auto& s : mesh.streamsSegments)
            {
                string sIndex = "ss#" + to_string(s.Index);
                string sLength = "   length = " + to_string(s.Length3d / 2);
                string fromToMax = "   from, to, max = " + to_string(s.fromPointIndex) + ", " + to_string(s.toPointIndex) + ", " + to_string(s.stream.Points.size() - 1);
                string sSegment = (s.curveSegment == nullptr ? "" : ("   seg" + to_string(s.curveSegment->id)));
                string sIntersections = "   [I#" + to_string(s.intersectionIds[0]) + ",I#" + to_string(s.intersectionIds[1]) + "]";
                //draw.AddLabel(s.PointMid(), sIndex + sLength + fromToMax);
                if (s.curveSegment != nullptr)
                {
                    draw.AddEdge(s.PointMid(false), s.PointMid(true), Color3d(0.6, 0.6, 0.6));
                }
                draw.AddLabel(s.PointMid(true), sIndex + sIntersections + sSegment);
            }
        }
    }

    //
   // DEBUG - show quads
   //
    if (options.debug_show_quads)
    {
        int addcount = 0;
        for (auto& m : meshes) addcount += m.quads.size();
        draw.ReserveLabels(addcount);
        for (auto& mesh : meshes)
        {
            for (auto& q : mesh.quads)
            {
                string sIndex = "Q#" + to_string(q.Index);
                string sIntersections = "  [I#" + to_string(q.intersectionIds[0]) + ",I#" + to_string(q.intersectionIds[1]) + ",I#" + to_string(q.intersectionIds[2]) + ",I#" + to_string(q.intersectionIds[3]) + "]";
                draw.AddLabel(q.PointMid(), sIndex + sIntersections);
            }
        }
    }

    //
    // DEBUG - show loops
    //
    if (options.debug_show_loops)
    {
        int addcount = 0;
        for (auto& m : meshes) addcount += m.quadLoops.size();
        draw.ReserveEdges(addcount * 5);
        draw.ReserveLabels(addcount * 2);
        for (auto& mesh : meshes)
        {
            for (auto& loop : mesh.quadLoops)
            {
                auto color = (loop.div01[0] != loop.div01[1]) ? Color3d(1, 0, 0) : Color3d(0, 0, 1);
                for (auto& quad : loop.quads)
                {
                    draw.AddEdge(quad.segment0->PointMid(true), quad.segment1->PointMid(true), color);
                }
                draw.AddLabel(loop.segment01[0]->PointMid(true), to_string(loop.div01[0]), color);
                draw.AddLabel(loop.segment01[1]->PointMid(true), to_string(loop.div01[1]), color);
            }
        }
    }


    //
    // DEBUG - show connections
    //
    if (options.debug_show_connections)
    {
        auto color = Color3d(0.4, 0.4, 0.4);
        int addcount = allConnections.size();
        draw.ReserveLabels(addcount * 2);
        for (auto& connection : allConnections)
        {
            string text = connection.GetDivByText();
            draw.AddLabel(connection.connection.middlePoint, text, color, 5);
        }
    }
}
