#include "stdafx.h"
#include "Mesh.h"
#include "MeshTopologyLoop.h"
#include "MeshStreams.h"

MeshTopologyLoopPoint::MeshTopologyLoopPoint(const Mesh& _mesh, const MeshLoop& _loop, const MeshLoopPoint& _loopPoint)
    :mesh(_mesh), loop(_loop), point(mesh.V.row(_loopPoint.VertexId)), loopPoint(_loopPoint), CurvePrev(nullptr), CurveNext(nullptr)
{

}


II MeshTopologyLoopPoint::SizeOF() const
{
    return sizeof(MeshTopologyLoopPoint);
}


atomic_int MeshTopologyLoopCurve::nextTopologyLoopCurveID;


MeshTopologyLoopCurve::MeshTopologyLoopCurve(const Mesh& _mesh, const MeshLoop& _loop, MeshTopologyLoopPoint& _pointStart, MeshTopologyLoopPoint& _pointEnd)
    : id(nextTopologyLoopCurveID++), mesh(_mesh), loop(_loop), pointStart(_pointStart), pointEnd(_pointEnd)
{
    _pointStart.CurveNext = this;
    _pointEnd.CurvePrev = this;

    // detect count
    int indexStart = pointStart.loopPoint.Index;
    int indexEnd = pointEnd.loopPoint.Index;
    isCyclic = (indexStart == indexEnd);

    PointsCount = indexStart < indexEnd
        ? indexEnd - indexStart + 1
        : (loop.Length - indexStart) + (indexEnd + 1);
    EdgesCount = PointsCount - 1;
    assert(PointsCount >= 2 && "Curve must have at least 2 points");
    assert(EdgesCount >= 1 && "Curve must have at least 1 edge");


    // fill arrays
    Length3d = 0;
    points.resize(PointsCount, 3);
    loopPointIndexes.resize(PointsCount);
    int index = pointStart.loopPoint.Index;
    for (int i = 0; i < PointsCount; i++)
    {
        //set indexes
        loopPointIndexes(i) = index;
        // set points
        int vid = loop.points[index].VertexId;
        //points.row(i) = mesh.V.row(vid);
        points.row(i) = mesh.V.row(vid);

        // set length
        if (i < EdgesCount) // excluding last points, since we have edgesCount=pointsCount-1
        {
            Length3d += loop.edges[index].Length;
        }

        // move to next point
        int indexNext = (index == loop.Length - 1) // if index is end of the loop
            ? 0 // move to first index
            : index + 1; // continue to increment
        index = indexNext;
    }
    AvgEdgeLength3d = Length3d / EdgesCount;

    middlePoint = GetPointAtLengthPercent(0.5);
    middlePoint_DistToZeroPow2 = utils::point::DistToPointPow2(middlePoint, P3(0, 0, 0));
}

MeshPoint MeshTopologyLoopCurve::GetMeshPointAtLength(D atLength3d, int fromIndex, int count, bool allowPointOnVertex) const
{
    D tol = meshLogicOptions.MeshStream.snap_to_vertex_tol;
    assert(count <= EdgesCount);
    D summLength = 0;
    int start_vid = -1;
    int end_vid = -1;
    if (count > 0)
    {
        start_vid = loop.points[loopPointIndexes(CorrectCyclicIndex(fromIndex + 0))].VertexId;
        end_vid = loop.points[loopPointIndexes(CorrectCyclicIndex(fromIndex + (count-1) + 1))].VertexId;
    }
    for (int iCyclic = 0; iCyclic < count; iCyclic++)
    {
        int i = CorrectCyclicIndex(fromIndex + iCyclic);
        int loopIndex = loopPointIndexes(i);
        D currentEdgeLength = loop.edges[loopIndex].Length;
        if (summLength <= atLength3d && atLength3d <= summLength + currentEdgeLength)
        {
            D shiftPercent = (atLength3d - summLength) / currentEdgeLength;
            const D allowPointOnVertex_maxShiftPercent = tol;//good is 0.005f;
            if (allowPointOnVertex && (shiftPercent < allowPointOnVertex_maxShiftPercent || shiftPercent >1 - allowPointOnVertex_maxShiftPercent))
            {
                if (shiftPercent < allowPointOnVertex_maxShiftPercent)
                {
                    int vid0 = loop.points[loopIndex].VertexId;
                    if (vid0 != start_vid && vid0 != end_vid)
                    {
                        P3 v0 = loop.mesh.V.row(vid0);
                        return MeshPoint(MeshPointType::onVertex, vid0, v0);
                    }
                }
                if (shiftPercent > 1 - allowPointOnVertex_maxShiftPercent)
                {
                    int i1 = CorrectCyclicIndex(fromIndex + iCyclic + 1);
                    int loopIndex1 = loopPointIndexes(i1);
                    int vid1 = loop.points[loopIndex1].VertexId;
                    if (vid1 != start_vid && vid1 != end_vid)
                    {
                        P3 v1 = loop.mesh.V.row(vid1);
                        return MeshPoint(MeshPointType::onVertex, vid1, v1);
                    }
                }
            }
            int vid = loop.points[loopIndex].VertexId;
            P3 point = mesh.V.row(vid);
            V3 edgeDir = loop.edges[loopIndex].directionAlongEdge;
            return MeshPoint(MeshPointType::onEdge, loop.edges[loopIndex].EdgeId, point + edgeDir * shiftPercent);
        }

        // set length
        summLength += currentEdgeLength;
    }

    // if we reach this line, then most probably something is wrong
    assert(false && "check GetPointAtLength if it works fine"); // it shouldnt go here, so we have to check

    // shouldnt enter here, but just provide some meaningfull value for emergency situations
    P3 pointStart = points.row(fromIndex);
    P3 pointEnd = points.row(CorrectCyclicIndex(fromIndex + count - 1));

    return abs(atLength3d - 0) < abs(atLength3d - summLength)
        ? MeshPoint(MeshPointType::onVertex, loop.points[loopPointIndexes(CorrectCyclicIndex(fromIndex + 0))].VertexId, pointStart)
        : MeshPoint(MeshPointType::onVertex, loop.edges[loopPointIndexes(CorrectCyclicIndex(fromIndex + count - 1))].EndVertexId, pointEnd);
}

MeshPoint MeshTopologyLoopCurve::GetMeshPointAtLength(D atLength3d) const
{
    return GetMeshPointAtLength(atLength3d, 0, EdgesCount, false);
}

P3 MeshTopologyLoopCurve::GetPointAtLength(D atLength3d, int fromIndex, int count) const
{
    return GetMeshPointAtLength(atLength3d, fromIndex, count, false).point;
}

P3 MeshTopologyLoopCurve::GetPointAtLength(D atLength3d) const
{
    return GetPointAtLength(atLength3d, 0, EdgesCount);
}


P3 MeshTopologyLoopCurve::GetPointAtLengthPercent(D percent) const
{
    assert(percent >= 0 && percent <= 1);
    return GetPointAtLength(Length3d * percent);
}

P3 MeshTopologyLoopCurve::GetPointAtLengthPercent(D percent, int fromIndex, int count) const
{
    assert(percent >= 0 && percent <= 1);
    return GetPointAtLength(Length3d * percent, fromIndex, count);
}

D MeshTopologyLoopCurve::GetAtLengthFromPoint(const MeshPoint& point, int fromIndex, int count) const
{
    assert(count <= EdgesCount);
    assert(point.Type == MeshPointType::onVertex || point.Type == MeshPointType::onEdge);
    D summLength = 0;
    for (int iCyclic = 0; iCyclic < count; iCyclic++)
    {
        int i = CorrectCyclicIndex(fromIndex + iCyclic);
        int loopIndex = loopPointIndexes(i);
        const auto& edge = loop.edges[loopIndex];
        const auto& vertex = loop.points[loopIndex];

        switch (point.Type)
        {
            case MeshPointType::onVertex:
                if (vertex.VertexId == point.vid_eid_fid)
                {
                    return summLength;
                }
                break;
            case MeshPointType::onEdge:
                if (edge.EdgeId == point.vid_eid_fid)
                {
                    P3 vertexPoint = points.row(i);
                    D dist = utils::point::DistToPoint(vertexPoint, point.point);
                    assert(dist < edge.Length);
                    return summLength + dist;
                }
                break;
            default:
                return 0;
        }

        summLength += edge.Length;
    }

    // if it reach this code - maybe it is last point - lets check it
    if (point.Type == MeshPointType::onVertex)
    {
        int loopIndex = loopPointIndexes(CorrectCyclicIndex(fromIndex + count - 1));
        const auto& edge = loop.edges[loopIndex];
        if (edge.StartVertexId == point.vid_eid_fid || edge.EndVertexId == point.vid_eid_fid)
        {
            return summLength;
        }
    }

    // if we reach this line, then most probably something is wrong
    assert(false && "check GetAtLengthFromPoint if it works fine"); // it should'nt go here, so we have to check

    // shouldnt enter here, but just provide some meaningfull value for emergency situations
    P3 pointStart = points.row(fromIndex);
    P3 pointEnd = points.row(CorrectCyclicIndex(fromIndex + count - 1));
    return utils::point::DistToPointPow2(pointStart, point.point) < utils::point::DistToPointPow2(pointEnd, point.point)
        ? 0
        : summLength;
}

void MeshTopologyLoopCurve::GetPointsDividedByCount(int fromIndex, int count, int divisioncount, P3s& points, bool isPointsStoredInRevertedOrder, D segmentLength3d) const
{
    divisioncount = max(divisioncount, 1); // at least 1 must be
    points.resize(divisioncount + 1, 3);
    P3 pointStart = this->points.row(CorrectCyclicIndex(fromIndex + 0));
    P3 pointEnd = this->points.row(CorrectCyclicIndex(fromIndex + count));
    points.row(0) = !isPointsStoredInRevertedOrder ? pointStart : pointEnd;
    points.row(divisioncount) = !isPointsStoredInRevertedOrder ? pointEnd : pointStart;
    if (divisioncount == 1) return;

    int pointIndex = 1;
    if (segmentLength3d < 0)
    {
        segmentLength3d = 0;
        for (int iCyclic = 0; iCyclic < count; iCyclic++)
        {
            int i = CorrectCyclicIndex(fromIndex + iCyclic);
            int loopIndex = loopPointIndexes(i);
            D currentEdgeLength = loop.edges[loopIndex].Length;
            segmentLength3d += currentEdgeLength;
        }
    }
    D atLength3d = segmentLength3d / divisioncount;

    D summLength = 0;
    for (int iCyclic = 0; iCyclic < count; iCyclic++)
    {
        int i = CorrectCyclicIndex(fromIndex + iCyclic);
        int loopIndex = loopPointIndexes(i);
        D currentEdgeLength = loop.edges[loopIndex].Length;
        while (summLength <= atLength3d && atLength3d <= summLength + currentEdgeLength + currentEdgeLength * 0.0000001)  //plus add D error correction
        {
            D shiftPercent = (atLength3d - summLength) / currentEdgeLength;
            //int vid = loop.points[loopIndex].VertexId;
            //P3 point = mesh.V.row(vid);
            P3 point = this->points.row(CorrectCyclicIndex(i));
            V3 edgeDir = loop.edges[loopIndex].directionAlongEdge;
            P3 pointAtLength = point + edgeDir * shiftPercent;
            int pointIndexReversed = divisioncount - pointIndex;
            points.row(!isPointsStoredInRevertedOrder ? pointIndex : pointIndexReversed) = pointAtLength;
            pointIndex++;
            atLength3d = pointIndex * segmentLength3d / divisioncount;
            if (pointIndex == divisioncount) return;
        }
        summLength += currentEdgeLength;
    }
    if (pointIndex <= divisioncount)
    {
        cout << "!!! not all points are populated in  MeshTopologyLoopCurve::GetPointsDividedByCount" << endl;
        assert(pointIndex > divisioncount && "not all points are populated in  MeshTopologyLoopCurve::GetPointsDividedByCount");
    }
}

II MeshTopologyLoopCurve::SizeOF() const
{
    return sizeof(MeshTopologyLoopCurve);
}

void MeshTopologyLoopCurveSegment::Init(int _fromIndex, int _toIndex)
{
    PrevSegmentInLoop = nullptr;
    NextSegmentInLoop = nullptr;
    ConnectedTo = nullptr;
    topologyPointStart = nullptr;
    topologyPointEnd = nullptr;
    topologyConnection = nullptr;

    fromIndex = _fromIndex;
    toIndex = _toIndex;

    PointsCount = fromIndex < toIndex
        ? toIndex - fromIndex + 1
        : (curve.PointsCount - fromIndex) + (toIndex);
    EdgesCount = PointsCount - 1;

    isCyclic = curve.isCyclic && PointsCount == curve.PointsCount;

    assert(PointsCount >= 2); // minimum 2 points must be in segment (in this case segment will be exactly 1 edge)
    if (fromIndex > toIndex)
    {
        assert(curve.isCyclic && "cyclic indexing is valid only for cyclic curves");
    }

    // use curve Length3d if segment is completely cover curve
    if (PointsCount == curve.PointsCount)
    {
        Length3d = curve.Length3d;
    }
    else
    {
        Length3d = 0;
        for (int i = 0; i < EdgesCount; i++)
        {
            Length3d += EdgeLength(i);
        }
    }
    AvgEdgeLength3d = Length3d / EdgesCount;

    pointStart = Point(0);
    pointEnd = Point(PointsCount - 1);
    pointStart_vertexId = VertexId(0);
    pointEnd_vertexId = VertexId(PointsCount - 1);

    // init parameters for Topology search algorithm - these properties will be used to make search faster
    P3 pointStart2 = Point(1);// second point from start
    P3 pointEnd2 = Point(PointsCount - 1 - 1); // pre-last from from end
    positionInPseudo3dStart2 = pointStart2(0) + pointStart2(1) + pointStart2(2);
    positionInPseudo3dEnd2 = pointEnd2(0) + pointEnd2(1) + pointEnd2(2);
    if (PointsCount & 1) // if there are uneven count of point alike 3,5,..,11,13
    {
        P3 pointmid = Point((PointsCount - 1) / 2);
        positionInPseudo3dMid = pointmid(0) + pointmid(1) + pointmid(2);
    }
    else // if there are even count of points alike 2,4,..,10,12
    {
        P3 pointmid1 = Point(PointsCount / 2 - 1);
        P3 pointmid2 = Point(PointsCount / 2);
        P3 pointmid = (pointmid1 + pointmid2) / 2;
        positionInPseudo3dMid = pointmid(0) + pointmid(1) + pointmid(2);
    }
    edgeLengthStart = EdgeLength(0); // first edge length 
    edgeLengthEnd = EdgeLength(EdgesCount - 1); // last edge length
}


atomic_int MeshTopologyLoopCurveSegment::nextTopologyLoopCurveSegmentID;

MeshTopologyLoopCurveSegment::MeshTopologyLoopCurveSegment(const MeshTopologyLoopCurve& _curve)
    : MeshTopologyLoopCurveSegment(_curve, 0, _curve.PointsCount - 1)
{
}

MeshTopologyLoopCurveSegment::MeshTopologyLoopCurveSegment(const MeshTopologyLoopCurve& _curve, int _fromIndex, int _toIndex)
    : id(nextTopologyLoopCurveSegmentID++),
    curve(_curve), meshid(_curve.mesh.id),
    fromIndex(0), toIndex(0), isCyclic(false), PointsCount(0), EdgesCount(0), Length3d(0), AvgEdgeLength3d(0),
    positionInPseudo3dStart2(0), positionInPseudo3dEnd2(0), positionInPseudo3dMid(0), edgeLengthStart(0), edgeLengthEnd(0),
    pointStart_vertexId(-1), pointEnd_vertexId(-1),
    IndexInTopologyConnections(-1), PrevSegmentInLoop(nullptr), NextSegmentInLoop(nullptr), ConnectedTo(nullptr),
    topologyPointStart(nullptr), topologyPointEnd(nullptr), topologyConnection(0)
{
    Init(_fromIndex, _toIndex);
}

II MeshTopologyLoopCurveSegment::SizeOF() const
{
    return sizeof(MeshTopologyLoopCurveSegment);
}

MeshTopologyLoopCurveSegment::MeshTopologyLoopCurveSegment(const MeshTopologyLoopCurve& _curve, vector<int> edgeIndexes)
    : id(nextTopologyLoopCurveSegmentID++),
    curve(_curve), meshid(_curve.mesh.id),
    fromIndex(0), toIndex(0), isCyclic(false), PointsCount(0), EdgesCount(0), Length3d(0), AvgEdgeLength3d(0),
    positionInPseudo3dStart2(0), positionInPseudo3dEnd2(0), positionInPseudo3dMid(0), edgeLengthStart(0), edgeLengthEnd(0),
    pointStart_vertexId(-1), pointEnd_vertexId(-1),
    IndexInTopologyConnections(-1), PrevSegmentInLoop(nullptr), NextSegmentInLoop(nullptr), ConnectedTo(nullptr),
    topologyPointStart(nullptr), topologyPointEnd(nullptr), topologyConnection(0)
{
    assert(edgeIndexes.size() > 0);

    vector<int> edgeIndexes_save = edgeIndexes;
    utils::stdvector::sort(edgeIndexes);
    if (edgeIndexes[edgeIndexes.size() - 1] == edgeIndexes[0] + edgeIndexes.size() - 1) // if indexes are continious
    {
        Init(edgeIndexes[0], edgeIndexes[edgeIndexes.size() - 1] + 1); // there is no cyclic indexing 
    }
    else
    {
        for (int i = 0; i < edgeIndexes.size() - 1; i++) // find break in indexing between all edges
        {
            if (edgeIndexes[i] + 1 != edgeIndexes[i + 1]) // test where is break in indexing
            {
                Init(edgeIndexes[i + 1], edgeIndexes[i] + 1); // add cyclic indexing 
                return;
            }
        }
        assert(false && "this line shouldnt be reach");
    }
}


bool MeshTopologyLoopCurveSegment::PointIsSharp(int index) const
{
    int i = CorrectCyclicIndex(fromIndex + index);
    int loopIndex = curve.loopPointIndexes(i);
    const MeshLoopPoint& p = curve.loop.points[loopIndex];
    return p.IsSharp || p.probably_IsSharp;
}

P3 MeshTopologyLoopCurveSegment::MiddlePoint() const
{
    // use middle point of curve if segment is completely cover curve
    if (PointsCount == curve.PointsCount)
    {
        return curve.middlePoint;
    }
    return GetPointAtLengthPercent(0.5);
}

P3s MeshTopologyLoopCurveSegment::GetPointsForDraw(bool& isPointsOptimizedToLine) const
{
    isPointsOptimizedToLine = false;
    
    // try to return line - optimisation
    if (PointsCount > 3 && !isCyclic)
    {
        D maxDist = 0;
        for (int i = 1; i < 5; i++)
        {
            D dist = utils::vector::DistFromLineToPoint(pointStart, pointEnd, GetPointAtLengthPercent(i*(1.0 / 5.0)));
            maxDist = max(maxDist, dist);
        }
        //cout << "PointsCount=" << PointsCount << "         maxDist = "<< maxDist << endl;

        if (maxDist < AvgEdgeLength3d / 100)
        {
            isPointsOptimizedToLine = true;
            //cout << "PointsCount=" << PointsCount << " optimized to 1" << endl;
        }

        if (isPointsOptimizedToLine)
        {
            P3s points;
            points.resize(2, 3);
            points.row(0) = pointStart;
            points.row(1) = pointEnd;
            return points;
        }
    }

    // return all points
    return GetPoints();
}


P3s MeshTopologyLoopCurveSegment::GetPoints() const
{
    // use curve points if segment is completely cover curve
    if (PointsCount == curve.PointsCount)
    {
        return curve.points;
    }

    P3s points;
    points.resize(PointsCount, 3);
    for (int i = 0; i < PointsCount; i++)
    {
        points.row(i) = curve.points.row(CorrectCyclicIndex(fromIndex + i));
    }
    return points;
}

void MeshTopologyLoopCurveSegment::GetPoints(vector<StreamPoint>& points) const
{
    // use curve points if segment is completely cover curve
    //if (PointsCount == curve.PointsCount)
    //{
    //    return curve.GetPoints(points);
    //}

    auto addPoint = [&](int vid, D length, D totalLength3d, const V3& dirToNextPointNormalized)
    {
        const P3s& V = curve.mesh.V;
        P3 point = V.row(vid);
        points.push_back(StreamPoint(MeshPointType::onVertex, vid, point, -1));
        points.back().lengthToNextPoint = length;
        points.back().dirToNextPointNormalized = dirToNextPointNormalized;
        points.back().Length3dUntilThisPoint = totalLength3d;
    };
    points.reserve(PointsCount);
    D totalLength3d = 0;
    for (int i = 0; i < EdgesCount; i++)
    {
        int loopindex = curve.loopPointIndexes(CorrectCyclicIndex(fromIndex + i)); // translate segment index to loop index
        const MeshLoopEdge& edge = curve.loop.edges[loopindex];        
        addPoint(edge.StartVertexId, edge.Length, totalLength3d, edge.directionAlongEdge/edge.Length);
        totalLength3d += edge.Length;
        if (i == EdgesCount - 1)
        {
            addPoint(edge.EndVertexId, 0, totalLength3d, V3(0, 0, 0));
        }
    }
}

P3 MeshTopologyLoopCurveSegment::GetPointAtLength(D atLength3d) const
{
    return curve.GetPointAtLength(atLength3d, fromIndex, EdgesCount);
}

void MeshTopologyLoopCurveSegment::GetPointsDividedByCount(int divisioncount, P3s& points, bool isPointsStoredInRevertedOrder) const
{
    curve.GetPointsDividedByCount(fromIndex, EdgesCount, divisioncount, points, isPointsStoredInRevertedOrder, Length3d);
}

int MeshTopologyLoopCurveSegment::GetSegmentsCountInLoop() const
{
    int count = 1;
    MeshTopologyLoopCurveSegment* next = NextSegmentInLoop;
    while (next->id != id)
    {
        count++;
        next = next->NextSegmentInLoop;
    }
    return count;
}

MeshPoint MeshTopologyLoopCurveSegment::GetMeshPointAtLength(D atLength3d, bool allowPointOnVertex) const
{
    return curve.GetMeshPointAtLength(atLength3d, fromIndex, EdgesCount, allowPointOnVertex);
}

P3 MeshTopologyLoopCurveSegment::GetPointAtLengthPercent(D percent) const
{
    return GetPointAtLength(Length3d*percent);
}

D MeshTopologyLoopCurveSegment::GetAtLengthFromPoint(const MeshPoint& point) const
{
    return curve.GetAtLengthFromPoint(point, fromIndex, EdgesCount);
}

bool MeshTopologyLoopCurveSegment::IsOppositeDirections(const MeshTopologyLoopCurveSegment& anotherSegment) const
{
    D distStartStart = utils::point::DistToPointPow2(pointStart, anotherSegment.pointStart);
    D distStartEnd = utils::point::DistToPointPow2(pointStart, anotherSegment.pointEnd);
    return distStartStart > distStartEnd;
}


atomic_int MeshTopologyLoop::next_updateId;


MeshTopologyLoop::MeshTopologyLoop()
    : updateId(next_updateId)
{

}


II MeshTopologyLoop::SizeOF() const
{
    II r = sizeof(MeshTopologyLoop);
    for (const auto& point : points) r += point.SizeOF();
    for (const auto& curve : curves) r += curve.SizeOF();
    return r;
}

void MeshTopologyLoop::Update(const Mesh& mesh)
{
    updateId = next_updateId++;
    points.clear();
    curves.clear();

    // allocate points count for correctness making references - we need this to make shure our references to points will be stable (no reallocation will be for 'points' list)
    int pointsCountTotal = 0;
    for (const MeshLoop& loop : mesh.Loops)
        for (const MeshLoopPoint& point : loop.points)
            if (point.IsSharp) pointsCountTotal++;
    points.reserve(pointsCountTotal);

    // for every loop - add points, and create curves
    for (const MeshLoop& loop : mesh.Loops)
    {
        if (loop.points.size() == 0) continue;
        int loopPointIndexStart = points.size(); // remember size of points
        for (const MeshLoopPoint& point : loop.points) // add all sharp point in the loop to global list
            if (point.IsSharp) points.push_back(MeshTopologyLoopPoint(mesh, loop, point));
        int loopPointIndexEnd = points.size(); // take current size of points
        int pointsCount = loopPointIndexEnd - loopPointIndexStart; // calculate points count in current loop
        if (pointsCount == 0) // if there are no sharp points, we must anyway to take some point as begining and end of the curve
        {
            points.push_back(MeshTopologyLoopPoint(mesh, loop, loop.points[0])); //  add first point (no matter what points we will add)
            loopPointIndexEnd = points.size(); // take current size of points
            pointsCount = loopPointIndexEnd - loopPointIndexStart; // recalculate points count in current loop
        }

        for (int i = 0; i < pointsCount; i++)
        {
            int index0 = loopPointIndexStart + i; // index of first point of curve
            int index1 = (index0 == loopPointIndexEnd - 1) ? loopPointIndexStart : index0 + 1; // index of second point of curve
            curves.push_back(MeshTopologyLoopCurve(mesh, loop, points[index0], points[index1])); // we can add here points as references since 'points' will not be reallocated since we have preallocated space for all future comming points at begining of current method
        }
    }
}


