#include "stdafx.h"
#include "MeshStreams.h"
#include "Mesh.h"
#include "MeshSolverNrosy.h"
#include "MeshSolverUV.h"
#include "ViewerDrawObjects.h"
#include "Utils.h"
#include <igl/polyvector_field_matchings.h>
#include <igl/segment_segment_intersect.h>
#include <igl/rotation_matrix_from_directions.h>
#include "../../../libs/vector_class/vectori128.h"
#include "../../../libs/vector_class/vectorf128.h"

const MeshLogicOptions_MeshStream& options = meshLogicOptions.MeshStream;


atomic_int StreamAnchorPoint::nextStreamAnchorPointID;
atomic_int StreamStartPoint::nextStreamStartDirectionID;


MeshStream::MeshStream(int _index, int _streamStartPointId, int _streamAnchorPointId, ViewerDrawObjects& _draw, const Mesh& _mesh, const Color3d& _color, DividerIteratorStreamInfo _dividingIteration)
    : Index(_index), StreamStartPointId(_streamStartPointId), StreamAnchorPointId(_streamAnchorPointId), draw(_draw), mesh(_mesh), color(_color),
    IsMerged(false), IsMerged_with_streamIndex(-1), IsDeleted(false)
    , dividingIteration(_dividingIteration), Length3d(0), isCyclic(false), isInited_lengthToNextPoint(false)
{
}

void  MeshStream::Add(const StreamPoint& p)
{
    assert(!isnan(p.point[0]));
    Points.push_back(p);
    Set_IsInited_lengthToNextPoint(false);
}

void MeshStream::Remove(int index, int count)
{
    if (count == 1)
    {
        utils::stdvector::remove_at(Points, index);
    }
    else if (count > 1)
    {
        utils::stdvector::remove_at(Points, index, count);
    }
    Set_IsInited_lengthToNextPoint(false);
}
bool MeshStream::DuplicatePointsExists()
{
    int size = Points.size();
    for (int i = 0; i < size; i++)
    {
        const StreamPoint& p1 = Points[i];
        const StreamPoint& p2 = Points[i + 1];
        if (p1.Type == p2.Type && p1.vid_eid_fid == p2.vid_eid_fid && p1.intersectionID == p2.intersectionID)
        {
            return true;
        }
    }
    return false;
}
int MeshStream::RemoveDuplicatePoints(bool tracedebuginfo)
{
    // first check if duplicate point exists before iterating memory backward - speed optimization
    // we usually wont have duplicate points so we can assume that we can run loop once in forward direction instead of in backward direction
    if (!DuplicatePointsExists())
    {
        return 0;
    }

    int removedCount = 0;
    int size = Points.size();
    for (int i = size - 1; i > 0; i--)
    {
        const StreamPoint& p1 = Points[i];
        const StreamPoint& p2 = Points[i - 1];
        if (p1.Type == p2.Type && p1.vid_eid_fid == p2.vid_eid_fid && p1.intersectionID == p2.intersectionID)
        {
            if (tracedebuginfo) cout << "!!! indentical points detected in streamIndex=" << Index << "'  - removing this point: p1=" << p1.vid_eid_fid_toString() << "   p2=" << p2.vid_eid_fid_toString() << endl;
            removedCount++;
            Remove(i);
        }
    }

    return removedCount;
}

void MeshStream::Insert(int index, const StreamPoint& point)
{
    Points.insert(Points.begin() + index, point);
    Set_IsInited_lengthToNextPoint(false);
}
void MeshStream::SetNewPoints(const vector<StreamPoint>& newPoints, bool isInited_lengthToNextPoint_in_newPoints)
{
    bool lengthInited = isInited_lengthToNextPoint;
    Points = newPoints;
    if (isInited_lengthToNextPoint_in_newPoints)
    {
        Set_IsInited_lengthToNextPoint(true);
    }
    else
    {
        Set_IsInited_lengthToNextPoint(false);
        if (lengthInited)
        {
            Init_lengthToNextPoint();
        }
    }
}

void MeshStream::Set_IsInited_lengthToNextPoint(bool isInited)
{
    if (isInited)
    {
        // tell stream that 'lengthToNextPoint' and 'dirToNextPointNormalized' is already populated, and that we need update Length3d
        isInited_lengthToNextPoint = true;
        Length3d = 0;
        for (int i = 0; i < Points.size(); i++)
        {
            Points[i].Length3dUntilThisPoint = Length3d;
            Length3d += Points[i].lengthToNextPoint;
        }
    }
    else
    {
        // tell that we have to clear 'Length3d' because some changes were made in Points
        isInited_lengthToNextPoint = false;
        Length3d = 0;
    }
}

void MeshStream::Init_lengthToNextPoint()
{
    if (isInited_lengthToNextPoint) return;
    isInited_lengthToNextPoint = true;
    Length3d = 0;
    int size = Points.size();
    if (size == 0) return;
    P3 v0 = Points[0].point;
    for (int i = 0; i < size - 1; i++)
    {
        P3 v1 = Points[i + 1].point;
        V3 line = v1 - v0;
        D len = utils::vector::Length(line);
        Points[i].lengthToNextPoint = len;
        Points[i].dirToNextPointNormalized = line / len;
        Points[i].Length3dUntilThisPoint = Length3d;
        Length3d += len;
        v0 = v1;
    }
    Points[size - 1].lengthToNextPoint = 0;
    Points[size - 1].dirToNextPointNormalized = P3(0, 0, 0);
    Points[size - 1].Length3dUntilThisPoint = Length3d;
}

bool MeshStream::isEndsAtBorder() const
{
    if (Points.size() == 0) return false;
    return Points.back().isOnBorder(mesh);
}

P3 MeshStream::GetPointAtLength(D atLength) const
{
    assert(isInited_lengthToNextPoint);

    if (Points.size() != 0)
    {
        D summLength = 0;
        for (int i = 0; i < Points.size() - 1; i++)
        {
            D currentLength = Points[i].lengthToNextPoint;
            if (summLength <= atLength && atLength < summLength + currentLength)
            {
                D shift = (atLength - summLength);
                return Points[i].point + Points[i].dirToNextPointNormalized * shift;
            }
            summLength += currentLength;
        }
    }

    assert(false && "MeshStream::GetMidPoint()   cannot find point");
    cout << "! wrong  MeshStream::GetMidPoint()   cannot find point" << endl;
    if (Points.size() == 0)
    {
        return P3(0, 0, 0);
    }
    return Points.back().point;
}

P3 MeshStream::GetMidPoint() const
{
    return GetPointAtLength(Length3d / 2);
}

DD MeshStream::GetDistToLineIndexDD(const Point3d& point, int lineIndex) const
{
    Point3d v0 = convertP3ToEigenDouble(Points[lineIndex].point);
    Point3d v1 = convertP3ToEigenDouble(Points[lineIndex + 1].point);
    Vector3d line = v1 - v0;
    if (isInited_lengthToNextPoint)
    {
        line /= Points[lineIndex].lengthToNextPoint;
    }
    else
    {
        line.normalize();
    }
    DD dist = line.cross(point - v0).norm();
    return dist;
}


D MeshStream::GetClosestDist_Linear(int points_startFromIndex, const P3& point, P3& pointOnPoints, int& pointIndex, bool breakAtFirstContact, bool returnFirstLastIndexIfFail, D maxDistRatioForNextContact) const
{
    assert(isInited_lengthToNextPoint);
    assert(points_startFromIndex < size());
    // find closest point

    //
    // v1 - raw - sometimes doesnt work when when stream is circle
    //
    //int closestPointIndex = 0;
    //D closestDist = utils::point::LengthPow2(connectionPoint, points[0].point);
    //for (int i = 0; i < points.size(); i++)
    //{
    //    D closestDistI = utils::point::LengthPow2(connectionPoint, points[i].point);
    //    if (closestDistI < closestDist)
    //    {
    //        closestDist = closestDistI;
    //        closestPointIndex = i;
    //    }
    //}
    //closestDist = sqrt(closestDist);// get 'sqrt' since we were using LengthPow2
    //
    //pointIndex = closestPointIndex;
    //cout << "closestDist = " << closestDist << "    closestPointIndex = " << closestPointIndex << endl;
    // find closest dist to line
    //bool found = false;
    //pointOnPoints = points[closestPointIndex].point;
    //int closestPointIndexPrev = closestPointIndex - 1;
    //if (closestPointIndex >= 0)
    //{
    //    P3 pOnVector = utils::vector::ClosestPoint_ToLine(points[closestPointIndex].point, points[closestPointIndexPrev].point, connectionPoint);
    //    D distPrev = (connectionPoint - pOnVector).norm();
    //    if (!found || distPrev < closestDist)
    //    {
    //        found = true;
    //        closestDist = distPrev;
    //        pointOnPoints = pOnVector;
    //        pointIndex = closestPointIndexPrev;
    //        //cout << "closestDist = " << closestDist << "    closestPointIndexPrev = " << closestPointIndexPrev << "     distPrev = " << distPrev << endl;
    //    }
    //}
    //int closestPointIndexNext = closestPointIndex + 1;
    //if (closestPointIndexNext < points.size())
    //{
    //    P3 pOnVector = utils::vector::ClosestPoint_ToLine(points[closestPointIndex].point, points[closestPointIndexNext].point, connectionPoint);
    //    D distNext = (connectionPoint - pOnVector).norm();
    //    if (!found || distNext < closestDist)
    //    {
    //        found = true;
    //        closestDist = distNext;
    //        pointOnPoints = pOnVector;
    //        pointIndex = closestPointIndex;
    //        //cout << "closestDist = " << closestDist << "    closestPointIndexNext = " << closestPointIndexNext << "     distNext = " << distNext << endl;
    //    }
    //}
    //return closestDist;

    //
    // v2 - precise
    //
    pointIndex = -1;
    D closestDistPow2 = 0;
    D maxDistRatioForNextContactPow2 = maxDistRatioForNextContact * maxDistRatioForNextContact;
    bool closestDist_isSet = false;
    bool prev_isOnNextLine = false;
    P3 pOnLine(0, 0, 0);
    int iMax = Points.size() - 2;//  excluding last, since we want to take i+1 in loop
    for (int i = points_startFromIndex; i <= iMax; i++)
    {
        // watch the distances to break a search and protect from cyclic loops
        if (closestDist_isSet)
        {
            D currentDistPow2 = utils::point::DistToPointPow2(point, Points[i].point);
            if (currentDistPow2 > closestDistPow2*maxDistRatioForNextContactPow2) //  if new dist is 8 times higher from min, so we are really getting far away from touch point
            {
                break; // break a search
            }
            //cout << "closestDistPow2 = " << closestDistPow2 << "   currentDistPow2 = " << currentDistPow2 << endl;
        }

        //if (utils::vector::ClosestPoint_ToLine_IsOnLine(points[i].point, points[i + 1].point, connectionPoint))
        //{
        //    P3 pOnLine = utils::vector::ClosestPoint_ToLine(points[i].point, points[i + 1].point, connectionPoint);

        P3 linePoint0 = Points[i].point;
        P3 linePoint1 = Points[i + 1].point;
        V3 line = linePoint1 - linePoint0;
        D lineLength = Points[i].lengthToNextPoint;
        D lineLengthPow2 = lineLength * lineLength;
        V3 PQ = point - linePoint0;
        D PQ_dot_u = utils::vector::Dot(PQ, line);
        bool isOnPrevLine = PQ_dot_u < 0;
        bool isOnNextLine = PQ_dot_u > lineLengthPow2;
        bool isOnLine = !(isOnPrevLine || isOnNextLine);

        if (isOnLine)
        {
            pOnLine = linePoint0 + line * (PQ_dot_u / lineLengthPow2); //P3 pOnLine = utils::vector::ClosestPoint_ToLine(points[i].point, points[i + 1].point, connectionPoint);
        }
        else if (isOnPrevLine && prev_isOnNextLine)
        {
            isOnLine = true;
            pOnLine = linePoint0;
        }
        else if (i == iMax)
        {
            isOnLine = true;
            pOnLine = Points[Points.size() - 1].point;
        }
        else if (i == points_startFromIndex && isOnPrevLine)
        {
            isOnLine = true;
            pOnLine = Points[points_startFromIndex].point;
        }

        if (isOnLine) // if we touching stream we want to check dist, or if we already have touching distance we want to check if we need to break a loop to protect from cyclic streams around the circle
        {
            D distPow2 = utils::point::DistToPointPow2(point, pOnLine);
            if (!closestDist_isSet || distPow2 < closestDistPow2)
            {
                closestDist_isSet = true;
                pointIndex = i;
                pointOnPoints = pOnLine;
                closestDistPow2 = distPow2;
                if (breakAtFirstContact)
                {
                    break;
                }
            }
        }

        prev_isOnNextLine = isOnNextLine;
    }

    if (!closestDist_isSet)
    {
        closestDist_isSet = true;
        D distToFirstPow2 = utils::point::DistToPointPow2(point, Points[points_startFromIndex].point);
        D distToLastPow2 = utils::point::DistToPointPow2(point, Points.back().point);
        pointIndex = (distToFirstPow2 < distToLastPow2) ? 0 : Points.size() - 1;
        pointOnPoints = Points[pointIndex].point;
        closestDistPow2 = (distToFirstPow2 < distToLastPow2) ? distToFirstPow2 : distToLastPow2;
    }

    return sqrt(closestDistPow2);

}

D MeshStream::GetClosestDist_Quick(const P3& point, P3& pointOnPoints, int& pointIndex, int points_startIndex, int points_endIndex) const
{
    const int indexMAX = size() - 2;
    if (points_endIndex == -1) points_endIndex = indexMAX;
    int count = points_endIndex - points_startIndex + 1;

    // if we have many point - lets divide diapason by 4 and get closest index, until we will have less than 20 in diapason
    while (count > 20)
    {
        int minIndex = 0;
        D minDistPow2 = utils::point::DistToPointPow2(point, Points[points_startIndex].point);
        for (int k = 1; k <= 4; k++)
        {
            D distPow2 = utils::point::DistToPointPow2(point, Points[points_startIndex + count * k / 4 - 1].point);
            if (distPow2 < minDistPow2)
            {
                minDistPow2 = distPow2;
                minIndex = k;
            }
        }

        int newMidIndex = points_startIndex + count * minIndex / 4 - 1; //  set middle of new diapason  min index
        points_startIndex = max(0, newMidIndex - count / 4); //decrease diapson by 50%
        points_endIndex = min(newMidIndex + count / 4, indexMAX);  //decrease diapson by 50%

        count = points_endIndex - points_startIndex + 1;
    }

    // if there is few points to test - call linear method
    return GetClosestDist_Linear(max(0, points_startIndex - 1), point, pointOnPoints, pointIndex, true, true);
}

D MeshStream::GetTotalLength(int indexStart, int indexEnd) const
{
    assert(isInited_lengthToNextPoint);
    if (indexEnd == -1) indexEnd = size() - 1;
    D totalLength = 0;
    for (int i = indexStart; i < indexEnd; i++)
    {
        totalLength += Points[i].lengthToNextPoint;
    }
    return totalLength;
}

void MeshStream::GetPointsDividedByCount(int divisioncount, P3s& points, bool isPointsStoredInRevertedOrder) const
{
    assert(Points.size() >= 2);
    assert(isInited_lengthToNextPoint);
    P3 pointStart = Points[0].point;
    P3 pointEnd = Points.back().point;
    divisioncount = max(divisioncount, 1); // at least 1 must be
    points.resize(divisioncount + 1, 3);
    points.row(0) = !isPointsStoredInRevertedOrder ? pointStart : pointEnd;
    points.row(divisioncount) = !isPointsStoredInRevertedOrder ? pointEnd : pointStart;
    int pointIndex = 1;
    D atLength3d = Length3d / divisioncount;
    //cout << "Length3d = " << Length3d << "   GetTotalLength()="<< GetTotalLength() << endl;

    D summLength = 0;
    for (int i = 0; i < size() - 1; i++)
    {
        D currentLineLength = Points[i].lengthToNextPoint;
        while (summLength <= atLength3d && atLength3d <= summLength + currentLineLength + currentLineLength * 0.0000001)  //plus add D error correction
        {

            P3 pointAtLength = Points[i].point + Points[i].dirToNextPointNormalized * (atLength3d - summLength);

            int pointIndexReversed = divisioncount - pointIndex;
            points.row(!isPointsStoredInRevertedOrder ? pointIndex : pointIndexReversed) = pointAtLength;
            pointIndex++;
            atLength3d = pointIndex * Length3d / divisioncount;
            if (pointIndex == divisioncount) return;
        }
        summLength += currentLineLength;
    }
    if (pointIndex <= divisioncount)
    {
        cout << "!!! not all points are populated in  MeshStream::GetPointsDividedByCount" << endl;
        assert(pointIndex > divisioncount && "not all points are populated in  MeshStream::GetPointsDividedByCount");
    }
}

II MeshStream::SizeOF() const
{
    II r = sizeof(MeshStream);
    r += Points.size() * sizeof(StreamPoint);
    return r;
}

bool MeshStream::GetConnection(ViewerDrawObjects& draw, const Mesh& mesh, MeshStream& stream1, MeshStream& stream2, D& connectionDist, int& connection_pointsIndex1, int& connection_pointsIndex2, int points1_maxIndex, int points2_maxIndex, D maxConnectionAngle_Directions, D maxConnectionAngle_Normals, bool debug, bool debugEveryPoint, D maxLenghtToTest)
{
    assert(stream1.isInited_lengthToNextPoint);
    assert(stream2.isInited_lengthToNextPoint);
    if (maxLenghtToTest < 0)
    {
        maxLenghtToTest = min(stream1.Length3d, stream2.Length3d);
    }
    D maxConnectionAngle_Directions_DR_COS = utils::angle::DegreesToCos(maxConnectionAngle_Directions); // streams should be somehow parallel
    D maxConnectionAngle_Normals_DR_COS = utils::angle::DegreesToCos(maxConnectionAngle_Normals); // streams normals should be reasoanble, at least lest from 120 degree

    stream1.Init_lengthToNextPoint();
    stream2.Init_lengthToNextPoint();
    if (points1_maxIndex == -1) points1_maxIndex = stream1.size() - 1; // default values is 'all indexes'
    if (points2_maxIndex == -1) points2_maxIndex = stream2.size() - 1; // default values is 'all indexes'
    points1_maxIndex = min(points1_maxIndex, stream1.size() - 1); //protection from out of index
    points2_maxIndex = min(points2_maxIndex, stream2.size() - 1); //protection from out of index

    struct sInfo
    {
        const MeshStream& points;
        int currentIndex;
        D currentLength;
        D currentEdgeLength;
        int indexMax;
        int bestIndex;
        D bestLength;
        int pointsSize;
        D cosStreamPrev;
        sInfo(const MeshStream&_points, int _indexMax)
            :points(_points), currentIndex(0), currentLength(0), currentEdgeLength(0), indexMax(_indexMax), bestIndex(0), bestLength(0)
        {
            pointsSize = points.size();
            cosStreamPrev = -10; // always correct at start
        }
    };
    sInfo ss[2]{ sInfo(stream1, points1_maxIndex), sInfo(stream2, points2_maxIndex) };

    D minDistPow2 = utils::point::DistToPointPow2(stream1[0].point, stream2[0].point);
    bool minDistFound = false;


    while (ss[0].currentIndex < ss[0].indexMax && ss[1].currentIndex < ss[1].indexMax)
    {
        // calculate what stream index should be extended to move at approximately same length
        int incrI = -1;
        for (int i = 0; i < 2; i++)
        {
            const sInfo& s = ss[i];
            if (s.currentIndex < s.indexMax)
            {
                int iOpposite = i ^ 1;
                D lineLength = s.points[s.currentIndex].lengthToNextPoint;
                if (s.currentLength + lineLength < ss[iOpposite].currentLength)
                {
                    incrI = i;
                }
            }
        }

        // if there is no winner stream to extend, choise with lowest current length
        for (int i = 0; i < 2; i++)
        {
            const sInfo& s = ss[i];
            if (s.currentIndex < s.indexMax)
            {
                int iOpposite = i ^ 1;
                if (s.currentLength < ss[iOpposite].currentLength)
                {
                    incrI = i;
                }
            }
        }

        // if there is no winner stream to extend, choise first possible
        if (incrI == -1)
        {
            incrI = ss[0].currentIndex < ss[0].indexMax ? 0 : 1;
        }

        // increase index of best stream
        sInfo& sincr = ss[incrI];
        //sInfo& sincrOpposite = ss[(incrI+1)%2];
        D incLength = sincr.points[sincr.currentIndex].lengthToNextPoint;
        sincr.currentLength += incLength;
        sincr.currentEdgeLength = incLength;
        sincr.currentIndex++;
        if (sincr.currentLength > maxLenghtToTest)
        {
            if (debug) cout << "breaking since test length " << sincr.currentLength << " is bigger from maximum requested length " << maxLenghtToTest << endl;
            break;
        }

        const StreamPoint& s0p0 = ss[0].points[ss[0].currentIndex];
        const StreamPoint& s0p1 = ss[0].points[ss[0].currentIndex + 1];
        const StreamPoint& s1p0 = ss[1].points[ss[1].currentIndex];
        const StreamPoint& s1p1 = ss[1].points[ss[1].currentIndex + 1];

        // angle between two streams vectors should be less from 40 degree and go opposite to each other
        D cosStream = utils::vector::Cos(s0p0.dirToNextPointNormalized, s1p0.dirToNextPointNormalized, true);
        D cosStreamPrev = sincr.cosStreamPrev;
        sincr.cosStreamPrev = cosStream;
        D cosNormal = 0;
        D pos = 0;
        D distPow2 = 0;
        bool accepted = false;
        bool acceptedCos = false;
        bool acceptedDist = false;
        // D angle = utils::vector::Angle(s0p0.dirToNextPointNormalized, -s1p0.dirToNextPointNormalized, true);
        // if (angle < maxConnectionAngle)
        if (cosStream < -maxConnectionAngle_Directions_DR_COS             // direction opposite and a almost 180 degree
            && cosStreamPrev < -maxConnectionAngle_Directions_DR_COS) // previous direction is same good as current - noise filter for streams - some edges may randomly be of different direction from avarage normal direction
        {
            cosNormal = utils::vector::Cos(mesh.F_normals.row(s0p0.fid), mesh.F_normals.row(s1p0.fid), true);
            if (cosNormal > maxConnectionAngle_Normals_DR_COS) // normals diff less 120 degree
            {
                acceptedCos = true;
                distPow2 = utils::point::DistToPointPow2(s0p0.point, s1p0.point);
                if (distPow2 < minDistPow2 || !minDistFound) // dist is acceptable
                {
                    acceptedDist = true;
                    pos = utils::vector::ClosestPoint_ToVector_Position(s0p0.point, s0p1 - s0p0, s0p0.lengthToNextPoint, s1p0.point);
                    if (pos > 0.1) // vector getting closer to each other - so they point each other to theirs vector-ends
                    {
                        minDistPow2 = distPow2;
                        ss[0].bestIndex = ss[0].currentIndex;
                        ss[0].bestLength = ss[0].currentLength;
                        ss[1].bestIndex = ss[1].currentIndex;
                        ss[1].bestLength = ss[1].currentLength;
                        accepted = true;
                        minDistFound = true;
                    }
                }

                // protection against cyclic streams
                if (minDistFound && distPow2 > minDistPow2*(8 * 8)) //  if new dist is 8 times higher from min, so we are really getting far away from connection point
                {
                    if (debug) cout << "   breaking since dist is higher from minimum dist by 8x times" << endl;
                    break;
                }
            }
        }



        if (debug && debugEveryPoint)
        {

            D angle = utils::vector::Angle(s0p0.dirToNextPointNormalized, -s1p0.dirToNextPointNormalized, true);
            D angleNormal = utils::vector::Angle(mesh.F_normals.row(s0p0.fid), mesh.F_normals.row(s1p0.fid), true);
            cout << "   point#" << ss[0].currentIndex << "   point#" << ss[1].currentIndex << "   lengths=[" << ss[0].currentEdgeLength << ", " << ss[1].currentEdgeLength << "]   angle=" << angle << "   angleNormal=" << angleNormal;
            if (acceptedCos)
            {
                cout << "  dist=" << sqrt(distPow2);
                if (acceptedDist)
                {
                    cout << "  pos=" << (pos * s0p0.lengthToNextPoint / stream1[0].lengthToNextPoint);
                    //P3 pointOnStream = utils::vector::ClosestPoint_ToVector(s0p0.point, s0p1 - s0p0, s0p0.lengthToNextPoint, s1p1.point);
                    //draw.AddEdge(s0p0.point, s1p0.point, Color3d(0.6,0.6,0.6));
                }
            }
            cout << (accepted ? "    accepted" : "");
            cout << endl;
        }
    }

    connection_pointsIndex1 = ss[0].bestIndex;
    connection_pointsIndex2 = ss[1].bestIndex;
    connectionDist = sqrt(minDistPow2);
    // increase connectionDist precision
    if (minDistFound)
    {
        // in loop we have measuring distances between start of the edges
        // now measure distance to edge itself
        P3 p1 = stream1.Points[connection_pointsIndex1].point;
        P3 p1End = stream1.Points[connection_pointsIndex1 + 1].point;
        P3 p2 = stream2.Points[connection_pointsIndex2].point;
        P3 p2End = stream2.Points[connection_pointsIndex2 + 1].point;
        P3 p1Closest = utils::vector::ClosestPoint_ToLine(p1, p1End, p2);
        D p1Closest_distPow2 = utils::point::DistToPointPow2(p1Closest, p2);
        if (p1Closest_distPow2 < minDistPow2)
        {
            connectionDist = sqrt(p1Closest_distPow2);
            // DEBUG show results
            //cout << "connectionDist = " << sqrt(minDistPow2) << "   p1Closest_distPow2 = " << sqrt(p1Closest_distPow2) << endl;
            //draw.AddPoint(p1);
            //draw.AddPoint(p2);
            //draw.AddPoint(p1Closest, Color3d(1, 0, 0), "updated");
        }
    }
    return minDistFound;
}

bool MeshStream::Remove_PrelastPoint_If_LastIsVertex_And3LastsHaveCommonFace()
{
    int c = size();
    if (c < 3) return false;
    const MeshPoint& p1 = Points[c - 3];
    const MeshPoint& p2 = Points[c - 2];
    const MeshPoint& p3 = Points[c - 1];
    if (p3.Type != MeshPointType::onVertex || p2.Type != MeshPointType::onEdge) return false;

    int commonFaceId1, commonFaceId2;
    if (mesh.CommonFaceIds(p1, p2, p3, commonFaceId1, commonFaceId2))
    {
        Remove(c - 2);
        return true;
    }

    return false;
}

bool MeshStream::FindBestCommonEdgeOrVertexBetween2Streams_InScope(const MeshStream& stream1, const MeshStream& stream2,
    int stream1_connectionIndex, int stream2_connectionIndex, int& stream1_commonPointIndex, int& stream2_commonPointIndex, int maxSeekInterval)
{
    stream1_commonPointIndex = -1;
    stream2_commonPointIndex = -1;

    int i1Start = max(0, stream1_connectionIndex - maxSeekInterval);
    int i1End = min(stream1.size(), stream1_connectionIndex + maxSeekInterval);
    int i2Start = max(0, stream2_connectionIndex - maxSeekInterval);
    int i2End = min(stream2.size(), stream2_connectionIndex + maxSeekInterval);


    // create map for all points
    map<int, int> map_eids_points1index;
    map<int, int> map_vids_points1index;
    for (int i1 = i1Start; i1 < i1End; i1++)
    {
        auto& p1 = stream1[i1];
        if (p1.Type == MeshPointType::onEdge)
        {
            map_eids_points1index[p1.vid_eid_fid] = i1;
        }
        if (p1.Type == MeshPointType::onVertex) // skip first vertex coz stream cannot connect at first vertex
        {
            map_vids_points1index[p1.vid_eid_fid] = i1;
        }
    }

    // find common edge id with min distance between streams
    D min_distPow2 = -1;
    for (int i2 = i2Start; i2 < i2End; i2++)
    {
        auto& p2 = stream2[i2];
        if (p2.Type == MeshPointType::onEdge)
        {
            auto find = map_eids_points1index.find(p2.vid_eid_fid);
            if (find != map_eids_points1index.end())
            {
                int i1 = find->second;
                D distPow2 = utils::point::DistToPointPow2(stream1[i1].point, stream2[i2].point);
                if (min_distPow2 < 0 || distPow2 < min_distPow2)
                {
                    stream1_commonPointIndex = i1;
                    stream2_commonPointIndex = i2;
                    min_distPow2 = distPow2;
                }
            }
        }
        if (p2.Type == MeshPointType::onVertex)
        {
            auto find = map_vids_points1index.find(p2.vid_eid_fid);
            if (find != map_vids_points1index.end())
            {
                int i1 = find->second;
                D distPow2 = 0; // distance between 2 same vertexes is 0
                stream1_commonPointIndex = i1;
                stream2_commonPointIndex = i2;
                break; // if they touch exactly in one point, then no need to search for best connection - it is already maximum of what we can get: dist=0
            }
        }
    }

    return stream1_commonPointIndex != -1;
}

bool MeshStream::FindBestCommonEdgeOrVertexBetween2Streams_AllPoints(const MeshStream& stream1, const MeshStream& stream2,
    int& stream1_commonPointIndex, int& stream2_commonPointIndex)
{
    return FindBestCommonEdgeOrVertexBetween2Streams_InScope(stream1, stream2, 0, 0, stream1_commonPointIndex, stream2_commonPointIndex, max(stream1.size(), stream2.size()));
}

bool MeshStream::FindBestCommonFaceBetween2Streams(const MeshStream& stream1, const MeshStream& stream2, int& commonPointIndex1, int& commonPointIndex2)
{
    commonPointIndex1 = -1;
    commonPointIndex2 = -1;

    // create map for faces
    map<int, int> map_fids_points1index;
    for (int i1 = 0; i1 < stream1.size(); i1++)
    {
        auto& p1 = stream1[i1];
        if (p1.fid != -1)
        {
            map_fids_points1index[p1.fid] = i1;
        }
    }
    // find common face
    int common_fid = -1;
    for (int i2 = 0; i2 < stream2.size(); i2++)
    {
        auto& p2 = stream2[i2];
        if (p2.fid != -1)
        {
            auto find = map_fids_points1index.find(p2.fid);
            if (find != map_fids_points1index.end())
            {
                int i1 = find->second;
                if (common_fid == -1 || abs(i1 + i2) < abs(commonPointIndex1 + commonPointIndex2)) // get most middle point
                {
                    commonPointIndex1 = i1;
                    commonPointIndex2 = i2;
                    common_fid = p2.fid;
                }
            }
        }
    }

    return commonPointIndex1 != -1;
}

void MeshStream::DebugShowStreamPoints(const Color3d& color, int maxIndex) const
{
    if (maxIndex == -1)
    {
        maxIndex = size() - 1;
    }
    //const StreamPoints& points = Points[streamId];
    int piMax = min(maxIndex, size() - 1);
    int showIdsCounter = 0;
    for (int pi = 0; pi < piMax; pi++)
    {
        if (options.show_dividing_streams)
        {
            draw.AddEdge(Points[pi].point, Points[pi + 1].point, color);
            if (options.show_dividing_streams__show_vid_eid_fid)
            {
                const StreamPoint& p = Points[pi + 1];
                switch (p.Type)
                {
                    case MeshPointType::onEdge:
                        draw.AddLabel(p.point, "   " + to_string(p.vid_eid_fid), Color3d(0.5, 0.5, 0.5));
                        break;
                    case MeshPointType::onVertex:
                        draw.AddLabel(p.point, "          vid = " + to_string(p.vid_eid_fid), Color3d(1, 0, 0));
                        break;
                    case MeshPointType::onFace:
                        draw.AddLabel(p.point, "   fid = " + to_string(p.vid_eid_fid), Color3d(0, 0, 1));
                        break;
                }
            }

            if (options.show_streamIndexes_Every10Points)
            {
                showIdsCounter++;
                if (showIdsCounter == 10)
                {
                    showIdsCounter = 0;
                    string text = "stream#" + to_string(Index);
                    if (IsMerged)
                    {
                        text += "+" + to_string(IsMerged_with_streamIndex);
                    }
                    draw.AddLabel(Points[pi].point, text, Color4d(0.3, 0.3, 0.3, 0.3));
                }
            }

            if (options.show_streamIndexes_InAllPoints)
            {
                string text = "s#" + to_string(Index) + "." + to_string(pi);
                if (IsMerged)
                {
                    text += "+" + to_string(IsMerged_with_streamIndex);
                }
                draw.AddLabel(Points[pi].point, text, Color4d(0.3, 0.3, 0.3, 0.3));
            }
        }
    }
    auto pointAtStart = [&]()
    {
        return Points[min(3, size() - 1)].point;
        //return isInited_lengthToNextPoint
        //    ? GetPointAtLength(Length3d*0.3)
        //    : Points[min(2, size() - 1)].point;
    };
    if (options.show_streamIndexes_atStart)
    {
        if (size() > 0)
        {
            P3 p = pointAtStart();
            draw.AddPoint(p, Color3d(0.3, 0.3, 0.3), "stream#" + to_string(Index));
            if (IsMerged && size() > 2)
            {
                p = Points[size() - 2].point;
                draw.AddPoint(p, Color3d(0.3, 0.3, 0.3), "stream#" + to_string(IsMerged_with_streamIndex));
            }
        }
    }
    if (options.show_stream_iterations)
    {
        if (size() > 0)
        {
            P3 p = pointAtStart();
            string text = "S#" + to_string(dividingIteration.globalStreamIndex);
            text += ", i#" + to_string(dividingIteration.iterationNum);
            //text += ", m#" + to_string(dividingIteration.meshid__at_first_iteration);
            draw.AddPoint(p, Color3d(0.3, 0.3, 0.3), text);
        }
    }
}

MeshStreamer::MeshStreamer(ViewerDrawObjects& _draw, const Mesh& _mesh, const vector<V3s>& _nrosyField, bool _IsNrosyFieldSymetric, const MatchABBAs& _match_ab_ba)
    : draw(_draw), mesh(_mesh), nrosyField(_nrosyField), IsNrosyFieldSymetric(_IsNrosyFieldSymetric), match_ab_ba(_match_ab_ba),
    start_direction(V3(0, 0, 0)), current_start_point(P3(0, 0, 0)), current_end_point(P3(0, 0, 0)), current_fid(0), current_direction(0), current_start_edgeid(0), current_end_pointType(MeshPointType::onVertex), current_end_vid_eid_fid(0), finished(false)
{
}


void MeshStreamer::RestartStream(MeshStream& Stream, int fid, MeshPointType pointType, int vid_eid_fid, const P3& point, const V3& direction)
{
    start_direction = direction;
    current_start_point = point;
    current_end_point = point;
    current_fid = fid;
    if (current_fid != -1)
        start_normal = mesh.F_normals.row(current_fid);
    else
        start_normal = mesh.V_normals.row(vid_eid_fid);
    //
    //v1 - use start direction as face nrosyfield
    //
    //int dirNi = GetClosestNrosyFieldDirectionNi(faceId, direction);

    ////DEBUG - show choised direction
    //if (index == 4)
    //{
    //    P3 p = mesh.F_Barycenters.row(faceId);
    //    V3 fieldDirBest = nrosyField[dirNi].row(faceId).transpose();
    //    draw.AddEdge(p, p + fieldDirBest*mesh.avg_edge_length * 5, Color3d(1, 1, 0));
    //    for (int ni = 0; ni < N; ni++)
    //    {
    //        V3 fieldDirN = nrosyField[ni].row(faceId).transpose();
    //        V3 pD = fieldDirN*mesh.avg_edge_length * 2;
    //        draw.AddEdge(p, p + pD, Color3d(0.3, 0.3, 0));
    //        D angle = utils::vector::Angle(direction, fieldDirN);
    //        draw.AddLabel(p + pD*1.2, "angle " + to_string(round(angle)), Color3d(1, 1, 0));
    //    }
    //}
    //current_directions(index) = dirNi;
    //
    //v2 - use start direction as singularity direction
    //
    current_direction = -1;
    current_start_edgeid = (pointType == MeshPointType::onEdge) ? vid_eid_fid : -1;
    current_end_pointType = pointType;
    current_end_vid_eid_fid = vid_eid_fid;
    finished = false;
    if (Stream.size() == 1)
    {
        Stream[0] = StreamPoint(pointType, vid_eid_fid, point, current_fid);
    }
    else
    {
        Stream.Points.clear();
        Stream.Add(StreamPoint(pointType, vid_eid_fid, point, current_fid));
    }
}

bool segments_intersect_fast(
    const V3 &p,
    const V3 &v1,
    const V3 &q,
    const V3 &s,
    D &a_t,
    D &a_u
)
{
    const D eps = 1e-6;
    const D epsPlus1 = 1 + eps;

    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    // Search intersection between two segments
    // p + t*v1 :  t \in [0,1]
    // q + u*s :  u \in [0,1]

    // p + t * v1 = q + u * s  // x s
    // t(v1 x s) = (q - p) x s
    // t = (q - p) x s / (v1 x s)

    // (v1 x s) ~ 0 --> directions are parallel, they will never cross
    V3 rxs = v1.cross(s);
    D rxs_norm = rxs.norm();
    if (rxs.norm() <= eps)
        return false;

    int sign;

    // u = (q - p) * v1 / (v1 * s)
    D u;
    V3 u1 = (q - p).cross(v1);
    sign = ((u1.dot(rxs)) > 0) ? 1 : -1;
    //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
    u = u1.norm() / rxs_norm;
    u = u * sign;

    if (u < -eps || u > epsPlus1)
    {
        return false;
    }

    // t = (q - p) * s / (v1 * s)
    D t;
    V3 t1 = (q - p).cross(s);
    sign = ((t1.dot(rxs)) > 0) ? 1 : -1;
    cout << "t sign = " << sign << endl;
    t = t1.norm() / rxs_norm;
    t = t * sign;

    //if (t < -eps || fabs(t) < eps)
    if (t < eps)
        return false;

    a_t = t;
    a_u = 0;

    return true;
};



bool segments_intersect_fast2(
    const V3 &p1, const V3 &v1,
    const V3 &p2, const V3 &v2,
    D &dist1,
    D &dist2
)
{
    const D eps = 1e-6;
    const D epsPlus1 = 1 + eps;

    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    // Search intersection between two segments
    // p1 + t*v1 :  t \in [0,1]
    // p2 + u*v2 :  u \in [0,1]

    // p1 + t * v1 = p2 + u * v2  // x v2
    // t(v1 x v2) = (p2 - p1) x v2
    // t = (p2 - p1) x v2 / (v1 x v2)

    // (v1 x v2) ~ 0 --> directions are parallel, they will never cross
    V3 rxs = v1.cross(v2);
    D rxs_norm = rxs.norm();
    if (rxs.norm() <= eps)
        return false;

    int sign;

    // u = (p2 - p1) * v1 / (v1 * v2)
    D u;
    V3 u1 = (p2 - p1).cross(v1);
    sign = ((u1.dot(rxs)) > 0) ? 1 : -1;
    //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
    u = u1.norm() / rxs_norm;
    u = u * sign;

    if (u < -eps || u > epsPlus1)
    {
        return false;
    }

    // t = (p2 - p1) * v2 / (v1 * v2)
    D t;
    V3 t1 = (p2 - p1).cross(v2);
    sign = ((t1.dot(rxs)) > 0) ? 1 : -1;
    //cout << "t sign = " << sign << endl;
    t = t1.norm() / rxs_norm;
    t = t * sign;

    //if (t < -eps || fabs(t) < eps)
    if (t < eps)
        return false;

    dist1 = t;
    dist2 = u;

    return true;
};

bool segments_intersect_fast3(
    const V3 &p1, const V3 &v1,
    const V3 &p2, const V3 &v2,
    D &dist1,
    D &dist2
)
{
    const D eps = 1e-6;
    const D epsPlus1 = 1 + eps;

    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    // Search intersection between two segments
    // p1 + t*v1 :  t \in [0,1]
    // p2 + u*v2 :  u \in [0,1]

    // p1 + t * v1 = p2 + u * v2  // x v2
    // t(v1 x v2) = (p2 - p1) x v2
    // t = (p2 - p1) x v2 / (v1 x v2)

    // (v1 x v2) ~ 0 --> directions are parallel, they will never cross
    V3 rxs = v1.cross(v2);
    D rxs_norm = rxs.norm();
    if (rxs.norm() <= eps)
        return false;

    int sign;

    // u = (p2 - p1) * v1 / (v1 * v2)
    D u;
    V3 u1 = (p2 - p1).cross(v1);
    sign = ((u1.dot(rxs)) > 0) ? 1 : -1;
    //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
    u = u1.norm() / rxs_norm;
    u = u * sign;

    if (u < -eps || u > epsPlus1)
    {
        return false;
    }

    // t = (p2 - p1) * v2 / (v1 * v2)
    D t;
    V3 t1 = (p2 - p1).cross(v2);
    sign = ((t1.dot(rxs)) > 0) ? 1 : -1;
    //cout << "t sign = " << sign << endl;
    t = t1.norm() / rxs_norm;
    t = t * sign;

    //if (t < -eps || fabs(t) < eps)
    if (t < eps)
        return false;

    dist1 = t;
    dist2 = u;

    return true;
};


__forceinline bool segments_intersect_fast3fast(
    const P3 &p1, const V3 &v1,
    const P3 &p2, const V3 &v2,
    D &pos1,
    D &pos2
)
{
    const D eps = 1e-6;
    const D epsPlus1 = 1 + eps;

    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    // Search intersection between two segments
    // p1 + t*v1 :  t \in [0,1]
    // p2 + u*v2 :  u \in [0,1]

    // p1 + t * v1 = p2 + u * v2  // x v2
    // t(v1 x v2) = (p2 - p1) x v2
    // t = (p2 - p1) x v2 / (v1 x v2)

    V3 fromP2toP1 = p2 - p1;

    // (v1 x v2) ~ 0 --> directions are parallel, they will never cross

    V3 rxs = utils::vector::Cross(v1, v2);
    D rxs_norm = utils::vector::Length(rxs);
    if (rxs_norm <= eps) // if there is suspicious to be directions parallel - test precisely
    {
        V3 rxsNorm = v1.normalized().cross(v2.normalized());
        //D  rxsDot = v1.normalized().dot(v2.normalized());
        D rxsNorm_norm = utils::vector::Length(rxsNorm);
        if (rxsNorm_norm <= eps)
        {
            return false;
        }
    }


    int sign;

    // pos2 = (p2 - p1) × v1 / (v1 × v2)
    V3 u1 = utils::vector::Cross(fromP2toP1, v1);
    sign = ((utils::vector::Dot(u1, rxs)) > 0) ? 1 : -1;
    //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
    pos2 = utils::vector::Length(u1) / rxs_norm;
    pos2 = pos2 * sign;

    if (isnan(pos2) || pos2 < -eps || pos2 > epsPlus1)
    {
        return false;
    }

    //pos1 = (p2 − p1) × v2 / (v1 × v2)
    V3 t1 = utils::vector::Cross(fromP2toP1, v2);
    sign = ((utils::vector::Dot(t1, rxs)) > 0) ? 1 : -1;
    //cout << "t sign = " << sign << endl;
    pos1 = utils::vector::Length(t1) / rxs_norm;
    pos1 = pos1 * sign;

    //if (t < -eps || fabs(t) < eps)
    if (pos1 < eps)
        return false;

    return true;
};

bool segments_intersect_fast4(
    const P3 &p1, const V3 &v1,
    const P3 &p2, const V3 &v2,
    D &pos2
)
{
    const D eps = 1e-6;
    const D epsPow2 = eps * eps;
    const D epsPlus1 = 1 + eps;
    const D epsPlus1Pow2 = epsPlus1 * epsPlus1;

    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    // Search intersection between two segments
    // p1 + t*v1 :  t \in [0,1]
    // p2 + u*v2 :  u \in [0,1]

    // p1 + t * v1 = p2 + u * v2  // x v2
    // t(v1 x v2) = (p2 - p1) x v2
    // t = (p2 - p1) x v2 / (v1 x v2)

    V3 fromP2toP1 = p2 - p1;

    // (v1 x v2) ~ 0 --> directions are parallel, they will never cross

    V3 rxs = utils::vector::Cross(v1, v2);
    D rxs_normPow2 = utils::vector::LengthPow2(rxs);
    if (rxs_normPow2 <= epsPow2) // if there is suspicious to be directions parallel - test precisely
    {
        V3 rxsNorm = v1.normalized().cross(v2.normalized());
        //D  rxsDot = v1.normalized().dot(v2.normalized());
        D rxsNorm_normPow2 = utils::vector::LengthPow2(rxsNorm);
        if (rxsNorm_normPow2 <= epsPow2)
        {
            return false;
        }
    }


    int sign;

    // pos2 = (p2 - p1) × v1 / (v1 × v2)
    V3 u1 = utils::vector::Cross(fromP2toP1, v1);
    sign = ((utils::vector::Dot(u1, rxs)) > 0) ? 1 : -1;
    //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
    D pos2Pow2 = utils::vector::LengthPow2(u1) / rxs_normPow2;
    pos2Pow2 = pos2Pow2 * sign;

    if (isnan(pos2Pow2) || pos2Pow2 < -epsPow2 || pos2Pow2 > epsPlus1Pow2)
    {
        return false;
    }

    //pos1 = (p2 − p1) × v2 / (v1 × v2)
    V3 t1 = utils::vector::Cross(fromP2toP1, v2);
    sign = ((utils::vector::Dot(t1, rxs)) > 0) ? 1 : -1;
    //cout << "t sign = " << sign << endl;
    D pos1Pow2 = utils::vector::LengthPow2(t1) / rxs_normPow2;
    pos1Pow2 = pos1Pow2 * sign;

    if (pos1Pow2 < epsPow2)
        return false;


    pos2 = utils::sse::sqrt(pos2Pow2);
    if (isnan(pos2)) pos2 = 0;
    return true;
};

__forceinline bool segments_intersect_fast4_sse41(
    const P3 &p1, const V3 &v1,
    const P3 &p2, const V3 &v2,
    D &pos2
)
{
    const D eps = 1e-6;
    const D epsPow2 = eps * eps;
    const D epsPlus1 = 1 + eps;
    const D epsPlus1Pow2 = epsPlus1 * epsPlus1;

    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    // Search intersection between two segments
    // p1 + t*v1 :  t \in [0,1]
    // p2 + u*v2 :  u \in [0,1]

    // p1 + t * v1 = p2 + u * v2  // x v2
    // t(v1 x v2) = (p2 - p1) x v2
    // t = (p2 - p1) x v2 / (v1 x v2)

    V3 fromP2toP1 = p2 - p1;

    // (v1 x v2) ~ 0 --> directions are parallel, they will never cross

    V3 rxs = utils::vector::Cross(v1, v2);
    D rxs_normPow2 = utils::vector::LengthPow2_sse41(rxs);
    if (rxs_normPow2 <= epsPow2) // if there is suspicious to be directions parallel - test precisely
    {
        V3 rxsNorm = v1.normalized().cross(v2.normalized());
        //D  rxsDot = v1.normalized().dot(v2.normalized());
        D rxsNorm_normPow2 = utils::vector::LengthPow2_sse41(rxsNorm);
        if (rxsNorm_normPow2 <= epsPow2)
        {
            return false;
        }
    }


    int sign;

    // pos2 = (p2 - p1) × v1 / (v1 × v2)
    V3 u1 = utils::vector::Cross(fromP2toP1, v1);
    sign = ((utils::vector::Dot_sse41(u1, rxs)) > 0) ? 1 : -1;
    //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
    D pos2Pow2 = utils::vector::LengthPow2_sse41(u1) / rxs_normPow2;
    pos2Pow2 = pos2Pow2 * sign;

    if (isnan(pos2Pow2) || pos2Pow2 < -epsPow2 || pos2Pow2 > epsPlus1Pow2)
    {
        return false;
    }

    //pos1 = (p2 − p1) × v2 / (v1 × v2)
    V3 t1 = utils::vector::Cross(fromP2toP1, v2);
    sign = ((utils::vector::Dot_sse41(t1, rxs)) > 0) ? 1 : -1;
    //cout << "t sign = " << sign << endl;
    D pos1Pow2 = utils::vector::LengthPow2_sse41(t1) / rxs_normPow2;
    pos1Pow2 = pos1Pow2 * sign;

    if (pos1Pow2 < epsPow2)
        return false;


    pos2 = utils::sse::sqrt(pos2Pow2);
    if (isnan(pos2)) pos2 = 0;
    return true;
};

int MeshStreamer::ExtendStream(MeshStream& Stream, int extendCount, bool storeNewPointInPointsVector)
{
    int breakReason = 0;
    bool isDebug = options.show_ExtendStream_info_in_console;
    const D cos30degrees = utils::angle::DegreesToCos(30);
    const D cos45degrees = utils::angle::DegreesToCos(45);
    D max_dist_to_start_pointPow2 = 0; // protection against cyclic streams
    int max_dist_to_start_pointPow2__IsOutOfLineSign = 0;
    D max_cos_between_normals = 1; // protection against cyclic streams
    D min_cos_between_normals = 0; // protection against cyclic streams
    V3 dir0 = start_direction;
    dir0.normalize();
    V3 normal0 = start_normal;
    normal0.normalize();
    D max_cos_between_dirs = 1; // protection against cyclic streams
    D min_cos_between_dirs = 0; // protection against cyclic streams

    bool checkCyclic = extendCount > 30 && storeNewPointInPointsVector;
    int extendedTimes = 0;
    for (int i = 0; i < extendCount; i++)
    {
        if (finished) break;
        current_start_point = current_end_point;
        if (ExtendStreamDirect(Stream))
        {
            // protection against cyclic streams
            if (checkCyclic && current_fid != -1) // only if this flag is set, since in other cases we use this class manully and want a full control
            {
                D distPow2 = utils::point::DistToPointPow2(Stream[0].point, current_end_point); // take dist from newpoint to start point of the stream
                if (distPow2 > max_dist_to_start_pointPow2) max_dist_to_start_pointPow2 = distPow2; // remember max dist
                if (distPow2*(8 * 8) < max_dist_to_start_pointPow2) //  if new dist is 8 times lower from max, so we are really getting closer to start of stream
                {
                    int sign = utils::vector::ClosestPoint_ToLine_IsOutOfLineSign(current_start_point, current_end_point, Stream[0].point);
                    // break if stream is cyclic by sign to stream 
                    if (sign == 0 || (sign == -1 && max_dist_to_start_pointPow2__IsOutOfLineSign == 1)) // closest point is inside the stream, or, prev is outNext and current is outPrev - so means this stream-line is closest to point, so we reach start of the stream
                    {
                        //cout << "breaking stream#" << Id << " since it is cyclic" << endl;
                        finished = true;// finish stream to avoid cycles
                        Stream.isCyclic = true;
                        breakReason = 1;
                        break;
                    }
                    max_dist_to_start_pointPow2__IsOutOfLineSign = sign; // remember current sign
                }

                // break if stream is cyclic by normals
                V3 normal = mesh.F_normals.row(current_fid);
                D cosBetweenNormals = utils::vector::Cos(start_normal, normal, true); // take cos between start normal and current
                if (cosBetweenNormals < max_cos_between_normals) max_cos_between_normals = cosBetweenNormals;// remember max coz between normals
                if (max_cos_between_normals < -cos30degrees) // if we had almost 180 degree 
                {
                    if (cosBetweenNormals > min_cos_between_normals) min_cos_between_normals = cosBetweenNormals;
                    if (cosBetweenNormals > cos30degrees)  // and now almost 0 degree to first normal - stream is cyclic
                    {
                        finished = true;// finish stream to avoid cycles
                        Stream.isCyclic = true;
                        breakReason = 2;
                        break;
                    }
                }

                // break if stream is cyclic by dir
                D cosBetweenDirs = utils::vector::Cos(dir0, current_end_point - current_start_point); // take cos between start normal and current
                if (cosBetweenDirs < max_cos_between_dirs) max_cos_between_dirs = cosBetweenDirs;// remember max coz between normals
                if (max_cos_between_dirs < -cos45degrees) // if we had almost 180 degree 
                {
                    if (cosBetweenDirs > min_cos_between_normals) min_cos_between_normals = cosBetweenDirs;
                    if (cosBetweenDirs > cos45degrees)  // and now almost 0 degree to first normal - stream is cyclic
                    {
                        finished = true;// finish stream to avoid cycles
                        Stream.isCyclic = true;
                        breakReason = 3;
                        break;
                    }
                }

                //DEBUG
                //if (Id == 16) cout << "dist=" << distPow2 << "     maxdist=" << max_dist_to_start_pointPow2 << endl;
            }


            extendedTimes++;
            if (storeNewPointInPointsVector)
            {
                Stream.Add(StreamPoint(current_end_pointType, current_end_vid_eid_fid, current_end_point, current_fid));
            }
        }
        else
        {
            finished = true;
        }
    }


    if (isDebug && checkCyclic)
    {
        cout << "stream#" << Stream.Index << "  extended " << Stream.size() << " times";

        if (breakReason == 1)
        {
            cout << "   break reason = cyclic stream";
        }
        else if (breakReason == 2)
        {
            D angleBetweenNormalsMax = utils::angle::RadiansToDegrees(acos(max_cos_between_normals));
            D angleBetweenNormalsMin = utils::angle::RadiansToDegrees(acos(min_cos_between_normals));
            cout << "   break reason = cyclic face normal: " << "    max angle between normals = " << angleBetweenNormalsMax << "    min angle between normals = " << angleBetweenNormalsMin;
        }
        else if (breakReason == 3)
        {
            D angleBetweenDirsMax = utils::angle::RadiansToDegrees(acos(max_cos_between_dirs));
            D angleBetweenDirsMin = utils::angle::RadiansToDegrees(acos(min_cos_between_dirs));
            cout << "   break reason = cyclic dir: " << "    max angle between dirs = " << angleBetweenDirsMax << "    min angle between dirs = " << angleBetweenDirsMin;
        }
        cout << endl;
    }
    return extendedTimes;
}

II MeshStreamer::SizeOF() const
{
    II r = sizeof(MeshStreamer);

    return r;
}

bool MeshStreamer::ExtendStreamDirect(MeshStream& Stream)
{
    //bool use_sse41 = utils::cpu::isSupportedSSE4 && options.use_sse41;
    bool use_sse41 = false; // false because this doenst make visible improvement, and i just want to make usage of CPU cache smaller - instead of 2 method there will be only one
    D tol = options.snap_to_vertex_tol;
    const bool TEST = false;
    const P3s& V = mesh.V;
    const auto& F = mesh.F;
    const auto& FE = mesh.FE;
    const auto& EF = mesh.EF;

    if (nrosyField.size() == 0) return false;
    if (finished) return false;
    //DEBUG - show only one stream
    if (TEST)
    {
        if (Stream.Index != 243) return false;
        //cout << "Id = " << Id << endl;

        // test #5 stream enter middle of the edge and point to vertex with angle 3 degree
        //if (Id != 4) return false;
        //TestFile = TestFiles::Lock_door_4;
        //TestFileDensity = TestFilesDensity::medium;
    }

    int fid = current_fid;
    if (fid == -1 || current_direction == NOMATCH)// reach boundary
    {
        finished = true;
        return false;
    }
    int ni = current_direction;
    int eidStart = current_start_edgeid;
    //if (eidStart == 39636)
    //{
    //    cout << "eidStart=" << eidStart << endl;
    //}

    // the starting point of the vector
    P3 startPoint = current_start_point;
    // the direction where we are trying to go
    V3 direction;
    if (ni < 0)
    {
        direction = start_direction;
    }
    else
    {
        assert(ni <= 4);
        direction = getnrosyField(ni, fid);
    }
    //cout << "i=" << i << ",  p=" << p(0) << "," << p(1) << "," << p(2) << "," << "    v1 = " << v1(0) << ", " << v1(1) << ", " << v1(2) << endl;
    //V3 directionNonNormalized = direction;
    //direction.normalize(); // we dont need normalize direction - depend on direction length will depend result 't' from method 'segments_intersect'

    // new state,
    bool found = false;
    bool found_forced_to_vertex_because_it_was_close = false;
    D pos1 = 1111;
    D pos2 = 1111;
    int manualNext_vid = -1;
    for (int tryNum = 1; tryNum <= 2; tryNum++) //  second try to fix issue when direction is parallel to some edge - in this case we shift a start point a bit to face center
    {
        if (found) break;
        // possible casses of fail at first try
        // a) direction is parallel to edge 
        // b) direction trying to back to same edge         
        // c) direction pointing to vertex
        if (tryNum == 2)
        {
            // calculate next fid manually
            int manualfidNext;
            P3 manualEndPoint;
            V3 manualNextDirection;
            bool found_ManuallyGetNextFaceFromStartPoint = (manualNext_vid == -1)
                ? ManuallyGetNextFaceFromStartPoint(Stream, fid, startPoint, direction, manualEndPoint, manualfidNext, manualNextDirection, manualNext_vid)
                : ManuallyGetNextFaceFromStartPoint(Stream, fid, manualNext_vid, direction, manualEndPoint, manualfidNext, manualNextDirection);

            if (found_ManuallyGetNextFaceFromStartPoint)
            {
                current_end_point = manualEndPoint;
                current_fid = -1;
                current_direction = -1;
                if (manualfidNext != -1)
                {
                    current_fid = manualfidNext;
                    if (ni != -2)
                    {
                        current_direction = GetClosestNrosyFieldDirectionNi(Stream, manualfidNext, manualNextDirection);
                    }
                }
                assert(current_direction <= 4);
                current_end_pointType = MeshPointType::onVertex;
                current_end_vid_eid_fid = manualNext_vid;
                current_start_edgeid = -1;
                found = true;
                pos1 = 1111;
                pos2 = 1111;
                break;
            }

            // works only for case 'a' (when direction is parallel to edge)
            // this is last chance to find something
            P3 pCenter = mesh.F_Barycenters.row(fid);
            startPoint = startPoint + (pCenter - startPoint) / 100;  // shift point a bit to face center
            current_start_edgeid = -1;
        }

        for (int k = 0; k < 3; ++k)
        {
            int eid = FE(fid, k);
            if (eid == current_start_edgeid) continue;

            //DEBUG
            //int f1 = mesh.FF(fid, k);
            //cout << "k=" << k << ",  fid=" << fid << "    f1=" << f1  << endl;

            // edge vertices
            int edgeStartPoint_vid = F(fid, k);
            P3 edgeStartPoint = V.row(edgeStartPoint_vid);
            int qs_vid = F(fid, (k + 1) % 3);
            P3 qs = V.row(qs_vid);
            // edge direction
            V3 edgeDirection = qs - edgeStartPoint;
            //edgeDirection.normalize(); - do not normalize - algorithm segments_intersect will not work correct!
            //if (igl::segments_intersect(startPoint, direction, edgeStartPoint, edgeDirection, t, u))
            bool areIntersects = use_sse41
                ? segments_intersect_fast4_sse41(startPoint, direction, edgeStartPoint, edgeDirection, pos2)
                : segments_intersect_fast4(startPoint, direction, edgeStartPoint, edgeDirection, pos2);
            if (areIntersects)
            {
                bool is_pos2_very_close_to_vertex = pos2 < tol || 1 - tol < pos2; // disallow end point on vertexes - just fail in such case and move calculation on tryNum == 2         
                int outside_vid = (pos2 < 0.5) ? edgeStartPoint_vid : qs_vid; // set vid for tryNum2 step - we will try to find next direction from vertex
                // check if we can move to point
                if (is_pos2_very_close_to_vertex)
                {
                    if (Stream.Points.size() >= 1)
                    {
                        I2 start__vids(-1, -1);
                        if (Stream.Points[0].Type == MeshPointType::onEdge) start__vids = mesh.EV.row(Stream.Points[0].vid_eid_fid);
                        if (Stream.Points[0].Type == MeshPointType::onVertex) start__vids = I2(Stream.Points[0].vid_eid_fid, -1);
                        if (outside_vid == start__vids(0) || outside_vid == start__vids(1))
                        {
                            is_pos2_very_close_to_vertex = false;
                        }
                    }
                }

                if (is_pos2_very_close_to_vertex)
                {
                    manualNext_vid = outside_vid; // set vid for tryNum2 step - we will try to find next direction from vertex
                    found_forced_to_vertex_because_it_was_close = true;
                    if (manualNext_vid == 48660)
                    {
                        //cout << "manualNext_vid=" << manualNext_vid << endl;
                        int temp = 0;
                    }
                }
                else
                {
                    // point on next face
                    //end_point = startPoint + t * direction; // for segments_intersect_fast3
                    current_end_point = edgeStartPoint + pos2 * edgeDirection; // for segments_intersect_fast4
                    current_end_pointType = MeshPointType::onEdge;
                    current_end_vid_eid_fid = eid;
                    int fidNext = mesh.FF(fid, k);
                    current_fid = fidNext;
                    if (fidNext != -1)
                    {
                        //DEBUG - check if we need normalize 'direction' - no we dont need
                        //if (directionNonNormalized.norm() < 0.98)
                        //{
                        //    D t2;
                        //    segments_intersect_fast3(startPoint, directionNonNormalized, edgeStartPoint, edgeDirection, t2);
                        //    
                        //    cout << "t=" << t << ",  t2=" << t2 << "   dist diff = " << (t * direction - t2 * directionNonNormalized).norm() << endl;
                        //}

                        //if (start_edgeid == 11137)
                        //{
                        //    cout << "start_edgeid=" << start_edgeid << "   next eid=" << eid << " at pos " << pos2 << "   vid0=" << F(fid, k) << "  vid1="<< F(fid, (k + 1) % 3) << endl;;
                        //}

                        //int fidnext = current_faces(i);
                        //V3 startPoint2 = end_points.row(i);
                        //D u2;
                        //D t2;
                        //igl::segments_intersect(startPoint2, direction, q, edgeDirection, t2, u2);
                        //D u3;
                        //D t3;
                        //int ninext = GetClosestNrosyFieldDirectionNi(fidnext, direction);
                        //V3 directionNext = nrosyField[ninext].row(fidnext);
                        //igl::segments_intersect(startPoint2, directionNext, q, edgeDirection, t3, u3);
                        //directionNext.normalize();
                        //D angle = utils::vector::Angle(direction, directionNext);
                        //D angleToNormal = utils::vector::Angle(mesh.F_normals.row(fid).transpose(), direction);
                        //D angleToNormalNext = utils::vector::Angle(mesh.F_normals.row(fidnext).transpose(), directionNext);
                        //P3 p1 = V.row(F(fid, k));
                        //P3 p2 = V.row(F(fid, (k + 1) % 3));
                        //D startPointDist = mesh.EdgeDistToPoint(mesh.FE(fid, k), startPoint);
                        //D startPoint2Dist = mesh.EdgeDistToPoint(mesh.FE(fid, k), startPoint2);


                        // matching direction on next face
                        if (ni < 0)
                        {
                            if (ni == -1)
                            {
                                current_direction = GetClosestNrosyFieldDirectionNi(Stream, fidNext, direction);
                                assert(current_direction <= 4);
                            }
                        }
                        else
                        {
                            if (mesh.EF(eid, 0) == fid)
                                current_direction = match_ab_ba(eid).ab(ni);
                            else
                                current_direction = match_ab_ba(eid).ba(ni);
                            assert(current_direction <= 3);
                        }
                        current_start_edgeid = eid; // next face will start from this edge - and will skipp it at check - since we can go to same edge from what we came from
                    }
                    else
                    {
                        current_start_edgeid = -1;
                    }
                    manualNext_vid = -1; // clear this flag since it can be setted on some step 'for (int k = 0; k < 3; ++k)'
                    found = true;
                    break;
                }
            }
        }
    }

    //DEBUG - show numbers 
    if (TEST)
    {
        //if (fid == 96)
        //{
        //    P3 pCenter = mesh.F_Barycenters.row(fid);
        //    draw.AddEdge(pCenter, pCenter + direction);
        //}
        if (fid == 10)
        {
            draw.AddEdge(startPoint, startPoint + direction, Color3d(1, 0, 0));
            V3 f57_dir0 = nrosyField[0].row(57);
            V3 f57_dir1 = nrosyField[1].row(57);
            V3 f10_dir0 = nrosyField[0].row(10);
            V3 f10_dir1 = nrosyField[1].row(10);
            V3 next_direction = nrosyField[current_direction].row(current_fid);
            P3 pCenter = mesh.F_Barycenters.row(current_fid);
            draw.AddEdge(pCenter, pCenter + next_direction, Color3d(0, 0, 1));
            found = false;
        }
        draw.AddLabel(current_end_point, "         " + to_string(Stream.Index));
        draw.AddPoint(current_end_point, Color3d(0, 0, 1));
        cout << "fid=" << fid << ",  eid=" << eidStart << "    ***   fid next=" << current_fid << "    nexteid=" << current_start_edgeid << "    next ni=" << current_direction << "    pos1=" << pos1 << "    pos2=" << pos2 << endl;
    }
    if (current_fid == -1
        && (!mesh.V_isborder[mesh.F(fid, 0)]) && !mesh.V_isborder[mesh.F(fid, 1)] && !mesh.V_isborder[mesh.F(fid, 2)])
    {
        cout << "!!! ExtendStream for stream #" << Stream.Index << "  failed in face " << fid << endl;

        //draw.AddLabel(mesh.F_Barycenters.row(fid), "failed to extend stream for face " + to_string(fid));
    }


    //
    // we forced to go stream trought vertex - now we have to remove all reduntant points from stream before this vertex
    // this is very important since it removes big angles jumps and also amost redundant points that complecates work with stream
    //
    auto isEdgeConnectesToVertex = [&](int eid, int vid)
    {
        return mesh.EV(eid, 0) == vid || mesh.EV(eid, 1) == vid;
    };
    auto isFaceConnectesToVertex = [&](int fid, int vid)
    {
        return mesh.F(fid, 0) == vid || mesh.F(fid, 1) == vid || mesh.F(fid, 2) == vid;
    };
    if (found && manualNext_vid != -1)
    {
        int countOfStreamPointsThatConnectsToVertex = 0;
        for (int i = Stream.Points.size() - 1; i >= 0; i--)
        {
            const StreamPoint& p = Stream.Points[i];
            bool connects = false;
            switch (p.Type)
            {
                case MeshPointType::onVertex:
                    for (int k = 0; k < mesh.VE.size(p.vid_eid_fid); k++)
                    {
                        int eidk = mesh.VE(p.vid_eid_fid, k);
                        if (isEdgeConnectesToVertex(eidk, manualNext_vid))
                        {
                            connects = true;
                            break; // break iterating edges that connects to vertex
                        }
                    }
                    break;
                case MeshPointType::onEdge:
                    connects = isEdgeConnectesToVertex(p.vid_eid_fid, manualNext_vid);
                    if (p.fid != -1 && isFaceConnectesToVertex(p.fid, manualNext_vid))
                    {
                        connects = true;
                    }
                    break;
                default:
                    connects = false;
                    break;
            }
            if (connects)
            {
                countOfStreamPointsThatConnectsToVertex++;
            }
            else
            {
                break; // stop searching when some point is not move connected to vertex
            }

        }

        // remove points from stream
        if (countOfStreamPointsThatConnectsToVertex > 1)
        {
            //cout << "manualNext_vid="<< manualNext_vid<<"    countOfStreamPointsThatConnectsToVertex = " << countOfStreamPointsThatConnectsToVertex << endl;
            int removePointsCount = countOfStreamPointsThatConnectsToVertex - 1;
            if (removePointsCount >= Stream.Points.size()) // we cant remove all points!:) at least start points should stay
            {
                removePointsCount = Stream.Points.size() - 1;
            }
            Stream.Remove(Stream.Points.size() - removePointsCount, removePointsCount);
        }
    }

    //cout << (end_pointType == MeshPointType::onVertex ? "vid=" : "eid=") << end_vid_eid_fid << endl;
    return found;
}


int MeshStreamer::GetClosestNrosyFieldDirectionNi(MeshStream& Stream, int faceId, const V3& direction)
{
    int dirId;
    if (IsNrosyFieldSymetric && nrosyField.size() == 2 && utils::cpu::isSupportedSSE4)
    {
        V3 dir0 = getnrosyField(0, faceId);
        V3 dir1 = getnrosyField(1, faceId);
        //__m128 dirs_sqrt = _mm_sqrt_ss(_mm_set_ps(dir0.normPow2_sse41(), 1, dir1.normPow2_sse41(), 1));
        //dir0 /= dirs_sqrt.m128_f32[0];
        //dir1 /= dirs_sqrt.m128_f32[2];
        D dir0normPow2 = dir0.normPow2_sse41();
        D dir1normPow2 = dir1.normPow2_sse41();
        D dot0 = utils::vector::Dot_sse41(dir0, direction);
        D dot1 = utils::vector::Dot_sse41(dir1, direction);
        D cmp0 = (dot0 * dot0)*dir1normPow2; // instead of (dot0*dot0)/dir0normPow2
        D cmp1 = (dot1 * dot1)*dir0normPow2;
        if (dot0 < 0) cmp0 = -cmp0;
        if (dot1 < 0) cmp1 = -cmp1;
        D maxcmp = cmp0;
        dirId = 0;
        if (cmp1 > maxcmp)
        {
            maxcmp = cmp1;
            dirId = 1;
        }
        if (-cmp0 > maxcmp)
        {
            maxcmp = -cmp0;
            dirId = 2;
        }
        if (-cmp1 > maxcmp)
        {
            dirId = 3;
        }
    }
    else
    {
        //cout << "direction.norm() = " << direction.norm() << endl;
        int dirId_slow = -1;
        D minDirAngle = 0;
        for (int ni = 0; ni < nrosyField.size(); ni++)
        {
            V3 fieldDirN = getnrosyField(ni, faceId);
            //cout << "    fieldDirN.norm() = " << fieldDirN.norm() << endl;
            D angle = utils::vector::Angle(direction, fieldDirN); // TODO can be optimized using 'IsNrosyFieldSymetric' flag - we can use symetric 
            if (angle < minDirAngle || dirId_slow == -1)
            {
                dirId_slow = ni;
                minDirAngle = angle;
            }
            if (IsNrosyFieldSymetric)
            {
                int niSymetric = 2 + ni; // symetric ni
                D angleSymetric = 180 - angle;
                if (angleSymetric < minDirAngle || dirId_slow == -1)
                {
                    dirId_slow = niSymetric;
                    minDirAngle = angleSymetric;
                }
            }
        }
        dirId = dirId_slow;
    }
    // DEBUG test correctness of fast method
    //if (dirId != dirId_slow)
    //{
    //    cout << "dirId=" << dirId << "    dirId_slow=" << dirId_slow << endl;
    //}
    return dirId;
}

bool MeshStreamer::ManuallyGetNextFaceFromVertex(MeshStream& Stream, int fid, int vid, const V3& direction, const P3& endPoint, bool TEST, int& nextfid, V3& nextdirection)
{
    const P3s& V = mesh.V;
    const auto& F = mesh.F;
    const auto& FE = mesh.FE;

    // set results
    nextfid = -1;
    nextdirection = direction;

    //
    //2) get closest face to direction ( this section can fail - this mean this will be the last stream iteration - stream will finish)
    //
    int bestfid = -1;
    D bestfid_angleDiff = 0;
    for (int fi = 0; fi < mesh.VF.size(vid); fi++)
    {
        int nfid = mesh.VF(vid, fi);
        if (nfid == fid) continue;
        V3 directionOnNextface = utils::vector::Translate(direction, mesh.F_normals.row(fid), mesh.F_normals.row(nfid), true);
        D angleDiff;
        if (vid == 48433 && nfid == 95576)
        {
            int temp = 0;
        }
        mesh.IsDirectionInsideFace(nfid, vid, directionOnNextface, angleDiff);
        if (angleDiff > bestfid_angleDiff || bestfid == -1)
        {
            bestfid = nfid;
            bestfid_angleDiff = angleDiff;
        }
    }
    // fix tolerance - when using utils::vector::Translate to new face direction can be sligtly changed and always outside of faces - let set 3 degree for tolerance
    if (bestfid_angleDiff < 0 && !mesh.V_isborder[vid])
    {
        if (-3 < bestfid_angleDiff)
        {
            if (TEST)
            {
                cout << "   bestfid_angleDiff  = " << bestfid_angleDiff << "   and will be set to zero as tolerance fix" << endl;
            }
            bestfid_angleDiff = 0;
        }
        else
        {
            cout << "! warning:  ManuallyGetNextFaceFromVertex   cant find proper face, angle is negative:  bestfid_angleDiff=" << bestfid_angleDiff << "   fid=" << fid << "   vid=" << vid << endl;
            #if DEBUG
            draw.AddPoint(mesh.V.row(vid), Color3d(1, 0, 0), "vid=" + to_string(vid) + "   bestfid_angleDiff=" + to_string(bestfid_angleDiff));
            #endif
        }
    }


    // if direction inside face (should be >=0 but we set some tolerance angle)
    if (TEST && bestfid_angleDiff < 0 && !mesh.V_isborder[vid])
    {
        cout << "!!! bestfid_angleDiff  = " << bestfid_angleDiff << "    in fid=" << fid << "  vid=" << vid << endl;
        draw.AddPoint(V.row(vid), Color3d(1, 0, 0));
        draw.AddLabel(V.row(vid), "     bestfid_angleDiff " + to_string(bestfid_angleDiff), Color3d(1, 0, 0));
        draw.AddEdge(V.row(vid), V.row(vid).transpose() + direction * mesh.avg_edge_length / 2, Color3d(1, 0, 0));
        draw.AddLabel(V.row(vid).transpose() + direction * mesh.avg_edge_length / 2, "direction", Color3d(1, 0, 0));
        for (int fi = 0; fi < mesh.VF.size(vid); fi++)
        {
            int nfid = mesh.VF(vid, fi);
            if (nfid == fid)
            {
                cout << "!!! nfid " << nfid << " skipped because if it euqal to fid" << endl;
                continue;
            }
            V3 directionOnNextface = utils::vector::Translate(direction, mesh.F_normals.row(fid), mesh.F_normals.row(nfid), true);
            D angleDiff;
            mesh.IsDirectionInsideFace(nfid, vid, directionOnNextface, angleDiff);
            cout << "!!! nfid " << nfid << "    angleDiff = " << angleDiff << endl;
            if (angleDiff > bestfid_angleDiff || bestfid == -1)
            {
                bestfid = nfid;
                bestfid_angleDiff = angleDiff;
            }
        }
        cout << "!!! bestfid " << bestfid << "    bestfid_angleDiff = " << bestfid_angleDiff << endl;
        V3 directionOnNextface = utils::vector::Translate(direction, mesh.F_normals.row(fid), mesh.F_normals.row(bestfid), true);
        draw.AddEdge(V.row(vid), V.row(vid).transpose() + directionOnNextface * mesh.avg_edge_length / 2, Color3d(0, 1, 0));
        draw.AddLabel(V.row(vid).transpose() + direction * mesh.avg_edge_length / 2, "directionOnNextface", Color3d(0, 1, 0));
        cout << "angle between direction and fid " << utils::vector::Angle(mesh.F_normals.row(fid), direction) << endl;
        cout << "angle between directionOnNextface and bestfid " << utils::vector::Angle(mesh.F_normals.row(bestfid), directionOnNextface) << endl;
        cout << "angle between directionOnNextface and direction " << utils::vector::Angle(direction, directionOnNextface) << endl;
        cout << "angle between normals fid and bestfid " << utils::vector::Angle(mesh.F_normals.row(fid), mesh.F_normals.row(bestfid)) << endl;
        draw.AddPoint(mesh.F_Barycenters.row(bestfid), Color3d(1, 0, 0));
        D angleDiff;
        mesh.IsDirectionInsideFace(bestfid, vid, directionOnNextface, angleDiff);
    }

    //if (bestfid != -1 && bestfid_angleDiff > -0.01)
    if (bestfid != -1)
    {
        nextfid = bestfid;
        V3 directionOnNextface = utils::vector::Translate(direction, mesh.F_normals.row(fid), mesh.F_normals.row(bestfid), true);
        nextdirection = directionOnNextface;
        //DEBUG show next face
        //if (TEST)
        //{
        //    draw.AddPoint(mesh.F_Barycenters.row(bestfid), Color3d(1, 0, 0));
        //    draw.AddLabel(mesh.F_Barycenters.row(bestfid), "     Next Face " + to_string(bestfid) + "   with angle diff " + utils::angle::ToString(bestfid_angleDiff), Color3d(1, 0, 0));
        //    draw.AddEdge(endPoint, endPoint + directionOnNextface.normalized()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}
        return true;
    }

    return false;
}

MeshStreamerUV::MeshStreamerUV(ViewerDrawObjects& _draw, const Mesh& _mesh, const vector<UV>& _uvField)
    :draw(_draw), mesh(_mesh), V(mesh.V.data()), F((I3*)mesh.F.data()), FE((I3*)mesh.FE.data()), EF((I2*)mesh.EF.data()), uvField(_uvField),
    start_direction(), end_direction(), current_fid(-1), current_start_direction(), current_end_direction(), finished(false)
{
}

void MeshStreamerUV::RestartStream(MeshStream& Stream, const MeshUV& uvBegin, const UV& uvEnd)
{
    start_direction = uvBegin;
    end_direction = uvEnd;
    current_fid = (uvBegin.Type == MeshPointType::onEdge) ? mesh.EF(uvBegin.vid_eid_fid, 0) : -1;
    current_start_direction = uvBegin;
    current_end_direction = uvBegin;
    finished = false;
    if (Stream.size() == 1)
    {
        Stream[0] = StreamPoint(uvBegin.Type, uvBegin.vid_eid_fid, uvBegin.point, current_fid);
    }
    else
    {
        Stream.Points.clear();
        Stream.Add(StreamPoint(uvBegin.Type, uvBegin.vid_eid_fid, uvBegin.point, current_fid));
    }
}

int MeshStreamerUV::ExtendStream(MeshStream& Stream, int extendCount, bool storeNewPointInPointsVector)
{
    int extendedTimes = 0;
    if (uvField.size() != 0)
    {
        for (int i = 0; i < extendCount; i++)
        {
            if (finished) break;
            current_start_direction = current_end_direction;
            if (ExtendStreamDirect(Stream))
            {
                extendedTimes++;
                if (storeNewPointInPointsVector)
                {
                    Stream.Add(StreamPoint(current_end_direction.Type, current_end_direction.vid_eid_fid, current_end_direction.point, current_fid));
                }
            }
            else
            {
                finished = true;
            }
        }
    }
    return extendedTimes;
}

II MeshStreamerUV::SizeOF() const
{
    II r = sizeof(MeshStreamerUV);

    return r;
}

bool MeshStreamerUV::ExtendStreamDirect(MeshStream& Stream)
{
    D tol = options.snap_to_vertex_tol;
    const bool TEST = false;

    if (finished) return false;
    //DEBUG - show only one stream
    if (TEST)
    {
        if (Stream.Index != 243) return false;
        //cout << "Id = " << Id << endl;

        // test #5 stream enter middle of the edge and point to vertex with angle 3 degree
        //if (Id != 4) return false;
        //TestFile = TestFiles::Lock_door_4;
        //TestFileDensity = TestFilesDensity::medium;
    }
    if (current_end_direction.isOnBorder(mesh) && current_end_direction.vid_eid_fid != start_direction.vid_eid_fid)// reach boundary
    {
        finished = true;
        return false;
    }

    bool found = false;
    P3 end = end_direction.toP3();
    P3 start = start_direction.uv.toP3();
    int best_fid = -1;
    MeshUV best_edge_uv;
    MeshUV best_vertext_uv;
    D best_distPow2_to_endUV = UV::DistPow2(current_start_direction.uv, end_direction);
    auto check = [&](int eid, int vid0, int vid1) -> bool
    {
        UV uv0 = uvField[vid0];
        UV uv1 = uvField[vid1];
        P3 uv0P3 = uv0.toP3();
        P3 uv1P3 = uv1.toP3();
        D pos2;
        if (utils::vector::FindIntersectionBetween_Vectors(start, end - start, uv0P3, uv1P3 - uv0P3, pos2))
        {
            //if (pos2 < (D)0) pos2 = (D)0;
            //if (pos2 > (D)1) pos2 = (D)1;
            if (pos2 < (D)0 || pos2 >(D)1) return false;
            UV closestPointUV = uv0 + (uv1 - uv0)*pos2;
            D distPow2 = utils::point::DistToPointPow2(closestPointUV.toP3(), end);
            if (distPow2 < best_distPow2_to_endUV)
            {
                P3 v0 = V[vid0];
                P3 v1 = V[vid1];
                P3 closestPoint3d = v0 + (v1 - v0)*pos2;
                best_edge_uv = MeshUV(MeshPointType::onEdge, eid, closestPoint3d, { closestPointUV[0], closestPointUV[1] });
                best_distPow2_to_endUV = distPow2;
                return true;
            }
        }
        return false;
    };
    if (current_start_direction.Type == MeshPointType::onEdge)
    {
        int eid = current_start_direction.vid_eid_fid;
        //I2 edge_vids = mesh.EV.row(eid);
        //int edge_vid0 = edge_vids(0);
        //int edge_vid1 = edge_vids(1);
        //if (edge_vid1 < edge_vid0) swap(edge_vid0, edge_vid1);
        I2 edge_fids = (current_fid == -1)
            ? EF[eid]
            : I2(current_fid, -1);
        for (int i = 0; i < 2; i++)
        {
            int fid = edge_fids(i);
            if (fid == -1) continue;
            I3 face = F[fid];
            I3 face_eids = FE[fid];
            for (int k = 0; k < 3; k++)
            {
                int face_eid = face_eids(k);
                if (face_eid == eid) continue;
                int face_vid0 = face(k);
                int face_vid1 = face((k + 1) % 3);
                if (face_vid1 < face_vid0) swap(face_vid0, face_vid1);
                if (check(face_eids(k), face_vid0, face_vid1))
                {
                    best_fid = fid;
                }
            }
        }
    }
    else if (current_start_direction.Type == MeshPointType::onVertex)
    {
        int vid = current_start_direction.vid_eid_fid;
        cout << "!warning    MeshStreamerUV::ExtendStreamDirect    if (dir.Type == MeshPointType::onVertex)   not implemented" << endl;
        //for(int i = 0; i < mesh.V
    }
    if (!best_edge_uv.isUndefined())
    {
        found = true;
        current_fid = -1;
        if (best_edge_uv.Type == MeshPointType::onEdge)
        {
            I2 edge_fids = EF[best_edge_uv.vid_eid_fid]; // get faces connected to edge
            if (edge_fids(0) != best_fid) current_fid = edge_fids(0); // get opposite face of tested face
            if (edge_fids(1) != best_fid) current_fid = edge_fids(1); // get opposite face of tested face
        }
        current_end_direction = best_edge_uv;
    }
    else if (!best_vertext_uv.isUndefined())
    {
        found = true;
        current_end_direction = best_vertext_uv;
    }

    // DEBUG show points
    //cout << "#" << Stream.Index << "." << Stream.size() << "   current_start_direction=" << current_start_direction.toString() << "   start_dist=" << currentStartDist << "               new_dist=" << sqrt(best_distPow2_to_endUV) << "   best_edge_uv=" << best_edge_uv.toString() << endl;
    //draw.AddEdge(current_start_direction.point, current_end_direction.point, Color3d(1,0,0), "#" + to_string(Stream.Index) + "." + to_string(Stream.size()));
    //cout << "#" << Stream.Index << "." << Stream.size()<< "   currentDist=" << currentDist << endl;

    // Test end point correctness
    #if DEBUG
    if (found && current_end_direction.isOnBorder(mesh))
    {
        D currentDistPow2 = UV::DistPow2(current_end_direction.uv, end_direction);
        D currentDist_tol = 0.01; // 1%
        D currentDist_tolPow2 = currentDist_tol * currentDist_tol;
        if (currentDistPow2 > currentDist_tolPow2)
        {
            assert(currentDistPow2 < currentDist_tolPow2 && "current_end_direction should be very close to end_direction");
            cout << "!warning   MeshStreamerUV::ExtendStreamDirect   current_end_direction should be very close to end_direction but is " << sqrt(currentDistPow2) << endl;
        }
        //current_end_direction.uv = end_direction;
    }
    #endif

    return found;
}

bool MeshStreamer::ManuallyGetNextFaceFromStartPoint(MeshStream& Stream, int fid, int vid, const V3& direction, P3& endPoint, int& nextfid, V3& nextdirection)
{
    const bool TEST = false;
    const P3s& V = mesh.V;

    if (TEST)
    {
        cout << endl;
        cout << "     ManuallyGetNextFaceFromStartPoint    fid = " << fid << endl;
    }

    if (TEST)
    {
        cout << "     vid  " + to_string(vid) << endl;
    }

    endPoint = V.row(vid);
    //DEBUG show next vertex
    //if (TEST)
    //{
    //    draw.AddPoint(V.row(vid), Color3d(1, 0, 0));
    //    draw.AddLabel(V.row(vid), "     Next Vertex " + to_string(vid), Color3d(1, 0, 0));
    //}

    // finish stream line if we reach border
    if (mesh.V_isborder[vid])
    {
        nextfid = -1;
        return true;
    }


    if (ManuallyGetNextFaceFromVertex(Stream, fid, vid, direction, endPoint, TEST, nextfid, nextdirection))
    {
        //cout << "ManuallyGetNextFace    manualNextDirection ni =" << current_directions(i) << ",  fid next=" << current_faces(i) << endl;
        // if we found next face - lets check if new-face-direction is'nt pointing to friend face - in this case lets choise that face and that face-friend-direction
        if (nextfid != -1)
        {
            int nextdirection_ni = GetClosestNrosyFieldDirectionNi(Stream, nextfid, nextdirection);
            V3 newdirection = getnrosyField(nextdirection_ni, nextfid); // what direction will be on new face
            V3 newdirectionOnCurrentFace = utils::vector::Translate(newdirection, mesh.F_normals.row(nextfid), mesh.F_normals.row(fid), true);// translate next direction to current face space
            int manualfidNext_friend;
            P3 manualEndPoint_friend_temp;
            V3 manualNextDirection_friend;
            if (false)
            {
                draw.AddEdge(V.row(vid), V.row(vid).transpose() + newdirectionOnCurrentFace * mesh.avg_edge_length / 2, Color3d(0, 1, 0));
                draw.AddLabel(V.row(vid).transpose() + newdirectionOnCurrentFace * mesh.avg_edge_length / 2, "newdirectionOnCurrentFace", Color3d(0, 1, 0));
            }
            if (ManuallyGetNextFaceFromVertex(Stream, fid, vid, newdirectionOnCurrentFace, manualEndPoint_friend_temp, TEST, manualfidNext_friend, manualNextDirection_friend))
            {
                if (manualfidNext_friend != -1
                    && manualfidNext_friend != nextfid
                    && mesh.CommonEdgeId_FaceFace(manualfidNext_friend, nextfid) != -1)
                {
                    nextfid = manualfidNext_friend;
                    nextdirection = manualNextDirection_friend;
                }
            }
        }
    }

    return true;
}


bool MeshStreamer::ManuallyGetNextFaceFromStartPoint(MeshStream& Stream, int fid, const P3& startPoint, const V3& direction, P3& endPoint, int& nextfid, V3& nextdirection, int& vid)
{
    const bool TEST = false;
    const P3s& V = mesh.V;

    if (TEST)
    {
        cout << endl;
        cout << "     ManuallyGetNextFaceFromStartPoint    fid = " << fid << endl;
    }

    int vidDirBest = mesh.GetClosestVertexIdFromDirectionInsideFace(fid, startPoint, direction);
    D vid_distPow2;
    int vidDistBest = mesh.GetClosestVertexIdFromPoint(fid, startPoint, vid_distPow2);

    V3 tovDirBest = V.row(vidDirBest).transpose() - startPoint;
    D angleDirBest = utils::vector::Angle(tovDirBest, direction);
    V3 tovDistBest = V.row(vidDistBest).transpose() - startPoint;
    D angleDistBest = utils::vector::Angle(tovDistBest, direction);
    D dist_tovDirBest = tovDirBest.norm();
    D dist_tovDistBest = tovDistBest.norm();
    if (dist_tovDistBest < 0.00000000001) dist_tovDistBest = 0.00000000001;

    // choise best vertex
    vid = vidDirBest;
    if (vidDirBest != vidDistBest  // if 'closest vertex' and 'closest to dir vertex' are different
        && angleDirBest > 30 // and 'closest to dir vertex' is not exactly on the direction
        && angleDistBest > -1 // and start point is not same as vidDistBest
        && angleDistBest < 90
        && dist_tovDirBest / dist_tovDistBest > 5) // and 'closest vertex' is much closer to startPoint compare to 'closest to dir vertex'
    {
        vid = vidDistBest; // then we will choise 'closest vertex' to avoid far jumps to far vertex
        if (TEST)
        {
            cout << "     vid  is set to vidDistBest " + to_string(vid) << "  for fid " << fid << endl << endl;
        }
    }

    if (TEST && vid == vidDistBest && vidDirBest != vidDistBest)//
    {
        draw.AddPoint(V.row(vidDirBest), Color3d(1, 0, 0));
        draw.AddLabel(V.row(vidDirBest), "     vidDirBest " + to_string(vidDirBest) + "  with dist " + to_string(dist_tovDirBest) + " with angle to direction  " + utils::angle::ToString(angleDirBest), Color3d(1, 0, 0));
        draw.AddPoint(V.row(vidDistBest), Color3d(1, 0, 0));
        draw.AddLabel(V.row(vidDistBest), "     vidDistBest  " + to_string(vidDistBest) + "  with dist " + to_string(dist_tovDistBest) + " with angle to direction  " + utils::angle::ToString(angleDistBest), Color3d(1, 0, 0));
        cout << "     vidDirBest " + to_string(vidDirBest) + "  with dist " + to_string(dist_tovDirBest) + " with angle to direction  " + utils::angle::ToString(angleDirBest) << endl;
        cout << "     vidDistBest  " + to_string(vidDistBest) + "  with dist " + to_string(dist_tovDistBest) + " with angle to direction  " + utils::angle::ToString(angleDistBest) << endl;
    }


    return ManuallyGetNextFaceFromStartPoint(Stream, fid, vid, direction, endPoint, nextfid, nextdirection);
}



//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//-------- Streams ----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
vector<V3s> dummy_nrosyField;
MatchABBAs dummy_ABBAs;
vector<UV> dummy_uvs;
MeshStreams::MeshStreams(ViewerDrawObjects& _draw, const Mesh& _mesh, const MeshSolverNrosy& solver)
    : draw(_draw), mesh(_mesh), nrosyField(solver.Result.Field), IsNrosyFieldSymetric(solver.Result.IsFieldSymetric), match_ab_ba(solver.Result.Field_match_ab_ba)
    , uvs(dummy_uvs)
{
}

MeshStreams::MeshStreams(ViewerDrawObjects& _draw, const Mesh& _mesh, const MeshSolverUV& solver)
    : draw(_draw), mesh(_mesh), nrosyField(dummy_nrosyField), IsNrosyFieldSymetric(false), match_ab_ba(dummy_ABBAs)
    , uvs(solver.Result.Field)
{
}


void MeshStreams::Reserve(int count)
{
    streams.reserve(streams.capacity() + count);
}

void MeshStreams::ClearStreams()
{
    streams.clear();
    streamers.clear();
    streamersUV.clear();
}

void MeshStreams::Add(int streamStartPointId, int streamAnchorPointId, int fid, MeshPointType pointType, int vid_eid_fid, const P3& point, const V3& direction, const Color3d& color, DividerIteratorStreamInfo dividingIteration)
{
    int streamIndex = streams.size();
    streams.push_back(MeshStream(streamIndex, streamStartPointId, streamAnchorPointId, draw, mesh, color, dividingIteration));
    streamers.push_back(MeshStreamer(draw, mesh, nrosyField, IsNrosyFieldSymetric, match_ab_ba));
    streamers.back().RestartStream(streams.back(), fid, pointType, vid_eid_fid, point, direction);
}

void MeshStreams::Add(int streamStartPointId, int streamAnchorPointId, const MeshUV& uvStart, const UV& uvEnd, const Color3d& color, DividerIteratorStreamInfo dividingIteration)
{
    int streamIndex = streams.size();
    streams.push_back(MeshStream(streamIndex, streamStartPointId, streamAnchorPointId, draw, mesh, color, dividingIteration));
    streamersUV.push_back(MeshStreamerUV(draw, mesh, uvs));
    streamersUV.back().RestartStream(streams.back(), uvStart, uvEnd);
}


int MeshStreams::Count() const
{
    return streams.size();
}

II MeshStreams::SizeOF() const
{
    II r = sizeof(MeshStreams);
    for (const auto &s : streams) r += s.SizeOF();
    for (const auto &s : streamers) r += s.SizeOF();
    return r;
}


int MeshStreams::ExtendStreams(int extendCount, bool storeNewPointInPointsVector, int preserveStreamsUntilDividingIteration)
{
    for (int i = 0; i < Count(); ++i)
    {
        if (preserveStreamsUntilDividingIteration != -1 && streams[i].dividingIteration.iterationNum <= preserveStreamsUntilDividingIteration) continue; //  skip some  streams from extending
        streams[i].reserve(streams[i].Points.capacity() + min(extendCount, 1000));
    }

    int extendedTimes = 0;
    if (streamers.size() != 0)
    {
        for (int i = 0; i < Count(); i++)
        {
            if (preserveStreamsUntilDividingIteration != -1 && streams[i].dividingIteration.iterationNum <= preserveStreamsUntilDividingIteration) continue; //  skip some  streams from extending
            extendedTimes += streamers[i].ExtendStream(streams[i], extendCount, storeNewPointInPointsVector);
            if (extendCount != 1) streams[i].shrink_to_fit();
        }
    }
    if (streamersUV.size() != 0)
    {
        for (int i = 0; i < Count(); i++)
        {
            if (preserveStreamsUntilDividingIteration != -1 && streams[i].dividingIteration.iterationNum <= preserveStreamsUntilDividingIteration) continue; //  skip some  streams from extending
            extendedTimes += streamersUV[i].ExtendStream(streams[i], extendCount, storeNewPointInPointsVector);
            if (extendCount != 1) streams[i].shrink_to_fit();
        }
    }
    return extendedTimes;
}


