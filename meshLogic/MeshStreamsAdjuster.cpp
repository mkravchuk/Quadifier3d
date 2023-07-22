#include "stdafx.h"
#include "MeshStreamsAdjuster.h"
#include "Divider.h"
#include "MeshStreams.h"

MeshLogicOptions_MeshStreamsAdjuster& options = meshLogicOptions.StreamAdjuster;

const D rotationDegree = 1;

MeshStreamsAdjuster::MeshStreamsAdjuster(const Mesh& _mesh, const MeshSolverNrosy& _solver, ViewerDrawObjects& _draw)
    : mesh(_mesh), solver(_solver), draw(_draw), callsCount_directMethod(0), callsCount_iterativeMethod(0)
{
}

bool MeshStreamsAdjuster::AdjustStream(const P3& connectionPoint, StreamStartPoint& sp, MeshStream& origin_points, MeshStream& adjusted_points)
{
    //options.DebugEnabled = true;
    //options.debug_debugStreamId = 1;

    bool res = false;
    //auto time = utils::time::Now();

    origin_points.Init_lengthToNextPoint();
    if (options.debug_TestPerformanceCall1000Times)
    {
        bool saveDebugEnabled = options.DebugEnabled;
        options.DebugEnabled = false;
        for (int i = 0; i < 999; i++)
        {
            MeshStream adjusted_points_temp(adjusted_points.Index, sp.id, sp.StreamAnchorPointId, draw, mesh, adjusted_points.color, -1);
            AdjustStream_Ditsts(connectionPoint, origin_points, sp, adjusted_points_temp);
        }
        options.DebugEnabled = saveDebugEnabled;
    }
    switch (options.Algorithm)
    {
        //case MeshLogicOptions_MeshStreamsAdjuster::AlgorithmType::StartAngle:
        //    res = AdjustStream_StartAngle(connectionPoint, origin_points, sp, adjusted_points);
        //    break;F
        case MeshLogicOptions_MeshStreamsAdjuster::AlgorithmType::Ditsts:
            res = AdjustStream_Ditsts(connectionPoint, origin_points, sp, adjusted_points);
            break;
    }
    //cout << "AdjustStream done in " << utils::time::ElapsedSecondsStr(time) << endl;
    //options.DebugEnabled = false;

    return res;
}

//bool MeshStreamsAdjuster::AdjustStream_StartAngle(const P3& connectionPoint, const StreamPoints& origin_points, StreamStartDirection& sp, StreamPoints& adjusted_points)
//{
//    bool isDebug = options.DebugEnabled;
//    if (isDebug && options.debug_debugStreamId != -1 && sp.ID != options.debug_debugStreamId) return false;
//    int vertexIndex = sp.vid_eid_fid;
//    vector<VertexToFacesSortedInfo> faces;
//    vector<D> facesAngleSegments;
//    D angleSumm360;
//    bool isRotationCircular;
//    int rotationCount;
//    if (!Divider::GetVertexFacesAngleSegments(mesh, draw, vertexIndex, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
//    {
//        return false;
//    }
//    auto faceIndexFromRI = [&facesAngleSegments, &faces](D ri)
//    {
//        int faceIndex = 0;
//        for (int i = 0; i < faces.size(); i++)
//        {
//            faceIndex = i;//must be before if - since if statement can be always false
//            if (rotationDegree*ri < facesAngleSegments[i + 1])
//            {
//                break;
//            }
//        }
//        return faceIndex;
//    };
//
//    D join_tollerance = options.tollerance*mesh.avg_edge_length;
//    int singularityExtendCount = isDebug ? options.debug_singularityExtendNum : meshLogicOptions.Divider.SingularityExtendCount;
//    int isoLinesExtensionsCount = min(singularityExtendCount, mesh.FacesCount);
//    D current_maxAngleChangeForDir = sp.maxAngleChangeForDir;
//    V3 current_dir = sp.dir;
//    D current_ri = sp.dir_ri;
//    D min_ri = sp.dir_ri - sp.maxAngleChangeForDir;
//    D max_ri = sp.dir_ri + sp.maxAngleChangeForDir;
//    int current_fid = sp.fid;
//    P3 pointOnPoints;
//    int pointOnPoints_pointIndex;
//    D startDistFromConnectionPointToStream = origin_points.GetClosestDistToPoints(0, connectionPoint, pointOnPoints, pointOnPoints_pointIndex);
//    D minDistFromConnectionPointToStream = startDistFromConnectionPointToStream;
//    V3 faceNormal = mesh.F_normals.row(sp.fid);
//    if (isDebug)
//    {
//        cout << "Divider::AdjustStream -     origin dist = " << startDistFromConnectionPointToStream << endl;
//    }
//    auto correctRI_minmax = [&min_ri, &max_ri](D& ri)
//    {
//        if (ri < min_ri) ri = min_ri;
//        if (ri > max_ri) ri = max_ri;
//    };
//    auto correctRI_360 = [&angleSumm360](D ri)
//    {
//        if (ri < 0) ri += angleSumm360;
//        if (ri > angleSumm360) ri -= angleSumm360;
//        return ri;
//    };
//
//
//    D riMiddle = sp.dir_ri;
//    MeshStreams ss(draw, mesh, solver);
//    ss.Add(sp.fid, sp.Type, sp.vid_eid_fid, sp.point, sp.dir, Color3d(0, 0.7, 0));
//    MeshStream& s = ss[0];
//    int fails_count_in_row = 0;
//    int tryid = 0;
//    for (; tryid < options.iterative_calculations__max_iterations_count; tryid++)
//    {
//        correctRI_minmax(riMiddle);
//        D riLeft = riMiddle - current_maxAngleChangeForDir;
//        D riRight = riMiddle + current_maxAngleChangeForDir;
//        correctRI_minmax(riLeft);
//        correctRI_minmax(riRight);
//        D riset[3] = { riLeft, riMiddle, riRight };
//        int mintestid = -1;
//        D mintestid_dist = minDistFromConnectionPointToStream;
//        bool mintestid_isset = false;
//
//        for (int testid = 0; testid < 3; testid++)
//        {
//            D ri = correctRI_360(riset[testid]);
//            V3 vertexPlaneTangentI = Divider::getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, ri);
//            vertexPlaneTangentI.normalize();
//            int fid = faces[faceIndexFromRI(ri)].FaceId;
//
//            s.RestartStream(fid, sp.Type, sp.vid_eid_fid, sp.point, vertexPlaneTangentI);
//
//            int non_min_founds_int_row = 0;
//            D min_dist_ext = utils::point::DistToPointPow2(connectionPoint, origin_points[0].point);
//            int extendi = 0;
//            for (; extendi < isoLinesExtensionsCount; extendi++)
//            {
//                if (s.ExtendStream(1, false) == 0) break;
//                D dist_ext = utils::point::DistToPointPow2(connectionPoint, s.end_point);
//                if (dist_ext < min_dist_ext)
//                {
//                    min_dist_ext = dist_ext;
//                    non_min_founds_int_row = 0;
//                }
//                else
//                {
//                    non_min_founds_int_row++;
//                    if (non_min_founds_int_row > 10) break;// speed optimization - we dont need to extend till end - if 10 times extension not minimize distance - we go out from our target 'connectionPoint'
//                }
//
//                if (isDebug && (options.debug_debugIterationNum == -1 || options.debug_debugIterationNum == tryid))
//                {
//                    draw.AddEdge(s.start_point, s.end_point, Color3d(0.7, 0.7, 0.7));
//                }
//            }
//            //cout << "extendi = " << extendi << endl;
//
//            D dist = s.Points.GetClosestDistToPoints(0, connectionPoint, pointOnPoints, pointOnPoints_pointIndex);
//
//            if (isDebug && (options.debug_debugIterationNum == -1 || options.debug_debugIterationNum == tryid))
//            {
//                draw.AddEdge(connectionPoint, pointOnPoints, Color3d(1, 0, 0));
//                draw.AddPoint(pointOnPoints, Color3d(0, 0, 1));
//                //draw.AddPoint(origin_points[0] + vertexPlaneTangentI*mesh.avg_edge_length, Color3d(0, 0, 1));
//                draw.AddLabel(origin_points[0].point + vertexPlaneTangentI * mesh.avg_edge_length, "   " + to_string(ri), Color3d(0, 0, 0.5));
//                //draw.AddEdge(origin_points[0], origin_points[0] + vertexPlaneTangentI*mesh.avg_edge_length, Color3d(0, 0, 1));
//            }
//
//            if (isDebug)
//            {
//                cout << "Divider::AdjustStream -  step#" << tryid << "." << testid << (dist < minDistFromConnectionPointToStream ? " accept" : " fail  ") << "   fid = " << fid << "   ri = " << ri << "   maxAngleChange = " << current_maxAngleChangeForDir << "   (dist = " << minDistFromConnectionPointToStream << "   new dist = " << dist << ")" << endl;
//            }
//
//            if (dist < mintestid_dist)
//            {
//                mintestid = testid;
//                mintestid_dist = dist;
//                mintestid_isset = true;
//            }
//        }
//
//        if (mintestid_isset && mintestid_dist < minDistFromConnectionPointToStream && mintestid != 1) // if we found closer stream and it is not middle stream 
//        {
//            riLeft = riset[0];
//            riRight = riset[2];
//            current_maxAngleChangeForDir = current_maxAngleChangeForDir * 0.75; // decrease angle increment to take closer to our connectionPoint
//            if (mintestid == 0)
//            {
//                //riMiddle = riLeft + current_maxAngleChangeForDir;
//                riMiddle = riLeft;
//                if (isDebug) cout << "selected  riLeft = " << riMiddle << endl;
//            }
//            else
//            {
//                //riMiddle = riRight - current_maxAngleChangeForDir;
//                riMiddle = riRight;
//                if (isDebug) cout << "selected  riRight = " << riMiddle << endl;
//            }
//            minDistFromConnectionPointToStream = mintestid_dist;
//            fails_count_in_row = 0;
//            //adjusted_points = s.StreamPoints[0];
//            //for (int i = 0; i < adjusted_points.size() - 1; i++)
//            //{
//            //    draw.AddEdge(adjusted_points[i], adjusted_points[i + 1], Color3d(0, 0, 0.7));
//            //}
//        }
//        else
//        {
//            riMiddle = riMiddle;// leave riMiddle the same
//            if (isDebug)cout << "selected  riMiddle = " << riMiddle << endl;
//            current_maxAngleChangeForDir = current_maxAngleChangeForDir * 0.95; // decrease angle increment to take closer to our connectionPoint
//            fails_count_in_row++;
//            if (fails_count_in_row > 10 && current_maxAngleChangeForDir < 0.001) break;// speed optimization - we dont need to extend till end - if 5 times extension not minimize distance - we reach limit
//        }
//
//        if (isDebug && (options.debug_debugIterationNum != -1 && options.debug_debugIterationNum == tryid))
//        {
//            break;
//        }
//
//        if (mintestid_dist < mesh.avg_edge_length*options.tollerance)
//        {
//            if (isDebug)
//            {
//                cout << "Divider::AdjustStream -  solution found!" << endl;
//            }
//            break;
//        }
//    }
//
//
//    if (minDistFromConnectionPointToStream < startDistFromConnectionPointToStream)
//    {
//        if (isDebug)
//        {
//            cout << "Divider::AdjustStream -  found!  origin dist = " << startDistFromConnectionPointToStream << "   new dist = " << minDistFromConnectionPointToStream << endl;
//            return false;
//        }
//
//
//        V3 vertexPlaneTangentI = Divider::getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, riMiddle);
//        vertexPlaneTangentI.normalize();
//        int fid = faces[faceIndexFromRI(riMiddle)].FaceId;
//
//        s.RestartStream(fid, sp.Type, sp.vid_eid_fid, sp.point, vertexPlaneTangentI);
//        s.ExtendStream(isoLinesExtensionsCount);
//        adjusted_points = s.Points;
//
//        correctRI_minmax(riMiddle);
//        sp.dir = vertexPlaneTangentI;
//        sp.dir_ri = correctRI_360(riMiddle);
//        sp.fid = fid;
//
//
//        return true;
//    }
//    else
//    {
//        if (fails_count_in_row >= 10)
//        {
//            cout << "!!! warning:   Divider::AdjustStream -  startDistFromConnectionPointToStream < dist  after 10 tryis" << endl;
//        }
//        return false;
//    }
//}









struct Minerr
{
    bool isset;
    int test_id;
    D err;
    D dist;
    int test_fid;
    StreamPoint point;
    int pointOnStream_pointIndex;
    D angleToStream;
    D err_min_change;
    Minerr(const StreamPoint& _point, D _err_min_change)
        : isset(false), test_id(-1), err(0), dist(0), test_fid(-1), point(_point), pointOnStream_pointIndex(0), angleToStream(0), err_min_change(_err_min_change)
    {
    }

    bool IsErrorAcceptable(D _err)
    {
        return (!isset || _err < err - err_min_change);
    }
    bool Remember(int _test_id, D _err, D _dist, int _test_fid, StreamPoint _point, int _pointOnStream_pointIndex, D _angleToStream)
    {
        if (IsErrorAcceptable(_err))
        {
            isset = true;
            test_id = _test_id;
            err = _err;
            dist = _dist;
            test_fid = _test_fid;
            point = _point;
            pointOnStream_pointIndex = _pointOnStream_pointIndex;
            angleToStream = _angleToStream;
            return true;
        }
        return false;
    }
};


bool MeshStreamsAdjuster::TryRememberNewPointTo__test_min(D max_stream_angle_diff, int testid, D ri, StreamPoint& newpoint, const StreamPoint& point,
    D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
    const Minerr& try_min, Minerr& test_min, int test_fid,
    DD shift_coeff, const MeshStream& points, int points_startFromIndex, int rot_sign,
    bool isDebug, bool isDebugThisPoint, bool isDebugThisIteration, D newpointEdgePosPercent)
{
    // calculate error
    //v0
    //D newpoint_linelength = utils::point::DistToPoint(newpoint.point, point.point);
    //v1 - precise
    DD newpoint_linelength = (convertP3ToEigenDouble(newpoint.point) - convertP3ToEigenDouble(point.point)).norm();

    int pointOnStream_pointIndex = 0;
    P3 pointOnStream;
    D distD = points.GetClosestDist_Linear(max(0, points_startFromIndex), newpoint.point, pointOnStream, pointOnStream_pointIndex, true, true);
    DD dist = points.GetDistToLineIndexDD(convertP3ToEigenDouble(newpoint.point), pointOnStream_pointIndex); // recalculate dist to get better precision

    DD desiredDist = 0;
    switch (options.testDistMethod)
    {
        default:
        case MeshLogicOptions_MeshStreamsAdjuster::TestDistMethod::NewStream:
        {
            // v0 - base on new stream
            DD newStreamCurrentLength = new_lengthTo_pointOnPoints;
            newStreamCurrentLength += newpoint_linelength;
            desiredDist = pow(newStreamCurrentLength, static_cast<int>(options.ShiftSmoothing) + 1) * shift_coeff;
        }
        break;
        case MeshLogicOptions_MeshStreamsAdjuster::TestDistMethod::OriginStream:
        {
            // v1 - base on original stream
            assert(points.IsInited_lengthToNextPoint());
            DD originalStreamCurrentLength = points[pointOnStream_pointIndex].Length3dUntilThisPoint;
            originalStreamCurrentLength += utils::point::DistToPoint(points[pointOnStream_pointIndex].point, pointOnStream);
            desiredDist = pow(originalStreamCurrentLength, static_cast<int>(options.ShiftSmoothing) + 1) * shift_coeff;
        }
        break;
        //case MeshLogicOptions_MeshStreamsAdjuster::TestDistMethod::NewAndOrigin:
        //{
        //    // v3 - base on new stream and original stream
        //    D newStreamCurrentLength = new_lengthTo_pointOnPoints;
        //    newStreamCurrentLength += newpoint_linelength;
        //    desiredDist = pow(sqrt(newStreamCurrentLength*newStreamCurrentLength - dist * dist), static_cast<int>(options.ShiftSmoothing) + 1) * shift_coeff;
        //}
        break;
    }

    // increase dist if newPoint is crossing stream
    {
        V3 rot_vector_newpoint = newpoint.point - point.point;
        V3 rot_vector_newpoint_normal = mesh.F_normals.row(point.fid);
        V3 rot_vector_stream = pointOnStream - point.point;
        V3 rot_vector_cross = utils::vector::Cross(rot_vector_stream, rot_vector_newpoint);
        int new_rot_sign = utils::vector::SameDirectionIfTheyParallel(rot_vector_cross, rot_vector_newpoint_normal) ? -1 : 1; // 1 if clockwise, and -1 is counterclockwise
        //if (new_rot_sign != rot_sign) dist += startDistFromConnectionPointToStream * 10;
        if (new_rot_sign != rot_sign) dist = -dist;
    }

    D err = (D)(abs(desiredDist - dist) / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist));


    //
    // snap to vertex at dist 0.7% of edge length
    //
    auto canMoveToVertex = [newpoint, point](const P3& v)
    {
        V3 origindir = newpoint - point;
        V3 shiftedDir = v - newpoint.point;
        return  (utils::vector::LengthPow2(shiftedDir) < 0.00000001) // newpoint is almost vertex - so we must to shift to vertex anyway since vector will be 0 and method 'SameDirectionIfTheyParallel' will fail
            || utils::vector::SameDirectionIfTheyParallel(origindir, shiftedDir);
    };
    bool isVeryCloseToVertex = false;
    if (newpoint.Type == MeshPointType::onEdge)
    {
        const D minPRECENT = meshLogicOptions.MeshStream.snap_to_vertex_tol;// good is 0.7%
        const D maxPRECENT = 1 - minPRECENT;
        if (!isnan(newpointEdgePosPercent))
        {
            if (newpointEdgePosPercent < minPRECENT)
            {
                int vid0 = mesh.EV(newpoint.vid_eid_fid, 0);
                P3 v0 = mesh.V.row(vid0);
                if (canMoveToVertex(v0))
                {
                    isVeryCloseToVertex = true;
                    newpoint.Type = MeshPointType::onVertex;
                    newpoint.vid_eid_fid = vid0;
                    newpoint.point = v0;
                }
            }
            if (newpointEdgePosPercent > maxPRECENT)
            {
                int vid1 = mesh.EV(newpoint.vid_eid_fid, 1);
                P3 v1 = mesh.V.row(vid1);
                if (canMoveToVertex(v1))
                {
                    isVeryCloseToVertex = true;
                    newpoint.Type = MeshPointType::onVertex;
                    newpoint.vid_eid_fid = vid1;
                    newpoint.point = v1;
                }
            }
        }
        else
        {
            int vid0 = mesh.EV(newpoint.vid_eid_fid, 0);
            int vid1 = mesh.EV(newpoint.vid_eid_fid, 1);
            P3 v0 = mesh.V.row(vid0);
            P3 v1 = mesh.V.row(vid1);

            D edgeLengthPow2 = utils::point::DistToPointPow2(v0, v1);
            D edgeposPow2 = utils::point::DistToPointPow2(v0, newpoint.point);
            if (edgeLengthPow2 > 0) // defend from devision by zero
            {
                D percentPow2 = edgeposPow2 / edgeLengthPow2;
                bool isVeryCloseToVertex0 = (percentPow2 < minPRECENT*minPRECENT);
                bool isVeryCloseToVertex1 = (percentPow2 > maxPRECENT*maxPRECENT);
                if (isVeryCloseToVertex0 && canMoveToVertex(v0))
                {
                    isVeryCloseToVertex = true;
                    newpoint.Type = MeshPointType::onVertex;
                    newpoint.vid_eid_fid = vid0;
                    newpoint.point = v0;
                }
                if (isVeryCloseToVertex1 && canMoveToVertex(v1))
                {
                    isVeryCloseToVertex = true;
                    newpoint.Type = MeshPointType::onVertex;
                    newpoint.vid_eid_fid = vid1;
                    newpoint.point = v1;
                }
            }
        }
    }

    //
    // calculate angle to original stream
    //
    D angleToStream = 0; // calculate angleToStream only if this try can be remembered - only if error is acceptable by test_min
    V3 adjustedVector(0, 0, 0); // calculate adjustedVector only if this try can be remembered - only if error is acceptable by test_min
    if (test_min.IsErrorAcceptable(err) || (isDebug && options.debug_trace_iterations &&  options.debug_trace_iterations__tests && isDebugThisPoint))
    {
        V3 streamVector = points[pointOnStream_pointIndex].dirToNextPointNormalized;
        adjustedVector = newpoint.point - point.point;
        angleToStream = utils::vector::Angle(streamVector, adjustedVector / newpoint_linelength, true);

        // if angle is not acceptable, lets calculate angle more precisely
        if (angleToStream > max_stream_angle_diff)
        {
            streamVector = utils::vector::Translate(streamVector, mesh.F_normals.row(points[pointOnStream_pointIndex].fid), mesh.F_normals.row(point.fid), true);
            angleToStream = utils::vector::Angle(streamVector, adjustedVector);
        }

        //DEBUG show streamVector and adjustedVector
        //if (testid < 0)
        //{
        //    draw.AddEdge(point.point, streamVector, 1, Color3d(0, 0, 1), "streamVector");
        //    draw.AddEdge(point.point, adjustedVector, 1, Color3d(1, 0, 0), "adjustedVector");
        //}
    }

    // remember lowest error
    bool accepted = (angleToStream < max_stream_angle_diff || isVeryCloseToVertex) // ignore 'angleToStream' if we snaped newpoint to vertex
        && test_min.Remember(testid, err, dist, test_fid, newpoint, pointOnStream_pointIndex, angleToStream);

    // recalculate next faceid from direction if we forced to be a newpoint of vertex type
    if (accepted && newpoint.Type == MeshPointType::onVertex)
    {
        vector<VertexToFacesSortedInfo> faces;
        vector<D> facesAngleSegments;
        D angleSumm360;
        bool isRotationCircular;
        int rotationCount;
        auto faceIndexFromRI = [&facesAngleSegments, &faces](D ri)
        {
            int faceIndex = 0;
            for (int i = 0; i < faces.size(); i++)
            {
                faceIndex = i;//must be before if - since if statement can be always false
                if (rotationDegree*ri < facesAngleSegments[i + 1])
                {
                    break;
                }
            }
            return faceIndex;
        };
        if (mesh.GetVertexFacesAngleSegments(newpoint.vid_eid_fid, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
        {
            V3 dir_translated_toface0 = utils::vector::Translate(adjustedVector, mesh.F_normals.row(point.fid), mesh.F_normals.row(faces[0].FaceId), true);
            D riStart = utils::vector::AngleFull(faces[0].RightSideDirection, dir_translated_toface0, mesh.F_normals.row(faces[0].FaceId));
            if (riStart < 0) riStart += angleSumm360;
            newpoint.fid = faces[faceIndexFromRI(riStart)].FaceId;
            test_min.point.fid = newpoint.fid;
        }
    }


    if (isDebug && options.debug_trace_iterations &&  options.debug_trace_iterations__tests && isDebugThisPoint)
    {
        cout << "      test#" << testid << "   ri=" << ri << "   angleToStream=" << angleToStream << "   desiredDist=" << desiredDist << "   dist=" << dist << "   err=" << err << (accepted ? "   accepted" : "") << (isVeryCloseToVertex ? "  isVeryCloseToVertex" : "") << endl;
    }

    if (isDebug && isDebugThisPoint && isDebugThisIteration)
    {
        //draw.AddEdge(s.start_point, s.end_point, Color3d(0.7, 0.7, 0.7));
        draw.AddEdge(point.point, newpoint.point, Color3d(0, 1, 0));
        //draw.AddPoint(newpoint.point, Color3d(0, 1, 0), "test#" + to_string(testid) + ", ri=" + to_string(ri) + (accepted ? "   accepted" : ""));
        draw.AddPoint(newpoint.point, Color3d(0, 1, 0), "test#" + to_string(testid) + ", err=" + to_string(err) + (accepted ? "   accepted" : ""));

        //draw.AddEdge(point.point, pointOnStream, Color3d(1, 0, 0), "rot_vector_stream");
        //draw.AddEdge(point.point, test_dir, 1, Color3d(0, 0, 0));

        //draw.AddEdge(point.point, rot_vector_stream, mesh.avg_edge_length, Color3d(0, 0, 1), "rot_vector_stream");
        //draw.AddEdge(point.point, rot_vector_connectionPoint, mesh.avg_edge_length, Color3d(0, 1, 0), "rot_vector_connectionPoint");

        //if (newpoint.Type == MeshPointType::onEdge)
        //{
        //    int eid = newpoint.vid_eid_fid;
        //    P3 eid_v0 = mesh.V.row(mesh.EV(eid, 0));
        //    P3 eid_v1 = mesh.V.row(mesh.EV(eid, 1));
        //    D distToEdge = utils::vector::DistFromLineToPoint(eid_v0, eid_v1, newpoint.point);
        //    cout << "distToEdge = " << (distToEdge * 1000 * 1000) << endl;
        //}
    }





    return accepted;
}


bool MeshStreamsAdjuster::TryRememberNewPointTo__harder(D max_stream_angle_diff, const StreamPoint& point, const V3& dir, D angleSumm360,
    D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
    const Minerr& try_min, Minerr& test_min,
    D shift_coeff, const MeshStream& points, int points_startFromIndex, int rot_sign,
    bool isDebug, bool isDebugThisPoint, bool isDebugThisIteration,
    const vector<VertexToFacesSortedInfo>& faces, D& riMiddle, int& testsMade)
{
    bool foundBetterResult = false;


    int fidStart = try_min.isset ? try_min.test_fid : point.fid;
    const int edgesMAX = 20;
    int fids[edgesMAX] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, };
    int eids_opposite[edgesMAX] = { -1,-1 ,-1 ,-1,-1,-1 ,-1 ,-1,-1,-1 ,-1 ,-1,-1,-1 ,-1 ,-1,-1,-1 ,-1 ,-1, };
    int try_harder_iterations_count_on_edge = options.hard_calculations__iterations_count_on_edge;

    if (point.Type == MeshPointType::onVertex)
    {
        int index = 0;
        int vid = point.vid_eid_fid;
        for (int vi = 0; vi < mesh.VF.size(vid); vi++)
        {
            if (index >= edgesMAX) break;
            int fid = mesh.VF(vid, vi);
            for (int k = 0; k < 3; k++)//TODO we can use mesh.VFi to get edge index instead of loop
            {
                if (mesh.F(fid, k) == vid)
                {
                    fids[index] = fid;
                    eids_opposite[index] = mesh.FE(fid, (k + 1) % 3);
                    index++;
                    if (index >= edgesMAX) break;
                }
            }
        }
    }
    if (point.Type == MeshPointType::onEdge)
    {
        int eid = point.vid_eid_fid;
        int index = 0;
        for (int ei = 0; ei < 2; ei++) // for every face that contacted with this edge
        {
            int fid = mesh.EF(eid, ei);
            if (fid == -1) continue;
            for (int k = 0; k < 3; k++) // find opposite edges
            {
                if (mesh.FE(fid, k) == eid)
                {
                    fids[index] = fid;
                    fids[index + 1] = fid;
                    eids_opposite[index] = mesh.FE(fid, (k + 1) % 3);
                    eids_opposite[index + 1] = mesh.FE(fid, (k + 2) % 3);
                    index += 2;
                    break;
                }
            }
        }
    }
    if (point.Type == MeshPointType::onFace)
    {
        return false;
    }
    if (isDebug && isDebugThisPoint)
    {
        //cout << "      TryRememberNewPointTo__harder:   fidStart=" << fidStart << ",  " << point.vid_eid_fid_ToStr() << ",   test harder on edges: {";
        cout << "      test harder on edges: {";
        for (int i = 0; i < edgesMAX; i++)
        {
            if (fids[i] == -1) break;
            if (i != 0) cout << ",";
            cout << eids_opposite[i];
        }
        cout << "}" << "      fidStart=" << fidStart << ",  " << point.vid_eid_fid_toString();
        if (point.Type == MeshPointType::onEdge)
        {
            int vid0 = mesh.EV(point.vid_eid_fid, 0);
            int vid1 = mesh.EV(point.vid_eid_fid, 1);
            P3 v0 = mesh.V.row(vid0);
            P3 v1 = mesh.V.row(vid1);
            D edgeLength = (v1 - v0).norm();
            D edgepos = (point.point - v0).norm();
            cout << ",  edgepos=" << ((edgepos / edgeLength) * 100) << "%";
        }
        cout << endl;
    }
    for (int ie = 0; ie < edgesMAX; ie++)
    {
        int eid = eids_opposite[ie];
        int fid = fids[ie];
        if (eid == -1) continue;
        int nextfid = mesh.EF(eid, 0) == fid ? mesh.EF(eid, 1) : mesh.EF(eid, 0);
        int vid0 = mesh.EV(eid, 0);
        int vid1 = mesh.EV(eid, 1);
        P3 v0 = mesh.V.row(vid0);
        P3 v1 = mesh.V.row(vid1);
        V3 edgedir = v1 - v0;
        StreamPoint newpoint(MeshPointType::onEdge, eid, P3(0, 0, 0), nextfid);

        int iMax = try_harder_iterations_count_on_edge;
        testsMade += iMax;
        if (isDebug && options.debug_trace_iterations && options.debug_trace_iterations__tests && isDebugThisPoint)
        {
            cout << endl << "     iteration#" << ie << " at eid=" << eid << ", vid0=" << vid0 << ", vid1=" << vid1 << endl; // << ", percent=" << (percent * 100) << "%"
        }
        for (int i = 0; i <= iMax; i++)
        {
            D percent = D(i) / iMax;
            newpoint.Type = MeshPointType::onEdge;
            newpoint.vid_eid_fid = eid;
            newpoint.point = v0 + edgedir * percent;
            if (i == 0 || i == iMax)
            {
                newpoint.Type = MeshPointType::onVertex;
                newpoint.vid_eid_fid = (i == 0) ? vid0 : vid1;
                newpoint.point = (i == 0) ? v0 : v1;
            }

            D ri = percent;
            int test_id = 1;// always 1 as Middle - we must set it to Middle since it doesnt changed when moving to next step
            int test_fid = fid;
            if (TryRememberNewPointTo__test_min(max_stream_angle_diff, test_id, ri, newpoint, point,
                new_lengthTo_pointOnPoints, lengthTo_pointOnPoints,
                try_min, test_min, test_fid,
                shift_coeff, points, points_startFromIndex, rot_sign,
                isDebug, isDebugThisPoint, isDebugThisIteration, percent))
            {
                foundBetterResult = true;
                // update riMiddle, since next iteration will continue from this ri
                V3 newdir = newpoint.point - point.point;
                if (point.Type == MeshPointType::onVertex)
                {
                    V3 dir_translated_toface0 = utils::vector::Translate(newdir, mesh.F_normals.row(fid), mesh.F_normals.row(faces[0].FaceId), true);
                    riMiddle = utils::vector::AngleFull(faces[0].RightSideDirection, dir_translated_toface0, mesh.F_normals.row(faces[0].FaceId));
                    if (riMiddle < 0) riMiddle += angleSumm360;
                }
                if (point.Type == MeshPointType::onEdge)
                {
                    riMiddle = utils::vector::AngleFull(dir, newdir, mesh.F_normals.row(fid));
                }
            }
        }
    }
    return foundBetterResult;
}

void MeshStreamsAdjuster::AdjustStream_Ditsts_iterative(const P3& connectionPoint, const MeshStream& origin_points, int points_startFromIndex,
    int pointNum, const StreamPoint& point, const V3& dir,
    D shift_coeff, int rot_sign, D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
    D join_tollerance, D ri_max_change_angle,
    bool isDebug, bool isDebugThisPoint,
    Minerr& try_min, int& testsMade)
{
    int try__failed_times_in_row_MAX = isDebug ? options.debug_max_times_failed_in_row : options.iterative_calculations__max_times_failed_in_row;
    D max_stream_angle_diff = options.max_stream_angle_diff;

    //
    // define varaibles for vertex iterations
    //
    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    auto faceIndexFromRI = [&facesAngleSegments, &faces](D ri)
    {
        int faceIndex = 0;
        for (int i = 0; i < faces.size(); i++)
        {
            faceIndex = i;//must be before if - since if statement can be always false
            if (rotationDegree*ri < facesAngleSegments[i + 1])
            {
                break;
            }
        }
        return faceIndex;
    };
    D min_ri = 0;
    D max_ri = 0;
    auto correctRI_minmax = [&min_ri, &max_ri, &angleSumm360](D& ri)
    {
        if (ri < min_ri) ri = min_ri;
        if (ri > max_ri) ri = max_ri;
    };
    auto correctRI_360 = [&min_ri, &max_ri, &angleSumm360](D ri)
    {
        if (ri < 0) ri += angleSumm360;
        if (ri > angleSumm360) ri -= angleSumm360;
        return ri;
    };

    D join_tollerance_div1000 = join_tollerance / 1000; //  defence from very small invisible iterations, be very carefull with this value - dont make it higher since it happens that iteration is small but anyway important

    bool isStreamFinished = true;
    D riStart = 0;
    if (point.Type == MeshPointType::onVertex)
    {
        if (!mesh.GetVertexFacesAngleSegments(point.vid_eid_fid, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
        {
            return;
        }
        V3 dir_translated_toface0 = utils::vector::Translate(dir, mesh.F_normals.row(point.fid), mesh.F_normals.row(faces[0].FaceId), true);
        riStart = utils::vector::AngleFull(faces[0].RightSideDirection, dir_translated_toface0, mesh.F_normals.row(faces[0].FaceId));
        if (riStart < 0) riStart += angleSumm360;
    }
    if (point.Type == MeshPointType::onEdge)
    {
        riStart = 0;
    }
    if (point.Type == MeshPointType::onFace)
    {
        return; // we dont support this type
    }
    min_ri = riStart - ri_max_change_angle;
    max_ri = riStart + ri_max_change_angle;
    D riMiddle = (min_ri + max_ri) / 2;
    MeshStreams ss(draw, mesh, solver);
    ss.Add(-1, -1, point.fid, point.Type, point.vid_eid_fid, point.point, dir, Color3d(0, 0.7, 0), -1);
    MeshStream& stream = ss.streams[0];
    MeshStreamer& streamer = ss.streamers[0];
    int try__failed_times_in_row = 0; // protection for useless iterations
    int iterationNum = 0;
    D current_maxAngleChangeForDir = 1; // 1 degree in each side (left or right) (clockwise or counter-clockwise)
    for (; iterationNum < options.iterative_calculations__max_iterations_count; iterationNum++)
    {
        bool isDebugThisIteration = (iterationNum == options.debug_debugIterationNum);
        // get scope for ri
        D riLeft = riMiddle - current_maxAngleChangeForDir;
        D riRight = riMiddle + current_maxAngleChangeForDir;
        correctRI_minmax(riLeft);
        correctRI_minmax(riRight);
        const int risetMAX = 3;
        D riset[risetMAX] = { riLeft, riMiddle, riRight };

        // precalculate 'ri', 'test_dir', 'test_fid' for all tests
        D ri_tests[risetMAX];
        V3 test_dir_tests[risetMAX];
        int test_fid_tests[risetMAX];
        for (int testid = 0; testid < risetMAX; testid++)
        {
            D& ri = ri_tests[testid];
            V3& test_dir = test_dir_tests[testid];
            int& test_fid = test_fid_tests[testid];
            // get dir and fid from ri for vertex
            if (point.Type == MeshPointType::onVertex)
            {
                ri = correctRI_360(riset[testid]);
                test_dir = Divider::getAngleFacePlaneVector(faces, facesAngleSegments, point.vid_eid_fid, angleSumm360, mesh, ri);
                //test_dir.normalize();
                test_fid = faces[faceIndexFromRI(ri)].FaceId;
            }
            // get dir and fid from ri for edge
            if (point.Type == MeshPointType::onEdge)
            {
                ri = riset[testid];
                test_dir = utils::vector::Rotate(dir, mesh.F_normals.row(point.fid), ri, true);
                test_fid = point.fid;
            }
        }

        // test left, middle, right for lowest error
        Minerr test_min(point, 0);
        isStreamFinished = true;
        int test__is_newpoint_vertex_times = 0;
        testsMade += risetMAX;
        for (int testid = 0; testid < risetMAX; testid++)
        {
            D ri = ri_tests[testid];
            V3 test_dir = test_dir_tests[testid];
            int test_fid = test_fid_tests[testid];

            // extend stream
            streamer.RestartStream(stream, test_fid, point.Type, point.vid_eid_fid, point.point, test_dir);
            streamer.current_direction = -2;// set to special reserved value '-2' to avoid calculating next direction, coz it is slow and we dont need it at all
            if (streamer.ExtendStream(stream, 1, false) == 0) break;
            isStreamFinished = false;
            StreamPoint newpoint(streamer.current_end_pointType, streamer.current_end_vid_eid_fid, streamer.current_end_point, streamer.current_fid);

            TryRememberNewPointTo__test_min(max_stream_angle_diff, testid, ri, newpoint, point,
                new_lengthTo_pointOnPoints, lengthTo_pointOnPoints,
                try_min, test_min, test_fid,
                shift_coeff, origin_points, points_startFromIndex, rot_sign,
                isDebug, isDebugThisPoint, isDebugThisIteration);

            if (test_min.point.Type == MeshPointType::onVertex) test__is_newpoint_vertex_times++;
        }

        // stop if we reach border
        if (isStreamFinished) break;

        // if we found not very good solution and almost giveup since failed many times - try harder on opposite edges 
        if (options.run_hard_calculations)
        {
            if ((try_min.err > 0.1 || !try_min.isset) && (try__failed_times_in_row == try__failed_times_in_row_MAX - 1 || test__is_newpoint_vertex_times == risetMAX))
            {
                if (isDebug && isDebugThisPoint)
                {
                    //cout << "      about to fail with high err " << try_min.err << " after " << try__failed_times_in_row << " fails. trying to get better result for point#" << pointNum << " in face=" << fid << " at " << (point.Type == MeshPointType::onVertex ? "vid=" : "eid=") << point.vid_eid_fid << " by " << options.try_harder_iterations_count_on_edge << " iterations" << endl;
                    cout << "      about to fail with high err=" << try_min.err << " after " << try__failed_times_in_row << " fails at iteration#" << iterationNum << ". trying to get better result in " << options.hard_calculations__iterations_count_on_edge << " iterations from point at " << point.vid_eid_fid_toString() << endl;
                }
                D riMiddle_before_try_harder = riMiddle;
                if (TryRememberNewPointTo__harder(max_stream_angle_diff, point, dir, angleSumm360,
                    new_lengthTo_pointOnPoints, lengthTo_pointOnPoints,
                    try_min, test_min,
                    shift_coeff, origin_points, points_startFromIndex, rot_sign,
                    isDebug, isDebugThisPoint, isDebugThisIteration,
                    faces, riMiddle, testsMade))
                {
                    if (isDebug && isDebugThisPoint)
                    {
                        cout << "      found better result with err " << test_min.err << " at " << test_min.point.vid_eid_fid_toString() << " (old ri=" << riMiddle_before_try_harder << ",  new ri=" << riMiddle << ")" << endl;
                    }
                }
                iterationNum += 100;// increase counter by 100 to be shure that we want trap into very long loop in worst cases
            }
        }

        // protection for useless iterations
        if (iterationNum > 100)
        {
            try_min.err_min_change = join_tollerance_div1000;
        }

        // try to remember newpoint if error is smaller from prevoius tryis
        bool isTestAccepted = test_min.isset && try_min.Remember(test_min.test_id, test_min.err, test_min.dist, test_min.test_fid, test_min.point, test_min.pointOnStream_pointIndex, test_min.angleToStream);
        if (isTestAccepted)
        {
            try__failed_times_in_row = 0;
        }
        else
        {
            try__failed_times_in_row++;
            if (try__failed_times_in_row > try__failed_times_in_row_MAX) break;// speed optimization - we dont need to extend till end - if 10 times extension not minimize distance - we break iterations
        }

        // DEBUG - show debug info for current iteration
        if (isDebug && options.debug_trace_iterations && isDebugThisPoint)
        {
            string accept_str = isTestAccepted ? "accept  err=" + to_string(try_min.err) : "test__is_newpoint_vertex_times=" + to_string(test__is_newpoint_vertex_times) + "  fail#" + to_string(try__failed_times_in_row);
            cout << "   iteration#" << iterationNum << "   ri=[" << riLeft << ", " << riRight << "]   best=#" << test_min.test_id << "   dist=" << test_min.dist << "   err=" << test_min.err << "   " << accept_str << endl;
            if (options.debug_trace_iterations && options.debug_trace_iterations__tests) cout << endl; // add additional separator to separate iterations from tests
        }


        // update riMiddle and current_maxAngleChangeForDir
        const D angleDecrementMiddle = 0.9; // for middle - we can shorten angle diapason more than for left and right, since we dont need to move to left or to right . since triangle edges-contour is not linear lets move slow at speed 0.9 to avoid losing precise results
        const D angleDecrementLeftRight = 0.99; // for left and right - lets give chance to move faster without losing speed (25 degrees for 30 steps). But anyway shorten angle to make sure that our iteration algorithm will get closer to result at every step
        if (test_min.isset && !(test_min.test_id == 1)) // if we found closer stream and it is not middle stream 
        {
            riMiddle = riset[test_min.test_id];
            current_maxAngleChangeForDir = current_maxAngleChangeForDir * angleDecrementLeftRight; // decrease angle increment to take closer to our connectionPoint
        }
        else
        {
            riMiddle = riMiddle; // dont change riMiddle
            current_maxAngleChangeForDir *= angleDecrementMiddle; // decrease angle increment to take closer to our connectionPoint -  
        }

        // stop if we test some tryNum
        if (isDebug && isDebugThisIteration && options.debug_debugPointNum != -1 && isDebugThisPoint)
        {
            break;
        }

        // stop if err is lower from desired
        if (try_min.isset && try_min.err < join_tollerance)
        {
            break;
        }
    }

}



//---------------------------------------------------------------------------
// solve cubic equation x^3 + a*x^2 + b*x + c
// x - array of size 3
// In case 
//         3 real roots: x[0], x[1], x[2], return 3
//         2 real roots: x[0], x[1],          return 2
//         1 real root : x[0], x[1] ± i*x[2], return 1
// https://github.com/sasamil/Quartic
// v0 - origin
//unsigned int solveP3(double a, double b, double c, double *x)
//{
//    const double PI = 3.141592653589793238463L;
//    const double M_2PI = 2 * PI;
//    const double eps = 1e-12;
//
//    double a2 = a * a;
//    double q = (a2 - 3 * b) / 9;
//    double r = (a*(2 * a2 - 9 * b) + 27 * c) / 54;
//    double r2 = r * r;
//    double q3 = q * q*q;
//    if (r2 < q3)
//    {
//        double t = r / sqrt(q3);
//        if (t < -1) t = -1;
//        if (t > 1) t = 1;
//        t = acos(t);
//        a /= 3; q = -2 * sqrt(q);
//        x[0] = q * cos(t / 3) - a;
//        x[1] = q * cos((t + M_2PI) / 3) - a;
//        x[2] = q * cos((t - M_2PI) / 3) - a;
//        return 3;
//    }
//    else
//    {
//        double A = -pow(fabs(r) + sqrt(r2 - q3), 1. / 3);
//        if (r < 0) A = -A;
//        double B = (0 == A ? 0 : q / A);
//
//        a /= 3;
//        x[0] = (A + B) - a;
//        x[1] = -0.5*(A + B) - a;
//        x[2] = 0.5*sqrt(3.)*(A - B);
//        if (fabs(x[2]) < eps)
//        {
//            x[2] = x[1];
//            return 2;
//        }
//
//        return 1;
//    }
//}
// v1 - optimized
unsigned int solveP3(double a, double b, double c, double *x)
{
    const double PI = 3.141592653589793238463L;
    const double M_2PI = 2 * PI;
    const double eps = 1e-12;

    double a2 = a * a;
    double q = (a2 - 3 * b);
    double q9 = q / 9;
    double r = (a*(2 * a2 - 9 * b) + 27 * c);
    double r54 = r / 54;
    double rpow2 = r * r;
    double r54pow2 = r54 * r54;
    double qpow3 = q * q*q;
    double q9pow3 = q9 * q9*q9;
    //if (r54pow2 < q9pow3)
    //if (rpow2 / (4 * 9 * 9 * 9) < qpow3 / (9 * 9 * 9))
    if (rpow2 < qpow3 * 4)
    {
        //double t = r54 / sqrt(q9pow3);
        //double t = (r / 54) / sqrt(qpow3 / (9 * 9 * 9));
        //double t = (r / 54) / (sqrt(qpow3) / (3 * 3 * 3));
        double t = (r / 2) / sqrt(qpow3);
        if (t < -1) t = -1;
        if (t > 1) t = 1;
        t = acos(t);
        a /= 3;
        q9 = -2. / 3 * sqrt(q);
        x[0] = q9 * cos(t / 3) - a;
        x[1] = q9 * cos((t + M_2PI) / 3) - a;
        x[2] = q9 * cos((t - M_2PI) / 3) - a;
        return 3;
    }
    else
    {
        //double A = -pow(fabs(r54) + sqrt(r54pow2 - q9pow3), 1. / 3);
        //double A = -pow(fabs(r54) + sqrt(rpow2 / (4 * 9 * 9 * 9) - qpow3 / (9 * 9 * 9)), 1. / 3);
        //double A = -pow(fabs(r54) + sqrt(rpow2 - 4*qpow3)*sqrt(1/ (4 * 9 * 9 * 9)), 1. / 3);
        //double A = -pow(fabs(r / 54) + sqrt(rpow2 - 4 * qpow3)*(1 / (2 * 3 * 3 * 3)), 1. / 3);
        //double A = -pow(fabs(r/54) + sqrt(rpow2 - 4 * qpow3)*(1. / 54), 1. / 3);
        double A = -pow(fabs(r) + sqrt(rpow2 - 4 * qpow3), 1. / 3) * pow(1. / 54, 1. / 3);
        //double A = -pow(fabs(r) + sqrt(rpow2 - 4 * qpow3), 1. / 3) * 0.26456684199469991245861760654538;
        if (r < 0) A = -A;
        double B = (A == 0 ? 0 : q9 / A);

        a /= 3;
        x[0] = (A + B) - a;
        x[1] = -0.5*(A + B) - a;
        x[2] = 0.5*sqrt(3.)*(A - B);
        if (fabs(x[2]) < eps)
        {
            x[2] = x[1];
            return 2;
        }

        return 1;
    }
}
//---------------------------------------------------------------------------
// solve quartic equation x^4 + a*x^3 + b*x^2 + c*x + d
//  this function produce 4 complex<double> solutions
// https://github.com/sasamil/Quartic
void solve_quartic__complex(double a, double b, double c, double d, complex<double>* x)
{
    const double eps = 1e-12;

    double a3 = -b;
    double b3 = a * c - 4.*d;
    double c3 = -a * a*d - c * c + 4.*b*d;

    // cubic resolvent
    // y^3 − b*y^2 + (ac−4d)*y − a^2*d−c^2+4*b*d = 0

    double x3[3];
    unsigned int iZeroes = solveP3(a3, b3, c3, x3);

    double q1, q2, p1, p2, sqD;

    double y = x3[0];
    // The essence - choosing Y with maximal absolute value.
    if (iZeroes != 1)
    {
        if (fabs(x3[1]) > fabs(y)) y = x3[1];
        if (fabs(x3[2]) > fabs(y)) y = x3[2];
    }

    // h1+h2 = y && h1*h2 = d  <=>  h^2 -y*h + d = 0    (h === q)

    double D = y * y - 4 * d;
    if (fabs(D) < eps) //in other words - D==0
    {
        q1 = q2 = y * 0.5;
        // g1+g2 = a && g1+g2 = b-y   <=>   g^2 - a*g + b-y = 0    (p === g)
        D = a * a - 4 * (b - y);
        if (fabs(D) < eps) //in other words - D==0
            p1 = p2 = a * 0.5;

        else
        {
            sqD = sqrt(D);
            p1 = (a + sqD) * 0.5;
            p2 = (a - sqD) * 0.5;
        }
    }
    else
    {
        sqD = sqrt(D);
        q1 = (y + sqD) * 0.5;
        q2 = (y - sqD) * 0.5;
        // g1+g2 = a && g1*h2 + g2*h1 = c       ( && g === p )  Krammer
        p1 = (a*q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }


    // solving quadratic eq. - x^2 + p1*x + q1 = 0
    D = p1 * p1 - 4 * q1;
    if (D < 0.0)
    {
        x[0].real(-p1 * 0.5);
        x[0].imag(sqrt(-D) * 0.5);
        x[1] = std::conj(x[0]);
    }
    else
    {
        sqD = sqrt(D);
        x[0].real((-p1 + sqD) * 0.5);
        x[1].real((-p1 - sqD) * 0.5);
    }

    // solving quadratic eq. - x^2 + p2*x + q2 = 0
    D = p2 * p2 - 4 * q2;
    if (D < 0.0)
    {
        x[2].real(-p2 * 0.5);
        x[2].imag(sqrt(-D) * 0.5);
        x[3] = std::conj(x[2]);
    }
    else
    {
        sqD = sqrt(D);
        x[2].real((-p2 + sqD) * 0.5);
        x[3].real((-p2 - sqD) * 0.5);
    }
}

// solve quartic equation x^4 + a*x^3 + b*x^2 + c*x + d
//this function returns count of stored solutions in 'x' (at max 4).
//https://github.com/sasamil/Quartic
int solve_quartic(double a, double b, double c, double d, double* x)
{
    const double eps = 1e-12;

    double a3 = -b;
    double b3 = a * c - 4.*d;
    double c3 = -a * a*d - c * c + 4.*b*d;

    // cubic resolvent
    // y^3 − b*y^2 + (ac−4d)*y − a^2*d−c^2+4*b*d = 0

    double x3[3];
    unsigned int iZeroes = solveP3(a3, b3, c3, x3);

    double q1, q2, p1, p2, sqD;

    double y = x3[0];
    // The essence - choosing Y with maximal absolute value.
    if (iZeroes != 1)
    {
        if (fabs(x3[1]) > fabs(y)) y = x3[1];
        if (fabs(x3[2]) > fabs(y)) y = x3[2];
    }

    // h1+h2 = y && h1*h2 = d  <=>  h^2 -y*h + d = 0    (h === q)

    double D = y * y - 4 * d;
    if (D < -eps)
    {
        return 0;
    }

    if (fabs(D) < eps) //in other words - D==0
    {
        q1 = q2 = y * 0.5;
        // g1+g2 = a && g1+g2 = b-y   <=>   g^2 - a*g + b-y = 0    (p === g)
        D = a * a - 4 * (b - y);
        if (fabs(D) < eps) //in other words - D==0
            p1 = p2 = a * 0.5;

        else
        {
            sqD = sqrt(D);
            p1 = (a + sqD) * 0.5;
            p2 = (a - sqD) * 0.5;
        }
    }
    else
    {
        sqD = sqrt(D);
        q1 = (y + sqD) * 0.5;
        q2 = (y - sqD) * 0.5;
        // g1+g2 = a && g1*h2 + g2*h1 = c       ( && g === p )  Krammer
        p1 = (a*q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    int possibleSolutionsCount = 0;
    // solving quadratic eq. - x^2 + p1*x + q1 = 0
    D = p1 * p1 - 4 * q1;
    sqD = (D < 0.0) ? 0 : sqrt(D);
    x[possibleSolutionsCount] = (-p1 + sqD) * 0.5;
    x[possibleSolutionsCount + 1] = (-p1 - sqD) * 0.5;
    possibleSolutionsCount += (D < 0.0 ? 1 : 2);

    // solving quadratic eq. - x^2 + p2*x + q2 = 0
    D = p2 * p2 - 4 * q2;
    sqD = (D < 0.0) ? 0 : sqrt(D);
    x[possibleSolutionsCount] = (-p2 + sqD) * 0.5;
    x[possibleSolutionsCount + 1] = (-p2 - sqD) * 0.5;
    possibleSolutionsCount += (D < 0.0 ? 1 : 2);
    return possibleSolutionsCount;
}

void MeshStreamsAdjuster::AdjustStream_Ditsts_direct(const P3& connectionPoint, const MeshStream& origin_points, int points_startFromIndex,
    int pointNum, const StreamPoint& point, const V3& dir,
    D shift_coeff, int rot_sign, D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
    D join_tollerance, D ri_max_change_angle,
    bool isDebug, bool isDebugThisPoint,
    Minerr& try_min, int& testsMade)
{
    // we support only linear equation - for quadratic need to resolve formula and extend "solveH, solveL" methods
    if (options.ShiftSmoothing != MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType::Linear)
    {
        return;
    }
    bool isDebugOnlyThisPoint = (options.debug_debugPointNum != -1 && pointNum == options.debug_debugPointNum);

    // solving without taking into account new point distance - works not good - new line is sharp 
    auto solveSimple = [](D t, D h, D sin, D cos, D s, D a, D& c1, D& c2)
    {
        c1 = (h - t * s) / sin;
        c2 = (h + t * s) / sin;
    };

    // t == new_lengthTo_pointOnPoints
    // h == dist from v0 to stream
    // sin == sin angle between stream and edge
    // cos == cos angle betwen two edges of triangle in vertex v0
    // s == shift_coeff
    // a == dist from v0 to point.point    
    auto solveH = [](D t, D h, D sin, D cos, D s, D a, DD& solution, int takeSolutionNum)
    {
        /*
            desiredDist = (totalLength + dirLength)*shift_coeff
            desiredDist = c1 * sin
            h = h1 + h2 = c1 * sin + c2 * sin = desiredDist + h2 = desiredDist + c2 * sin
            desiredDist = h - h2 = h - c2 * sin

            h - c2 * sin = (totalLength + dirLength)*shift_coeff


            lets simplify : dirLength = >d, shift_coeff = >s, totalLength = >t, c2 = >c
            h - c * sin = (t + d)*s
            c = (h - (t + d)*s)*sin
            d = (h - c * sin) / s - t
            = h / s - c * (sin / s) - t
            = h / s - t - c * (sin / s)
            = w + cn(where w = (h / s - t) and n = -(sin / s))



            d ^ 2 = a ^ 2 + c ^ 2 - 2ac*cos(Law of Cosines)
            (a + b) ^ 2 = a ^ 2 + 2ab + b ^ 2



            d ^ 2 = (w + cn) ^ 2
            = w ^ 2 + 2wcn + c ^ 2 * n ^ 2

            w ^ 2 + 2wcn + c ^ 2 * n ^ 2 = a ^ 2 + c ^ 2 - 2ac*cos
            c ^ 2 * n ^ 2 - c ^ 2 + 2wcn + 2ac*cos + w ^ 2 - a ^ 2 = 0
            c ^ 2 * (n ^ 2 - 1) + c * (2wn + 2a*cos) + (w ^ 2 - a ^ 2) = 0

            c ^ 2 * A + c * B + C = 0, where A = (n ^ 2 - 1), B = (2wn + 2a*cos), C = (w ^ 2 - a ^ 2)


            c = (-B + -sqrt(B ^ 2 - 4AC)) / (2A)
        */

        DD w = (h / s) - t;
        DD n = -(sin / s);

        DD A = n * n - 1;
        DD B = 2 * w*n + 2 * a*cos;
        DD C = w * w - a * a;

        DD det = sqrt(B*B - 4 * A*C);
        solution = (takeSolutionNum == 1)
            ? (-B - det) / (2 * A)
            : (-B + det) / (2 * A);
    };

    // t == new_lengthTo_pointOnPoints
    // L == dist from v0 to intersection point of edge-stream
    // sin == sin angle between stream and edge
    // cos == cos angle betwen two edges of triangle in vertex v0
    // s == shift_coeff
    // a == dist from v0 to point.point  
    //auto solveL = [](D t, D L, D sin, D cos, D s, D a, D& solution, int takeSolutionNum)
    //{
    //    /*
    //         L = x + X = > X = L - x
    //         H = (t + d)s
    //         H = X * sin == (L - x)*sin

    //         (t + d)s == (L - x)*sin
    //         (t + d) == (L - x)*sin / s
    //         d == (L - x)*sin / s - t
    //         d == x * (-sin / s) + L * sin / s - t
    //         d == xw + n(where w == (-sin / s), n == L * sin / s - t)

    //         d ^ 2 == a ^ 2 + x ^ 2 - 2ax*cos
    //         x ^ 2 - 2ax*cos - d ^ 2 + a ^ 2 == 0
    //         x ^ 2 - 2ax*cos - (xw + n) ^ 2 + a ^ 2 == 0
    //         x ^ 2 - 2ax*cos - (x ^ 2 * w ^ 2 + 2xwn + n ^ 2) + a ^ 2 == 0
    //         x ^ 2 - 2ax*cos - x ^ 2 * w ^ 2 - 2xwn - n ^ 2 + a ^ 2 == 0
    //         (x ^ 2 - x ^ 2 * w ^ 2) - (2ax*cos + 2xwn) - n ^ 2 + a ^ 2 == 0
    //         x ^ 2(1 - w ^ 2) + x(-2a*cos - 2wn) + (a ^ 2 - n ^ 2) == 0
    //         x ^ 2 * A + x * B + C == 0
    //         where A == 1 - w ^ 2, B == -2a*cos - 2wn, C == a ^ 2 - n ^ 2

    //         x(1, 2) == (-B + -sqrt(B*B - 4AC)) / (2A)
    //     */
    //    D w = (-sin / s);
    //    D n = (L * sin / s) - t;

    //    D A = 1 - w * w;
    //    D B = -2 * a*cos - 2 * w*n;
    //    D C = a * a - n * n;

    //    D det = sqrt(B*B - 4 * A*C);
    //    solution = (takeSolutionNum == 1)
    //        ? (-B + det) / (2 * A)
    //        : (-B - det) / (2 * A);
    //};


    // t == new_lengthTo_pointOnPoints
    // L == dist from v0 to intersection point of edge-stream
    // sin == sin angle between stream and edge
    // cos == cos angle betwen two edges of triangle in vertex v0
    // s == shift_coeff
    // a == dist from v0 to point.point  
    auto solveLOptimized = [](D t, D L, D sin, D cos, D s, D a, DD& solution, int takeSolutionNum)
    {
        /*
        L = x + X = > X = L - x
        H = (t + d)s
        H = X * sin == (L - x)*sin

        (t + d)s == (L - x)*sin
        (t + d) == (L - x)*sin / s
        d == (L - x)*sin / s - t
        d == x * (-sin / s) + L * sin / s - t
        d == xw + n(where w == (-sin / s), n == L * sin / s - t)

        d ^ 2 == a ^ 2 + x ^ 2 - 2ax*cos
        x ^ 2 - 2ax*cos - d ^ 2 + a ^ 2 == 0
        x ^ 2 - 2ax*cos - (xw + n) ^ 2 + a ^ 2 == 0
        x ^ 2 - 2ax*cos - (x ^ 2 * w ^ 2 + 2xwn + n ^ 2) + a ^ 2 == 0
        x ^ 2 - 2ax*cos - x ^ 2 * w ^ 2 - 2xwn - n ^ 2 + a ^ 2 == 0
        (x ^ 2 - x ^ 2 * w ^ 2) - (2ax*cos + 2xwn) - n ^ 2 + a ^ 2 == 0
        x ^ 2(1 - w ^ 2) + x(-2a*cos - 2wn) + (a ^ 2 - n ^ 2) == 0
        x ^ 2 * A + x * B + C == 0
        where A == 1 - w ^ 2, B == -2a*cos - 2wn, C == a ^ 2 - n ^ 2

        x(1, 2) == (-B + -sqrt(B*B - 4AC)) / (2A)
        */
        DD w = (-sin / s);
        DD n = (L * sin / s) - t;

        DD A = 1 - w * w;
        DD B = a * cos + w * n;
        DD C = a * a - n * n;

        DD det = utils::sse::sqrt(B*B - A * C);
        solution = (takeSolutionNum == 1)
            ? (B + det) / A
            : (B - det) / A;
    };

    auto solveX = [](D t, D L, D sin, D cos, D s, D a, DD& solution, int takeSolutionNum)
    {
        /* ----------
             x--> X
             ----------
             x = L + X

             ----------
             d--> X
             ----------
             H = (t + d)s
             H = X * sin
             (t + d)s == X * sin
             (t + d) == X * sin / s
             d == X * sin / s - t
             d == Xw + n(where w == sin / s, n == -t)

             ----------
             Law of Cosines
             ----------
             d ^ 2 == a ^ 2 + x ^ 2 - 2ax*cos

             ----------
             Expand equation
             ----------

             (Xw + n) ^ 2 == a ^ 2 + (L + X) ^ 2 - 2a(L + X)*cos
             X ^ 2 * w ^ 2 + n ^ 2 + 2Xwn == a ^ 2 + L ^ 2 + X ^ 2 + 2LX - 2aL*cos - 2aX*cos
             X ^ 2 * w ^ 2 + n ^ 2 + 2Xwn - a ^ 2 - L ^ 2 - X ^ 2 - 2LX + 2aL*cos + 2aX*cos == 0
             X ^ 2(w ^ 2 - 1) + X(2wn - 2L + 2a*cos) + n ^ 2 - a ^ 2 - L ^ 2 + 2aL*cos == 0
             X ^ 2(w ^ 2 - 1) + X(2wn - 2L + 2a*cos) + n ^ 2 - (a ^ 2 + L ^ 2 - 2aL*cos) == 0


             x ^ 2 * A + x * B + C == 0
         where A == w ^ 2 - 1, B == 2wn - 2L + 2a*cos, C == n ^ 2 - (a ^ 2 + L ^ 2 - 2aL*cos)
             X(1, 2) == (-B + -sqrt(B*B - 4AC)) / (2A)
         */

        DD w = sin / s;
        DD n = -t;

        DD A = w * w - 1;
        DD B = 2 * w*n - 2 * L + 2 * a*cos;
        DD C = n * n - (a*a + L * L - 2 * a*L*cos);

        DD det = sqrt(B*B - 4 * A*C);
        solution = (takeSolutionNum == 1)
            ? L - (-B + det) / (2 * A)
            : L - (-B - det) / (2 * A);
    };

    auto solve3d = []()
    {

        /*
        given: p,r,t  and q,s,u and h,d (h length, d vector from q to r)

        p + tr = q + us + hd

        (p + tr) × s = (q + us + d) × s

         t (r × s) = (q − p) × s + d × s




        */
    };

    auto algorithm = options.DirectAlgorithm;
    if (pointNum == 0 && algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::V)
    {
        algorithm = MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::L;
    }
    bool precalculateHLXhelpers = (isDebug)// && isDebugOnlyThisPoint
        || (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::H)
        || (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::L)
        || (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::X);

    int fidStart = try_min.isset ? try_min.test_fid : point.fid;
    if (fidStart == -1) return;

    // gain all edges that should be tested
    const int edgesMAX = 20;
    enum TestSide
    {
        Left, Right
    };
    TestSide targetSide = (rot_sign == 1) ? TestSide::Right : TestSide::Left;
    struct TestEdge
    {
        int fid;
        int eid;
        TestSide side;
        int testsMade;
        D minerr;
        TestEdge()
            : fid(0), eid(0), side(TestSide::Left), testsMade(0), minerr(0)
        {

        }
    };
    TestEdge testEdges[edgesMAX];
    int testEdgesAddedCount = 0;
    int eidStart = -1;
    if (point.Type == MeshPointType::onEdge)
    {
        eidStart = point.vid_eid_fid;
        for (int k = 0; k < 3; k++)
        {
            int eid = mesh.FE(fidStart, k);
            if (eid == point.vid_eid_fid)
            {
                TestEdge& tR = testEdges[testEdgesAddedCount];
                tR.fid = fidStart;
                tR.eid = mesh.FE(fidStart, (k + 1) % 3);
                tR.side = TestSide::Right;
                testEdgesAddedCount++;
                TestEdge& tL = testEdges[testEdgesAddedCount];
                tL.fid = fidStart;
                tL.eid = mesh.FE(fidStart, (k + 2) % 3);
                tL.side = TestSide::Left;
                testEdgesAddedCount++;
                break;
            }
        }
        if (testEdgesAddedCount == 0) return;
        //test firstly edges that are on the way of 'dir'
        TestEdge& t2 = testEdges[1];
        int t2_vid0 = mesh.EV(t2.eid, 0);
        int t2_vid1 = mesh.EV(t2.eid, 1);
        P3 t2_v0 = mesh.V.row(t2_vid0);
        P3 t2_v1 = mesh.V.row(t2_vid1);
        //mesh.EV(eids_opposite[1]
        D edgePosPow2;
        if (utils::vector::FindIntersectionBetween_EdgeAndVectorPow2(t2_v0, t2_v1, point.point, dir, edgePosPow2) // if we can say something about edge intersection 
            && (edgePosPow2 > 0 * 0 && edgePosPow2 < 1 * 1)) // and intersection point is in edge
        {
            //swap positions, so edge#2 will be tested firstly
            swap(testEdges[0].eid, testEdges[1].eid);
            swap(testEdges[0].side, testEdges[1].side);
        }
    }
    if (point.Type == MeshPointType::onVertex)
    {
        int vid = point.vid_eid_fid;
        for (int k = 0; k < 3; k++)
        {
            if (mesh.F(fidStart, k) != vid) continue;
            //add opposite edge to vertex of same face
            TestEdge& tM = testEdges[testEdgesAddedCount];
            tM.fid = fidStart;
            tM.eid = mesh.FE(fidStart, (k + 1) % 3);
            tM.side = TestSide::Right;
            testEdgesAddedCount++;

            //add opposite edges to vertex of friend faces to left and right edges of startface
            int eidsLeftRight[2] = { mesh.FE(fidStart, (k + 2) % 3), mesh.FE(fidStart, k) };
            for (int n = 0; n < 2; n++)
            {
                int eid_friend = eidsLeftRight[n];
                for (int m = 0; m < 2; m++)
                {
                    int fid_friend = mesh.EF(eid_friend, m);
                    if (fid_friend == -1 || fid_friend == fidStart) continue;
                    for (int kn = 0; kn < 3; kn++)
                    {
                        if (mesh.F(fid_friend, kn) != vid) continue;
                        //add opposite edge to vertex of neighborhood face
                        TestEdge& t = testEdges[testEdgesAddedCount];
                        t.fid = fid_friend;
                        t.eid = mesh.FE(fid_friend, (kn + 1) % 3);
                        t.side = TestSide::Right;
                        testEdgesAddedCount++;
                    }
                }
            }
        }
    }
    for (int i = 0; i < testEdgesAddedCount; i++)
    {
        testEdges[i].testsMade = 0;
        testEdges[i].minerr = 0;
    }
    #if DEBUG
    vector<TestEdge> testEdgesDEBUG;
    testEdgesDEBUG.reserve(testEdgesAddedCount);
    for (int i = 0; i < testEdgesAddedCount; i++)testEdgesDEBUG.push_back(testEdges[i]);
    #endif

    //DEBUG - replace totalLength of new stream - just for debuging 'solve' method in start of stream
    if (!(options.direct_calculations__replace__new_lengthTo_pointOnPoints < 0))
    {
        new_lengthTo_pointOnPoints = options.direct_calculations__replace__new_lengthTo_pointOnPoints;
    }
    // DEBUG - show info about what we gonna do into console
    if (isDebug && isDebugOnlyThisPoint)
    {
        cout << "   direct solver:  fid=" << fidStart << "   " << point.vid_eid_fid_toString() << "   testing edges=";
        for (int tid = 0; tid < testEdgesAddedCount; tid++)
        {
            if (tid != 0) cout << ",";
            cout << testEdges[tid].eid;
        }
        cout << endl;
    }

    if (testEdgesAddedCount == 0) return;

    bool precise_direction_translation = options.direct_calculations__precise_direction_translation;
    D newTotalLength = new_lengthTo_pointOnPoints;

    D targetTollerance = options.tollerance;
    Minerr test_min(point, 0);
    int iteration = -1; // iterations made on edges, and test on stream lines
    int testidMAX = min(options.direct_calculations__test_streams_count, origin_points.size() - 1 - points_startFromIndex);
    int successfulTestsCount = 0;
    int edgePosIsOutCount = 0;
    for (bool testOppositeSide : {false, true}) // firstly to test points that on same side as 'sign'(right or left) (since 'solve' method can return two points)
    {
        if (test_min.isset && test_min.err < targetTollerance) break;
        TestSide testSide = (!testOppositeSide) ? targetSide : (targetSide == TestSide::Left ? TestSide::Right : TestSide::Left);
        for (int tid = 0; tid < testEdgesAddedCount; tid++)
        {
            if (test_min.isset && test_min.err < targetTollerance) break;
            iteration++;
            bool isDebugThisIteration = (iteration == options.debug_debugIterationNum);

            const TestEdge& t = testEdges[tid];
            int eid = t.eid;
            int fid = t.fid;
            V3 fnormal = mesh.F_normals.row(fid);


            // get edge vid0, vid1
            int vid0 = 0;
            int vid1 = 0;
            for (int k = 0; k < 3; k++)
            {
                if (mesh.FE(fid, k) == eid)
                {
                    vid0 = mesh.F(fid, k);
                    vid1 = mesh.F(fid, (k + 1) % 3);
                    break;
                }
            }
            // v0 should belong to some of vertexes of eidStart
            if (eidStart != -1)
            {
                if (vid1 == mesh.EV(eidStart, 0) || vid1 == mesh.EV(eidStart, 1))
                {
                    swap(vid0, vid1);

                }
            }


            // edge direction
            P3 v0 = mesh.V.row(vid0);
            P3 v1 = mesh.V.row(vid1);
            D edgeLength = mesh.E_Length(eid);//utils::vector::Length(edgedir);
            V3 edgedir = v1 - v0;
            V3 edgedirNormalized = edgedir / edgeLength;


            //DEBUG - show edges
            if (isDebug && isDebugOnlyThisPoint && isDebugThisIteration)
            {
                draw.AddPoint(v0, Color3d(0, 0, 0), "vid0=" + to_string(vid0));
                draw.AddPoint(v1, Color3d(0, 0, 0), "vid1=" + to_string(vid1));
                draw.AddLabel((v0 + v1) / 2, "  eid=" + to_string(eid), Color3d(0, 0, 0));
                draw.AddEdge(v0, v1);
            }

            if (isDebug && options.debug_trace_iterations && options.debug_trace_iterations__tests && isDebugThisPoint)
            {
                cout << "     iteration#" << iteration << "  side=" << (testSide == TestSide::Left ? "Left" : "Right") << ", fid=" << fid << ",  eid=" << eid << endl;
            }

            // calculate cos
            D dist_v0_point = 0;
            D cos = 0;
            if (precalculateHLXhelpers)
            {
                dist_v0_point = utils::point::DistToPoint(v0, point.point);
                V3 dir_v0_point = point.point - v0;
                V3 dir_v0_pointNormalized = dir_v0_point / dist_v0_point; //normalize
                cos = utils::vector::Cos(edgedirNormalized, dir_v0_pointNormalized, true);
            }
            int takeSolution = (testSide == t.side) ? 1 : 2;

            // create newpoint
            int nextfid = mesh.EF(eid, 0) == fid ? mesh.EF(eid, 1) : mesh.EF(eid, 0);

            int newpointIsOutOfEdgeCounter = 0;
            for (int testid = 0; testid < testidMAX; testid++)// test few origin_points
            {
                testsMade++;

                // cache translated stream vectors for 'fid'??
                // stream
                const StreamPoint& streamp0 = origin_points[points_startFromIndex + testid];
                const StreamPoint& streamp1 = origin_points[points_startFromIndex + testid + 1];
                P3 streamv0 = streamp0.point;
                P3 streamv0Translated = streamv0;
                P3 streamv1 = streamp1.point;
                P3 streamv1Translated = streamv1;
                V3 streamdir = streamv1 - streamv0;
                V3 streamdirNormalized = streamp0.dirToNextPointNormalized;
                int fid_from = origin_points[points_startFromIndex + testid].fid;
                if (options.testDistMethod == MeshLogicOptions_MeshStreamsAdjuster::TestDistMethod::OriginStream)
                {
                    assert(origin_points.IsInited_lengthToNextPoint());
                    newTotalLength = streamp0.Length3dUntilThisPoint;
                }

                V3 streamdirTranslatedNormalized(0, 0, 0);
                D sin = 0;
                if (precalculateHLXhelpers)
                {
                    V3 streamdirTranslated = streamdir;
                    if (fid_from != fid)
                    {
                        streamdirTranslated = precise_direction_translation
                            ? utils::vector::Translate(streamdir, mesh.F_normals.row(fid_from), fnormal, true) //v0 - slower but more precise in very deformed surfaces
                            : utils::vector::Translate(streamdir, fnormal, false); //v1 - faster but less precise in very deformed surfaces
                    }
                    //
                    ////v3
                    //streamv0Translated = utils::point::ProjectToPlane(streamv0, v0, fnormal);
                    //streamv1Translated = utils::point::ProjectToPlane(streamv1, v0, fnormal);
                    //streamdirTranslated = streamv1Translated - streamv0Translated;

                    streamdirTranslatedNormalized = streamdirTranslated / utils::vector::Length(streamdirTranslated); // normalize
                    sin = utils::vector::Sin(streamdirTranslatedNormalized, edgedirNormalized, true);
                }

                DD edgePos;
                if (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::H)
                {
                    // closest point on stream for v0
                    P3 pointOnStream = utils::vector::ClosestPoint_ToVector(streamv0Translated, streamdirTranslatedNormalized, 1, v0);
                    D dist_v0_pointOnStream = utils::point::DistToPoint(v0, pointOnStream);
                    if (isDebug && isDebugOnlyThisPoint && isDebugThisIteration)
                    {
                        draw.AddEdge(v0, pointOnStream, Color3d(0.7, 0.7, 0.7));
                    }
                    if (takeSolution == 1)
                        solveH(newTotalLength, dist_v0_pointOnStream, sin, cos, shift_coeff, dist_v0_point, edgePos, takeSolution);
                    else
                        solveH(newTotalLength, -dist_v0_pointOnStream, -sin, cos, shift_coeff, dist_v0_point, edgePos, takeSolution);
                    //solveSimple(new_lengthTo_pointOnPoints, dist_v0_pointOnStream, sin, cos, shift_coeff, dist_v0_point, c[0], c[1]);
                }
                else if (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::L)
                {
                    // intersection stream vs edge
                    D  intersectionEdgePos;
                    if (!utils::vector::FindIntersectionBetween_Vectors(streamv0Translated, streamdirTranslatedNormalized, v0, edgedirNormalized, intersectionEdgePos)) continue;
                    //DEBUG - show edges
                    if (isDebug && isDebugOnlyThisPoint && isDebugThisIteration)
                    {
                        // draw intersection lines for stream and edge
                        D intersectionStreamPos;
                        utils::vector::FindIntersectionBetween_Vectors(streamv0Translated, streamdirTranslatedNormalized, v0, edgedirNormalized, intersectionStreamPos, intersectionEdgePos);
                        draw.AddEdge(v0, v0 + edgedirNormalized * intersectionEdgePos, Color3d(0.7, 0.7, 0.7));
                        draw.AddEdge(streamv0Translated, streamv0Translated + streamdirTranslatedNormalized * intersectionStreamPos, Color3d(0.7, 0.7, 0.7));
                        draw.AddPoint(streamv0Translated + streamdirTranslatedNormalized * intersectionStreamPos, Color3d(0.7, 0.7, 0.7));
                    }
                    if (takeSolution == 1)
                        solveLOptimized(newTotalLength, intersectionEdgePos, sin, cos, shift_coeff, dist_v0_point, edgePos, takeSolution);
                    else
                        solveLOptimized(newTotalLength, intersectionEdgePos, -sin, cos, shift_coeff, dist_v0_point, edgePos, takeSolution);
                }
                else if (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::X)
                {
                    // intersection stream vs edge
                    D intersectionStreamPos, intersectionEdgePos;
                    if (!utils::vector::FindIntersectionBetween_Vectors(streamv0Translated, streamdirTranslatedNormalized, v0, edgedirNormalized, intersectionStreamPos, intersectionEdgePos)) continue;
                    //DEBUG - show edges
                    if (isDebug && isDebugOnlyThisPoint && isDebugThisIteration)
                    {
                        // draw intersection lines for stream and edge
                        draw.AddEdge(v0, v0 + edgedirNormalized * intersectionEdgePos, Color3d(0.7, 0.7, 0.7));
                        draw.AddEdge(streamv0Translated, streamv0Translated + streamdirTranslatedNormalized * intersectionStreamPos, Color3d(0.7, 0.7, 0.7));
                        draw.AddPoint(streamv0Translated + streamdirTranslatedNormalized * intersectionStreamPos, Color3d(0.7, 0.7, 0.7));
                    }
                    if (takeSolution == 1)
                        solveX(newTotalLength, intersectionEdgePos, sin, cos, shift_coeff, dist_v0_point, edgePos, takeSolution);
                    else
                        solveX(newTotalLength, intersectionEdgePos, -sin, cos, shift_coeff, dist_v0_point, edgePos, takeSolution);
                }
                else if (algorithm == MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::V)
                {
                    ////DEBUG - show edges
                    if (isDebug && isDebugOnlyThisPoint && isDebugThisIteration)
                    {
                        // draw intersection lines for stream and edge
                        D intersectionStreamPos, intersectionEdgePos;
                        if (utils::vector::FindIntersectionBetween_Vectors(streamv0Translated, streamdirTranslatedNormalized, v0, edgedirNormalized, intersectionStreamPos, intersectionEdgePos)) continue;
                        draw.AddEdge(v0, v0 + edgedirNormalized * intersectionEdgePos, Color3d(0.7, 0.7, 0.7));
                        draw.AddEdge(streamv0Translated, streamv0Translated + streamdirTranslatedNormalized * intersectionStreamPos, Color3d(0.7, 0.7, 0.7));
                        draw.AddPoint(streamv0Translated + streamdirTranslatedNormalized * intersectionStreamPos, Color3d(0.7, 0.7, 0.7));
                    }

                    Vector3d s = convertV3ToEigenDouble(streamp0.dirToNextPointNormalized);
                    s.normalize();
                    DD Lnew = newTotalLength;
                    Vector3d H = convertP3ToEigenDouble(v0) - convertP3ToEigenDouble(point.point);
                    Vector3d e = convertV3ToEigenDouble(edgedir) / edgeLength;
                    Vector3d E = e * edgePos;
                    Vector3d A = s.cross(convertP3ToEigenDouble(v0) - convertP3ToEigenDouble(streamv0));
                    Vector3d B = s.cross(E);
                    Vector3d b = s.cross(e);
                    DD s2 = shift_coeff * shift_coeff;
                    DD c1 = b.dot(b) - s2;
                    DD c2 = 2 * A.dot(b) - 2 * H.dot(e)* s2;
                    DD c3 = A.dot(A) - Lnew * Lnew*s2 - H.dot(H)* s2;
                    DD c4 = 2 * Lnew*s2;
                    DD coef4 = c1 * c1;
                    DD coef3 = 2 * c1*c2;
                    DD coef2 = 2 * c1*c3 + c2 * c2 - c4 * c4;
                    DD coef1 = 2 * c2*c3 - 2 * c4*c4*H.dot(e);
                    DD coef0 = (c3*c3 - c4 * c4*H.dot(H));
                    double xs[4] = { 0,0,0,0 };
                    int resCount = solve_quartic(coef3 / coef4, coef2 / coef4, coef1 / coef4, coef0 / coef4, xs);
                    auto getError = [&](DD x, DD& lnew, const Vector3d& normal, int& sign)
                    {
                        DD desiredDist = sqrt(A.dot(A) + b.dot(b)*x*x + 2 * A.dot(b)*x);
                        Point3d xp = convertP3ToEigenDouble(v0) + e * x;
                        Vector3d W = xp - convertP3ToEigenDouble(streamv0);
                        Vector3d s_cross_W = s.cross(W);
                        DD dist = s_cross_W.norm();
                        sign = s_cross_W.dot(normal) > 0 ? 1 : -1;
                        //DD dist = utils::point::DistToPoint(xp, pointOnStream);
                        DD err = abs(desiredDist - dist) / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);
                        lnew = (xp - convertP3ToEigenDouble(point.point)).norm();
                        return err;
                    };
                    double errs[4] = { 0,0,0,0 };
                    double lnews[4] = { 0,0,0,0 };
                    int signs[4] = { 0,0,0,0 };
                    for (int i = 0; i < resCount; i++) errs[i] = getError(xs[i], lnews[i], convertV3ToEigenDouble(fnormal), signs[i]);
                    int takeSign = (takeSolution == 1) ? 1 : -1;
                    int bestSolutionIndex = -1;
                    for (int i = 0; i < resCount; i++)
                    {
                        if (takeSign != signs[i]) continue;
                        if (bestSolutionIndex == -1 || errs[i] < errs[bestSolutionIndex])
                        {
                            bestSolutionIndex = i;
                        }
                    }
                    if (bestSolutionIndex == -1) continue;
                    edgePos = xs[bestSolutionIndex];
                }
                else
                {
                    continue;
                }

                //validate solution
                D newpointEdgePosPercent = (edgePos / edgeLength);
                bool edgePosIsOut = edgePos < 0 || edgePos > edgeLength;
                if (edgePosIsOut)
                {
                    if (edgePos < 0) newpointIsOutOfEdgeCounter -= 1;
                    if (edgePos > edgeLength) newpointIsOutOfEdgeCounter += 1;
                    edgePosIsOutCount++;
                    if (isDebug && options.debug_trace_iterations && options.debug_trace_iterations__tests && isDebugOnlyThisPoint)
                    {
                        cout << "      test#" << testid << "   streamPoint#" << points_startFromIndex + testid << "   edgePos=" << (edgePos / edgeLength) << "%   edgePosIsOut" << endl;
                    }
                    continue;
                }
                successfulTestsCount++;

                // test new points
                //StreamPoint newpoint(MeshPointType::onEdge, eid, v0 + edgedirNormalized * edgePos, nextfid);
                Point3d newpointDD = convertP3ToEigenDouble(v0) + convertV3ToEigenDouble(edgedir)*(edgePos / (DD)edgeLength);
                StreamPoint newpoint(MeshPointType::onEdge, eid, convertEigenToP3(newpointDD), nextfid);
                D newpoint_IsOutOfStreamPercent = utils::vector::ClosestPoint_ToLine_IsOnLinePercent(streamv0, streamv1, newpoint.point);

                #if DEBUG
                if (isDebug)
                {
                    DD newpoint_linelength = (newpointDD - convertP3ToEigenDouble(point.point)).norm();
                    DD desiredDist = pow(newpoint_linelength + newTotalLength, static_cast<int>(options.ShiftSmoothing) + 1) * shift_coeff;
                    int pointOnStream_pointIndex = points_startFromIndex + testid;
                    DD dist = origin_points.GetDistToLineIndexDD(newpointDD, points_startFromIndex + testid);
                    DD err = abs(desiredDist - dist) / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);

                    P3 pointOnStream = streamv0 + streamdir * newpoint_IsOutOfStreamPercent;
                    D err_angle = utils::vector::Angle(streamdir, newpoint.point - pointOnStream, false);

                    int temp = 0;
                }
                #endif

                if (isDebug && isDebugOnlyThisPoint)
                {
                    //D newpoint_linelength = utils::point::DistToPoint(newpoint.point, point.point);
                    DD newpoint_linelength = (newpointDD - convertP3ToEigenDouble(point.point)).norm();
                    //D desiredDist = pow(newpoint_linelength + newTotalLength, static_cast<int>(options.ShiftSmoothing) + 1) * shift_coeff;
                    DD desiredDist = pow(newpoint_linelength + newTotalLength, static_cast<int>(options.ShiftSmoothing) + 1) * shift_coeff;
                    //int pointOnStream_pointIndex = 0;
                    //P3 pointOnStream;
                    //D dist = utils::origin_points.GetClosestDistToPoints(max(0, points_startFromIndex - 2), newpoint.point, pointOnStream, pointOnStream_pointIndex);
                    int pointOnStream_pointIndex = points_startFromIndex + testid;
                    P3 pointOnStream = streamv0 + streamdir * newpoint_IsOutOfStreamPercent;
                    //D dist = utils::point::DistToPoint(newpoint.point, pointOnStream);
                    //DD dist = (newpointDD - convertP3ToEigenDouble(pointOnStream)).norm();
                    DD dist = origin_points.GetDistToLineIndexDD(newpointDD, points_startFromIndex + testid);
                    //D err = abs(desiredDist - dist) / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);
                    DD err = abs(desiredDist - dist) / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);
                    D err_angle = utils::vector::Angle(streamdir, newpoint.point - pointOnStream, false);// DEBUG


                    //DEBUG testing new method
                    // new method formulates formula: 
                    //                                      desiredDist_newmethod = (Lnew+lnew)*shift
                    //                                      sqrt(A.dot(A) + B.dot(B) + 2 * A.dot(B)) = (Lnew + sqrt(H.dot(H) + E.dot(E) + 2 * H.dot(E)))*shift
                    //                       or            sqrt(A.dot(A) + b.dot(b)*x*x + 2 * A.dot(b)*x) = (Lnew + sqrt(H.dot(H) + e.dot(e)*x*x + 2 * H.dot(e)*x))*shift
                    Vector3d W = convertP3ToEigenDouble(newpoint.point) - convertP3ToEigenDouble(streamv0);
                    Vector3d s = convertV3ToEigenDouble(streamp0.dirToNextPointNormalized);
                    s.normalize();
                    DD Lnew = newTotalLength;
                    DD lnew = (newpointDD - convertP3ToEigenDouble(point.point)).norm();
                    Vector3d H = convertP3ToEigenDouble(v0) - convertP3ToEigenDouble(point.point);
                    Vector3d e = convertV3ToEigenDouble(edgedir) / edgeLength;
                    Vector3d E = e * edgePos;
                    Vector3d A = s.cross(convertP3ToEigenDouble(v0) - convertP3ToEigenDouble(streamv0));
                    Vector3d B = s.cross(E);
                    Vector3d b = s.cross(e);
                    DD x = edgePos;
                    DD dist_newmethod = s.cross(W).norm(); // DEBUG - testing new method of getting dist from point to line. source: https://answers.unity.com/questions/568773/shortest-distance-from-a-point-to-a-vector.html
                    DD lnew_newmethod = sqrt(H.dot(H) + E.dot(E) + 2 * H.dot(E));
                    DD desiredDist_newmethod = sqrt(A.dot(A) + B.dot(B) + 2 * A.dot(B));
                    DD lnew_newmethodX = sqrt(H.dot(H) + x * x + 2 * H.dot(e)*x);
                    DD desiredDist_newmethodX = sqrt(A.dot(A) + b.dot(b)*x*x + 2 * A.dot(b)*x);
                    DD desiredDist_newmethod_calculated = (Lnew + lnew)*shift_coeff;
                    DD err_newmethod = abs(desiredDist_newmethod - dist) / (desiredDist_newmethod < 0.000000000001 ? 0.000000000001 : desiredDist_newmethod);
                    DD err_newmethodX = abs(desiredDist_newmethodX - dist) / (desiredDist_newmethodX < 0.000000000001 ? 0.000000000001 : desiredDist_newmethodX);
                    DD s2 = shift_coeff * shift_coeff;
                    DD c1 = b.dot(b) - s2;
                    DD c2 = 2 * A.dot(b) - 2 * H.dot(e)* s2;
                    DD c3 = A.dot(A) - Lnew * Lnew*s2 - H.dot(H)* s2;
                    DD c4 = 2 * Lnew*s2;
                    DD coef4 = c1 * c1;
                    DD coef3 = 2 * c1*c2;
                    DD coef2 = 2 * c1*c3 + c2 * c2 - c4 * c4;
                    DD coef1 = 2 * c2*c3 - 2 * c4*c4*H.dot(e);
                    DD coef0 = (c3*c3 - c4 * c4*H.dot(H));
                    DD eq4 = x * x*x*x*coef4 + x * x*x*coef3 + x * x*coef2 + x * coef1 + coef0;
                    double res[4] = { 0,0,0,0 };
                    int resCount = solve_quartic(coef3 / coef4, coef2 / coef4, coef1 / coef4, coef0 / coef4, res);
                    auto getError = [&](DD x, DD& lnew, const Vector3d& normal, int& sign)
                    {
                        DD desiredDist = sqrt(A.dot(A) + b.dot(b)*x*x + 2 * A.dot(b)*x);
                        Point3d xp = convertP3ToEigenDouble(v0) + e * x;
                        Vector3d W = xp - convertP3ToEigenDouble(streamv0);
                        Vector3d s_cross_W = s.cross(W);
                        DD dist = s_cross_W.norm();
                        sign = s_cross_W.dot(normal) > 0 ? 1 : -1;
                        //DD dist = utils::point::DistToPoint(xp, pointOnStream);
                        DD err = abs(desiredDist - dist) / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);
                        lnew = (xp - convertP3ToEigenDouble(point.point)).norm();
                        return err;
                    };
                    double errs[4] = { 0,0,0,0 };
                    double lnews[4] = { 0,0,0,0 };
                    int signs[4] = { 0,0,0,0 };
                    for (int i = 0; i < resCount; i++) errs[i] = getError(res[i], lnews[i], convertV3ToEigenDouble(fnormal), signs[i]);

                    if (options.debug_trace_iterations && options.debug_trace_iterations__tests)
                    {
                        int newpoint_IsOutOfStreamSign = utils::vector::ClosestPoint_ToLine_IsOutOfLineSign(streamv0, streamv1, newpoint.point);
                        cout << "      test#" << testid << "   streamPoint#" << points_startFromIndex + testid << "   edgePos=" << (edgePos / edgeLength) << "%   IsOutOfStreamSign=" << newpoint_IsOutOfStreamSign << "   IsOutOfStreamPercent=" << newpoint_IsOutOfStreamPercent << "%   err" << err << endl;
                    }
                    draw.AddPoint(newpoint.point, Color3d(1, 0, 0), "test#" + to_string(testid) + ",  err=" + to_string(err) + ",  side=" + (testSide == TestSide::Left ? "Left" : "Right"));
                }

                // newpoint should be in stream
                if (newpoint_IsOutOfStreamPercent < -0.1 || newpoint_IsOutOfStreamPercent > 1.1) continue;// 10% tolerance

                // try accept new point
                bool is_vid_swaped = (mesh.EV(eid, 0) != vid0);
                if (is_vid_swaped) newpointEdgePosPercent = 1 - newpointEdgePosPercent;
                TryRememberNewPointTo__test_min(ri_max_change_angle, testid, testSide == TestSide::Right ? 1 : -1, newpoint,
                    point, abs(new_lengthTo_pointOnPoints), lengthTo_pointOnPoints, try_min,
                    test_min, fid, shift_coeff, origin_points, points_startFromIndex + testid,
                    rot_sign, isDebug, isDebugThisPoint, isDebugThisIteration, newpointEdgePosPercent);

                if (test_min.isset && test_min.err < targetTollerance) break;
            }

            // if all newpoints is out of edge in same direction - means newpoint is on the vertex (only for first edge and only for same side)
            // - works sometimes incorrect - so dont make desicions in such case but let it to decide to iterative method
            //if ((newpointIsOutOfEdgeCounter == -testidMAX || newpointIsOutOfEdgeCounter == testidMAX) && tid == 0 && !testOppositeSide)
            //{
            //    //int out_test_vid = (newpointIsOutOfEdgeCounter == -testidMAX) ? vid0 : vid1; - wrong
            //    int out_test_vid = vid1; // always vid1, since there is only possible to go forward (vid0 is always go backward)
            //    if (isDebug && options.debug_trace_iterations && options.debug_trace_iterations__tests && isDebugOnlyThisPoint)
            //    {
            //        cout << "      all tests point to vertex vid=" << out_test_vid << " - testing vertex..." << endl;
            //    }
            //    newpoint.point = (newpointIsOutOfEdgeCounter == -testidMAX) ? v0 : v1;
            //    TryRememberNewPointTo__test_min(ri_max_change_angle, 0, 0, newpoint,
            //        point, abs(new_lengthTo_pointOnPoints), lengthTo_pointOnPoints, try_min,
            //        test_min, fidStart, shift_coeff, origin_points, points_startFromIndex,
            //        rot_sign, isDebug, isDebugThisPoint, true);
            //}
        }
    }


    if (test_min.isset)
    {
        try_min.Remember(test_min.test_id, test_min.err, test_min.dist, test_min.test_fid, test_min.point, test_min.pointOnStream_pointIndex, test_min.angleToStream);
        //if (isDebug)
        //{
        //    draw.AddPoint(try_min.point.point, Color3d(0, 1, 0), "#" + to_string(pointNum) + ",  err" + to_string(try_min.err));
        //}
    }
}

bool MeshStreamsAdjuster::AdjustStream_Ditsts(const P3& connectionPoint, const MeshStream& origin_points, const StreamStartPoint& sp, MeshStream& adjusted_points)
{
    //
    // check debug options
    //
    bool isDebug = options.DebugEnabled;

    if (isDebug && options.debug_debugStreamIndex != -1 && sp.Index != options.debug_debugStreamIndex) return false;
    if (origin_points.size() < 2) return false;


    //
    // init skip_points_count
    //
    int pointOnStream_pointIndex_start = sp.joinedAtIndex;
    int skip_points_count = max(0, options.skip_first_points);
    if (skip_points_count * 3 > pointOnStream_pointIndex_start) // ignore option if there are small amount of point to adjust - we have to make smooth adjusment so we can ignore some points only if we have a lot of points
    {
        skip_points_count = 0;
    }

    //
    // calculate 'lengthTo_pointOnPoints' on original stream
    //
    D lengthTo_pointOnPoints = 0;
    assert(origin_points.IsInited_lengthToNextPoint());
    for (int i = skip_points_count; i < sp.joinedAtIndex; i++)
    {
        lengthTo_pointOnPoints += origin_points[i].lengthToNextPoint;
    }
    P3 pointOnOriginStream = utils::vector::ClosestPoint_ToVector(origin_points[sp.joinedAtIndex].point, origin_points[sp.joinedAtIndex].dirToNextPointNormalized, 1, connectionPoint);
    lengthTo_pointOnPoints += utils::point::DistToPoint(origin_points[sp.joinedAtIndex].point, pointOnOriginStream);

    // define  desired dist
    D desiredDist = utils::point::DistToPoint(connectionPoint, pointOnOriginStream);

    // avoid adjustments if our stream is already enought close to connection points
    if (desiredDist < mesh.avg_edge_length * options.tollerance)
    {
        adjusted_points.clear();
        adjusted_points.Points = origin_points.Points;
        return true;
    }

    // prolong a bit target point on new stream (since we need hypotenuse) - this is very important for heavy shifted connection points - improves precision a lot
    lengthTo_pointOnPoints = sqrt(lengthTo_pointOnPoints * lengthTo_pointOnPoints + desiredDist * desiredDist);

    //
    // adjust stream first time
    //
    if (!AdjustStream_Ditsts_SingleIteration(0, lengthTo_pointOnPoints, connectionPoint, origin_points, sp, adjusted_points))
    {
        // for debuging - show  those unconnected streams - to see where is the issue
        if (options.DebugEnabled)
        {
            return true;
        }
        if (desiredDist < mesh.avg_edge_length *options.tollerance) // try to return true if possible - if desiredDist is of 25% of avarage mesh edge length
        {
            adjusted_points.clear();
            adjusted_points.Points = origin_points.Points;
            return true;
        }
        else
        {
            return false;
        }
    }

    //
    // calculate 'lengthTo_pointOnPoints' and 'adjustedDist' on adjusted stream
    //
    adjusted_points.Init_lengthToNextPoint();
    int pointOnAdjustedStream_pointIndex = 0;
    P3 pointOnAdjustedStream;
    bool breakAtFirstContact = false; // must be false, because in complicated objects some lines goes 90 degrees to others and with 'breakAtFirstContact=true' distance will be calculated very baddly
    D adjustedDist = adjusted_points.GetClosestDist_Linear(skip_points_count, connectionPoint, pointOnAdjustedStream, pointOnAdjustedStream_pointIndex, breakAtFirstContact, true);

    // if by luck we got fine result from first try - just return it, no need for additional adjustments
    if (mesh.min_edge_length > 0.000000000001 &&  adjustedDist / mesh.min_edge_length < options.tollerance)
    {
        return true;
    }


    //
    // iterate stream adjustment using more precisely calculated 'lengthTo_pointOnPoints'
    //
    if (options.iterate_adjustments)
        for (int shiftIterationIndex = 1; shiftIterationIndex <= options.iterate_adjustments__iterations_count; shiftIterationIndex++)
        {
            D err = adjustedDist / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);
            if (err < options.tollerance)
            {
                break; // we dont need search for better result since our error inside desired scope
            }

            // calculate 'lengthTo_pointOnPoints' this time using adjusted stream - so it will be more precise this time, and thus adjustedStream should be closer to connection point
            D lengthTo_pointOnPoints2 = 0;
            for (int i = skip_points_count; i < pointOnAdjustedStream_pointIndex; i++)
            {
                lengthTo_pointOnPoints2 += adjusted_points[i].lengthToNextPoint;
            }
            lengthTo_pointOnPoints2 += utils::point::DistToPoint(adjusted_points[pointOnAdjustedStream_pointIndex].point, pointOnAdjustedStream);

            // create adjusted_points2
            MeshStream adjusted_points2(adjusted_points.Index, adjusted_points.StreamStartPointId, adjusted_points.StreamAnchorPointId, adjusted_points.draw, adjusted_points.mesh, adjusted_points.color, adjusted_points.dividingIteration);

            // adjust stream this time using more precisely calculated 'lengthTo_pointOnPoints'
            if (!AdjustStream_Ditsts_SingleIteration(shiftIterationIndex, lengthTo_pointOnPoints2, connectionPoint, origin_points, sp, adjusted_points2))
            {
                break;
            }

            // calculate dist to more precisely caluclate adjusted stream
            adjusted_points2.Init_lengthToNextPoint();
            D adjustedDist2 = adjusted_points2.GetClosestDist_Linear(skip_points_count, connectionPoint, pointOnAdjustedStream, pointOnAdjustedStream_pointIndex, breakAtFirstContact, true);
            D err2 = adjustedDist2 / (desiredDist < 0.000000000001 ? 0.000000000001 : desiredDist);
            bool improved = adjustedDist2 < adjustedDist*0.95;// new dists improved from previous result at least for 5%


            // DEBUG - show iteration progress in console
            if (options.iterate_adjustments__trace_progress_in_console)
            {
                cout << "stream#" << sp.Index << "   shiftIterationIndex#" << shiftIterationIndex << "   was [err=" << err << ",adjustedDist=" << adjustedDist << "]     readjusted [err2=" << err2 << ", adjustedDist2=" << adjustedDist2 << "]      " << (improved ? "improved, accepted" : "not better, rejected") << endl;
            }
            // remember better results
            if (improved)
            {
                adjustedDist = adjustedDist2;                              // remember closest dist
                adjusted_points.SetNewPoints(adjusted_points2.Points); // remember closest 'adjusted_points' to 'connectionPoint'
            }
            else
            {
                // new adjustment didnt make results better, so no more use in iterations
                break;
            }
        }

    return true;
}


bool MeshStreamsAdjuster::AdjustStream_Ditsts_SingleIteration(int shiftIterationIndex, D lengthTo_pointOnPoints, const P3& connectionPoint, const MeshStream& origin_points, const StreamStartPoint& sp, MeshStream& adjusted_points)
{
    //
    // init common variables
    //
    bool isDebug = options.DebugEnabled;
    //if (mesh.id == 64 && origin_points.Index == 17)
    //{
    //    isDebug = true;
    //    options.debug_debugStreamId = 1;
    //}
    if (isDebug && options.debug_debugStreamIndex != -1 && sp.Index != options.debug_debugStreamIndex) return false;
    if (origin_points.size() < 2) return false;

    //D join_tollerance = options.tollerance*mesh.avg_edge_length; // v1 - edge tolerance
    D join_tollerance = options.tollerance; // v2 - desiredDist tolerance
    int singularityExtendCount = isDebug ? options.debug_singularityExtendNum : meshLogicOptions.Divider.SingularityExtendCount;
    int isoLinesExtensionsCount = min(singularityExtendCount, mesh.FacesCount);

    //
    // get closest point on original stream
    //
    // v1 - calculate manually
    //P3 pointOnStream;
    //int pointOnStream_pointIndex_start;    
    //D startDistFromConnectionPointToStream = origin_points.GetClosestDistToPoints(0, connectionPoint, pointOnStream, pointOnStream_pointIndex_start);
    // v2 - get information from 'sp'
    int pointOnStream_pointIndex_start = sp.joinedAtIndex;
    P3 pointOnStream = utils::vector::ClosestPoint_ToVector(origin_points[sp.joinedAtIndex].point, origin_points[sp.joinedAtIndex].dirToNextPointNormalized, 1, connectionPoint); // must be to vector since here distance is very improtant and if we take dist to line - dist can be bigger from ortogonal distance to vector
    D startDistFromConnectionPointToStream = utils::point::DistToPoint(pointOnStream, connectionPoint);

    //
    // get rotational angle sign
    //
    V3 rot_vector_stream = origin_points[pointOnStream_pointIndex_start + 1].point - origin_points[pointOnStream_pointIndex_start].point;
    V3 rot_vector_stream_normal = mesh.F_normals.row(origin_points[pointOnStream_pointIndex_start].fid);
    V3 rot_vector_connectionPoint = connectionPoint - origin_points[pointOnStream_pointIndex_start].point;
    V3 rot_vector_cross = utils::vector::Cross(rot_vector_stream, rot_vector_connectionPoint);
    int rot_sign = utils::vector::SameDirectionIfTheyParallel(rot_vector_cross, rot_vector_stream_normal) ? -1 : 1; // 1 if clockwise, and -1 is counterclockwise
    // DEBUG - show rot vectors
    if (isDebug)
    {
        //draw.AddEdge(pointOnStream, rot_vector_cross, mesh.avg_edge_length, Color3d(1, 0, 0), "rot_vector_cross");
        //draw.AddEdge(origin_points[pointOnStream_pointIndex_start].point, rot_vector_stream, mesh.avg_edge_length, Color3d(0, 0, 1), "rot_vector_stream");
        //draw.AddEdge(origin_points[pointOnStream_pointIndex_start].point, rot_vector_connectionPoint, mesh.avg_edge_length, Color3d(0, 1, 0), "rot_vector_connectionPoint");
        //V3 pointOnStream_to_connectionPoint_vector = connectionPoint - pointOnStream;
        //draw.AddPoint(pointOnStream, Color3d(0, 0, 1), " pointOnStream");
        //draw.AddEdge(connectionPoint, pointOnStream, Color3d(0, 0, 1), "pointOnStream_to_connectionPoint");
    }

    //
    // init skip_points_count
    //
    int skip_points_count = max(0, options.skip_first_points);
    if (skip_points_count * 3 > pointOnStream_pointIndex_start) // ignore option is there are small amount of point to adjust - we have to make smooth adjusment so we can ignore some points only if we have a lot of points
    {
        skip_points_count = 0;
    }


    //
    // calculate 'shift_coeff'
    //
    D lengthTo_pointOnPoints_mult_2 = lengthTo_pointOnPoints * 2;
    D shift_coeff = startDistFromConnectionPointToStream / pow(lengthTo_pointOnPoints, static_cast<int>(options.ShiftSmoothing) + 1);
    //D shift_coeff = startDistFromConnectionPointToStream / pow(sqrt(lengthTo_pointOnPoints*lengthTo_pointOnPoints+ startDistFromConnectionPointToStream* startDistFromConnectionPointToStream), static_cast<int>(options.ShiftSmoothing) + 1);

    //
    // clear adjusted_points
    //
    adjusted_points.clear();
    adjusted_points.reserve(origin_points.size() * 2);
    for (int i = 0; i <= skip_points_count; i++)
    {
        adjusted_points.Add(origin_points[i]);
    }


    //
    // DEBUG - show connection point and pointOnStream
    //
    if (isDebug)
    {
        //cout << "Divider::AdjustStream -     origin dist = " << startDistFromConnectionPointToStream << endl;
        string text = ",  dist=" + to_string(startDistFromConnectionPointToStream) + ",  length=" + to_string(lengthTo_pointOnPoints) + ",  rot_sign=" + to_string(rot_sign) + ", pointOnStream_pointIndex_start=" + to_string(pointOnStream_pointIndex_start) + ", shortest length=" + to_string(utils::point::DistToPoint(pointOnStream, origin_points[0].point));
        cout << "stream#" << sp.Index << "   pointIndex=" << pointOnStream_pointIndex_start << text << endl;
        draw.AddPoint(pointOnStream, Color3d(1, 0, 0), "pointOnStream" + text);
        if (options.debug_show_stream_to_connection_point)
        {
            int iMax = pointOnStream_pointIndex_start;
            if (origin_points.size() - 2 < iMax) iMax = origin_points.size() - 2;
            for (int i = 0; i <= iMax; i++)
            {
                draw.AddPoint(origin_points[i].point, Color3d(1, 0, 0), to_string(i));
                draw.AddEdge(origin_points[i].point, i == iMax ? pointOnStream : origin_points[i + 1].point, Color3d(1, 0, 0));
            }
            //draw.AddEdge(origin_points[iMax].point, pointOnPoints, Color3d(1, 0, 0));
        }
    }

    //
    // Init loop
    //
    V3 dir = origin_points[skip_points_count + 1].point - origin_points[skip_points_count].point;
    D new_lengthTo_pointOnPoints = 0;
    int pointOnStream_pointIndex__prev_point = 0; // speed optimization for very long streams - start searching from prev point
    D ri_max_change_angle = adjusted_points.size() == 1 && adjusted_points[0].Type == MeshPointType::onVertex
        ? min(options.iterative_calculations__max_change_angle, sp.maxAngleChangeForDir)
        : options.iterative_calculations__max_change_angle;
    D prev_newpoint_linelength = 0;

    //
    // start loop
    //
    bool isStreamAdjusted = false;
    int pointNum = skip_points_count;
    while (pointNum < isoLinesExtensionsCount)
    {
        if (new_lengthTo_pointOnPoints > lengthTo_pointOnPoints_mult_2)
        {
            isStreamAdjusted = true;
            break;
        }
        const StreamPoint& point = adjusted_points[adjusted_points.size() - 1];
        bool isDebugOnlyThisPoint = (options.debug_debugPointNum != -1 && pointNum == options.debug_debugPointNum);
        bool isDebugThisPoint = (options.debug_debugPointNum == -1 || pointNum == options.debug_debugPointNum);
        if (point.fid == -1)
        {
            isStreamAdjusted = (new_lengthTo_pointOnPoints > lengthTo_pointOnPoints*0.8);
            break; // if we reach border - stop 
        }

        Minerr try_min(point, 0);
        try_min.pointOnStream_pointIndex = pointOnStream_pointIndex__prev_point;

        if (isDebug && sp.Index == options.debug_debugStreamIndex
            && (options.debug_debugPointNum == -1 || (options.debug_debugPointNum != -1 && options.debug_debugPointNum - 3 <= pointNum && pointNum <= options.debug_debugPointNum + 3)))
        {
            cout << endl << "point#" << pointNum << "   fid=" << point.fid << (point.Type == MeshPointType::onVertex ? "   vid=" : "   eid=") << point.vid_eid_fid << endl;
        }

        int testsMade = 0;

        // direct
        if (options.run_direct_calculations)
        {
            AdjustStream_Ditsts_direct(connectionPoint, origin_points, pointOnStream_pointIndex__prev_point,
                pointNum, point, dir,
                shift_coeff, rot_sign, new_lengthTo_pointOnPoints, lengthTo_pointOnPoints,
                join_tollerance, ri_max_change_angle,
                isDebug, isDebugThisPoint,
                try_min, testsMade);
            callsCount_directMethod++;

            // update dir if err is still higher from desired 
            // - bad idea, since if direct method calculate wrong direction - iterative method can fail as well since direction will be worng from start
            //if (try_min.isset && (try_min.err > join_tollerance && try_min.point.Type != MeshPointType::onVertex))// || try_min.angleToStream > 20
            //{
            //    dir = try_min.point.point - point.point;
            //    dir.normalize();
            //}
        }

        // iterative
        if (options.run_iterative_calculations)
        {
            if (!try_min.isset || try_min.err > join_tollerance) 
            {
                Minerr saved_try_min = try_min;
                AdjustStream_Ditsts_iterative(connectionPoint, origin_points, pointOnStream_pointIndex__prev_point,
                    pointNum, point, dir,
                    shift_coeff, rot_sign, new_lengthTo_pointOnPoints, lengthTo_pointOnPoints,
                    join_tollerance, ri_max_change_angle,
                    isDebug, isDebugThisPoint,
                    try_min, testsMade);
                callsCount_iterativeMethod++;

                if (isDebug && isDebugThisPoint && options.debug_trace_iterations)
                {
                    cout << "   called iterative method for ";
                    if (try_min.isset)
                    {
                        cout << saved_try_min.point.vid_eid_fid_toString() << "  err=" << saved_try_min.err;
                    }
                    else
                    {
                        cout << "!try_min.isset";
                    }
                    cout << "       new values:  " << try_min.point.vid_eid_fid_toString() << "  err=" << try_min.err << endl;
                }
            }
        }

        // stop if we reach border
        if (!try_min.isset)
        {
            #if DEBUG
            cout << endl << "point#" << (pointNum + 1) << "   fid=" << "   eid=" << endl << "   solution not found!   direct and iterative methods failed to get next point! " << endl;
            #endif
            isStreamAdjusted = (new_lengthTo_pointOnPoints > lengthTo_pointOnPoints*0.8);
            break;
        }

        // remember best pointIndex to start next iteration from this point - this will improve speed since we will cut some points on stream from check if they close to new point
        pointOnStream_pointIndex__prev_point = try_min.pointOnStream_pointIndex;

        // store new point in result output
        //utils::vector::Translate(dir, mesh.F_normals.row(point.fid), mesh.F_normals.row(try_min.point.fid));
        adjusted_points.Add(try_min.point);

        // check if prelast point-faceid has changed during change angle
        // this can heppend when start point is vertex and we change angle so the next faceid is changed
        if (adjusted_points.size() >= 2)
        {
            const StreamPoint& pointLast1 = adjusted_points[adjusted_points.size() - 1];
            StreamPoint& pointLast2 = adjusted_points[adjusted_points.size() - 2];
            if (pointLast2.Type == MeshPointType::onVertex)
            {
                if (pointLast1.Type == MeshPointType::onEdge)
                {
                    int commonFaceId1;
                    int commonFaceId2;
                    mesh.CommonFaceIds_VertexEdge(pointLast2.vid_eid_fid, pointLast1.vid_eid_fid, commonFaceId1, commonFaceId2);
                    if (commonFaceId1 != -1 && commonFaceId1 != pointLast2.fid &&commonFaceId2 == -1)
                    {
                        pointLast2.fid = commonFaceId1;
                    }
                }
                if (pointLast1.Type == MeshPointType::onVertex)
                {
                    int commonFaceId1;
                    int commonFaceId2;
                    mesh.CommonFaceIds_VertexVertex(pointLast2.vid_eid_fid, pointLast1.vid_eid_fid, commonFaceId1, commonFaceId2);
                    if (commonFaceId1 != -1 && commonFaceId1 != pointLast2.fid && commonFaceId2 != pointLast2.fid)
                    {
                        pointLast2.fid = commonFaceId1;
                    }
                }
            }
        }

        // check if stream was forced to move to vertex - it can happend if newpoint was very close to vertex
        // if 3 points are belong to same face - we have to remove middle point to avoid high angle jumps
        if (adjusted_points.Remove_PrelastPoint_If_LastIsVertex_And3LastsHaveCommonFace())
        {
            new_lengthTo_pointOnPoints -= prev_newpoint_linelength;
            if (isDebug && isDebugThisPoint)
            {
                cout << "      removed prelast point since last 3 points had same face" << endl;
            }
        }

        const StreamPoint& newpoint = adjusted_points.Points.back();
        const StreamPoint& prevpoint = adjusted_points[adjusted_points.size() - 2];

        // DEBUG - show new point
        if (isDebug && isDebugThisPoint)
        {
            if (sp.Index == options.debug_debugStreamIndex)
            {
                cout << "   solution found:  err=" << try_min.err << ",   angleToStream=" + to_string(try_min.angleToStream) << ",  testsMade=" << testsMade << endl;
            }
            if (options.debug_draw_accepted_points)
            {
                Color3d color = try_min.err < (options.tollerance) ? Color3d(0, 1, 0) : Color3d(1, 0, 0);
                draw.AddPoint(newpoint.point, color, " #" + to_string(pointNum) + "  err=" + to_string(try_min.err) + "   angleToStream=" + to_string(try_min.angleToStream) + "   testsMade=" + to_string(testsMade));
                //draw.AddPoint(end.point, Color3d(0, 1, 0), "dist=" + to_string(try_min.dist) + ",  err=" + to_string(try_min.err));
                //draw.AddEdge(origin_points[i].point, i == iMax ? pointOnStream : origin_points[i + 1].point, Color3d(0, 0, 1));                            
            }
            if (options.debug_debugPointNum == -1 && (options.debug_debugStreamIndex == -1 || options.debug_debugStreamIndex == sp.Index))
            {
                draw.AddEdge(prevpoint.point, newpoint.point, Color3d(0, 1, 0));
                draw.AddPoint(newpoint.point, Color3d(0, 1, 0), " #" + to_string(pointNum));
            }
        }

        // increment 'new_lengthTo_pointOnPoints'
        D newpoint_linelength = utils::point::DistToPoint(newpoint.point, prevpoint.point);
        new_lengthTo_pointOnPoints += newpoint_linelength;
        prev_newpoint_linelength = newpoint_linelength;

        // stop if we reach border
        if (newpoint.isOnBorder(mesh))
        {
            isStreamAdjusted = (new_lengthTo_pointOnPoints > lengthTo_pointOnPoints*0.8);
            break;
        }

        // stop if we did enought work
        if (try_min.pointOnStream_pointIndex > pointOnStream_pointIndex_start + 20 && new_lengthTo_pointOnPoints > lengthTo_pointOnPoints) // 20 more steps in advance just to make more posibilities for connection two streams
        {
            isStreamAdjusted = (new_lengthTo_pointOnPoints > lengthTo_pointOnPoints);
            break;
        }

        //
        // Accept new point
        //
        // increase point num
        pointNum++;
        // set new dir
        dir = newpoint.point - prevpoint.point;
        if (newpoint.fid != -1)
        {
            dir = utils::vector::Translate(dir, mesh.F_normals.row(prevpoint.fid), mesh.F_normals.row(newpoint.fid), true);
        }
        dir.normalize();

        // DEBUG - show direction for next point
        isDebugOnlyThisPoint = (options.debug_debugPointNum != -1 && pointNum == options.debug_debugPointNum);
        isDebugThisPoint = (options.debug_debugPointNum == -1 || pointNum == options.debug_debugPointNum);
        if (isDebug && isDebugThisPoint && options.debug_draw_accepted_points && isDebugOnlyThisPoint)
        {
            if (isDebugOnlyThisPoint)
            {
                draw.AddEdge(newpoint.point, dir, mesh.avg_edge_length, Color3d(0, 0, 0), "dir");
                //P3 pointOnDir = utils::vector::ClosestPoint_ToVector2(dir, prevpoint.point, newpoint.point);
                //draw.AddEdge(newpoint.point, pointOnDir, Color3d(0, 0, 0), "dir-to-newpoint");
                //cout << "newpoint to dir dist=" <<  << endl;
            }
        }
    }


    return isStreamAdjusted;
}

