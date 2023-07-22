#include "stdafx.h"
#include "MeshCutter.h"
#include "Divider.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshStreams.h"
#include "MeshSolverNrosy.h"
#include "MeshSurface.h"
#include <unordered_map>

const MeshLogicOptions_MeshCutter& options = meshLogicOptions.MeshCutter;



MeshCutter::MeshCutter(ViewerDrawObjects& _draw, const Mesh& _mesh, bool _splitNakedTriangleWith2NakedEdge, bool _correctFacesEdgeIndexes, bool _optimizeVertexIndexesForGPU)
    : mesh_original(_mesh), draw(_draw), avg_edge_length(_mesh.avg_edge_length),
    splitNakedTriangleWith2NakedEdge(_splitNakedTriangleWith2NakedEdge), correctFacesEdgeIndexes(_correctFacesEdgeIndexes), optimizeVertexIndexesForGPU(_optimizeVertexIndexesForGPU)
{
}

II MeshCutter::SizeOF() const
{
    II r = sizeof(MeshCutter);
    for (auto& m : cutedMeshes) r += m.SizeOF();
    return r;
}

void MeshCutter::checkStreamsForDuplicatedPoints(vector<MeshStream>& streams, string callerName)
{
    for (auto& stream : streams)
    {
        bool debug = false;
        #if DEBUG
        debug = options.DebugEnabled;
        #endif
        int removedCount = stream.RemoveDuplicatePoints(debug);
        if (removedCount > 0 && debug)
        {
            cout << "!!! removed " << removedCount << " indentical points in meshid=" << mesh_original.id << " at method '" << callerName << endl;
        }
    }
}

void MeshCutter::Cut(const vector<MeshStream>& divider_streams)
{
    //
    // copy stream points excluding empty streams (that was removed when merging streams)
    //
    vector<MeshStream> streams;
    for (const MeshStream& s : divider_streams)
    {
        if (s.size() > 0)
        {
            streams.push_back(s);
        }
    }
    checkStreamsForDuplicatedPoints(streams, "divider.streams.streams");
    checkStreamsForDuplicatedPoints(streams, "divider.streams.streams 2");

    //
    // copy mesh to be able edit it
    //
    //Mesh m = mesh_original;
    const Mesh& m = mesh_original;
    P3s m_V = mesh_original.V; // duplicate V since we can change it in method 'ImproveMeshToConnectStreamLines'


    //
    // ImproveStreamLines
    //
    if (options.ImproveStreamLines)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::ImproveStreamLines ..." << endl;
        ImproveStreamLines(m, m_V, streams);
        checkStreamsForDuplicatedPoints(streams, "ImproveStreamLines1");
    }

    //
    // MergeClosePointsOnEdges
    //
    int intersectoinInfo_currentId = 0;
    bool debug_show_intersections = options.DebugEnabled && options.Debug_show_intersections;
    if (options.MergeClosePointsOnEdges)
    {
        MergeClosePointsOnEdges(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        checkStreamsForDuplicatedPoints(streams, "MergeClosePointsOnEdges");
    }

    //
    // ImproveMesh
    //
    if (options.ImproveMeshToConnectStreamLines)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::ImproveMeshToConnectStreamLines ..." << endl;
        bool allow_improvements_on_borders = options.ImproveMeshToConnectStreamLines__allow_improvements_on_borders;
        bool dissalow_move_other_streams = options.ImproveMeshToConnectStreamLines__dissalow_move_other_streams;
        ImproveMeshToConnectStreamLines(m, m_V, streams, allow_improvements_on_borders, dissalow_move_other_streams);
        checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines");
    }

    //
    // ImproveStreamLines second times
    //
    if (options.ImproveStreamLines)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::ImproveStreamLines ..." << endl;
        ImproveStreamLines(m, m_V, streams);
        checkStreamsForDuplicatedPoints(streams, "ImproveStreamLines2");
    }

    //
    // FindStreamIntersections
    //
    if (options.FindStreamIntersections)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::FindStreamIntersections ..." << endl;
        FindStreamIntersections_Vertex(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Vertex");
        FindStreamIntersections_Edge(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Edge");
        FindStreamIntersections_Face(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Face");
        FindStreamEdgeIntersection(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        checkStreamsForDuplicatedPoints(streams, "FindStreamEdgeIntersection");
    }

    //
    // Validate intersections ids
    //
    ValidateIntersectionsIds(streams, intersectoinInfo_currentId);

    //
    //CutMesh
    //
    cutedMeshes.clear();
    vector<Mesh>& ms = cutedMeshes;
    if (options.CutMesh && streams.size() > 0)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::CutMesh ..." << endl;
        CutMesh(m, m_V, streams, ms);
    }
    else
    {
        ms.push_back(m);
    }


    if (options.DebugEnabled && options.Debug_ShowModifiedMesh)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::Debug_ShowWorkMesh ..." << endl;
        for (int i = 0; i < ms.size(); i++)
        {
            Debug_ShowWorkMesh(ms[i], options.Debug_ShowModifiedMesh_onlyborders);
        }
    }
}
int MeshCutter::FindStreamIntersections(vector<MeshStream>& streams, bool debug_show_intersections)
{
    return FindStreamIntersections(streams, options.ImproveStreamLines, options.MergeClosePointsOnEdges, options.ImproveMeshToConnectStreamLines, debug_show_intersections);
}
int MeshCutter::FindStreamIntersections(vector<MeshStream>& streams, bool improveStreamLines, bool mergeClosePointsOnEdges, bool improveMeshToConnectStreamLines, bool debug_show_intersections)
{
    int intersectoinInfo_currentId = 0;

    //
    // copy mesh to be able edit it
    //
    const Mesh& m = mesh_original;
    const P3s& m_V = m.V;  // we dont need duplicate V since we dont call a method 'ImproveMeshToConnectStreamLines'

    //
    // ImproveStreamLines
    //
    if (improveStreamLines)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::ImproveStreamLines ..." << endl;
        ImproveStreamLines(m, m_V, streams);
    }

    //
    // MergeClosePointsOnEdges
    //
    if (mergeClosePointsOnEdges)
    {
        MergeClosePointsOnEdges(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
    }

    //
    // ImproveMesh
    //
    if (improveMeshToConnectStreamLines)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::ImproveMeshToConnectStreamLines ..." << endl;
        ImproveMeshToConnectStreamLines_removeReduntantPoints(m, m_V, streams);
    }

    //
    // ImproveStreamLines second times
    //
    if (improveStreamLines)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::ImproveStreamLines ..." << endl;
        ImproveStreamLines(m, m_V, streams);
    }



    //
    // FindStreamIntersections
    //
    if (options.FindStreamIntersections)
    {
        //if (options.DebugEnabled) cout << "MeshCutter::FindStreamIntersections ..." << endl;
        FindStreamIntersections_Vertex(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        FindStreamIntersections_Edge(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
        FindStreamIntersections_Face(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
    }


    //
    // Validate intersections ids
    //
    ValidateIntersectionsIds(streams, intersectoinInfo_currentId);


    return intersectoinInfo_currentId;
}





void MeshCutter::Debug_ShowWorkMesh(const Mesh& mi, bool onlyborders)
{
    draw.ReserveEdges(mi.EV.rows());
    for (int i = 0; i < mi.EV.rows(); i++)
    {
        if (onlyborders && !mi.E_isborder[i]) continue;
        Color3d color = mi.E_isborder[i] ? Color3d(0, 0, 0) : Color3d(0.7, 0.7, 0.7);
        int vid0 = mi.EV(i, 0);
        int vid1 = mi.EV(i, 1);
        P3 p0 = mi.V.row(vid0);
        P3 p1 = mi.V.row(vid1);
        draw.AddEdge(p0, p1, color);
    }
}

int MeshCutter::MergeClosePointsOnEdgesOnce(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections)
{
    //test file: AddPresetTestFile(TestFiles::korzyna_small, TestFilesDensity::low, 1, "F128 F131 F132 F135  F009 F010 F111");

    //
    // get information what edges have at least 2 streampoints 
    // 
    Is edgesCrossesCount = Is::Zero(m.EdgesCount);
    int unique_edgesCrossesCount = 0;
    for (MeshStream& s : streams)
    {
        for (int i = 0; i < s.Points.size(); i++)
        {
            const StreamPoint& p = s.Points[i];
            if (p.Type == MeshPointType::onEdge)
            {
                int eid = p.vid_eid_fid;
                edgesCrossesCount(eid)++;
                if (edgesCrossesCount(eid) == 2) unique_edgesCrossesCount++; // when at lease 2 streams intersects edge then we count this edge                
            }
        }
    }
    if (unique_edgesCrossesCount == 0) return 0;

    //
    // get edges where streams intersects
    //
    struct intersectoinInfo
    {
        MeshStream* s;
        StreamPoint* p;
        int point_index; // point index in stream
        D from_v0_to_p_lenPow2; // length from start of the edge to current stream point (will be used for sorting only if there will be more than 2 points on edge)
        StreamPoint getPoint(int shiftIndex) const
        {
            return s->Points[point_index + shiftIndex];
        }
    };
    map<int, vector<intersectoinInfo>> map_eid_intersectoinInfos;
    for (MeshStream& s : streams)
    {
        for (int i = 0; i < s.Points.size(); i++)
        {
            StreamPoint& p = s.Points[i];
            if (p.Type == MeshPointType::onEdge)
            {
                int eid = p.vid_eid_fid;
                if (edgesCrossesCount(eid) >= 2) // at least two streams crosses this edge
                {
                    auto find = map_eid_intersectoinInfos.find(eid);
                    if (find == map_eid_intersectoinInfos.end())
                    {
                        map_eid_intersectoinInfos[eid] = vector<intersectoinInfo>();
                        find = map_eid_intersectoinInfos.find(eid);
                    }
                    vector<intersectoinInfo>& iis = find->second;
                    intersectoinInfo ii;
                    ii.s = &s;
                    ii.p = &p;
                    ii.point_index = i;
                    ii.from_v0_to_p_lenPow2 = 0; // set undefined value
                    iis.push_back(ii);
                }
            }
        }
    }

    //
    // sort 'vector<intersectoinInfo>' in 'map_eid_intersectoinInfos'
    //
    for (auto& o : map_eid_intersectoinInfos)
    {
        int eid = o.first;
        vector<intersectoinInfo>& iis = o.second;
        //sort only if we have more then 2 points on edge
        if (iis.size() > 2)
        {
            int vid0 = m.EV(eid, 0);
            P3 v0 = m_V.row(vid0);
            for (auto& ii : iis)
            {
                ii.from_v0_to_p_lenPow2 = utils::point::DistToPointPow2(v0, ii.p->point);
            }
            // sort values of map_eid_intersectoinInfos
            std::sort(iis.begin(), iis.end(), [](const intersectoinInfo &a, const intersectoinInfo &b)
            {
                return a.from_v0_to_p_lenPow2 < b.from_v0_to_p_lenPow2;
            });
        }
    }

    //
    // merge points on same edge if distance and angle between them is small 
    //
    int iterationNum = -1;
    int mergedPointsCount = 0;
    int mergedPointsCounti = 1; // set 1 just to make first time 'while' condion true: 'while (mergedPointsCounti > 0)'
    D edge_percent_tolerance = options.MergeClosePointOnEdges__edge_percent_tolerance;
    D tol = avg_edge_length * edge_percent_tolerance;
    D tolPow2 = tol * tol;
    D max_angle_connection_COS = utils::angle::DegreesToCos(options.MergeClosePointOnEdges__connection_angle);
    D max_angle_change_belowConnectionAngle_COS = utils::angle::DegreesToCos(options.MergeClosePointOnEdges__max_angle_change_belowConnectionAngle);
    bool isDisabled__angle_change_below = (options.MergeClosePointOnEdges__max_angle_change_belowConnectionAngle < 0);
    D max_angle_change_aboveConnectionAngle_COS = utils::angle::DegreesToCos(options.MergeClosePointOnEdges__max_angle_change_aboveConnectionAngle);
    bool isDisabled__angle_change_above = (options.MergeClosePointOnEdges__max_angle_change_aboveConnectionAngle < 0);
    //while (mergedPointsCounti > 0)  doesnt need for now - but if there will be an issue that will be needed few iterations - code is ready
    {
        iterationNum++;
        mergedPointsCounti = 0;
        StreamPoint zeroStreamPoint(MeshPointType::onEdge, 0, P3(0, 0, 0), 0);
        pair<StreamPoint, StreamPoint> zero = { zeroStreamPoint , zeroStreamPoint };
        pair<StreamPoint, StreamPoint> friend_points[4]{ zero, zero, zero, zero };
        for (auto& o : map_eid_intersectoinInfos)
        {
            int eid = o.first;
            vector<intersectoinInfo>& iis = o.second;
            if (iis.size() < 2) continue;

            // set 'intersectionID' for those points that are close to each other
            for (int i = 0; i < iis.size() - 1; i++)
            {
                const intersectoinInfo& ii0 = iis[i];
                const intersectoinInfo& ii1 = iis[i + 1];
                if (ii0.p->intersectionID != -1 && ii0.p->intersectionID == ii1.p->intersectionID) continue;

                // check dist
                D distPow2 = utils::point::DistToPointPow2(ii0.p->point, ii1.p->point);
                if (distPow2 > tolPow2) continue;

                // check angle connection
                V3 dir0 = (ii0.point_index < ii0.s->size() - 1)
                    ? (ii0.getPoint(1) - ii0.getPoint(0))
                    : (ii0.getPoint(0) - ii0.getPoint(-1));
                V3 dir1 = (ii1.point_index < ii1.s->size() - 1)
                    ? (ii1.getPoint(1) - ii1.getPoint(0))
                    : (ii1.getPoint(0) - ii1.getPoint(-1));
                dir0.normalize();
                dir1.normalize();
                D angle_connection_COS = utils::vector::Cos(dir0, dir1, true);
                if (angle_connection_COS < 0) angle_connection_COS = utils::vector::Cos(dir0, -dir1, true); // if vectors opposite - change direction, so we will get: if (angle>90) angle=180-angle;
                bool isOrtogonal = (abs(angle_connection_COS) < max_angle_connection_COS);
                D max_angle_change_COS = isOrtogonal // set max angle change base on currection connection angle: for ortogonal connections lets set max angle 1 and for parallel 35
                    ? max_angle_change_aboveConnectionAngle_COS
                    : max_angle_change_belowConnectionAngle_COS;
                if (isOrtogonal && iterationNum > 0) continue; // iterative process made only for parallel connection - for ortogonal iterations will not change anything so execution code below is useless
                if (isOrtogonal && isDisabled__angle_change_above) continue;
                if (!isOrtogonal && isDisabled__angle_change_below) continue;

                // check angle after
                P3 point_changed = (ii0.p->point + ii1.p->point) / 2;
                int friend_points_count = 0;
                if (ii0.point_index > 0) friend_points[friend_points_count++] = { ii0.getPoint(0), ii0.getPoint(-1) };
                if (ii0.point_index < ii0.s->size() - 1) friend_points[friend_points_count++] = { ii0.getPoint(0), ii0.getPoint(+1) };
                int friend0_points_count = friend_points_count;
                if (ii1.point_index > 0) friend_points[friend_points_count++] = { ii1.getPoint(0), ii1.getPoint(-1) };
                if (ii1.point_index < ii1.s->size() - 1) friend_points[friend_points_count++] = { ii1.getPoint(0), ii1.getPoint(+1) };
                D best_angle_change_COSi = isOrtogonal ? 1 : 0; // smallest angle for parallel and highest for ortogonal
                for (int k = 0; k < friend_points_count; k++)
                {
                    P3 point = friend_points[k].first.point;
                    P3 friend_point = friend_points[k].second.point;
                    V3 dirOrigin = (friend_point - point).normalized();
                    V3 dirNew = (friend_point - point_changed).normalized();
                    D angle_change_COS = utils::vector::Cos(dirOrigin, dirNew, true);
                    if (angle_change_COS < 0) angle_change_COS = utils::vector::Cos(dirOrigin, -dirNew, true); // if vectors opposite - change direction, so we will get: if (angle>90) angle=180-angle;
                    if (isOrtogonal)
                    {
                        if (angle_change_COS < best_angle_change_COSi) best_angle_change_COSi = angle_change_COS; // remember highest angle
                    }
                    else
                    {
                        if (angle_change_COS > best_angle_change_COSi) best_angle_change_COSi = angle_change_COS; // remember lowest angle
                    }
                }
                if (best_angle_change_COSi < max_angle_change_COS) continue; // skip merging if lowest change angle is higher from maximum allowed

                // merge intersectionID's
                int intersectionID = ii0.p->intersectionID;
                if (intersectionID == -1)  // check if 'intersectionID' is not already defined
                {
                    intersectionID = intersectoinInfo_currentId;
                    intersectoinInfo_currentId++;
                }
                ii0.p->intersectionID = intersectionID;
                ii1.p->intersectionID = intersectionID;
                mergedPointsCounti++;
            }
        }
        mergedPointsCount += mergedPointsCounti;
        //if (options.DebugEnabled)
        //{
        //    cout << "@@@   iterationNum=" << iterationNum << "    mergedPointsCounti=" << mergedPointsCounti << endl;
        //}
    }
    if (mergedPointsCount == 0) return 0;

    //
    // set new 3d coordinates for merged points that are close to each other
    //
    vector<int> close_points_intersectionIDs;
    close_points_intersectionIDs.reserve(100);
    for (auto& o : map_eid_intersectoinInfos)
    {
        int eid = o.first;
        vector<intersectoinInfo>& iis = o.second;
        if (iis.size() < 2) continue;

        // populate 'close_points_intersectionIDs'
        close_points_intersectionIDs.clear();
        for (int i = 0; i < iis.size() - 1; i++)
        {
            const intersectoinInfo& ii0 = iis[i];
            const intersectoinInfo& ii1 = iis[i + 1];
            if (ii0.p->intersectionID != -1 && ii0.p->intersectionID == ii1.p->intersectionID)
            {
                int intersectionID = ii0.p->intersectionID;
                if (!utils::stdvector::exists(close_points_intersectionIDs, intersectionID))
                {
                    close_points_intersectionIDs.push_back(intersectionID);
                }
            }
        }

        // merge 3d coordinates
        for (int intersectionID : close_points_intersectionIDs)
        {
            P3 middlePoint(0, 0, 0);
            D count = 0;
            int size = iis.size();
            for (int i = 0; i < size; i++)
            {
                const intersectoinInfo& ii = iis[i];
                if (ii.p->intersectionID != intersectionID) continue;
                count++;
                middlePoint += ii.p->point;
            }
            if (count >= 2)
            {
                middlePoint /= count;
                if (options.DebugEnabled && options.Debug_show_mergedclose_points)
                {
                    draw.AddPoint(middlePoint, Color3d(1, 0, 0), "merdedPoint#" + to_string(intersectionID));
                }
                for (int i = 0; i < size; i++)
                {
                    const intersectoinInfo& ii = iis[i];
                    if (ii.p->intersectionID != intersectionID) continue;
                    ii.p->point = middlePoint;
                    ii.s->Set_IsInited_lengthToNextPoint(false);  // deinit lengths information
                }

            }
        }
    }

    //
    // init for deinited where we have updated some points
    //
    for (MeshStream& s : streams)
    {
        for (auto& stream : streams)
        {
            bool debug = false;
            //#if DEBUG
            //debug = options.DebugEnabled;
            //#endif
            int removedDuplicatesCount = stream.RemoveDuplicatePoints(debug);
        }
        s.Init_lengthToNextPoint();
    }

    return mergedPointsCount;
}

void MeshCutter::MergeClosePointsOnEdges(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections)
{
    //int iterationNum = 0;
    //while (MergeClosePointsOnEdgesOnce(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections) > 0)
    //{
    //    iterationNum++;
    //    if (iterationNum > 1000)
    //    {
    //        cout << "!warning   MeshCutter::MergeClosePointsOnEdgesOnce runned for a 1000 times!" << endl;
    //        break;
    //    }
    //}

    MergeClosePointsOnEdgesOnce(m, m_V, streams, intersectoinInfo_currentId, debug_show_intersections);
}

void MeshCutter::ImproveStreamLines(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams)
{
    D edge_percent_tolerance = options.ImproveStreamLines__edge_percent_tolerance;
    D tol = avg_edge_length * edge_percent_tolerance;
    D tolPow2 = tol * tol;
    for (MeshStream& points : streams)
    {
        vector<int> indexesToDelete;
        for (int i = 0; i < points.size() - 1; i++)  //exluding last
        {
            int iv = i;
            int ie = i + 1;
            if (points[ie].Type == MeshPointType::onVertex && points[iv].Type == MeshPointType::onEdge)
            {
                swap(iv, ie);
            }
            if (points[iv].Type == MeshPointType::onVertex && points[ie].Type == MeshPointType::onEdge)
            {
                int eid = points[ie].vid_eid_fid;
                int vid0 = m.EV(eid, 0);
                int vid1 = m.EV(eid, 1);
                if (vid0 == points[iv].vid_eid_fid || vid1 == points[iv].vid_eid_fid) // if edge cantacting vertex
                {
                    D distPow2 = utils::point::DistToPointPow2(points[iv].point, points[ie].point); // and if dist to vertex is small
                    if (distPow2 < tolPow2)
                    {
                        if (options.DebugEnabled && options.Debug_trace_removed)
                        {
                            D dist = sqrt(distPow2);
                            cout << "Removing point on edge " << points[ie].vid_eid_fid << " that is very close to vertex " << points[iv].vid_eid_fid << "   dist = " << dist << "  what is " << dist / avg_edge_length << "% of avg edge length" << endl;
                        }
                        indexesToDelete.push_back(ie);
                    }
                }
            }
        }
        //cout << "points.size() = " << points.size() << endl;
        utils::stdvector::sort(indexesToDelete);
        for (int i = indexesToDelete.size() - 1; i >= 0; i--)
        {
            points.Remove(indexesToDelete[i]);
        }
        //cout << "points.size() = " << points.size() << endl;
    }
}

void MeshCutter::ImproveMeshToConnectStreamLines(const Mesh& m, P3s& m_V, vector<MeshStream>& streams, bool allow_improvements_on_borders, bool dissalow_move_other_streams)
{
    const I2* EV = (const I2*)m.EV.data();
    bool debug_show_steps = options.DebugEnabled && options.Debug_show_steps;

    auto getClosestVertex = [](const Mesh& m, const P3s& m_V, const StreamPoint& p, int& closest_vid, D& closest_distPow2)
    {
        int eid = p.vid_eid_fid;
        int vid0 = m.EV(eid, 0);
        int vid1 = m.EV(eid, 1);
        P3 v0 = m_V.row(vid0);
        P3 v1 = m_V.row(vid1);
        D distPow2_0 = utils::point::DistToPointPow2(v0, p.point);
        D distPow2_1 = utils::point::DistToPointPow2(v1, p.point);
        if (distPow2_0 < distPow2_1)
        {
            closest_vid = vid0;
            closest_distPow2 = distPow2_0;
        }
        else
        {
            closest_vid = vid1;
            closest_distPow2 = distPow2_1;
        }
    };

    struct StreamPointRememberedPosition
    {
        StreamPoint* point;
        D edgePercent;
        StreamPoint* pointPrev;
        StreamPoint* pointNext;
    };

    auto rememberPositionOnMovedEdges = [](const Mesh& m, const P3s& m_V, ViewerDrawObjects& draw, vector<MeshStream>& streams, const vector<Vector<bool>>& streams_usedVertexes, int vid, vector<StreamPointRememberedPosition>& state)
    {
        state.clear();

        for (int sid = 0; sid < streams.size(); sid++)
        {
            const Vector<bool>& usedVertexes = streams_usedVertexes[sid];
            if (!usedVertexes(vid)) continue; // if this stream dont use a searched vertex - the skip this stream from search
            MeshStream& points = streams[sid];
            for (int i = 0; i < points.size(); i++)
            {
                StreamPoint& p = points[i];
                if (p.Type == MeshPointType::onEdge)
                {
                    int eid = p.vid_eid_fid;
                    //v0
                    int vid0 = m.EV(eid, 0);
                    int vid1 = m.EV(eid, 1);
                    if (vid0 == vid || vid1 == vid)
                        //v1 - maybe this method is faster from v0 ???
                        //if (m.VE.exists(vid, eid);
                    {
                        P3 p0 = m_V.row(vid0);
                        P3 p1 = m_V.row(vid1);
                        D edgeLengthPow2 = utils::point::DistToPointPow2(p0, p1); // we dont use edge length because edge length could be change after mesh improvement
                        if (edgeLengthPow2 < 0.000000000001) continue;// protection fro devision by zero
                        D distTov0Pow2 = utils::point::DistToPointPow2(p0, p.point);
                        StreamPointRememberedPosition r;
                        r.point = &points[i];
                        r.edgePercent = (distTov0Pow2 / edgeLengthPow2);
                        r.pointPrev = nullptr;
                        r.pointNext = nullptr;
                        if (i > 0 && i < points.size() - 1)
                        {
                            r.pointPrev = &points[i - 1];
                            r.pointNext = &points[i + 1];
                        }
                        state.push_back(r);

                        //if (eid == 530)
                        //{
                        //    draw.AddPoint(p0, Color3d(1, 0, 0));
                        //    draw.AddLabel(p0, "  p0", Color3d(1, 0, 0));
                        //    draw.AddPoint(p1, Color3d(1, 0, 0));
                        //    draw.AddLabel(p1, "  p1", Color3d(1, 0, 0));
                        //    draw.AddPoint(p.point, Color3d(0, 0, 1));
                        //    draw.AddLabel(p.point, "  edgePercent=" + to_string(r.edgePercent) + "   distTov0="+to_string(distTov0), Color3d(0, 0, 1));
                        //}
                    }
                }
            }
        }
    };

    auto updatePositionOnMovedEdges = [](const Mesh& m, const P3s& m_V, ViewerDrawObjects& draw, vector<MeshStream>& streams, const vector<Vector<bool>>& streams_usedVertexes, int vid, const vector<StreamPointRememberedPosition>& state)
    {
        for (int sid = 0; sid < streams.size(); sid++)
        {
            const Vector<bool>& usedVertexes = streams_usedVertexes[sid];
            if (!usedVertexes(vid)) continue; // if this stream dont use a searched vertex - the skip this stream from search
            MeshStream& points = streams[sid];
            for (int i = 0; i < points.size(); i++)
            {
                StreamPoint& p = points[i];
                if (p.Type == MeshPointType::onVertex && p.vid_eid_fid == vid)
                {
                    p.point = m_V.row(vid);
                }
            }
        }

        for (const StreamPointRememberedPosition& s : state)
        {
            StreamPoint* p = s.point;
            int eid = p->vid_eid_fid;
            int vid0 = m.EV(eid, 0);
            int vid1 = m.EV(eid, 1);
            P3 p0 = m_V.row(vid0);
            P3 p1 = m_V.row(vid1);
            // try first get intersection point of moved edge to prev and next lines - this method preserves lines curve
            if (s.pointPrev != nullptr && s.pointNext != nullptr)
            {
                P3 closetPoint;
                int closestPoint_IsOnCornerPoint012 = 0;

                D minDistPrev;
                P3 closetPointPrev;
                P3 closetPointPrevEdge;
                int line1_ClosestPointIsOnCornerPoint012_prev;
                utils::vector::ClosestPoint_BetweenLines(p0, p1, s.pointPrev->point, s.point->point, closetPointPrevEdge, closetPointPrev, minDistPrev, line1_ClosestPointIsOnCornerPoint012_prev);

                D minDistNext;
                P3 closetPointNext;
                P3 closetPointNextEdge;
                int line1_ClosestPointIsOnCornerPoint012_next;
                utils::vector::ClosestPoint_BetweenLines(p0, p1, s.pointNext->point, s.point->point, closetPointNextEdge, closetPointNext, minDistNext, line1_ClosestPointIsOnCornerPoint012_next);

                if (minDistPrev < minDistNext)
                {
                    closetPoint = closetPointPrevEdge;
                    closestPoint_IsOnCornerPoint012 = line1_ClosestPointIsOnCornerPoint012_prev;
                }
                else
                {
                    closetPoint = closetPointNextEdge;
                    closestPoint_IsOnCornerPoint012 = line1_ClosestPointIsOnCornerPoint012_next;
                }

                if (closestPoint_IsOnCornerPoint012 == 1) // if point moved to edge vertex vid0
                {
                    p->point = p0;
                    p->vid_eid_fid = vid0;
                    p->Type = MeshPointType::onVertex;
                }
                else if (closestPoint_IsOnCornerPoint012 == 2) // if point moved to edge vertex vid1
                {
                    p->point = p1;
                    p->vid_eid_fid = vid1;
                    p->Type = MeshPointType::onVertex;
                }
                else
                {
                    p->point = closetPoint;
                }
                continue;
            }

            // if we fail first method - lets check if we can preserve almost same position of intersection with edge

            if (m.V_isborder[vid] && p->Type == MeshPointType::onEdge && m.E_isborder[p->vid_eid_fid])
            {
                P3 closestPoint = utils::vector::ClosestPoint_ToLine(p0, p1, p->point);
                if (options.DebugEnabled)
                {
                    cout << "***  border vertex moved - updating point on edge#" << p->vid_eid_fid << endl;
                }
                p->point = closestPoint;
                continue;
            }

            // if we fail second method - lets take same point on edge - this method works fine, but can make lines a bit curved
            //cout << "updatePositionOnMovedEdges   failed first method for eid = " << eid << endl;
            D edgeLength = utils::point::DistToPoint(p0, p1);
            V3 dirFrom_v0_To_v1 = p1 - p0;
            dirFrom_v0_To_v1.normalize();
            P3 newPoint = p0 + dirFrom_v0_To_v1 * (edgeLength*s.edgePercent);
            p->point = newPoint;

            //if (eid == 530)
            //{
            //    draw.AddPoint(p0, Color3d(0, 1, 0));
            //    draw.AddLabel(p0, "        p0", Color3d(0, 1, 0));
            //    draw.AddPoint(p1, Color3d(0, 1, 0));
            //    draw.AddLabel(p1, "        p1", Color3d(0, 1, 0));
            //    draw.AddPoint(p->point, Color3d(0, 1, 0));
            //    draw.AddLabel(p->point, "                                                                                                                         edgePercent=" + to_string(s.edgePercent) + "   distTov0=" + to_string((edgeLength*s.edgePercent)), Color3d(0, 1, 0));
            //}
        }
    };

    struct ImprovedPoint
    {
        bool enabled;
        int pointIndex;
        int eid;
        int closest_vid;
        P3 point;
        bool isOnBorder;
    };
    vector<ImprovedPoint> improvements;
    improvements.reserve(1000);

    // get locked vids for each stream
    vector<Vector<bool>> streams_usedVertexes(streams.size());
    for (int sid = 0; sid < streams.size(); sid++) streams_usedVertexes[sid].setConstant(m.VertexCount, false);//  set all values to 0
    if (streams.size() <= 1) dissalow_move_other_streams = false; //  if only 1 stream on mesh - we dont care because this stream cant conflict with another
    if (dissalow_move_other_streams)
    {
        for (int sid = 0; sid < streams.size(); sid++)
        {
            Vector<bool>& usedVertexes = streams_usedVertexes[sid];
            for (const StreamPoint& p : streams[sid].Points)
            {
                // lock vertex
                if (p.Type == MeshPointType::onVertex)
                {
                    int vid = p.vid_eid_fid;
                    usedVertexes(vid) = true;
                }
                // lock both start and end vertexes of edge
                if (p.Type == MeshPointType::onEdge)
                {
                    int eid = p.vid_eid_fid;
                    I2 evids = EV[eid];
                    int vid0 = evids(0);
                    int vid1 = evids(1);
                    usedVertexes(vid0) = true;
                    usedVertexes(vid1) = true;
                }
            }
        }
    }


    int foundSkipedImprovementsCount = 1;
    int iterationsCount = 0;
    while (foundSkipedImprovementsCount > 0 && iterationsCount < 100)
    {
        iterationsCount++;
        foundSkipedImprovementsCount = 0;

        Matrix<char, Dynamic, 1> usedVertexes_SummForAllStreams = Matrix<char, Dynamic, 1>::Zero(m.VertexCount);//  set all values to 0. only 256 count is allowed what is anought because usually we will have at max 16 edges connected to one vertex
        for (int sid = 0; sid < streams.size(); sid++)
        {
            Vector<bool>& usedVertexes = streams_usedVertexes[sid];
            for (int vid = 0; vid < m.VertexCount; vid++)
            {
                if ((vid & (boolBitsCount -1)) == 0) // every 32-th vertex check at once 32 vertexes - and skip them if in next 32 vertexes we dont use - speed optimzation
                {
                    int boolBits_isVertexUsed = usedVertexes.data()[vid / boolBitsCount];
                    if (boolBits_isVertexUsed == 0)
                    {
                        vid += (boolBitsCount -1);
                        continue;
                    }
                }
                if (usedVertexes(vid)) usedVertexes_SummForAllStreams(vid)++;
            }
        }

        int step = -1;
        int stopAtStep = (options.DebugEnabled && options.Debug_show_steps) ? options.Debug_show_steps__stop_at_step : -1;
        D edge_percent_tolerance = options.ImproveMeshToConnectStreamLines__edge_percent_tolerance;
        D tol = avg_edge_length * edge_percent_tolerance;
        D tolPow2 = tol * tol;

        // for all streams
        for (int streamId = 0; streamId < streams.size(); streamId++)
        {
            MeshStream& points = streams[streamId];

            // decrease lock for current stream, and if lock is 0 then it means that only this stream is using vertex so we allowed to move it
            Vector<bool>& usedVertexes = streams_usedVertexes[streamId];
            improvements.clear();
            if (improvements.capacity() < points.size() / 5) improvements.reserve(points.size() / 5);
            int iMin = 1; //exluding first and last line - since they are start from some point or reach some edge 
            int iMax = points.size() - 2; //exluding first and last line - since they are start from some point or reach some edge 
            if (allow_improvements_on_borders)
            {
                iMin = 0;
                iMax = points.size() - 1;
            }
            // find all improvements for all points on edges in stream
            for (int i = iMin; i <= iMax; i++)  //exluding first and last line - since they are start from some point or reach some edge 
            {
                const StreamPoint& p = points[i];
                if (p.Type != MeshPointType::onEdge) continue;

                int eid = p.vid_eid_fid;
                int closest_vid;
                D closest_distPow2;
                getClosestVertex(m, m_V, p, closest_vid, closest_distPow2);
                char usedVertexes_SummForAllStreams_value = usedVertexes_SummForAllStreams(closest_vid);
                int usedVertexes_value = (int)usedVertexes(closest_vid);
                if ((int)usedVertexes_SummForAllStreams_value - usedVertexes_value > 0) // if lock more from what we use in this stream
                {
                    continue;
                }
                if (closest_distPow2 < tolPow2 && (!m.V_isborder[closest_vid] || allow_improvements_on_borders))
                {
                    // check if border vertex can be moved
                    if (m.V_isborder[closest_vid])
                    {
                        vector<VertexToFacesSortedInfo> vid0faces = m.VertexToFacesSorted(closest_vid, m_V);
                        D angleSumm = 0;
                        for (int i0 = 0; i0 < vid0faces.size(); ++i0)
                        {
                            angleSumm += vid0faces[i0].angle;
                        }
                        //if (options.DebugEnabled && options.Debug_trace_removed) cout << "border vid = " << vid0 <<"   abs(angleSumm - 180) = " << abs(angleSumm - 180) << endl;
                        if (abs(angleSumm - 180) > 20) continue;
                    }
                    if (options.DebugEnabled && options.Debug_trace_removed)
                    {
                        D dist = sqrt(closest_distPow2) / 2;
                        draw.AddPoint(p.point, Color3d(1, 0, 0));
                        cout << "Removing line between edge " << p.vid_eid_fid << "  and vertex " << closest_vid << "      dist = " << dist << "  what is " << dist / avg_edge_length << "% of avg edge length" << endl;
                    }
                    improvements.push_back({ true, i, eid, closest_vid , p.point , m.E_isborder[eid] });
                }
            }
            //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines  #" + to_string(streamId)+ "  allow_improvements_on_borders");

            Bs isPointRemoved = Bs::Zero(points.size());//  set all values to false
            int isPointRemoved_count = 0;
            int improvementsCount = improvements.size();
            for (int i = 0; i < improvementsCount; i++)  //exluding first and last line - since they are start from some point or reach some edge 
            {
                auto improvement = improvements[i];
                if (!improvement.enabled) continue;

                int vid = improvement.closest_vid;
                if ((int)usedVertexes_SummForAllStreams(vid) - (int)usedVertexes(vid) > 0) // if lock more from what we use in this stream
                {
                    foundSkipedImprovementsCount++;
                    continue;
                }

                step++;
                if (debug_show_steps) if (stopAtStep != -1 && step > stopAtStep) break;
                bool debug_current_step = debug_show_steps && (stopAtStep == -1 || (stopAtStep != -1 && step == stopAtStep));
                if (debug_current_step) cout << "MeshCutter::ImproveMeshToConnectStreamLines   Step#" << step << endl;
                /*if (vid == 254)
                {
                    vid = vid;
                }*/
                int iEnd = i + 1;
                while (iEnd < improvementsCount && improvements[iEnd].enabled && vid == improvements[iEnd].closest_vid)
                {
                    iEnd++;
                }
                P3 mergePoint = P3(0, 0, 0);
                int mergedPointsCount = (iEnd - i);
                if (debug_current_step)cout << "merging ";
                for (int k = i; k < iEnd; k++)
                {
                    //P3 pK = points[k].point;
                    P3 pK = improvements[k].point;
                    mergePoint += pK;
                    if (debug_current_step)
                    {
                        cout << points[improvements[k].pointIndex].vid_eid_fid_toString();
                        if (k != iEnd - 1) cout << ", ";
                    }
                }
                if (debug_current_step)cout << " for vertex " << vid;
                mergePoint /= (D)mergedPointsCount;
                // force movement of border vertex only along border edge
                bool ignoreThisImprovement = false;
                if (allow_improvements_on_borders)
                {
                    bool founPointOnBorder = false;
                    for (int k = i; k < iEnd; k++)
                    {
                        if (improvements[k].isOnBorder)
                        {
                            if (!m.V_isborder[vid])
                            {
                                ignoreThisImprovement = true;
                                break;
                            }
                            founPointOnBorder = true;
                            P3 pK = improvements[k].point;
                            mergePoint = pK;
                            if (debug_current_step) cout << ",  taking border point of edge " << points[improvements[k].pointIndex].vid_eid_fid << endl;
                            break;
                        }
                    }
                    if (!founPointOnBorder && m.V_isborder[vid])
                    {
                        ignoreThisImprovement = true;
                    }
                }
                if (debug_current_step) cout << endl; // finish print

                // clear improvements
                for (int k = i; k < iEnd; k++)
                {
                    improvements[k].enabled = false;
                }
                if (ignoreThisImprovement) continue;

                // remember points that are on edges of moved vertex
                //vector<VertexToFacesSortedInfo> fs = m.VertexToFacesSorted(vid, m_V); //TODO can be improved - we need only eid connected to vertex
                vector<StreamPointRememberedPosition> state;
                rememberPositionOnMovedEdges(m, m_V, draw, streams, streams_usedVertexes, vid, state);
                // move vertex
                m_V.row(vid) = mergePoint;
                //lock this vertex and all connected vertexes 
                usedVertexes_SummForAllStreams(vid)++;
                for (int k = 0; k < m.VE.size(vid); k++)
                {
                    int locked_eid = m.VE(vid, k);
                    int locked_vid0 = m.EV(locked_eid, 0);
                    int locked_vid1 = m.EV(locked_eid, 1);
                    if (locked_vid0 != vid) usedVertexes_SummForAllStreams(locked_vid0)++;
                    if (locked_vid1 != vid) usedVertexes_SummForAllStreams(locked_vid1)++;
                }
                //  update points that are on edges of moved vertex
                updatePositionOnMovedEdges(m, m_V, draw, streams, streams_usedVertexes, vid, state);

                // update point
                //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines  #" + to_string(streamId) + "  update point - before");
                int pointIndex = improvements[i].pointIndex;
                points[pointIndex].point = mergePoint;
                points[pointIndex].Type = MeshPointType::onVertex;
                points[pointIndex].vid_eid_fid = vid;
                //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines  #" + to_string(streamId) + "  update point - after");
                // remove merged points
                for (int iremove = 0; iremove < mergedPointsCount - 1; iremove++)
                {
                    isPointRemoved(pointIndex + 1 + iremove) = true;
                }
                isPointRemoved_count += mergedPointsCount - 1;
                if (debug_current_step)
                {
                    draw.AddLabel(mergePoint, "              step#" + to_string(step) + "   merged " + to_string(mergedPointsCount) + " points at vid=" + to_string(vid));
                }
            }

            //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines  #" + to_string(streamId) + "  isPointRemoved");

            if (isPointRemoved_count > 0)
            {
                int newIndex = 0;
                for (int i = 0; i < points.size(); i++)
                {
                    if (isPointRemoved(i)) continue; // skip copying removed points
                    points[newIndex] = points[i];
                    newIndex++;
                }
                points.Remove(newIndex, isPointRemoved_count);
                // remove other duplicated points, that can appears when we deleted some points
                bool removedSomePoints = true;
                while (removedSomePoints)
                {
                    removedSomePoints = false;
                    for (int i = 0; i < points.size() - 1; i++)
                    {
                        const StreamPoint& p1 = points[i];
                        const StreamPoint& p2 = points[i + 1];
                        if (p1.Type == p2.Type && p1.vid_eid_fid == p2.vid_eid_fid && p1.intersectionID == p2.intersectionID)
                        {
                            points.Remove(i);
                            removedSomePoints = true;
                            //cout << "removed duplicated point#"<<i << endl;
                            break;
                        }
                    }
                }
            }
            //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines  #" + to_string(streamId) + "  isPointRemoved_count > 0");

            // remove point on edge between two points on vertexes - all 3 points have same face (this can happend after improvement)
            for (int i = points.size() - 2; i > 0; i--) // i=[points.size()-2..1], so iPrev can be points.size()-1 and iNext can be 0
            {
                int iPrev = i - 1;
                int iNext = i + 1;
                if (points[iPrev].Type == MeshPointType::onVertex
                    && points[i].Type == MeshPointType::onEdge
                    && points[iNext].Type == MeshPointType::onVertex)
                {
                    int eid = points[i].vid_eid_fid;
                    //v1 - works only when edge contaxct both vertexes
                    //int edge_v0 = m.EV(eid, 0);
                    //int edge_v1 = m.EV(eid, 1);
                    //if ((edge_v0 == points[iPrev].vid_eid_fid && edge_v1 == points[iNext].vid_eid_fid)
                    //    || (edge_v0 == points[iNext].vid_eid_fid && edge_v1 == points[iPrev].vid_eid_fid))
                    //{
                    //    utils::stdvector::remove_at(points, i);
                    //}
                    //v2 - works if edge and both vertex have common face
                    bool remove = false;
                    int vid1 = points[iPrev].vid_eid_fid;
                    int vid2 = points[iNext].vid_eid_fid;

                    for (int k = 0; k <= 1; k++)
                    {
                        int fidk = m.EF(eid, k);
                        if (fidk != -1)
                        {
                            if (m.VF.exists(vid1, fidk) && m.VF.exists(vid2, fidk))
                            {
                                remove = true;
                            }
                        }
                    }
                    if (remove)
                    {
                        points.Remove(i);
                    }
                }
            }
            //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines  #"+to_string(streamId) + "  remove point on edge between two points on vertexes");

        }

        ImproveMeshToConnectStreamLines_removeReduntantPoints(m, m_V, streams);
        //checkStreamsForDuplicatedPoints(streams, "ImproveMeshToConnectStreamLines_removeReduntantPoints");
    }
}

void MeshCutter::ImproveMeshToConnectStreamLines_removeReduntantPoints(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams)
{
    auto isReduntantPoint = [](const Mesh& m, ViewerDrawObjects& draw, MeshStream& points, int i0, int i1, int i2)
    {
        const StreamPoint& p0 = points[i0];
        const StreamPoint& p1 = points[i1];
        const StreamPoint& p2 = points[i2];
        if (p0.Type != MeshPointType::onVertex) return false;
        if (p1.Type != MeshPointType::onEdge) return false;
        if (p2.Type != MeshPointType::onEdge) return false;
        int f1_fid0 = m.EF(p1.vid_eid_fid, 0);
        int f1_fid1 = m.EF(p1.vid_eid_fid, 1);
        int f2_fid0 = m.EF(p2.vid_eid_fid, 0);
        int f2_fid1 = m.EF(p2.vid_eid_fid, 1);
        int samefid = -1;
        if (f1_fid0 != -1 && f1_fid0 == f2_fid0) samefid = f1_fid0;
        if (f1_fid0 != -1 && f1_fid0 == f2_fid1) samefid = f1_fid0;
        if (f1_fid1 != -1 && f1_fid1 == f2_fid0) samefid = f1_fid1;
        if (f1_fid1 != -1 && f1_fid1 == f2_fid1) samefid = f1_fid1;
        if (samefid != -1)
        {
            int vid = p0.vid_eid_fid;
            if (m.VF.exists(vid, samefid))
            {
                return true;
            }
        }
        return false;
    };

    for (int streamId = 0; streamId < streams.size(); streamId++)
    {
        MeshStream& points = streams[streamId];

        // remove point on edge between point on edge and vertex of same face
        bool removedReduntantPoint = true;
        while (removedReduntantPoint)
        {
            removedReduntantPoint = false;
            for (int i = 0; i < points.size(); i++)
            {
                const StreamPoint& p = points[i];
                if (p.Type != MeshPointType::onVertex) continue;
                if (i >= 2)
                {
                    int iPrev = i - 1;
                    int iPrevPrev = i - 2;
                    if (isReduntantPoint(m, draw, points, i, iPrev, iPrevPrev))
                    {
                        points.Remove(iPrev);
                        removedReduntantPoint = true;
                        break;
                    }
                }
                if (i < points.size() - 2)
                {
                    int iNext = i + 1;
                    int iNextNext = i + 2;
                    if (isReduntantPoint(m, draw, points, i, iNext, iNextNext))
                    {
                        points.Remove(iNext);
                        removedReduntantPoint = true;
                        break;
                    }
                }

            }
        }
    }

}


//one stream on edge, second intersect this edge
void MeshCutter::FindStreamIntersections_Edge(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections)
{
    //
    // get information what edges have streampoints 
    //
    //Is debug_edgesCrossesCount = Is::Zero(m.EdgesCount);
    Vector<bool> isEdgesCrosses;
    isEdgesCrosses.setConstant(m.EdgesCount, false);
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size(); i++)
        {
            const StreamPoint& p = points[i];
            if (p.Type == MeshPointType::onEdge)
            {
                int eid = p.vid_eid_fid;
                //debug_edgesCrossesCount(eid)++;
                isEdgesCrosses(eid) = true;
            }
        }
    }

    //
    // get edges where streams intersects
    //
    struct intersectoinInfo
    {
        P3 p;
        int id;
        D distPow2;
    };
    map<int, vector<intersectoinInfo>> map_eid_intersecions;
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size() - 1; i++)
        {
            const StreamPoint& p0 = points[i];
            const StreamPoint& p1 = points[i + 1];
            if (p0.Type == MeshPointType::onVertex && p1.Type == MeshPointType::onVertex)
            {
                // here we need to undestand what edge id is for this two vertexes
                int vid0 = p0.vid_eid_fid;
                int vid1 = p1.vid_eid_fid;
                int commoneid = m.CommonEdgeId_VertexVertex(vid0, vid1);
                if (commoneid != -1 && isEdgesCrosses(commoneid))
                {
                    map_eid_intersecions[commoneid] = vector<intersectoinInfo>();
                }
            }
        }
    }
    if (map_eid_intersecions.size() == 0) return;

    //
    // populate intersection points
    //
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size(); i++)
        {
            StreamPoint& p = points[i];
            if (p.Type == MeshPointType::onEdge)
            {
                int eid = p.vid_eid_fid;
                auto find = map_eid_intersecions.find(eid);
                if (find != map_eid_intersecions.end())
                {
                    intersectoinInfo ii;
                    ii.p = p.point;
                    if (p.intersectionID == -1) // check if 'intersectionID' is not already defined
                    {
                        p.intersectionID = intersectoinInfo_currentId;
                        ii.id = p.intersectionID;
                        intersectoinInfo_currentId++;
                    }
                    else
                    {
                        ii.id = p.intersectionID; // reuse 'intersectionID'
                    }
                    // dont add 'intersectionID' twice - since intersection with same id are same points, so we have to avoid duplicates
                    bool is_intersectionID_alreadyAdded = false;
                    for (auto& oii : find->second)
                    {
                        if (oii.id == ii.id)
                        {
                            is_intersectionID_alreadyAdded = true;
                            break;
                        }
                    }
                    if (!is_intersectionID_alreadyAdded)
                    {
                        find->second.push_back(ii);

                        // DEBUG imitate 3 intersectoin points on same edge
                        //P3 v0 = m_V.row(vid0);
                        //P3 v1 = m_V.row(vid1);
                        //V3 dir01 = v1 - v0;
                        //D dir01length = dir01.norm();
                        //dir01.normalize();
                        ////for (int k = 0; k < 3; k++)
                        //    for (int k = 2; k >=0; k--)
                        //{
                        //    intersectoinInfo ii;
                        //    ii.p = v0 + dir01*dir01length*0.25*(k+1);
                        //    ii.id = intersectoinInfo_currentId;
                        //    p.intersectionID = intersectoinInfo_currentId;
                        //    intersectoinInfo_currentId++;
                        //    intersecionEdgeIdsAndPoints[eid].push_back(ii);
                        //    draw.AddPoint(ii.p, Color3d(0, 0, 1));
                        //}
                    }
                }
            }
        }
    }

    //
    // insert new points in stream
    //
    for (MeshStream& points : streams)
    {
        int i = 0;
        int size = points.size();
        while (i < size - 1)
        {
            const StreamPoint& p0 = points[i];
            const StreamPoint& p1 = points[i + 1];
            if (p0.Type == MeshPointType::onVertex && p1.Type == MeshPointType::onVertex)
            {
                // here we need to undestand what edge id is for this two vertexes
                int vid0 = p0.vid_eid_fid;
                int vid1 = p1.vid_eid_fid;
                int commoneid = m.CommonEdgeId_VertexVertex(vid0, vid1);
                if (commoneid != -1 && isEdgesCrosses(commoneid))
                {
                    vector<intersectoinInfo>& iis = map_eid_intersecions[commoneid];
                    for (auto& ii : iis)
                    {
                        ii.distPow2 = utils::point::DistToPointPow2(p0.point, ii.p);
                    }
                    vector<unsigned int> sorted_indexes = utils::stdvector::sort_indexes_custom(iis.size(), [&iis](unsigned int i1, unsigned int i2)
                    {
                        return iis[i1].distPow2 < iis[i2].distPow2;
                    });
                    for (auto index : sorted_indexes)
                    {
                        const intersectoinInfo& ii = iis[index];
                        StreamPoint s(MeshPointType::onEdge, commoneid, ii.p, p0.fid);
                        s.intersectionID = ii.id;
                        points.Insert(i + 1, s);
                        size++;
                        if (debug_show_intersections)
                        {
                            //cout << "stream intersection#" << ii.id << ": eid = " << commoneid << ",   intersections count = " << debug_edgesCrossesCount(commoneid) << endl;
                            draw.AddPoint(ii.p, Color3d(0, 0, 1));
                            draw.AddLabel(ii.p, "  edge intersection#" + to_string(ii.id), Color3d(0, 0, 1), 2);
                        }
                        i++;
                    }
                }
            }
            i++;
        }
    }
}

// two stream intersects at one vertex
void MeshCutter::FindStreamIntersections_Vertex(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections)
{
    //
    // get information what vertex have streampoints 
    //
    Is vertexIntersectionCount = Is::Zero(m.VertexCount);
    Is& vertexIntersectionIds = vertexIntersectionCount; // reuse same space for speed optimization
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size(); i++)
        {
            const StreamPoint& p = points[i];
            if (p.Type == MeshPointType::onVertex)
            {
                int vid = p.vid_eid_fid;
                if (i == 0)
                {
                    StreamPoint& lastPoint = points.Points.back();
                    if (lastPoint.Type == MeshPointType::onVertex && lastPoint.vid_eid_fid == p.vid_eid_fid) continue; // skip adding 2 time intersection for cyclic streams - add only 1 time for last  point (if some other stream will connect to same point then count will be already 2 and intersection point will be registered)
                }
                vertexIntersectionCount(vid)--;
            }
        }
    }

    //
    // set ids to points
    //
    for (int vid = 0; vid < m.VertexCount; vid++)
    {
        int count = -vertexIntersectionCount(vid);
        if (count < 2) continue; // at least two streams should use this vertex
        vertexIntersectionIds(vid) = intersectoinInfo_currentId + 1; // increase by 1 to avoid 0 values
        intersectoinInfo_currentId++;

        //DEBUG - show intersection points
        if (debug_show_intersections)
        {
            //cout << "stream intersection#" << vertexIntersectionIds(vid) << ": vid = " << vid << ",   intersections count = " << vertexIntersectionCount(vid) << endl;
            P3 intersectoinPoint = m_V.row(vid);
            draw.AddPoint(intersectoinPoint, Color3d(0, 0, 1));
            draw.AddLabel(intersectoinPoint, "  vertex intersection#" + to_string(vertexIntersectionIds(vid) - 1), Color3d(0, 0, 1), 2); // decrease by 1 since before we were doing increase by 1 to avoid 0 value
        }
    }


    //
    // set ids to streams
    //
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size(); i++)
        {
            StreamPoint& p = points[i];
            if (p.Type == MeshPointType::onVertex)
            {
                int vid = p.vid_eid_fid;
                if (vertexIntersectionIds(vid) > 0)
                {
                    p.intersectionID = vertexIntersectionIds(vid) - 1; // decrease by 1 since before we were doing increase by 1 to avoid 0 value
                }
            }
        }
    }
}

void MeshCutter::FindStreamIntersections_Face(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections)
{
    //
    // get information what face have stream lines
    //
    Is faceIntersectionCount = Is::Zero(m.FacesCount);
    bool foundPossibleIntersections = false;
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size() - 1; i++) // exluding last to be able to take 'i+1'
        {
            const StreamPoint& p1 = points[i];
            const StreamPoint& p2 = points[i + 1];
            if ((p1.Type == MeshPointType::onVertex && p2.Type == MeshPointType::onEdge)
                || (p1.Type == MeshPointType::onEdge && p2.Type == MeshPointType::onVertex))
            {
                int vid = (p1.Type == MeshPointType::onVertex) ? p1.vid_eid_fid : p2.vid_eid_fid;
                int eid = (p2.Type == MeshPointType::onEdge) ? p2.vid_eid_fid : p1.vid_eid_fid;
                if (!m.VE.exists(vid, eid))
                {
                    int commonfid1;
                    int commonfid2;
                    m.CommonFaceIds_VertexEdge(vid, eid, commonfid1, commonfid2);
                    if (commonfid1 != -1)
                    {
                        faceIntersectionCount(commonfid1)++;
                        if (faceIntersectionCount(commonfid1) >= 2) foundPossibleIntersections = true;
                    }
                }
            }
            if (p1.Type == MeshPointType::onEdge && p2.Type == MeshPointType::onEdge)
            {
                int eid1 = p1.vid_eid_fid;
                int eid2 = p2.vid_eid_fid;
                int commonfid = m.CommonFaceIds_EdgeEdge(eid1, eid2);
                if (commonfid != -1)
                {
                    faceIntersectionCount(commonfid)++;
                    if (faceIntersectionCount(commonfid) >= 2) foundPossibleIntersections = true;
                }
            }
        }
    }
    if (!foundPossibleIntersections) return;

    //DEBUG
    //
    // calculate contour length for face that have more than 2 stream lines
    //
    //Ds faceContoursLength = Ds::Zero(m.FacesCount); //DEBUG
    //for (int fid = 0; fid < m.FacesCount; fid++)
    //{
    //    if (faceIntersectionCount(fid) < 2) continue;
    //        int vid0 = m.F(fid, 0);
    //        int vid1 = m.F(fid, 1);
    //        int vid2 = m.F(fid, 2);
    //        P3 v0 = m_V.row(vid0);
    //        P3 v1 = m_V.row(vid1);
    //        P3 v2 = m_V.row(vid2);
    //        faceContoursLength[fid] = utils::point::DistToPoint(v1, v0) + utils::point::DistToPoint(v2, v1) + utils::point::DistToPoint(v0, v2);
    //}

    //
    // store stream lines for faces that have more than 2 stream lines
    //
    auto getContourPosition = [&m, &m_V](int fid, int eid, P3 p)
    {
        int vid0 = m.F(fid, 0);
        int vid1 = m.F(fid, 1);
        int vid2 = m.F(fid, 2);
        D pos = 0;
        for (int k = 0; k < 3; k++)
        {
            int vidk = m.F(fid, k);
            P3 vidkp = m_V.row(vidk);
            int faceeidk = m.FE(fid, k);
            if (faceeidk == eid)
            {
                pos += utils::point::DistToPoint(p, vidkp);
                break;
            }
            else
            {
                //TODO may we use m.E_Length or this data will be outdated after mesh improvement ?
                int vidkNext = m.F(fid, (k + 1) % 3);
                P3 vidNextp = m_V.row(vidkNext);
                pos += utils::point::DistToPoint(vidNextp, vidkp);
            }
        }
        return pos;
    };
    auto getEdgeContactedWithVertexAndEdge = [&m, &faceIntersectionCount](int vid, int eid, int& commonfid)
    {
        if (!m.VE.exists(vid, eid))
        {
            int commonfid1;
            int commonfid2;
            m.CommonFaceIds_VertexEdge(vid, eid, commonfid1, commonfid2);
            if (commonfid1 != -1 && faceIntersectionCount(commonfid1) >= 2)
            {
                for (int k = 0; k < 3; k++)
                {
                    if (vid == m.F(commonfid1, k))
                    {
                        commonfid = commonfid1;
                        return m.FE(commonfid1, k);
                    }
                }
            }
        }
        return -1;
    };
    struct intersectoinInfo
    {
        P3 p;           // intersection point
        int id;         // intersection id
        D distPow2; // utils::point::DistToPoint(points[sl.pointIndex0].point, ii.p);
    };
    struct FaceStreamLine
    {
        int streamId;
        int pointIndex0;
        int pointIndex1;
        int pointIndex0_intersectionID;
        int pointIndex1_intersectionID;
        int fid;
        P3 startPosition; // point in 'pointIndex1' or in 'pointIndex2'
        P3 endPosition; // point in 'pointIndex2' or in 'pointIndex1'
        D startContourPosition; //start position on face contour (3 lines create face contour)
        D endContourPosition;//end position on face contour (3 lines create face contour)
        vector<intersectoinInfo> intersections;
    };
    map<int, vector<FaceStreamLine>> map_fid_facesStreamLines;// we can allow such expensive construction because we will have very small amount of data
    for (int sid = 0; sid < streams.size(); sid++)
    {
        MeshStream& points = streams[sid];
        for (int i = 0; i < points.size() - 1; i++) // exluding last to be able to take 'i+1'
        {
            const StreamPoint& p0 = points[i];
            const StreamPoint& p1 = points[i + 1];

            int eid1 = -1;
            int eid2 = -1;
            int commonfid = -1;
            if (p0.Type == MeshPointType::onVertex && p1.Type == MeshPointType::onEdge)
            {
                eid1 = getEdgeContactedWithVertexAndEdge(p0.vid_eid_fid, p1.vid_eid_fid, commonfid);
                eid2 = p1.vid_eid_fid;
            }
            if (p0.Type == MeshPointType::onEdge && p1.Type == MeshPointType::onVertex)
            {
                eid1 = p0.vid_eid_fid;
                eid2 = getEdgeContactedWithVertexAndEdge(p1.vid_eid_fid, p0.vid_eid_fid, commonfid);
            }
            if (p0.Type == MeshPointType::onEdge && p1.Type == MeshPointType::onEdge)
            {
                commonfid = m.CommonFaceIds_EdgeEdge(p0.vid_eid_fid, p1.vid_eid_fid);
                if (commonfid != -1 && faceIntersectionCount(commonfid) >= 2)
                {
                    eid1 = p0.vid_eid_fid;
                    eid2 = p1.vid_eid_fid;
                }
            }

            if (eid1 != -1 && eid2 != -1)
            {
                if (map_fid_facesStreamLines.find(commonfid) == map_fid_facesStreamLines.end())
                {
                    map_fid_facesStreamLines[commonfid] = vector<FaceStreamLine>();
                }
                vector<FaceStreamLine>& stream_lines = map_fid_facesStreamLines[commonfid];
                FaceStreamLine sl;
                sl.streamId = sid;
                sl.pointIndex0 = i;
                sl.pointIndex1 = i + 1;
                sl.pointIndex0_intersectionID = p0.intersectionID;
                sl.pointIndex1_intersectionID = p1.intersectionID;
                sl.fid = commonfid;
                sl.startPosition = p0.point;
                sl.endPosition = p1.point;
                sl.startContourPosition = getContourPosition(commonfid, eid1, sl.startPosition);
                sl.endContourPosition = getContourPosition(commonfid, eid2, sl.endPosition);
                if (sl.endContourPosition < sl.startContourPosition)
                {
                    swap(sl.startContourPosition, sl.endContourPosition);
                    P3 temp = sl.startPosition;
                    sl.startPosition = sl.endPosition;
                    sl.endPosition = temp;
                }
                stream_lines.push_back(sl);

                //DEBUG - show streamlines
                //if (options.DebugEnabled && options.Debug_trace_intersections)
                //{
                //    for (int k = 0; k < 3; k++)
                //    {
                //        int vid0 = m.F(sl.fid, k);
                //        P3 vid0p = m_V.row(vid0);
                //        int vidNext = m.F(sl.fid, (k + 1) % 3);
                //        P3 vidNextp = m_V.row(vidNext);
                //        D edgeLength = utils::point::DistToPoint(vid0p, vidNextp);
                //        draw.AddLabel((vid0p+ vidNextp)/2, "   edge#" + to_string(k) + ",  length=" + to_string(edgeLength));
                //    }
                //    //streams[sl.streamId][sl.pointIndex2].point
                //    draw.AddLabel(sl.startPosition, "   streamId=" + to_string(sl.streamId) + ",  start=" + to_string(sl.startContourPosition));
                //    draw.AddLabel(sl.endPosition, "   streamId=" + to_string(sl.streamId) + ",  end=" + to_string(sl.endContourPosition));
                //}
            }
        }
    }
    //checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Face - store stream lines for faces that have more than 2 stream lines");

    //
    // find intersectoin for faces that have more than 2 stream lines - store them into 'map_fid_facesStreamLines[fid].intersections'
    //
    int foundIntersectionsCount = 0;
    for (int fid = 0; fid < faceIntersectionCount.size(); fid++)
    {
        if (faceIntersectionCount(fid) < 2) continue;
        vector<FaceStreamLine>& streamLines = map_fid_facesStreamLines[fid];
        for (int sli1 = 0; sli1 < streamLines.size(); sli1++)
        {
            for (int sli2 = sli1 + 1; sli2 < streamLines.size(); sli2++)
            {
                FaceStreamLine& sl1 = streamLines[sli1];
                FaceStreamLine& sl2 = streamLines[sli2];

                // if streamlines have some common point, then they cant intersects
                bool isSame0 = (sl1.pointIndex0_intersectionID != -1) && (sl1.pointIndex0_intersectionID == sl2.pointIndex0_intersectionID || sl1.pointIndex0_intersectionID == sl2.pointIndex1_intersectionID);
                bool isSame1 = (sl1.pointIndex1_intersectionID != -1) && (sl1.pointIndex1_intersectionID == sl2.pointIndex0_intersectionID || sl1.pointIndex1_intersectionID == sl2.pointIndex1_intersectionID);
                if (isSame0 || isSame1)
                {
                    continue;
                }

                // stream lines will intersects only if controur positions intersects
                D s1 = sl1.startContourPosition;// start is always smaller from end
                D e1 = sl1.endContourPosition;
                D s2 = sl2.startContourPosition;// start is always smaller from end
                D e2 = sl2.endContourPosition;
                if (s2 < s1) // since we dont need recognition what is smaller s1 or s2 - lets sort by start positions
                {
                    swap(s1, s2);
                    swap(e1, e2);
                }
                if (s1 < s2 && s2 < e1 && e1 < e2)
                {
                    P3 closestPointOnLine1;
                    P3 closestPointOnLine2;
                    D minDistBetweenLines;
                    int line1_ClosestPointIsOnCornerPoint012;
                    utils::vector::ClosestPoint_BetweenLines(sl1.startPosition, sl1.endPosition, sl2.startPosition, sl2.endPosition, closestPointOnLine1, closestPointOnLine2, minDistBetweenLines, line1_ClosestPointIsOnCornerPoint012);
                    intersectoinInfo ii;
                    ii.id = intersectoinInfo_currentId;
                    ii.p = (closestPointOnLine1 + closestPointOnLine2) / 2;
                    ii.distPow2 = 0;
                    intersectoinInfo_currentId++;
                    sl1.intersections.push_back(ii);
                    sl2.intersections.push_back(ii);
                    foundIntersectionsCount += 2;

                    //DEBUG - show intersectoin point in face
                    if (debug_show_intersections)
                    {
                        //cout << "stream intersection#" << ii.id << ": fid = " << fid << ",   intersections count = " << streamLines.size() << endl;
                        P3 p = ii.p;
                        draw.AddPoint(p, Color3d(0, 0, 1));
                        draw.AddLabel(p, "  face intersection#" + to_string(ii.id), Color3d(0, 0, 1), 2);
                    }
                }
            }
        }
    }
    if (foundIntersectionsCount == 0) return;
    //checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Face - find intersectoin for faces that have more than 2 stream lines - store them into FaceStreamLine.intersections");


    //
    // merge intersections inside faces (this avoids duplicate points and very small triangles)
    //
    vector<int> mergedIntersectionIDs; // IntersectionID that was merged and hence are removed
    if (options.MergeClosePointsOnEdges)
    {
        D edge_percent_tolerance = options.MergeClosePointOnEdges__edge_percent_tolerance;
        D tol = avg_edge_length * edge_percent_tolerance;
        D tolPow2 = tol * tol;
        for (int fid = 0; fid < faceIntersectionCount.size(); fid++)
        {
            if (faceIntersectionCount(fid) < 2) continue;
            vector<FaceStreamLine>& streamLines = map_fid_facesStreamLines[fid];
            // store all intersection from all streamlines for current fid
            vector<pair<FaceStreamLine*, intersectoinInfo*>> intersections;
            intersections.reserve(100);
            for (auto& sl : streamLines)
            {
                for (auto& ii : sl.intersections) intersections.push_back({ &sl, &ii });
            }
            // merge intersections
            int size_minus_1 = intersections.size() - 1;
            for (int i = 0; i < intersections.size(); i++)
            {
                int iNext = (i == size_minus_1) ? 0 : i + 1;
                FaceStreamLine* sl0 = intersections[i].first;
                FaceStreamLine* sl1 = intersections[iNext].first;
                intersectoinInfo* ii0 = intersections[i].second;
                intersectoinInfo* ii1 = intersections[iNext].second;
                D distPow2 = utils::point::DistToPointPow2(ii0->p, ii1->p);
                if (distPow2 < tolPow2)
                {
                    int id0 = ii0->id;
                    int id1 = ii1->id;
                    if (id0 != id1)
                    {
                        for (auto& ii : intersections)
                        {
                            if (ii.second->id == id1 && ii.second->id != id0) // replace 'ii.second->id' from id1 to id0
                            {
                                int& mergedID = ii.second->id;
                                // remember removed id 
                                if (!utils::stdvector::exists(mergedIntersectionIDs, mergedID))
                                {
                                    mergedIntersectionIDs.push_back(mergedID); 
                                }
                                // merge id
                                mergedID = id0;
                            }
                        }
                    }
                }
            }
            // remove duplicate points from streamlines (since after merging intesections - two points may became one)
            for (auto& sl : streamLines)
            {
                bool needToFindDuplcates = true;
                while (needToFindDuplcates)
                {
                    needToFindDuplcates = false;
                    for (int i = 0; i < sl.intersections.size(); i++)
                    {
                        for (int i2 = 0; i2 < sl.intersections.size(); i2++)
                        {
                            if (i == i2) continue; // dont compate same intersection
                            if (sl.intersections[i].id == sl.intersections[i2].id)
                            {
                                //draw.AddEdge(mesh_original.V.row(mesh_original.F(fid, 0)), mesh_original.V.row(mesh_original.F(fid, 1)), Color3d(0, 0, 0));
                                //draw.AddEdge(mesh_original.V.row(mesh_original.F(fid, 1)), mesh_original.V.row(mesh_original.F(fid, 2)), Color3d(0, 0, 0));
                                //draw.AddEdge(mesh_original.V.row(mesh_original.F(fid, 2)), mesh_original.V.row(mesh_original.F(fid, 0)), Color3d(0, 0, 0));
                                utils::stdvector::remove_at(sl.intersections, i2);
                                needToFindDuplcates = true;
                                break;
                            }
                        }
                    }
                }
            }
        }

        // compact IntersectionIDs to avoid gaps - this will avoid later issues in indexing
        if (mergedIntersectionIDs.size() > 0)
        {
            utils::stdvector::sort(mergedIntersectionIDs);
            intersectoinInfo_currentId -= mergedIntersectionIDs.size();  //decrease counter since we merged some ids
            for (int fid = 0; fid < faceIntersectionCount.size(); fid++)
            {
                if (faceIntersectionCount(fid) < 2) continue;
                vector<FaceStreamLine>& streamLines = map_fid_facesStreamLines[fid];
                for (auto& sl : streamLines)
                {
                    for (auto& intersection : sl.intersections)
                    {
                        int mergedIdsBefore = 0;
                        for (int i = 0; i < mergedIntersectionIDs.size(); i++)
                        {
                            if (mergedIntersectionIDs[i] < intersection.id)
                            {
                                mergedIdsBefore++;
                            }
                        }
                        intersection.id -= mergedIdsBefore;
                    }
                }
            }
        }
    }

    //
    // accumulate FaceStreamLine for each stream
    //
    vector<vector<FaceStreamLine>> ss(streams.size());
    for (const auto& iter : map_fid_facesStreamLines)
    {
        int fid = iter.first;
        const vector<FaceStreamLine>& streamLines = iter.second;
        for (auto& sl : streamLines)
        {
            ss[sl.streamId].push_back(sl);
        }
    }

    //
    // insert intersection into streams
    //
    for (int sid = 0; sid < streams.size(); sid++)
    {
        MeshStream& points = streams[sid];
        vector<FaceStreamLine>& streamLines = ss[sid];
        if (streamLines.size() == 0) continue;
        // sort faceStreamLines for stream in descending order - since we want to insert new points
        vector<unsigned int> sorted_indexes = utils::stdvector::sort_indexes_custom(streamLines.size(), [&streamLines](unsigned int i1, unsigned int i2)
        {
            return streamLines[i1].pointIndex0 > streamLines[i2].pointIndex0;
        });
        for (auto index : sorted_indexes)
        {
            FaceStreamLine& sl = streamLines[index];
            vector<intersectoinInfo>& intersections = sl.intersections;
            for (auto& ii : intersections)
            {
                ii.distPow2 = utils::point::DistToPoint(points[sl.pointIndex0].point, ii.p);
            }
            // sort faceStreamline intersectoins in descending order - since we want to insert new points
            vector<unsigned int> sorted_indexes_intersection = utils::stdvector::sort_indexes_custom(intersections.size(), [&intersections](unsigned int i1, unsigned int i2)
            {
                return intersections[i1].distPow2 > intersections[i2].distPow2;
            });
            for (auto index_intersection : sorted_indexes_intersection)
            {
                const intersectoinInfo& ii = intersections[index_intersection];
                int insertIndex = sl.pointIndex1;
                StreamPoint p(MeshPointType::onFace, sl.fid, ii.p, sl.fid);
                p.intersectionID = ii.id;
                //if (insertIndex > 0 && points[insertIndex-1].Type == p.Type && points[insertIndex - 1].vid_eid_fid == 
                points.Insert(insertIndex, p);
            }
        }
        //checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Face - insert intersection into streams #" + to_string(sid));
    }
    //checkStreamsForDuplicatedPoints(streams, "FindStreamIntersections_Face - insert intersection into streams");


    //DEBUG - show intersection points
    //if (options.DebugEnabled && options.Debug_trace_intersections)
    //{
    //    for (int fid = 0; fid < faceIntersectionCount.size(); fid++)
    //    {
    //        if (faceIntersectionCount(fid) < 2) continue;
    //        cout << "stream intersection: fid = " << fid << ",   intersections count = " << faceIntersectionCount(fid) << endl;
    //        P3 p = m.F_Barycenters.row(fid);
    //        draw.AddPoint(p, Color3d(0, 0, 1));
    //        draw.AddLabel(p, "  face intersection#" + to_string(faceIntersectionIds(fid)), Color3d(0, 0, 1), 2);
    //    }
    //}
}

void MeshCutter::FindStreamEdgeIntersection(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, int& intersectoinInfo_currentId, bool debug_show_intersections)
{
    for (MeshStream& points : streams)
    {
        for (int i = 0; i < points.size(); i++)
        {
            StreamPoint& p = points[i];
            if (p.Type != MeshPointType::onVertex) // only edge and face intersections types can create new vertex
            {
                if (p.intersectionID < 0)
                {
                    p.intersectionID = intersectoinInfo_currentId;
                    intersectoinInfo_currentId++;

                    //DEBUG - show intersectoin point in face
                    //if (debug_show_intersections)
                    //{
                    //    //cout << "stream intersection#" << p.intersectionID << ": fid = " << fid << ",   intersections count = " << streamLines.size() << endl;
                    //    draw.AddPoint(p.point, Color3d(0.6, 0.6, 0.6));
                    //    draw.AddLabel(p.point, "  intersection#" + to_string(p.intersectionID), Color3d(0.6, 0.6, 0.6), 0);
                    //}
                }
            }
        }
    }
}



//
// define new structures
//
struct FaceStreamLine
{
    int fid;
    int eid_original;// if stream line is on the edge - remember edge id
    int streamId;
    int pointIndex0;
    int pointIndex1;
};
struct FacePoint
{
    int index; // index in 'vector<FacePoint> FacePoints'
    int vidnew;// what is vid in new mesh (it is in scope [0..m.VertexCount..])
    P3 point; //3d position of point
    int vid_original; // what was original vid, if point was on vertex
    int eid_original; // what was original eid, if point was on edge
    int intersectionID; // what is streams intersection id (can be -1 if this point was created from original vertex)
    int fid0;
    int fid1;
    FacePoint() // default contrsuctor for vector intialization
        : index(-1), vidnew(-1), point(P3(0, 0, 0)), vid_original(-1), eid_original(-1), intersectionID(-1), fid0(-1), fid1(-1)
    {

    }
    FacePoint(int _index, int _vidnew, P3 _point, int _vid_original = -1, int _eid_original = -1, int _intersectionID = -1)
        : index(_index), vidnew(_vidnew), point(_point), vid_original(_vid_original), eid_original(_eid_original), intersectionID(_intersectionID), fid0(-1), fid1(-1)
    {
    }
};
struct FacePointDist
{
    const FacePoint& p;
    D dist;
    int eid_original;
    FacePointDist(const FacePoint& _p, D _dist, int _eid_original)
        : p(_p), dist(_dist), eid_original(_eid_original)
    {
    }
};
struct FaceLine
{
    int index; // index in 'vector<FaceLine> FaceLines'
    int eidnew;// what is eid in new mesh (it is in scope [0..])
    int eid_original;
    int fid0;
    int fid1;// can be -1, if faceline is not on edge
    const FacePoint& p0;
    const FacePoint& p1;
    FaceStreamLine* streamLine; //can be null if there are no streamLine behind this line
    bool addedByNgonSeparation;
    FaceLine(int _index, int _eidnew, int _eid_original, const FacePoint& _p0, const FacePoint& _p1, FaceStreamLine* _streamLine = nullptr, int _fid0 = -1, int _fid1 = -1)
        : index(_index), eidnew(_eidnew), eid_original(_eid_original), p0(_p0), p1(_p1), streamLine(_streamLine), fid0(_fid0), fid1(_fid1), addedByNgonSeparation(false)
    {
    }
};
struct VectorAtPoint
{
    int faceLineIndex; // index of FaceLine in 'vector<FaceLine> faceLines'
    int pointIndex_Local_start; // local to fid
    int pointIndex_Local_end;  // local to fid
    int pointIndex_Global_start;
    int pointIndex_Global_end;
    V3 vectorNormalized; // normalized vector
    D angleToPrevVector;
    D angleToNextVector;
    bool isAlreadyUsedByNewFace;// flag for loop where we find next closest vector
    int edgeDirectionSign;//-1 opposite, 0 not in edge, 1 same direction as edge
    VectorAtPoint()
        : faceLineIndex(0), pointIndex_Local_start(0), pointIndex_Local_end(0), pointIndex_Global_start(0), pointIndex_Global_end(0), vectorNormalized(V3(0, 0, 0)), angleToPrevVector(0), angleToNextVector(0), isAlreadyUsedByNewFace(false), edgeDirectionSign(0)
    {
    }
    VectorAtPoint(int _faceLineIndex, int _pointIndex_Local_start, int _pointIndex_Local_end, int _pointIndex_Global_start, int _pointIndex_Global_end, V3 _vector, int _edgeDirectionSign)
        : faceLineIndex(_faceLineIndex), pointIndex_Local_start(_pointIndex_Local_start), pointIndex_Local_end(_pointIndex_Local_end), pointIndex_Global_start(_pointIndex_Global_start), pointIndex_Global_end(_pointIndex_Global_end),
        vectorNormalized(_vector), angleToPrevVector(0), angleToNextVector(0), isAlreadyUsedByNewFace(false), edgeDirectionSign(_edgeDirectionSign)
    {
        vectorNormalized.normalize();
    }
};
struct NgonPoint
{
    int facePointIndex; // index of FacePoint in 'vector<FacePoint> facesPoints'
    int faceLineIndex;  // index of FaceLine in 'vector<FaceLine> facesLines'
    V3 directionToNextPointNormalized; // normalized vector
    D angleAtThisPoint;// angle at point between edges
};
struct FaceNew
{
    int fidnew;// what is fid in new mesh (it is in scope [0..])
    int fid_original;
    const FaceLine& line1;
    const FaceLine& line2;
    const FaceLine& line3;
    FaceNew(int _fidnew, int _fid_original, const FaceLine& _line1, const FaceLine& _line2, const FaceLine& _line3)
        : fidnew(_fidnew), fid_original(_fid_original), line1(_line1), line2(_line2), line3(_line3)
    {
    }
};


void findCloseLoop(CompactVectorVector<VectorAtPoint>& point_lines, CompactVectorVector<NgonPoint>& ngons,
    int stage, int pointIndex_Local_start, int new_ngon_index,
    VectorAtPoint& v_recursive, int& iterarionsCount, bool& success)
{
    iterarionsCount++;
    // lock vector from D usage
    v_recursive.isAlreadyUsedByNewFace = true;
    // check if we reach start point
    if (v_recursive.pointIndex_Local_end == pointIndex_Local_start)
    {
        success = iterarionsCount >= 3; // valid ngong must have more then 2 edges
        // dont show waning messages as this is ok - we just ignore redundant lines that are not creating any ngongs
        //if (iterarionsCount < 3)
        //{
        //    assert(iterarionsCount >= 3 && "valid ngong must have at least 3 edges");
        //    cout << "warning:  MeshCutter.findCloseLoop()   valid ngong must have more then 2 edges" << endl;
        //}
    }
    else
    {
        // find next line of ngon 
        int nextVindex = -1;
        for (int n = 0; n < point_lines.size(v_recursive.pointIndex_Local_end); n++)
        {
            VectorAtPoint& v = point_lines(v_recursive.pointIndex_Local_end, n);
            if (v.faceLineIndex == v_recursive.faceLineIndex) // we found same VectorAtPoint but just opposite
            {
                //nextVindex = n + 1; // lets take next clockwise line (each next line should go counterclockwise, but since we want to populate our list from top to down - lets take next clockwise line)
                //if (nextVindex > point_lines.size(v_recursive.pointIndex_Local_end) - 1) nextVindex = 0;
                nextVindex = n - 1;
                if (nextVindex < 0) nextVindex = point_lines.size(v_recursive.pointIndex_Local_end) - 1;
                break;
            }
        }
        if (nextVindex >= 0)
        {
            //if (v_recursive.isAlreadyUsedByNewFace) continue;
            VectorAtPoint& v = point_lines(v_recursive.pointIndex_Local_end, nextVindex);
            if (!v.isAlreadyUsedByNewFace && v.edgeDirectionSign >= 0)
            {
                findCloseLoop(point_lines, ngons, stage, pointIndex_Local_start, new_ngon_index, v, iterarionsCount, success);
            }
        }
    }

    if (success)
    {
        if (stage == 2) // populate ngons points
        {
            NgonPoint p;
            p.faceLineIndex = v_recursive.faceLineIndex;
            p.facePointIndex = v_recursive.pointIndex_Global_start;
            p.directionToNextPointNormalized = v_recursive.vectorNormalized;
            p.angleAtThisPoint = 0; // at this point of time we dont know angle - we will populate it later
            ngons.add_inReversedOrder(new_ngon_index, p);// because we populate data in recursive method - we populate it from back to start - so to preserve order we have to populate data in reversed order
        }
    }
    else
    {
        //v_recursive.isAlreadyUsedByNewFace = false; // unlock vector in case of fail
        iterarionsCount--;
    }
};


void MeshCutter::CutMesh(const Mesh& m, const P3s& m_V, vector<MeshStream>& streams, vector<Mesh>& ms)
{
    const auto* mF = &m.F;

    //
    // grab all streams for face
    //
    CompactVectorVector<FaceStreamLine> facesStreamLines;
    facesStreamLines.resizeBegin(m.FacesCount);
    for (int stage = 0; stage < 2; stage++)
    {
        if (stage == 1)
        {
            facesStreamLines.resizeEnd();
        }
        for (int sid = 0; sid < streams.size(); sid++)
        {
            MeshStream& points = streams[sid];
            for (int i = 0; i < points.size() - 1; i++) // exluding last to be able to take 'i+1'
            {
                const StreamPoint& p1 = points[i];
                const StreamPoint& p2 = points[i + 1];

                int commonfid1 = -1;
                int commonfid2 = -1;
                int commoneid = -1;
                if (p1.Type == MeshPointType::onFace)
                {
                    commonfid1 = p1.vid_eid_fid;
                }
                if (p2.Type == MeshPointType::onFace)
                {
                    commonfid1 = p2.vid_eid_fid;
                }
                if (p1.Type == MeshPointType::onVertex && p2.Type == MeshPointType::onEdge)
                {
                    int vid = p1.vid_eid_fid;
                    int eid = p2.vid_eid_fid;
                    m.CommonFaceIds_VertexEdge(vid, eid, commonfid1, commonfid2);
                    if (commonfid1 != -1 && commonfid2 != -1)
                    {
                        commoneid = eid;
                    }
                }
                if (p1.Type == MeshPointType::onEdge && p2.Type == MeshPointType::onVertex)
                {
                    int eid = p1.vid_eid_fid;
                    int vid = p2.vid_eid_fid;
                    m.CommonFaceIds_VertexEdge(vid, eid, commonfid1, commonfid2);
                    if (commonfid1 != -1 && commonfid2 != -1)
                    {
                        commoneid = eid;
                    }
                }
                if (p1.Type == MeshPointType::onEdge && p2.Type == MeshPointType::onEdge)
                {
                    commonfid1 = m.CommonFaceIds_EdgeEdge(p1.vid_eid_fid, p2.vid_eid_fid);
                    if (p1.vid_eid_fid == p2.vid_eid_fid)
                    {
                        commoneid = p1.vid_eid_fid;
                        commonfid1 = m.EF(commoneid, 0);
                        commonfid2 = m.EF(commoneid, 1);
                        //DEBUG test dists
                        //int vid0 = m.EV(commoneid, 0);
                        //int vid1 = m.EV(commoneid, 1);
                        //P3 v0 = m_V.row(vid0);
                        //P3 v1 = m_V.row(vid1);
                        //D dist_p1v0 = utils::point::DistToPoint(p1.point, v0);
                        //D dist_p1v1 = utils::point::DistToPoint(p1.point, v1);
                        //D dist_p2v0 = utils::point::DistToPoint(p2.point, v0);
                        //D dist_p2v1 = utils::point::DistToPoint(p2.point, v1);
                        //int temp = 0;
                    }
                }

                if (commonfid1 != -1)
                {
                    if (stage == 0)
                    {
                        facesStreamLines.size(commonfid1)++;
                    }
                    else
                    {
                        FaceStreamLine f;
                        f.fid = commonfid1;
                        f.eid_original = commoneid;
                        f.streamId = sid;
                        f.pointIndex0 = i;
                        f.pointIndex1 = i + 1;
                        facesStreamLines.add(commonfid1, f);
                    }
                }

                if (commonfid2 != -1)
                {
                    if (stage == 0)
                    {
                        facesStreamLines.size(commonfid2)++;
                    }
                    else
                    {
                        FaceStreamLine f;
                        f.fid = commonfid2;
                        f.eid_original = commoneid;
                        f.streamId = sid;
                        f.pointIndex0 = i;
                        f.pointIndex1 = i + 1;
                        facesStreamLines.add(commonfid2, f);
                    }
                }
            }
        }
    }
    vector<int> facesStreamLines_NonEmptyFids;
    facesStreamLines_NonEmptyFids.reserve(m.FacesCount);
    for (int fid = 0; fid < facesStreamLines.size(); fid++)
    {
        if (facesStreamLines.size(fid) == 0) continue;
        facesStreamLines_NonEmptyFids.push_back(fid);
    }

    //
    // calculate how many edges and faces will be removed from original mesh
    //
    Bs edgeWillBeRemoved = Bs::Zero(m.EdgesCount); //  set all values to false
    Bs faceWillBeRemoved = Bs::Zero(m.FacesCount);//  set all values to false
    int removedEdgesCount = 0;
    int removedFacesCount = 0;
    int max_intersectionID = -1;
    for (int fid : facesStreamLines_NonEmptyFids)
    {
        int count = facesStreamLines.size(fid);
        if (count == 0) continue;
        if (!faceWillBeRemoved(fid)) removedFacesCount++;
        faceWillBeRemoved(fid) = true;
        for (int i = 0; i < count; i++)
        {
            const FaceStreamLine& sl = facesStreamLines(fid, i);
            const MeshStream& points = streams[sl.streamId];
            const StreamPoint& p1 = points[sl.pointIndex0];
            const StreamPoint& p2 = points[sl.pointIndex1];
            max_intersectionID = max(max_intersectionID, p1.intersectionID);
            max_intersectionID = max(max_intersectionID, p2.intersectionID);
            if (p1.Type == MeshPointType::onEdge)
            {
                int eid = p1.vid_eid_fid;
                if (!edgeWillBeRemoved(eid)) removedEdgesCount++;
                edgeWillBeRemoved(eid) = true;
            }
            if (p2.Type == MeshPointType::onEdge)
            {
                int eid = p2.vid_eid_fid;
                if (!edgeWillBeRemoved(eid)) removedEdgesCount++;
                edgeWillBeRemoved(eid) = true;
            }
        }
    }


    //
    // calculate new mesh indexes for vertex, edge, face
    //
    int next_vidnew = m.VertexCount;
    int next_eidnew = m.EdgesCount - removedEdgesCount;
    int next_fidnew = m.FacesCount - removedFacesCount;


    //
    // grab all points for all faces (next_vidnew)
    //
    vector<FacePoint> facesPoints;// first points are new points [0..max_intersectionID+1], rest are vertexes of edited faces
    facesPoints.reserve(max_intersectionID + 1 + removedFacesCount * 3);
    facesPoints.resize(max_intersectionID + 1);
    for (int i = 0; i < facesPoints.size(); i++)
    {
        facesPoints[i].index = i;
    }
    Is vid_To_facesPointId = Is::Constant(m.VertexCount, -1);
    // add points that are used by streams
    for (int fid : facesStreamLines_NonEmptyFids) // we have to use facesStreamLines since not all streamlines we need, but only those that create new faces
    {
        int count = facesStreamLines.size(fid);
        if (count == 0) continue;
        for (int i = 0; i < count; i++)
        {
            const FaceStreamLine& sl = facesStreamLines(fid, i);
            const MeshStream& points = streams[sl.streamId];
            int pi[2] = { sl.pointIndex0, sl.pointIndex1 };
            for (int pointIndex : pi)
            {
                const StreamPoint& p = points[pointIndex];
                if (p.intersectionID != -1) //if streams create new point
                {
                    if (p.intersectionID >= facesPoints.size())
                    {
                        assert(p.intersectionID < facesPoints.size() && "warning:  MeshCutter.CutMesh()    invalid condition    p.intersectionID >= facesPoints.size()");
                        cout << "warning:  MeshCutter.CutMesh()    invalid condition    p.intersectionID >= facesPoints.size()" << endl;
                    }
                    FacePoint& fp = facesPoints[p.intersectionID];
                    if (fp.intersectionID == -1) // if we didnt allocate new point yet
                    {
                        if (p.Type == MeshPointType::onVertex)
                        {
                            fp.vidnew = p.vid_eid_fid; // get original vid - reuse original vertex
                            vid_To_facesPointId(p.vid_eid_fid) = p.intersectionID;
                        }
                        else
                        {
                            fp.vidnew = next_vidnew; // allocate new vid - if point is on edge or on face
                            next_vidnew++;
                        }
                        fp.point = p.point;
                        if (p.Type == MeshPointType::onVertex) fp.vid_original = p.vid_eid_fid;
                        if (p.Type == MeshPointType::onEdge)fp.eid_original = p.vid_eid_fid;
                        fp.intersectionID = p.intersectionID;
                    }
                    if (p.Type == MeshPointType::onEdge || p.Type == MeshPointType::onFace)
                    {
                        if (fp.fid0 == -1 || fp.fid0 == fid)// if not used or is same - set fid at 0 index, else at index 1
                        {
                            fp.fid0 = fid;
                        }
                        else
                        {
                            fp.fid1 = fid;
                        }
                    }
                }
            }
        }
    }
    // add points that are not used by streams but used by face (vertexes of face that where not used by streams)
    for (int fid : facesStreamLines_NonEmptyFids)
    {
        int count = facesStreamLines.size(fid);
        if (count == 0) continue;
        for (int k = 0; k < 3; k++)
        {
            int vid = m.F(fid, k);
            if (vid_To_facesPointId(vid) == -1)// if vertex of face is not added yet
            {
                P3 v = m_V.row(vid);
                facesPoints.push_back(FacePoint(facesPoints.size(), vid, v, vid));
                vid_To_facesPointId(vid) = facesPoints.size() - 1;
            }
        }
    }



    //
    // grab all lines for all faces (next_eidnew)
    //
    auto StreamPoint_to_FacePoint = [&vid_To_facesPointId, &facesPoints](const StreamPoint& p)
    {
        int facesPointsIndex = (p.Type == MeshPointType::onVertex)
            ? vid_To_facesPointId(p.vid_eid_fid)
            : p.intersectionID;
        //const FacePoint& fp = facesPoints[facesPointsIndex];
        return facesPointsIndex;
    };
    vector<FaceLine> facesLines;// first points are new points [0..max_intersectionID+1], rest are vertexes of edited faces
    facesLines.reserve(facesPoints.size() * 2);
    Bs facesLines_eid_isProcessed = Bs::Zero(m.EdgesCount);//  set all values to false
    for (int fid : facesStreamLines_NonEmptyFids) // we have to use facesStreamLines since not all streamlines we need, but only those that create new faces
    {
        int count = facesStreamLines.size(fid);
        if (count == 0) continue;
        int facesPointsIds[3] = { -1,-1,-1 };
        for (int k = 0; k < 3; k++)
        {
            int vid = m.F(fid, k);
            facesPointsIds[k] = vid_To_facesPointId(vid);
        }
        // add every stream line that cross face
        for (int i = 0; i < count; i++)
        {
            const FaceStreamLine& sl = facesStreamLines(fid, i);
            const MeshStream& points = streams[sl.streamId];
            if (sl.eid_original != -1) continue; // for now ignore streamLines that are on the edge - we will process them later, when we will create lines for every face's edge
            const FacePoint& fp1 = facesPoints[StreamPoint_to_FacePoint(points[sl.pointIndex0])];
            const FacePoint& fp2 = facesPoints[StreamPoint_to_FacePoint(points[sl.pointIndex1])];
            int facesLineIndex = facesLines.size();
            facesLines.push_back(FaceLine(facesLineIndex, next_eidnew, -1, fp1, fp2, facesStreamLines.pointer(fid, i), fid));
            next_eidnew++;
        }
        // for every edge - create lines between vertexes and streamPoints
        int addedFaceLinesOnEdgeCount = 0;
        for (int k = 0; k < 3; k++)
        {
            int eid = m.FE(fid, k);
            if (facesLines_eid_isProcessed(eid)) continue;
            facesLines_eid_isProcessed(eid) = true;
            int vid0 = m.EV(eid, 0);
            int vid1 = m.EV(eid, 1);
            P3 v0 = m_V.row(vid0);
            P3 v1 = m_V.row(vid1);
            vector<FacePointDist> fpd;
            fpd.push_back(FacePointDist(facesPoints[vid_To_facesPointId(vid0)], 0, eid)); // add vertex v0 with dist=0, since dist from v0 to v0 is zero
            for (int i = 0; i < count; i++)
            {
                const FaceStreamLine& sl = facesStreamLines(fid, i);
                const MeshStream& points = streams[sl.streamId];
                const FacePoint& fp1 = facesPoints[StreamPoint_to_FacePoint(points[sl.pointIndex0])];
                const FacePoint& fp2 = facesPoints[StreamPoint_to_FacePoint(points[sl.pointIndex1])];
                if (fp1.eid_original == eid)
                {
                    D dist = utils::point::DistToPoint(v0, fp1.point);
                    bool fpalreadyExists = false;
                    for (auto& fpdi : fpd)if (fpdi.p.vidnew == fp1.vidnew) fpalreadyExists = true;
                    if (!fpalreadyExists) fpd.push_back(FacePointDist(fp1, dist, eid));
                }
                if (fp2.eid_original == eid)
                {
                    D dist = utils::point::DistToPoint(v0, fp2.point);
                    bool fpalreadyExists = false;
                    for (auto& fpdi : fpd)if (fpdi.p.vidnew == fp2.vidnew) fpalreadyExists = true;
                    if (!fpalreadyExists) fpd.push_back(FacePointDist(fp2, dist, eid));
                }
            }
            fpd.push_back(FacePointDist(facesPoints[vid_To_facesPointId(vid1)], utils::point::DistToPoint(v0, v1), eid)); // add vertex v1
            vector<unsigned int> fpd_sorted_indexes = utils::stdvector::sort_indexes_custom(fpd.size(), [&fpd](unsigned int i1, unsigned int i2)
            {
                return fpd[i1].dist < fpd[i2].dist;
            });
            //copy sorted loops to output result list
            for (int index = 0; index < fpd_sorted_indexes.size() - 1; index++)// excluding last to be able take 'index+1'
            {
                const FacePoint& p0 = fpd[fpd_sorted_indexes[index]].p;
                const FacePoint& p1 = fpd[fpd_sorted_indexes[index + 1]].p;
                //facesStreamLines.pointer(fid, i)
                int fid0 = m.EF(eid, 0);
                int fid1 = m.EF(eid, 1);
                int facesLineIndex = facesLines.size();
                facesLines.push_back(FaceLine(facesLineIndex, next_eidnew, eid, p0, p1, nullptr, fid0, fid1));
                next_eidnew++;
                addedFaceLinesOnEdgeCount++;
            }
        }
        // assign StreamLine to FaceLine's on edges - this can happend if streamLine goes from vertex to vertex (also can be intersecting by another stream line - then stream line on edge will be divided)
        for (int i = 0; i < count; i++)
        {
            const FaceStreamLine& sl = facesStreamLines(fid, i);
            const MeshStream& points = streams[sl.streamId];
            if (sl.eid_original == -1) continue; // take only streamLines that are on the edge - other we have already processed
            const FacePoint& fp0 = facesPoints[StreamPoint_to_FacePoint(points[sl.pointIndex0])];
            const FacePoint& fp1 = facesPoints[StreamPoint_to_FacePoint(points[sl.pointIndex1])];
            int eid = sl.eid_original;
            int vid0 = m.EV(sl.eid_original, 0);
            int vid1 = m.EV(sl.eid_original, 1);
            for (int k = 0; k < addedFaceLinesOnEdgeCount; k++)
            {
                FaceLine& fl = facesLines[facesLines.size() - 1 - k];
                if ((fl.p0.eid_original == eid || fl.p0.vid_original == vid0 || fl.p0.vid_original == vid1) // faceLine p0 is on same edge as streamLine
                    && (fl.p1.eid_original == eid || fl.p1.vid_original == vid0 || fl.p1.vid_original == vid1))// faceLine p1 is on same edge as streamLine
                {
                    if ((fl.p0.vidnew == fp0.vidnew || fl.p0.vidnew == fp1.vidnew)       // faceLine has common p0
                        && (fl.p1.vidnew == fp0.vidnew || fl.p1.vidnew == fp1.vidnew)) // and faceLine has common p1
                    {
                        fl.streamLine = facesStreamLines.pointer(fid, i); // add reference in FaceLine to StreamLine
                    }
                }
            }
        }
    }


    //
    // create new faces (next_fidnew)
    //
    // to improve performance first get max buffer size for points and lines per face
    Is pointsCountPerFace = Is::Zero(m.FacesCount);
    Is linesCountPerFace = Is::Zero(m.FacesCount);
    for (auto& fp : facesPoints)
    {
        if (fp.fid0 != -1)pointsCountPerFace(fp.fid0)++;
        if (fp.fid1 != -1)pointsCountPerFace(fp.fid1)++;
    }
    for (auto& fl : facesLines)
    {
        if (fl.fid0 != -1)pointsCountPerFace(fl.fid0)++;
        if (fl.fid1 != -1)pointsCountPerFace(fl.fid1)++;
    }
    int maxPointsPerFace = pointsCountPerFace.maxCoeff() + 3; // plus 3 vertexes
    int maxLinesPerFace = linesCountPerFace.maxCoeff();
    maxPointsPerFace *= 10; // multipliying by 10 we guarantee that we will never get out of buffer scope
    maxLinesPerFace *= 10; // multipliying by 10 we guarantee that we will never get out of buffer scope

    // for every face that crosses stream lines  - create new faces
    vector<int> facePointsIndexes; // indexes to FacePoint vector
    vector<int> faceLinesIndexes;// indexes to FaceLine vector
    facePointsIndexes.reserve(maxPointsPerFace);
    faceLinesIndexes.reserve(maxLinesPerFace);
    vector<pair<int, int>> faceLinesIds;// ids of faceLine: pait<point.index, point.index>
    faceLinesIds.reserve(maxLinesPerFace);
    Is global_facePointIndex_to_local = Is::Zero(facesPoints.size());
    vector<CompactVectorVector<NgonPoint>> facesStreamLines_ngonsTriangulated;
    facesStreamLines_ngonsTriangulated.reserve(facesStreamLines_NonEmptyFids.size());
    for (int fid : facesStreamLines_NonEmptyFids)
    {
        if (options.DebugEnabled && options.Debug_cutmesh__show_intersecting_faces && fid != options.Debug_cutmesh__show_intersecting_faces__show_only_for_face && options.Debug_cutmesh__show_intersecting_faces__show_only_for_face != -1) continue;

        V3 normal = m.F_normals.row(fid);

        // get all points for current face
        facePointsIndexes.clear();
        for (int i = 0; i < facesPoints.size(); i++)
        {
            const FacePoint& fp = facesPoints[i];
            int fp__vid_original = fp.vid_original;
            if (fp__vid_original == -1)
            {
                if (fp.fid0 != fid && fp.fid1 != fid) continue;// if point is not vertex and edge not belong to face
            }
            else
            {
                int vid0 = (*mF)(fid, 0);
                int vid1 = (*mF)(fid, 1);
                int vid2 = (*mF)(fid, 2);
                if (vid0 != fp__vid_original && vid1 != fp__vid_original && vid2 != fp__vid_original) continue; // if is vertex and this vertex not belong to face
            }
            global_facePointIndex_to_local(i) = facePointsIndexes.size(); // no need to clear 'global_facePointIndex_to_local' before this assignment since we never touch others indexes - so we use only those that we need and dont care for others
            facePointsIndexes.push_back(i);
        }

        // get all lines for current face
        faceLinesIndexes.clear(); // local faceLine indexes
        faceLinesIds.clear();
        if (facesLines.size() != 0)
        {
            const FaceLine* fl = facesLines.data();
            const FaceLine* flEnd = fl + facesLines.size() - 1;
            int i = 0;
            fl--;
            i--;
            while (fl < flEnd)
            {
                fl++;
                i++;
                // check if line belongs to current face
                if (fl->fid0 != fid && fl->fid1 != fid) continue;// if point is not vertex and edge not belong to face
                // check if both 2 points are allowed points
                int p0 = fl->p0.index;
                int p1 = fl->p1.index;
                if (!utils::stdvector::exists(facePointsIndexes, p0) || !utils::stdvector::exists(facePointsIndexes, p1)) continue; // both points should belong to this face
                // check if this line is not merged with another, so avoid duplicate lines
                pair<int, int> lineid = (p0 < p1) ? pair<int, int>(p0, p1) : pair<int, int>(p1, p0); // sort to get <min,max>
                if (utils::stdvector::exists(faceLinesIds, lineid)) continue; // dont add duplicate lines
                faceLinesIndexes.push_back(i);
                faceLinesIds.push_back(lineid); // remember ids of lines that we have added
            }
        }

        // build relation 'point to vector of lines'
        CompactVectorVector<VectorAtPoint> point_lines;// local point index to global line index (each point have few lines connecting to it)
        point_lines.resizeBegin(facePointsIndexes.size());
        for (int i = 0; i < faceLinesIndexes.size(); i++)
        {
            const FaceLine& fl = facesLines[faceLinesIndexes[i]];
            int localIndex0 = global_facePointIndex_to_local(fl.p0.index);
            point_lines.size(localIndex0)++;
            int localIndex1 = global_facePointIndex_to_local(fl.p1.index);
            point_lines.size(localIndex1)++;
        }
        point_lines.resizeEnd();
        for (int i = 0; i < faceLinesIndexes.size(); i++)
        {
            const FaceLine& fl = facesLines[faceLinesIndexes[i]];
            int localIndex0 = global_facePointIndex_to_local(fl.p0.index);
            int localIndex1 = global_facePointIndex_to_local(fl.p1.index);
            int edgeDirectionSign0 = 0; // by deafult we assume that faceLine dosnt belong to any edge
            int edgeDirectionSign1 = 0; // by deafult we assume that faceLine dosnt belong to any edge
            if (fl.eid_original != -1) // if faceLine belong to some edge
            {
                edgeDirectionSign0 = 1; // assume that edge direction is same as face edges-contour direction
                edgeDirectionSign1 = -1;// assume that edge direction is same as face edges-contour direction
                for (int k = 0; k < 3; k++)
                {
                    if (m.FE(fid, k) == fl.eid_original)
                    {
                        if (m.EV(m.FE(fid, k), 0) != m.F(fid, k))
                        {
                            swap(edgeDirectionSign0, edgeDirectionSign1);
                        }
                    }
                }
            }
            point_lines.add(localIndex0, VectorAtPoint(fl.index, localIndex0, localIndex1, fl.p0.index, fl.p1.index, fl.p1.point - fl.p0.point, edgeDirectionSign0));
            point_lines.add(localIndex1, VectorAtPoint(fl.index, localIndex1, localIndex0, fl.p1.index, fl.p0.index, fl.p0.point - fl.p1.point, edgeDirectionSign1));
        }

        // sort 'CompactVectorVector<VectorAtPoint> point_lines' counter-clockwise based on angles between them
        for (int i = 0; i < point_lines.size(); i++)
        {
            int count = point_lines.size(i);
            // dont show waning messages as this is ok - we just ignore redundant lines that are not creating any ngongs
            //if (count < 2)
            //{
            //    assert(count >= 2 && "warning:  MeshCutter.CutMesh()   at one point must be at least 2 lines");
            //    cout << "warning:  MeshCutter.CutMesh()   at one point must be at least 2 lines but is "<<count<<"   (meshid="<<m.id <<", fid="<<fid<<")"<< endl;
            //}
            if (count == 0) continue;

            VectorAtPoint& base = point_lines(i, 0);
            base.angleToPrevVector = 0;
            for (int n = 1; n < count; n++)
            {
                VectorAtPoint& next = point_lines(i, n);
                next.angleToPrevVector = utils::vector::AngleFull(base.vectorNormalized, next.vectorNormalized, normal, true);
            }
            point_lines.sort(i, [](const VectorAtPoint& a, const VectorAtPoint& b)
            {
                return a.angleToPrevVector < b.angleToPrevVector;
            });
            D angleSumm = 0;
            for (int k = 0; k < count; k++)
            {
                int kNext = k + 1;
                if (kNext > count - 1) kNext = 0;
                VectorAtPoint& prev = point_lines(i, k);
                VectorAtPoint& next = point_lines(i, kNext);
                D angle = next.angleToPrevVector - angleSumm;
                if (angle < 0) angle = 360 + angle;
                angleSumm += angle;
                prev.angleToNextVector = angle;
                next.angleToPrevVector = -angle;
            }
            int temp = 0;
        }

        // find ngons (loops from faceLines)
        CompactVectorVector<NgonPoint> ngons;
        for (int stage = 0; stage < 3; stage++) // do our algorithm in 3 stages:  1) get ngonsCount  2) get face points count 3) populate ngons points
        {
            int ngonsCount = 0; // clear at every stage
            for (int i = 0; i < point_lines.size(); i++)
            {
                int count = point_lines.size(i);
                for (int n = 0; n < count; n++)
                {
                    point_lines(i, n).isAlreadyUsedByNewFace = false;  // clear at every stage
                }
            }

            for (int i = 0; i < point_lines.size(); i++)
            {
                for (int n = 0; n < point_lines.size(i); n++)
                {
                    VectorAtPoint& v = point_lines(i, n);
                    if (v.isAlreadyUsedByNewFace) continue;
                    int new_ngon_index = ngonsCount;
                    int iterarionsCount = 0;
                    bool success = false;
                    //auto items = point_lines.items(i); //DEBUG
                    findCloseLoop(point_lines, ngons, stage, v.pointIndex_Local_start, new_ngon_index, v, iterarionsCount, success);
                    if (success)
                    {
                        ngonsCount++;
                        if (stage == 1) // get face points count 
                        {
                            assert(iterarionsCount >= 3);
                            ngons.size(ngonsCount - 1) = iterarionsCount;
                        }
                    }
                }
            }
            if (stage == 0) // get ngonsCount
            {
                ngons.resizeBegin(ngonsCount);
            }
            else if (stage == 1) // get face points count capacity
            {
                ngons.resizeEnd();
            }
        }

        // set angles for ngons at points (set 'angleAtThisPoint')
        for (int ni = 0; ni < ngons.size(); ni++)
        {
            int ngonPointsCount = ngons.size(ni);
            if (ngonPointsCount == 3) continue; // if ngon has only 3 points than this is triangle and we dont need to get additional information 'angleAtThisPoint'
            for (int i = 0; i < ngonPointsCount; i++)
            {
                NgonPoint& p = ngons(ni, i);
                int iPrev = i - 1;
                if (iPrev < 0) iPrev = ngonPointsCount - 1;
                const NgonPoint& pPrev = ngons(ni, iPrev);
                p.angleAtThisPoint = utils::vector::AngleFull(p.directionToNextPointNormalized, -pPrev.directionToNextPointNormalized, normal, true);
            }
        }

        // split ngons to triangles
        facesStreamLines_ngonsTriangulated.push_back(CompactVectorVector<NgonPoint>());
        CompactVectorVector<NgonPoint>& ngonsTriangulated = facesStreamLines_ngonsTriangulated.back();
        // do our algorithm in 2 stages:  1) set size and capacities  2) populate ngonsTriangulated points
        //
        // step#1: set size and capacities
        //
        int ngonsTriangulatedCount = 0;
        for (int ni = 0; ni < ngons.size(); ni++)
        {
            ngonsTriangulatedCount += 1 + (ngons.size(ni) - 3); // each ngon with more than 3 points will be splited to few triangles (ngons)
        }
        ngonsTriangulated.resizeBegin(ngonsTriangulatedCount, true);
        for (int ni = 0; ni < ngonsTriangulated.size(); ni++)
        {
            ngonsTriangulated.size(ni) = 3; // each triangulated ngon will have exactly 3 points 
        }
        ngonsTriangulated.resizeEnd();
        //
        // step#2: populate ngonsTriangulated points
        //
        int current_ngonsTriangulated_index = 0;
        for (int ni = 0; ni < ngons.size(); ni++)
        {
            // while this is ngon with more than 3 points
            assert(ngons.size(ni) >= 3);
            while (ngons.size(ni) > 3)
            {
                // get best triangle from ngon ( triangle with widest angles)
                int bestTriangle_Index = -1;
                D bestTriangle_AnglesSumm = 0;
                int ngonsize = ngons.size(ni);
                for (int i0 = 0; i0 < ngonsize; i0++)
                {
                    int i1 = (i0 == ngonsize - 1) ? 0 : i0 + 1;
                    int i2 = (i1 == ngonsize - 1) ? 0 : i1 + 1;
                    const NgonPoint& p0 = ngons(ni, i0);
                    const NgonPoint& p1 = ngons(ni, i1);
                    const NgonPoint& p2 = ngons(ni, i2);
                    D angleSumm = p0.angleAtThisPoint + p2.angleAtThisPoint - p1.angleAtThisPoint;
                    if (bestTriangle_Index == -1 || angleSumm > bestTriangle_AnglesSumm)
                    {
                        bestTriangle_Index = i0;
                        bestTriangle_AnglesSumm = angleSumm;
                    }
                }
                int i0 = bestTriangle_Index;
                int i1 = (i0 == ngonsize - 1) ? 0 : i0 + 1;
                int i2 = (i1 == ngonsize - 1) ? 0 : i1 + 1;
                NgonPoint& p0 = ngons(ni, i0);
                NgonPoint& p1 = ngons(ni, i1);
                NgonPoint& p2 = ngons(ni, i2);
                int vid0 = p0.facePointIndex;
                int vid1 = p1.facePointIndex;
                int vid2 = p2.facePointIndex;
                if (vid0 == vid1 || vid0 == vid2 || vid1 == vid2) // validate face - all vertexes must be different
                {
                    cout << "!!! warning    MeshCutter.CutMesh():   vid0 == vid1 || vid0 == vid2 || vid1 == vid2     vids=" << vid0 << "," << vid1 << "," << vid2 << "  ni=" << ni << endl;
                }
                // copy triangle to new ngonsTriangulated storage
                ngonsTriangulated.add(current_ngonsTriangulated_index, p0);
                ngonsTriangulated.add(current_ngonsTriangulated_index, p1);
                ngonsTriangulated.add(current_ngonsTriangulated_index, p2);
                NgonPoint& p0new = ngonsTriangulated(current_ngonsTriangulated_index, 0);
                NgonPoint& p1new = ngonsTriangulated(current_ngonsTriangulated_index, 1);
                NgonPoint& p2new = ngonsTriangulated(current_ngonsTriangulated_index, 2);
                p2new.directionToNextPointNormalized = facesPoints[p0.facePointIndex].point - facesPoints[p2.facePointIndex].point;
                p2new.directionToNextPointNormalized.normalize();
                //p0new.angleAtThisPoint = skip updating angle since we dont need it anymore
                //p2new.angleAtThisPoint = skip updating angle since we dont need it anymore
                p2new.faceLineIndex = facesLines.size(); // update faceLineIndex to point new FaceLine
                int facesLineIndex = facesLines.size();
                facesLines.push_back(FaceLine(facesLineIndex, next_eidnew, -1, facesPoints[p0.facePointIndex], facesPoints[p2.facePointIndex], nullptr, fid)); //  add new FaceLine
                facesLines.back().addedByNgonSeparation = true;
                next_eidnew++;
                current_ngonsTriangulated_index++;
                // remove point from 'ngons' storage
                p0.directionToNextPointNormalized = facesPoints[p2.facePointIndex].point - facesPoints[p0.facePointIndex].point; // set new direction from i0 to i2 (i1 we are removing from ngons)
                p0.directionToNextPointNormalized.normalize();
                int i0Prev = (i0 == 0) ? ngonsize - 1 : i0 - 1;
                p0.angleAtThisPoint = utils::vector::AngleFull(p0.directionToNextPointNormalized, -ngons(ni, i0Prev).directionToNextPointNormalized, normal, true);
                p2.angleAtThisPoint = utils::vector::AngleFull(p2.directionToNextPointNormalized, -p0.directionToNextPointNormalized, normal, true);
                ngons.remove(ni, i1);
            }

            // copy triangle to new ngonsTriangulated storage
            assert(ngons.size(ni) >= 3);
            if (ngons.size(ni) == 3)
            {
                int vid0 = ngons(ni, 0).facePointIndex;
                int vid1 = ngons(ni, 1).facePointIndex;
                int vid2 = ngons(ni, 2).facePointIndex;
                if (vid0 == vid1 || vid0 == vid2 || vid1 == vid2) // validate face - all vertexes must be different
                {
                    cout << "!!! warning    MeshCutter.CutMesh():   vid0 == vid1 || vid0 == vid2 || vid1 == vid2     vids=" << vid0 << "," << vid1 << "," << vid2 << endl;
                }
                ngonsTriangulated.add(current_ngonsTriangulated_index, ngons(ni, 0));
                ngonsTriangulated.add(current_ngonsTriangulated_index, ngons(ni, 1));
                ngonsTriangulated.add(current_ngonsTriangulated_index, ngons(ni, 2));
            }
            current_ngonsTriangulated_index++;
        }

        //DEBUG - show points and lines
        if (options.DebugEnabled && options.Debug_cutmesh__show_intersecting_faces && fid == options.Debug_cutmesh__show_intersecting_faces__show_only_for_face)
        {
            cout << "Cutmesh - create new faces:  fid = " << fid << " -  found " << facePointsIndexes.size() << " points and " << faceLinesIndexes.size() << " lines" << endl;

            for (int i = 0; i < point_lines.size(); i++)
            {
                cout << "point" << i << "  P#" << facePointsIndexes[i] << "  lines: ";
                for (int k = 0; k < point_lines.size(i); k++)
                {
                    cout << " L#" << point_lines(i, k).faceLineIndex;
                }
                cout << "       ";
                for (int k = 0; k < point_lines.size(i); k++)
                {
                    const FaceLine& fl = facesLines[point_lines(i, k).faceLineIndex];
                    cout << "  [" << fl.p0.index << "," << fl.p1.index << "]";
                }
                cout << "       degrees: ";
                for (int k = 0; k < point_lines.size(i); k++)
                {
                    cout << "  " << to_string(point_lines(i, k).angleToNextVector);
                }
                cout << endl;
            }

            for (int i = 0; i < ngons.size(); i++)
            {
                int count = ngons.size(i);
                cout << "ngon#" << i << "    points: ";
                for (int n = 0; n < count; n++)
                {
                    cout << " P#" << ngons(i, n).facePointIndex;
                }
                cout << endl;
            }

            for (int i = 0; i < ngonsTriangulated.size(); i++)
            {
                int count = ngonsTriangulated.size(i);
                cout << "ngonsTriangulated#" << i << "    points: ";
                for (int n = 0; n < count; n++)
                {
                    cout << " P#" << ngonsTriangulated(i, n).facePointIndex;
                }
                cout << endl;
            }
        }
        if (options.DebugEnabled && options.Debug_cutmesh__show_intersecting_faces)
        {
            //for (int ni = 0; ni < ngons.size(); ni++)
            //{
            //    P3 middle(0, 0, 0);
            //    int count = ngons.size(ni);
            //    for (int n = 0; n < count; n++)
            //    {
            //        const FacePoint& p = facesPoints[ngons(ni, n).facePointIndex];
            //        middle += p.point;
            //    }
            //    middle /= count;
            //    draw.AddLabel(middle, "ngon" + to_string(count) + "#" + to_string(ni), Color3d(0,0,1));
            //}
            for (int ni = 0; ni < ngonsTriangulated.size(); ni++)
            {
                P3 middle(0, 0, 0);
                int count = ngonsTriangulated.size(ni);
                for (int n = 0; n < count; n++)
                {
                    const FacePoint& p = facesPoints[ngonsTriangulated(ni, n).facePointIndex];
                    middle += p.point;
                }
                middle /= count;
                draw.AddLabel(middle, "ngonTri" + to_string(count) + "#" + to_string(ni), Color3d(1, 0.7, 0.7));
            }
        }
    }

    // DEBUG - show streams for face
    if (options.DebugEnabled && options.Debug_cutmesh__show_intersecting_faces)
    {
        // show face id, and how many streams crosses this face
        for (int fid = 0; fid < m.FacesCount; fid++)
        {
            if (fid == options.Debug_cutmesh__show_intersecting_faces__show_only_for_face || options.Debug_cutmesh__show_intersecting_faces__show_only_for_face == -1)
            {
                if (facesStreamLines.size(fid) > 0)
                {
                    //faceIsIntersectingStreamLine(fid)
                    //draw.FaceColors.row(fid) = Color3d(1, 0, 0); - we cant do this
                    draw.AddLabel(m.F_Barycenters.row(fid), "fid=" + to_string(fid) + ", streams = " + to_string(facesStreamLines.size(fid)), Color3d(0, 0, 1), 3);
                    //if (options.Debug_cutmesh__show_intersecting_faces__show_only_for_face != -1)
                    //{
                    //    for (int k = 0; k < 3; k++)
                    //    {
                    //        int vid = m.F(fid, k);
                    //        P3 v = m_V.row(vid);
                    //        draw.AddPoint(v, Color3d(0.6, 0.6, 0.6), "vid=" + to_string(vid));
                    //    }
                    //}
                }
                //for (int i = 0; i < facesStreamLines.size(fid); i++)
                //{
                //    const FaceStreamLine& fsl = facesStreamLines(fid, i);
                //    const StreamPoints& points = streams[fsl.streamId];
                //    const StreamPoint& p1 = points[fsl.pointIndex1];
                //    const StreamPoint& p2 = points[fsl.pointIndex2];
                //    draw.AddEdge(p1.point, p2.point, Color3d(1, 0, 0));
                //    //draw.AddPoint(p1.point, Color3d(1, 0, 0));
                //    //draw.AddPoint(p2.point, Color3d(1, 0, 0));
                //}
            }
        }
        const Color3d colorRedLight(1, 0.7, 0.7);
        const Color3d colorPurple(1, 0, 0.6);
        const Color3d colorGreen(0, 1, 0);
        const Color3d colorBlue(0, 0, 1);

        // show points
        for (int i = 0; i < facesPoints.size(); i++)
        {
            const FacePoint& fp = facesPoints[i];
            int fidOnly = options.Debug_cutmesh__show_intersecting_faces__show_only_for_face;
            if (fidOnly != -1 && fidOnly < m.F.rows())
            {
                if (fp.vid_original == -1 && fp.fid0 != fidOnly && fp.fid1 != fidOnly) continue;// if point is not vertex and edge not belong to limited face
                if (fp.vid_original != -1 && m.F(fidOnly, 0) != fp.vid_original && m.F(fidOnly, 1) != fp.vid_original && m.F(fidOnly, 2) != fp.vid_original) continue; // if is vertex and this vertex not belong to face
            }
            Color3d color = (fp.intersectionID != -1) ? colorPurple : colorGreen;
            string desc = "P#" + to_string(i) + ", vidnew=" + to_string(fp.vidnew);
            //if (fp.vid_original != -1) desc += ", vid=" + to_string(fp.vid_original);
            draw.AddPoint(fp.point, color, desc);
        }

        // show lines
        for (int i = 0; i < facesLines.size(); i++)
        {
            const FaceLine& fl = facesLines[i];
            int fidOnly = options.Debug_cutmesh__show_intersecting_faces__show_only_for_face;
            if (fidOnly != -1)
            {
                if (fl.fid0 != fidOnly && fl.fid1 != fidOnly) continue;// if point is not vertex and edge not belong to limited face
            }
            Color3d color = (fl.streamLine != nullptr) ? colorPurple : colorGreen;
            if (fl.addedByNgonSeparation) color = colorRedLight;
            draw.AddEdge(fl.p0.point, fl.p1.point, color);
            P3 flmidPoint = (fl.p0.point + fl.p1.point) / 2;
            draw.AddLabel(flmidPoint, "L#" + to_string(i), Color3d(0.5, 0.5, 0.5), -2);
        }

    }


    //
    // Create new mesh fom collected data
    //
    //  CompactVectorVector<FaceStreamLine> facesStreamLines;
    //  vector<CompactVectorVector<NgonPoint>> facesStreamLines_ngonsTriangulated;
    //  vector<FacePoint> facesPoints
    //  vector<FaceLine> facesLines
    //  int next_vidnew = m.VertexCount;
    //  int next_eidnew = m.EdgesCount - removedEdgesCount;
    //  int next_fidnew = m.FacesCount - removedFacesCount;


    // copy vertexes
    P3s V(next_vidnew, 3);
    // copy old vertexes
    //for (int i = 0; i < m_V.rows(); i++)
    //{
    //    V.row(i) = m_V.row(i);
    //}
    memcpy(V.data(), m_V.data(), sizeof(P3)*m_V.rows());
    // add new vertexes
    Vector<bool> VisInited = Vector<bool>::Zero(next_vidnew);//  set all values to false
    for (const FacePoint& fp : facesPoints)
    {
        if (fp.vidnew < m.VertexCount || VisInited(fp.vidnew)) continue;
        V.row(fp.vidnew) = fp.point.transpose();
        VisInited(fp.vidnew) = true;
    }
    #if DEBUG
    for (int i = m_V.rows(); i < V.rows(); i++)
    {
        assert(VisInited(i));
    }
    #endif

    // copy faces
    int addednewFacesCount = 0;
    for (const CompactVectorVector<NgonPoint>& ngonsTriangulated : facesStreamLines_ngonsTriangulated)
    {
        addednewFacesCount += ngonsTriangulated.size();
    }
    I3s F;
    F.resize(next_fidnew + addednewFacesCount, 3);
    // copy old faces and only those that was not removed
    int currentFrowindex = 0;
    for (int fid = 0; fid < facesStreamLines.size(); fid++)
    {
        if (facesStreamLines.size(fid) != 0) continue;
        I3 vids = m.F.row(fid);
        //F.row(currentFrowindex) = vids;
        F(currentFrowindex, 0) = vids(0);
        F(currentFrowindex, 1) = vids(1);
        F(currentFrowindex, 2) = vids(2);
        currentFrowindex++;
    }
    // copy new faces
    int debug_index = -1;
    for (const CompactVectorVector<NgonPoint>& ngonsTriangulated : facesStreamLines_ngonsTriangulated)
    {
        debug_index++;
        //if (debug_index == 10) continue;//DEBUG
        for (int ni = 0; ni < ngonsTriangulated.size(); ni++)
        {
            if (ngonsTriangulated.size(ni) != 3)
            {
                cout << "!!! warning    MeshCutter.CutMesh():   ngonsTriangulated.size(ni) != 3" << endl;
                continue; // skip empty triangles
            }
            const FacePoint& p0 = facesPoints[ngonsTriangulated(ni, 0).facePointIndex];
            const FacePoint& p1 = facesPoints[ngonsTriangulated(ni, 1).facePointIndex];
            const FacePoint& p2 = facesPoints[ngonsTriangulated(ni, 2).facePointIndex];
            int vid0 = p0.vidnew;
            int vid1 = p1.vidnew;
            int vid2 = p2.vidnew;
            if (vid0 == vid1 || vid0 == vid2 || vid1 == vid2) // validate face - all vertexes must be different
            {
                cout << "!!! warning    MeshCutter.CutMesh():   vid0 == vid1 || vid0 == vid2 || vid1 == vid2     vids=" << vid0 << "," << vid1 << "," << vid2 << endl;
                continue;
            }
            F(currentFrowindex, 0) = vid0;
            F(currentFrowindex, 1) = vid1;
            F(currentFrowindex, 2) = vid2;
            //if (debug_index == 10)
            //{
            //    cout << "currentFrowindex=" << currentFrowindex << endl;
            //    cout << "    vid0=" << vid0 << "          " << V.row(vid0) << endl;
            //    cout << "    vid1=" << vid1 << "          " << V.row(vid1) << endl;
            //    cout << "    vid2=" << vid2 << "          " << V.row(vid2) << endl;
            //    //cout << "vid0,vid1,vid2=" << vid0 <<","<< vid0 << "," << vid0 << "          " << V.row(vid0) << "   " << V.row(vid1) << "   " << V.row(vid2) << "   " << endl;
            //}
            //if (debug_index == 9|| debug_index == 10)
            //{
            //    cout << "debug_index=" << debug_index << "    currentFrowindex=" << currentFrowindex << endl;
            //    cout << "    vid0=" << vid0 << "          " << V.row(vid0) << endl;
            //    cout << "    vid1=" << vid1 << "          " << V.row(vid1) << endl;
            //    cout << "    vid2=" << vid2 << "          " << V.row(vid2) << endl;
            //    //cout << "vid0,vid1,vid2=" << vid0 <<","<< vid0 << "," << vid0 << "          " << V.row(vid0) << "   " << V.row(vid1) << "   " << V.row(vid2) << "   " << endl;
            //}
            //if (vid0 == 807 || vid1 == 807 || vid2 == 807)
            //{
            //    cout << "debug_index=" << debug_index << "    vids=" << vid0 << ", " << vid1 << ", " << vid2 << endl;
            //}

            currentFrowindex++;
        }
    }

    // decrease size of F if some faces was rejected by validation
    if (currentFrowindex != F.rows())
    {
        F.conservativeResize(currentFrowindex, 3);
    }

    //if (m.Name != "F009")
    //{
    //    V.resize(0, 3);
    //    F.resize(0, 3);
    //}

    // create mesh from new V and F
    Mesh mNew;
    Mesh::CreateFromVF(V, F, mNew, false, false, false, false, false);

    //
    // mark edges that holds stream as border edges (for those edges that hold streamlines we need to set flag mesh.E_isborder to true)
    //
    Vector<bool> VhasStreamsConnected;
    VhasStreamsConnected.setConstant(next_vidnew, false);//  set all values to false // just for speed optimization since asking map is quite long operation
    //v0
    //map<pair<int, int>, bool> edgeHoldsStream;
    //v1
    typedef union _LARGE_INTEGER
    {
        struct
        {
            __int32 LowPart;
            __int32 HighPart;
        } u;
        __int64 QuadPart;
    } LARGE_INTEGER;
    struct pairHasher
    {
        std::size_t operator()(const pair<int, int>& k) const
        {
            LARGE_INTEGER hash;
            hash.u.LowPart = k.first;
            hash.u.HighPart = k.second;
            return hash.QuadPart;
        }
    };
    unordered_map<pair<int, int>, bool, pairHasher> edgeHoldsStream; // lets use unordered_map to improve performance by 2% for large models
    edgeHoldsStream.reserve(mNew.EV.rows());
    auto markEdgeThatHoldStreamAsBorder = [&VhasStreamsConnected, &edgeHoldsStream](ViewerDrawObjects& draw, int vid0, int vid1)
    {
        if (vid1 < vid0)
        {
            swap(vid0, vid1);
        }
        VhasStreamsConnected(vid0) = true;
        VhasStreamsConnected(vid1) = true;
        edgeHoldsStream[{vid0, vid1}] = true;
    };

    for (const CompactVectorVector<NgonPoint>& ngonsTriangulated : facesStreamLines_ngonsTriangulated)
    {
        for (int ni = 0; ni < ngonsTriangulated.size(); ni++)
        {
            for (int k = 0; k < 3; k++)
            {
                const FaceLine& fl = facesLines[ngonsTriangulated(ni, k).faceLineIndex];
                if (fl.streamLine != nullptr)
                {
                    //draw.AddEdge(fl.p0.point, fl.p1.point, Color3d(1, 0, 0));
                    markEdgeThatHoldStreamAsBorder(draw, fl.p0.vidnew, fl.p1.vidnew);
                }
            }
        }
        for (int sid = 0; sid < streams.size(); sid++)
        {
            MeshStream& points = streams[sid];
            int size = points.size();
            if (size == 0) continue;
            const StreamPoint* p0 = &points.Points[0];
            const StreamPoint* p1 = p0 + 1;
            for (int i = 1; i < size; i++) // exluding last to be able to take 'i+1'
            {
                if (p0->Type == MeshPointType::onVertex && p1->Type == MeshPointType::onVertex)
                {
                    //draw.AddEdge(p0.point, p1.point, Color3d(1, 0, 0));
                    markEdgeThatHoldStreamAsBorder(draw, p0->vid_eid_fid, p1->vid_eid_fid);
                }
                p0 = p1;
                p1++;
            }
        }
    }
    for (int eid = 0; eid < mNew.EV.rows(); eid++)
    {
        int vid0 = mNew.EV(eid, 0);
        int vid1 = mNew.EV(eid, 1);
        if (vid1 < vid0)
        {
            swap(vid0, vid1);
        }
        if (VhasStreamsConnected(vid0) && VhasStreamsConnected(vid1) && edgeHoldsStream[{vid0, vid1}])
        {
            mNew.E_isborder[eid] = true;
            //DEBUG show edges that are recognized as stream holders
            if (options.DebugEnabled && options.Debug_cutmesh__show_edges_that_are_recognized_as_stream_holders)
            {
                P3 v0 = mNew.V.row(vid0);
                P3 v1 = mNew.V.row(vid1);
                draw.AddEdge(v0, v1, Color3d(1, 0, 0));
                draw.AddLabel(v0, "  " + to_string(vid0));
                draw.AddLabel(v1, "  " + to_string(vid1));
            }
        }
    }

    //
    // split mesh to few meshes based on stream lines that comes trought edges
    //
    mNew.ComputeParts_BaseOnEdgeToFaceConnections();
    ms.resize(mNew.Parts.size());
    for (int i = 0; i < mNew.Parts.size(); i++)
    {
        const MeshPart& part = mNew.Parts[i];
        part.ExtractMesh(ms[i], splitNakedTriangleWith2NakedEdge, correctFacesEdgeIndexes, optimizeVertexIndexesForGPU, true);
        ms[i].originalMeshId = m.id;
    }

    //ms.resize(1);
    //ms.push_back(m); //DEBUG temp
    //ms.push_back(mNew);
}

void MeshCutter::ValidateIntersectionsIds(const vector<MeshStream>& streams, int intersectoinInfo_currentId)
{
    #ifdef DEBUG
    auto IntersectionsCount = intersectoinInfo_currentId;
    vector<int>  allIntersectionsIds;
    for (const MeshStream& stream : streams)
    {
        if (stream.Points.size() < 2) continue; //skip invalid streams
        for (const auto& point : stream.Points)
        {
            if (point.intersectionID == -1)  continue;
            if (utils::stdvector::exists(allIntersectionsIds, point.intersectionID)) continue;
            allIntersectionsIds.push_back(point.intersectionID);
        }
    }
    utils::stdvector::sort(allIntersectionsIds);
    // validation#1
    if (allIntersectionsIds.size() != IntersectionsCount)
    {
        std::cout << "!!! Error:   MeshCutter::FindStreamIntersections() -  invalid intersection ids detected:  allIntersectionsIds.size()=" << allIntersectionsIds.size() << ",   IntersectionsCount=" << IntersectionsCount << std::endl;
        assert(allIntersectionsIds.size() == IntersectionsCount);
    }
    else
    {
        //validation#2
        for (int i = 0; i < allIntersectionsIds.size(); i++)
        {
            if (allIntersectionsIds[i] != i)
            {
                std::cout << "!!! Error:   MeshCutter::FindStreamIntersections() -  intersection #" << i << "  not found" << std::endl;
                assert(allIntersectionsIds.size() == IntersectionsCount);
                break;
            }
        }
    }
    #endif
}
