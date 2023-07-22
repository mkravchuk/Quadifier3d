#include "stdafx.h"
#include "DividerIterator.h"
#include "Divider.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshStreams.h"
#include "MeshSolverNrosy.h"
#include "MeshSurface.h"
#include "MeshStreamsJoiner.h"

const MeshLogicOptions_DividerIterator& options = meshLogicOptions.DividerIterator;





DividerDynamicIteration::DividerDynamicIteration(int _index_InOriginalDividersList, Divider& _divider, const MeshSolverNrosy* _solver)
    : index_InOriginalDividersList(_index_InOriginalDividersList), divider(_divider), solver(_solver), meshId(_divider.mesh.id), isNeedToRedivide(true)
{
    // populate StopVertexIds
    vector<MeshPoint> points;
    vector<int> dividesBy; //  how many lines comes from this points (singularity - 3 or 5, others - 1,2,3)
    vector<bool> isSingularPoints;
    divider.GetDividingPoints(points, dividesBy, isSingularPoints);
    StopVertexIds.clear();
    StopVertexIds.reserve(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i].Type == MeshPointType::onVertex)
        {
            StopVertexIds.push_back(points[i].vid_eid_fid);
        }
    }
}


EndStreamPointOnSegment::EndStreamPointOnSegment(const Topology* _topology, int _connectionIndex, int _segmentIndex, DividerDynamicIteration* _divider, const MeshPoint& _point, V3 _point_direction, V3 _point_directionNormal, int _streamIndex)
    : globalStreamIndex(_divider->divider.streams[_streamIndex].dividingIteration.globalStreamIndex), iterationNum(_divider->divider.streams[_streamIndex].dividingIteration.iterationNum), connectionIndex(_connectionIndex), segmentIndex(_segmentIndex),
    topology(_topology), divider(_divider), point(_point), point_direction(_point_direction), point_directionNormal(_point_directionNormal), stream_Index(_streamIndex)
{
    isTaken = false;
    isCyclicStream = false;
    meshId = divider->meshId;
    segment = &topology->Connections[connectionIndex].segments[segmentIndex];
    segmentId = segment->id;
    atLengthOriginal = segment->GetAtLengthFromPoint(point);
    bool atLengthIsValid = atLengthOriginal >= 0 && atLengthOriginal <= segment->Length3d;
    if (!atLengthIsValid)
    {
        assert(atLengthIsValid && "position must be inside segment");
        atLengthOriginal = segment->Length3d / 2;
    }
    isSegmentReverted = topology->Connections[connectionIndex].IsSegmentReverted(segmentIndex);
    atLengthRevertedToFirstSegment = (isSegmentReverted ? segment->Length3d - atLengthOriginal : atLengthOriginal);
}

DividerIterator::DividerIterator(const Topology& _topology, ViewerDrawObjects& _draw, vector<Divider>& _dividers, vector<const MeshSolverNrosy*> _solvers, D _meshSize)
    : dividerIterationNum(0), topology(_topology), draw(_draw), meshSize(_meshSize)
{
    // convert divider into wrapper
    dividersAll.clear();
    for (int i = 0; i < _dividers.size(); i++)
    {
        dividersAll.push_back(DividerDynamicIteration(i, _dividers[i], _solvers[i]));
    }
    // at start add all dividers into refresh list - all of them needs divide and join stream, since we dont know what mesh will have stream and what will be empty
    dividersNeedsToRedivide.reserve(dividersAll.size());
    for (int i = 0; i < dividersAll.size(); i++)
    {
        dividersNeedsToRedivide.push_back(&dividersAll[i]);
    }

    // create map <meshid, divider>
    for (int i = 0; i < dividersAll.size(); i++)
    {
        map_meshid_DividerDynamicIteration[dividersAll[i].meshId] = &dividersAll[i];
    }
}





bool findMergePoints(D maxDist, const vector<vector<EndStreamPointOnSegment*>>& newSegmentPoints, int& besti0, int& besti1, D& bestDist)
{
    besti0 = -1;
    besti1 = -1;
    bestDist = 0;
    for (int i0 = 0; i0 < newSegmentPoints[0].size(); i0++) // iterate all point on segment#0
    {
        const auto& p0 = *newSegmentPoints[0][i0];
        if (p0.isTaken) continue;
        D atLength0 = p0.atLengthOriginal;
        for (int i1 = 0; i1 < newSegmentPoints[1].size(); i1++) // iterate all point on segment#1
        {
            const auto& p1 = *newSegmentPoints[1][i1];
            if (p1.isTaken) continue;
            D atLength1 = p1.atLengthRevertedToFirstSegment;
            D dist = abs(atLength0 - atLength1);
            if (dist < maxDist && (dist < bestDist || besti0 == -1))
            {
                besti0 = i0;
                besti1 = i1;
                bestDist = dist;
            }
        }
    }
    return (besti0 != -1);
}

bool DividerIterator::addAnchorAndStartPoints(D newPos, bool allowPointOnVertex, const MeshTopologyLoopCurveSegment* segment, DividerIteratorStreamInfo dividingIteration, const Mesh& mesh, const MeshSolverNrosy& solver, vector<StreamStartPoint>& startStreamPoints, vector<StreamAnchorPoint>& anchorPoints, V3* direction, V3* directionNormal)
{
    if (solver.Result.Field.size() == 0)
    {
        #if DEBUG
        cout << "! warning:  DividerIterator::addAnchorAndStartPoints()    solver.Result.Field is empty" << endl;
        #endif
        return false;
    }
    bool positionIsValid = newPos >= 0 && newPos <= segment->Length3d;
    if (!positionIsValid)
    {
        #if DEBUG
        cout << "! warning:  DividerIterator::addAnchorAndStartPoints()    position must be inside segment:   newPos=" << newPos << "   segment->Length3d=" << segment->Length3d << endl;
        #endif
        assert(positionIsValid && "position must be inside segment");
        return false;
    }

    // get mesh point from position
    MeshPoint newPoint0 = segment->GetMeshPointAtLength(newPos, allowPointOnVertex); // always return point on mesh


    // add anchorPoint    
    anchorPoints.push_back(StreamAnchorPoint(newPoint0.Type, newPoint0.vid_eid_fid, newPoint0.point, true, false, 1, dividingIteration.iterationNum));

    // add startStreamPoint
    int eid = (newPoint0.Type == MeshPointType::onVertex) ? -1 : newPoint0.vid_eid_fid;
    int fid = (newPoint0.Type == MeshPointType::onVertex) ? -1 : mesh.EF(eid, 0);//for border edge there are only 1 face 
    V3 dir(0, 0, 0); // by default direction is undefined - we will define it in 
    if (newPoint0.Type == MeshPointType::onVertex)
    {

        //noting  - we will calculate dir in Divider::Divide

        //////int vid = newPoint0.vid_eid_fid;
        //////if (mesh.V_isborder(vid))
        //////{
        //////    V3 edgesNormals(0, 0, 0);
        //////    int edgesNormalsCount = 0;
        //////    for (int i = 0; i < mesh.VE.size(vid); i++)
        //////    {
        //////        int eidi = mesh.VE(vid, i);
        //////        if (mesh.E_isborder(eidi))
        //////        {
        //////            edgesNormalsCount++;
        //////            edgesNormals += mesh.EdgeNormalToFace(eidi, mesh.EF(eidi, 0)); // border edge will have only 1 face - the first face
        //////        }
        //////    }
        //////    if (edgesNormalsCount != 0)
        //////    {
        //////        edgesNormals /= edgesNormalsCount;
        //////        edgesNormals.normalize();
        //////        dir = edgesNormals;
        //////    }
        //////    else
        //////    {
        //////        // never will happend
        //////    }
        //////}
        //////else
        //////{
        //////    assert(direction != nullptr && "For internal vertexes direction must be provided, otherwise direction will be undefined and stream extension will be random");
        //////}        
    }
    else
    {
        dir = solver.Result.Field[1].row(fid);  // constrain from solver
    }

    if (direction != nullptr) // custom direction provided - this will happend when in DividerIterator where streams travel from mesh to mesh
    {
        dir = *direction;
        if (directionNormal != nullptr)
        {
            V3 newNormal = (newPoint0.Type == MeshPointType::onVertex)
                ? mesh.V_normals.row(newPoint0.vid_eid_fid)
                : mesh.F_normals.row(fid);
            dir = utils::vector::Translate(dir, *directionNormal, newNormal); // transform normal - this is very important for sharp connection between meshes where angle can be changed by 90 degree and without tranformation future directoin will be undefined
            dir.normalize();
        }
        // detect fid
        if (newPoint0.Type == MeshPointType::onVertex)
        {
            auto faces = mesh.VertexToFacesSorted(newPoint0.vid_eid_fid);
            D minAngle = 10000;
            for (auto face : faces)
            {
                V3 faceNormal = mesh.F_normals.row(face.FaceId);
                V3 dirProjectedOnFace = utils::vector::Translate(dir, faceNormal);
                D angle_R_dir = utils::vector::AngleFull(face.RightSideDirection, dirProjectedOnFace, faceNormal);
                D angle_dir_L = utils::vector::AngleFull(dirProjectedOnFace, face.LeftSideDirection, faceNormal);
                D angle = angle_R_dir + angle_dir_L;
                if (fid == -1 || angle < minAngle)
                {
                    fid = face.FaceId;
                    minAngle = angle;
                }
            }
        }
    }


    //if (allowPointOnVertex && newPoint0.Type == MeshPointType::onVertex)
    //{
    //    cout << "allowPointOnVertex && newPoint0.Type == MeshPointType::onVertex       vid=" << newPoint0.vid_eid_fid << mesh.GetMeshIdStr()<< "   dir.norm()="<< dir.norm() << endl;
    //    //draw.AddEdge(newPoint0.point, newPoint0.point + dir.normalized(), Color3d(1, 0, 0), "vid="+to_string(newPoint0.vid_eid_fid)+ ", " + "fid=" + to_string(fid));
    //}
    StreamStartPoint p(newPoint0.Type, newPoint0.vid_eid_fid, newPoint0.point);
    p.Index = startStreamPoints.size();
    p.StreamAnchorPointId = anchorPoints.back().id;
    p.dir = dir;
    p.dir_isCustomDefined = (direction != nullptr);
    p.maxAngleChangeForDir = 20;
    p.fid = fid;
    p.dir_ri = 0;
    p.dividesBy = 1;
    p.isBorderPoint = true;
    p.isSingularPoint = false;
    p.canBeJoined = false;
    p.joinedAtIndex = 0;
    p.dividingIteration = dividingIteration;
    startStreamPoints.push_back(p);
    return true;
}

void DividerIterator::Divide()
{
    extern bool IsOmpEnabled;
    Timer timer_Divide;
    Timer timer_DefiningJoins;
    Timer timer_Join;


    //
    // Iterate streams and get 'vector<EndStreamPointOnSegment>' for each iteration
    //
    vector<vector<EndStreamPointOnSegment>> newSegmentPoints;
    newSegmentPoints.resize(options.max_iterations_count);
    vector<vector<int>> streams_visited_meshids;  // visited meshes ids for 'stream->dividingIteration.globalStreamIndex' - size is same as size of newSegmentPoints[0] - since every point at iteration 0 represent global stream
    while (ProceedCurrentIteration_and_start_NextIteration(newSegmentPoints[dividerIterationNum], streams_visited_meshids)) // find merge points between 2 meshes, or place end of stream from 1 mesh to its friend as start point of stream (the max iteration is alike 1000)
    {
        if (options.DebugEnabled) cout << ".";

        // divide
        if (meshLogicOptions.Divider.Enabled)
        {
            timer_Divide.start();
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < dividersNeedsToRedivide.size(); i++)
            {
                DividerDynamicIteration* d = dividersNeedsToRedivide[i];
                if (meshLogicOptions.DividerIterator.joinStreamsAfterEachIteration && meshLogicOptions.Joiner.Enabled)
                {
                    d->divider.Divide(false); // we must redivide all streams since its possible to join old streams with new one
                }
                else
                {
                    d->divider.Divide(false, dividerIterationNum - 1); // create new streams only for just added points - we can ignore old streams, since we dont plan to join streams
                }
            }
            timer_Divide.stop();
        }

        // join streams
        if (meshLogicOptions.DividerIterator.joinStreamsAfterEachIteration && meshLogicOptions.Joiner.Enabled)
        {
            timer_Join.start();
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < dividersNeedsToRedivide.size(); i++)
            {
                DividerDynamicIteration* d = dividersNeedsToRedivide[i];
                MeshStreamsJoiner joiner(d->divider, meshSize);
                joiner.JoinStreams();
            }
            timer_Join.stop();
        }
    }
    newSegmentPoints.resize(dividerIterationNum); // truncate vector

    // DEBUG - show statistic
    if (options.DebugEnabled)
    {
        int newPontsCount = 0;
        for (auto& ps : newSegmentPoints) newPontsCount += ps.size();
        cout << endl << "Created " << newPontsCount << " points in " << dividerIterationNum << " iterations" << endl;
    }

    //
    // Join streams
    //
    if (!meshLogicOptions.DividerIterator.joinStreamsAfterEachIteration && meshLogicOptions.Joiner.Enabled)
    {
        timer_DefiningJoins.start();
        DefineJoinPoints(newSegmentPoints, streams_visited_meshids);
        timer_DefiningJoins.stop();
        // join streams
       /* if (meshLogicOptions.Joiner.Enabled)
        {
            timer_Join.start();
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < dividersAll.size(); i++)
            {
                DividerDynamicIteration* d = &dividersAll[i];
                MeshStreamsJoiner joiner(d->divider, meshSize);
                joiner.JoinStreams();
            }
            timer_Join.stop();
        }*/
    }

    //
    // DEBUG - show iterations on streams
    //
    if (options.DebugEnabled && options.debug_show_iteration_index_on_streams)
    {
        int totalPoints = 0;
        for (auto& ps : newSegmentPoints) totalPoints += ps.size();
        draw.ReserveLabels(totalPoints);
        for (auto& ps : newSegmentPoints)
        {
            for (auto& p : ps)
            {
                p.stream()->Init_lengthToNextPoint();
                MeshStream& stream = *p.stream();
                draw.AddLabel(stream.GetPointAtLength(stream.Length3d*0.3), "S#" + to_string(p.globalStreamIndex) + ", i#" + to_string(p.iterationNum));
            }
        }
    }

    cout << "time  (dividing:" << timer_Divide << ",  joining:" << timer_Join << ")" << endl;
}

void DividerIterator::GetEndStreamPoints(vector<EndStreamPoint>& points, int selectOnlyPointsWithIterationNum)
{
    extern bool IsOmpEnabled;

    // reserve size
    points.clear();
    int newPointsReserveSize = 0;
    for (int i = 0; i < dividersNeedsToRedivide.size(); i++)
    {
        DividerDynamicIteration& d = *dividersNeedsToRedivide[i];
        for (auto& s : d.divider.streams.streams)
        {
            if (selectOnlyPointsWithIterationNum == -1 || s.dividingIteration.iterationNum == selectOnlyPointsWithIterationNum)
            {
                newPointsReserveSize++;
            }
        }
    }
    points.reserve(newPointsReserveSize);

    // get all new points created by streams for current iteration
    for (int i = 0; i < dividersNeedsToRedivide.size(); i++)
    {
        DividerDynamicIteration& d = *dividersNeedsToRedivide[i];
        for (auto& s : d.divider.streams.streams)
        {
            if (selectOnlyPointsWithIterationNum != -1 && s.dividingIteration.iterationNum != selectOnlyPointsWithIterationNum) continue; // skip stream from previous iterations - since they already taken into account
            if (s.Points.size() == 0) continue; // skip empty streams
            if (s.IsDeleted) continue; // skip deleted streams
            if (s.IsMerged) continue; // skip merged streams - they already closed, and cannot be continued on friend mesh

            const StreamPoint& endStreamPoint = s.Points.back();

            points.push_back(EndStreamPoint(&d, endStreamPoint, &s)); // create new point
        }
    }

    // detect connectionIndex and segmentIndex for new points
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < points.size(); i++)
    {
        EndStreamPoint& endStreamPoint = points[i];
        int meshid = endStreamPoint.stream->mesh.id;

        // find connection index from stream
        int connectionIndex = -1;
        switch (endStreamPoint.point.Type)
        {
            case MeshPointType::onVertex:
            {
                const int vid = endStreamPoint.point.vid_eid_fid;
                if (endStreamPoint.stream->mesh.V_isborder[vid])
                {
                    auto find_meshid = topology.map_meshid_vid_connectionIndex.find(meshid); // get TopologyConnection from point on edge using cached map in Topoloty class for current file
                    if (find_meshid != topology.map_meshid_vid_connectionIndex.end())
                    {
                        auto find_vid = find_meshid->second.find(vid);
                        if (find_vid != find_meshid->second.end())
                        {
                            connectionIndex = find_vid->second;
                        }
                        // if point is on start or on end of the segment - ignore it
                        for (auto& sss : topology.Connections[connectionIndex].segments)
                        {
                            if (sss.meshid == meshid && (sss.VertexId(0) == vid || sss.VertexId(sss.PointsCount - 1) == vid))
                            {
                                connectionIndex = -1; // ignore point
                                break;
                            }
                        }
                    }
                }
            }
            break;
            case MeshPointType::onEdge:
            {
                const int eid = endStreamPoint.point.vid_eid_fid;
                if (endStreamPoint.stream->mesh.E_isborder[eid])
                {
                    auto find_meshid = topology.map_meshid_eid_connectionIndex.find(meshid); // get TopologyConnection from point on edge using cached map in Topoloty class for current file
                    if (find_meshid != topology.map_meshid_eid_connectionIndex.end())
                    {
                        auto find_eid = find_meshid->second.find(eid);
                        if (find_eid != find_meshid->second.end())
                        {
                            connectionIndex = find_eid->second;
                        }
                    }
                }
            }
            break;
            default:
                cout << "!!! error in method DividerIterator::GetEndStreamPoints() - this point type is not supported " << endl;
                continue;
        }
        if (connectionIndex == -1) continue;

        // DEBUG - trace in console found point
        if (options.DebugEnabled && options.debug_trace_iterations_in_console)
        {
            vector<int> segmentIds;
            for (auto& ss : topology.Connections[connectionIndex].segments) segmentIds.push_back(ss.id);
            cout << "found new start stream point for meshid=" << meshid << "  at " << endStreamPoint.point.vid_eid_fid_toString() << "   for segments " << utils::stdvector::toString(segmentIds) << endl;
        }

        // find segment index from stream
        int segmentIndex = -1;
        auto& segments = topology.Connections[connectionIndex].segments;
        if (segments.size() > 2)
        {
            cout << "! wrong  DividerIterator::GetEndStreamPoints():  manifold connections not supported " << endl;
            assert(segments.size() <= 2 && "manifold connections not supported");
            continue;
        }
        for (int si = 0; si < segments.size(); si++)
        {
            if (segments[si].curve.mesh.id == meshid)
            {
                segmentIndex = si;
            }
        }
        if (segmentIndex == -1)
        {
            cout << "! wrong  DividerIterator::GetEndStreamPoints():  failed to find appropiate segment from endStreamPoint " << endl;
            assert(segmentIndex != -1 && "failed to find appropiate segment from endStreamPoint");
            continue;
        }

        // update 'endStreamPoint'
        endStreamPoint.connectionIndex = connectionIndex;
        endStreamPoint.segmentIndex = segmentIndex;
    }
}

bool addAnchorAndStartPoints2(D newPos, bool allowPointOnVertex, const MeshTopologyLoopCurveSegment* segment, DividerDynamicIteration* divider, DividerIteratorStreamInfo dividingIteration, V3* direction = nullptr, V3* directionNormal = nullptr)
{
    if (DividerIterator::addAnchorAndStartPoints(newPos, allowPointOnVertex, segment, dividingIteration, divider->divider.mesh, *divider->solver, divider->divider.startPoints, divider->divider.anchorPoints, direction, directionNormal))
    {
        divider->isNeedToRedivide = true;
        return true;
    }
    return false;
}


bool DividerIterator::ProceedCurrentIteration_and_start_NextIteration(vector<EndStreamPointOnSegment>& newPoints, vector<vector<int>>& streams_visited_meshids)
{
    extern bool IsOmpEnabled;
    //
    //
    // Proceed information from current iteration
    //
    //


    // DEBUG - show iteration number
    if (options.DebugEnabled && options.debug_trace_iterations_in_console)
    {
        cout << endl << endl << "iteration#" << dividerIterationNum << "    needs to update " << dividersNeedsToRedivide.size() << " dividers from " << dividersAll.size() << endl;
    }

    //
    // terminate iterations when we debuging current iteration
    //
    if (options.DebugEnabled && options.debug_iteration_index == dividerIterationNum)
    {
        // TODO: show additional debug info for current debuged iteration
        return false;
    }

    //
    // terminate iterations when we reach max iteration allowed
    //
    if (dividerIterationNum >= options.max_iterations_count)
    {
        // warn if iterator finished only because iterations out of max allowed - probably cyclic stream
        if (dividerIterationNum > 3)
        {
            cout << "!!!   dividingIteration >= meshLogicOptions.DividerIterator.max_iterations_count - probably cyclic stream" << endl;
        }
        return false;
    }


    //
    // create list of new points from streams (from streams that was created from current iteration or all iterations)
    //
    vector<EndStreamPoint> newEndStreamPoints;
    GetEndStreamPoints(newEndStreamPoints, dividerIterationNum);
    newPoints.clear();
    newPoints.reserve(newEndStreamPoints.size());
    for (auto& newPoint : newEndStreamPoints)
    {
        if (newPoint.connectionIndex == -1 || newPoint.segmentIndex == -1) continue;
        if (dividerIterationNum == 0)
        {
            newPoint.stream->dividingIteration.globalStreamIndex = newPoints.size(); // set globalStreamIndex to stream
        }
        V3 newPoint_direction(0, 0, 0);
        V3 newPoint_directionNormal(0, 0, 0);
        if (newPoint.stream->Points.size() >= 2)
        {
            P3 p0 = newPoint.stream->Points[newPoint.stream->Points.size() - 2].point;
            P3 p1 = newPoint.stream->Points[newPoint.stream->Points.size() - 1].point;
            newPoint_direction = p1 - p0;
            newPoint_direction.normalize();
            if (newPoint.point.Type == MeshPointType::onVertex)
            {
                newPoint_directionNormal = newPoint.stream->mesh.V_normals.row(newPoint.point.vid_eid_fid);
            }
            if (newPoint.point.Type == MeshPointType::onEdge)
            {
                int fid = newPoint.stream->mesh.EF(newPoint.point.vid_eid_fid, 0);
                newPoint_directionNormal = newPoint.stream->mesh.F_normals.row(fid);
            }
        }
        newPoints.push_back(EndStreamPointOnSegment(&topology, newPoint.connectionIndex, newPoint.segmentIndex, newPoint.divider, newPoint.point, newPoint_direction, newPoint_directionNormal, newPoint.stream->Index));
    }
    // for cyclic streams we have to define globalStreamIndex independetly since they will not have new points
    int globalStreamIndex = newPoints.size();
    for (auto& newPoint : newEndStreamPoints)
    {
        if (dividerIterationNum == 0)
        {
            if (newPoint.stream->dividingIteration.globalStreamIndex == -1)
            {
                newPoint.stream->dividingIteration.globalStreamIndex = globalStreamIndex;
                globalStreamIndex++;
            }
        }
    }
    // create 'streams_visited_meshids' if it is first iteration
    if (dividerIterationNum == 0)
    {
        streams_visited_meshids.resize(newPoints.size());
        for (int i = 0; i < newPoints.size(); i++)
        {
            auto& newPoint = newPoints[i];
            streams_visited_meshids[i].push_back(newPoint.meshId); // register first mesh in the list of visited meshes by the stream
        }
    }



    //
    //
    // Start new iteration
    //
    //
    dividerIterationNum++;

    //
    // clear 'isNeedToRedivide' flag
    //
    for (int i = 0; i < dividersNeedsToRedivide.size(); i++)
    {
        DividerDynamicIteration& d = *dividersNeedsToRedivide[i];
        d.isNeedToRedivide = false;
    }


    //
    // Merge points on connections
    //    
    if (options.joinStreamsAfterEachIteration && meshLogicOptions.Joiner.Enabled)
    {
        // gather new points for each connection
        map<int, vector<vector<EndStreamPointOnSegment*>>> map_connectionIndex_newPoints; //map_connectionIndex_newPoints[connectionIndex][segmentIndex]
        for (auto& newPoint : newPoints)
        {
            const auto& segments = topology.Connections[newPoint.connectionIndex].segments;

            // create connection wrapper if it is not created yet
            if (map_connectionIndex_newPoints.find(newPoint.connectionIndex) == map_connectionIndex_newPoints.end())
            {
                vector<vector<EndStreamPointOnSegment*>> emptyNewPoints;
                emptyNewPoints.resize(segments.size());
                map_connectionIndex_newPoints[newPoint.connectionIndex] = emptyNewPoints;
            }
            // add point to apropriate 'connectionIndex' and 'segment'
            map_connectionIndex_newPoints[newPoint.connectionIndex][newPoint.segmentIndex].push_back(&newPoint);

            // debug - check if segments sizes are same
            //if (segments[0].PointsCount != segments[1].PointsCount)
            //{
            //    cout << "!!! segments[0].PointsCount != segments[1].PointsCount:  [seg=" << segments[0].id << ",PointsCount=" << segments[0].PointsCount << "]    [seg=" << segments[1].id << ",PointsCount=" << segments[1].PointsCount << "]" << endl;
            //}
        }

        // merge points in connections
        for (auto& o : map_connectionIndex_newPoints) //  for each connection
        {
            int connectionIndex = o.first;
            vector<vector<EndStreamPointOnSegment*>>& newSegmentPoints = o.second;
            auto& segments = topology.Connections[connectionIndex].segments;

            // debug - show newPoints for current connection
            if (options.DebugEnabled && options.debug_trace_iterations_in_console)
            {
                cout << "   connectionId=" << topology.Connections[connectionIndex].id << "  newpoints: ";
                for (int si = 0; si < newSegmentPoints.size(); si++)
                {
                    for (int i = 0; i < newSegmentPoints[si].size(); i++)
                    {
                        const EndStreamPointOnSegment& p = *newSegmentPoints[si][i];
                        cout << "{seg " << p.segmentId << ", atLength " << p.atLengthRevertedToFirstSegment << "}  ";
                    }
                }
                cout << endl;
            }

            if (newSegmentPoints.size() == 1) continue; // we cant merge points on border-segments or add point on friend mesh since there is no friend mesh on borders

                                                        // find what points are possible to merge in current iteration
            int besti0, besti1;
            D bestDist;
            int mergeId = 0;
            while (findMergePoints(meshSize, newSegmentPoints, besti0, besti1, bestDist))
            {
                auto& p0 = *newSegmentPoints[0][besti0];
                auto& p1 = *newSegmentPoints[1][besti1];
                p0.isTaken = true;
                p1.isTaken = true;

                // merge points
                p0.stream()->Init_lengthToNextPoint();
                p1.stream()->Init_lengthToNextPoint();
                D stream0Length = p0.stream()->GetTotalLength();
                D stream1Length = p1.stream()->GetTotalLength();
                D shift0Length = bestDist * stream0Length / (stream0Length + stream1Length);
                D shift1Length = bestDist * stream1Length / (stream0Length + stream1Length);
                D shift0Sign = p0.atLengthRevertedToFirstSegment < p1.atLengthRevertedToFirstSegment ? 1 : -1;
                if (p0.isSegmentReverted) shift0Sign = -shift0Sign;
                D shift1Sign = p1.atLengthRevertedToFirstSegment < p0.atLengthRevertedToFirstSegment ? 1 : -1;
                if (p1.isSegmentReverted) shift1Sign = -shift1Sign;
                D newPos0 = p0.atLengthOriginal + shift0Length * shift0Sign;
                D newPos1 = p1.atLengthOriginal + shift1Length * shift1Sign;

                // debug - trace merged points
                if (options.DebugEnabled && options.debug_trace_iterations_in_console)
                {
                    cout << "      merging points ";
                    cout << "{seg " << p0.segmentId << ", atLength " << p0.atLengthRevertedToFirstSegment << "}  ";
                    cout << "{seg " << p1.segmentId << ", atLength " << p1.atLengthRevertedToFirstSegment << "}  ";
                    cout << endl;
                    mergeId++;
                    draw.AddPoint((p0.point.point + p1.point.point) / 2, Color3d(1, 0, 0), "merge#" + to_string(mergeId));
                    //draw.AddPoint(p0.segment->GetPointAtLength(newPos0), Color3d(0, 0, 1), "newPos0 for segment#" + to_string(p0.segment->id));
                    //draw.AddPoint(p1.segment->GetPointAtLength(newPos1), Color3d(0, 0, 1), "newPos1 for segment#" + to_string(p1.segment->id));
                }


                // add with same dividingIteration as stream to be able join original startStream and new startStream (from same mesh)
                if (addAnchorAndStartPoints2(newPos0, true, p0.segment, p0.divider, p0.stream()->dividingIteration))
                {
                    auto& new_sp = p0.divider->divider.startPoints.back();// get reference to currently added startStreamPoint
                    auto& new_ap = p0.divider->divider.anchorPoints.back(); // get reference to currently added anchorPoint
                    StreamStartPoint& s0_sp = p0.divider->divider.startPoints[p0.stream()->Index];
                    StreamAnchorPoint* s0_ap = p0.divider->divider.FindAnchorPoint(p0.stream()->StreamAnchorPointId);
                    p0.divider->divider.SignContract(new_sp, new_ap, s0_sp, *s0_ap);
                    //if (options.DebugEnabled && options.debug_trace_iterations_in_console)
                    //{
                    //    draw.AddPoint(new_sp.point, Color3d(0,1,0), "startPoint0");
                    //}
                }
                if (addAnchorAndStartPoints2(newPos1, true, p1.segment, p1.divider, p1.stream()->dividingIteration))
                {
                    auto& new_sp = p1.divider->divider.startPoints.back();// get reference to currently added startStreamPoint
                    auto& new_ap = p1.divider->divider.anchorPoints.back(); // get reference to currently added anchorPoint
                    StreamStartPoint& s1_sp = p1.divider->divider.startPoints[p1.stream()->Index];
                    StreamAnchorPoint* s1_ap = p1.divider->divider.FindAnchorPoint(p1.stream()->StreamAnchorPointId);
                    p1.divider->divider.SignContract(new_sp, new_ap, s1_sp, *s1_ap);
                    //if (options.DebugEnabled && options.debug_trace_iterations_in_console)
                    //{
                    //    draw.AddPoint(new_sp.point, Color3d(0, 1, 0), "startPoint1");
                    //}
                }
            }
        }
    }


    //
    // gather new points for each mesh - to be able to run in multithreading
    //
    map<int, vector<EndStreamPointOnSegment*>> map_meshid_newPoints;
    for (auto& newPoint : newPoints)
    {
        if (newPoint.isTaken) continue; // if point was merged - skip it
        newPoint.isTaken = true;

        auto& segments = topology.Connections[newPoint.connectionIndex].segments;
        if (segments.size() == 1) continue; // we can't add point on friend mesh since there is no friend mesh on borders

        int si = newPoint.segmentIndex;
        int siOpposite = si == 0 ? 1 : 0;

        auto* segment = &segments[si];
        const auto* segmentOpposite = &segments[siOpposite];

        int meshidOpposite = segmentOpposite->meshid;
        // create wrapper if it is not created yet
        if (map_meshid_newPoints.find(meshidOpposite) == map_meshid_newPoints.end())
        {
            map_meshid_newPoints[meshidOpposite] = vector<EndStreamPointOnSegment*>();
            map_meshid_newPoints[meshidOpposite].reserve(100);
        }
        map_meshid_newPoints[meshidOpposite].push_back(&newPoint); // register that new point will create new friend point for this mesh
    }
    vector<int>newPointsMeshids;
    newPointsMeshids.reserve(dividersAll.size());
    for (auto iter = map_meshid_newPoints.begin(); iter != map_meshid_newPoints.end(); ++iter)
    {
        int meshid = iter->first;
        newPointsMeshids.push_back(meshid);
    }

    //
    // form new startStreamPoint's
    //
    #pragma omp parallel for  if(IsOmpEnabled)
    for (auto i = 0; i < newPointsMeshids.size(); i++)
    {
        int meshid = newPointsMeshids[i];
        for (auto newPoint : map_meshid_newPoints[meshid])
        {
            auto& segments = topology.Connections[newPoint->connectionIndex].segments;

            int si = newPoint->segmentIndex;
            int siOpposite = si == 0 ? 1 : 0;

            auto* segment = &segments[si];
            const auto* segmentOpposite = &segments[siOpposite];

            assert(segmentOpposite->meshid == meshid && "all segmentsOpposite must belong to current mesh for multithreading porpouse");

            // stop cyclic streams
            assert(newPoint->globalStreamIndex < streams_visited_meshids.size());
            auto& visitedMeshesIds = streams_visited_meshids[newPoint->stream()->dividingIteration.globalStreamIndex];
            if (utils::stdvector::exists(visitedMeshesIds, segmentOpposite->meshid))
            {
                newPoint->isCyclicStream = true;
                continue; // prevent from entering mesh by stream second time 
            }
            visitedMeshesIds.push_back(segmentOpposite->meshid); // block opposite meshid for second entrance


            bool IsSegmentReverted = topology.Connections[newPoint->connectionIndex].IsSegmentReverted(si);
            bool IsSegmentRevertedOpposite = topology.Connections[newPoint->connectionIndex].IsSegmentReverted(siOpposite);
            DividerDynamicIteration* dividerOpposite = map_meshid_DividerDynamicIteration[segmentOpposite->curve.mesh.id];
            //D newPos = segmentOpposite->GetAtLengthFromPoint(p.point);
            D newPos = newPoint->atLengthOriginal;
            if (IsSegmentReverted != IsSegmentRevertedOpposite) newPos = segmentOpposite->Length3d - newPos;
            //cout << p.segmentId << "  " << segment->id << endl;
            //if (dividingIteration.iterationNum == 4 && p.meshId == 128)
            //{
            //    draw.AddPoint(p.point.point, Color3d(1, 0, 0), p.point.vid_eid_fid_ToStr());
            //}

            //newSegmentPoint sOpposite(&topology, connectionIndex, segmentOppositeIndex, dividerOpposite, p.point, p.stream);
            //D newPos = IsSegmentRevertedOpposite ? segmentOpposite->Length3d - atLength : atLength;
            auto next_dividingIteration = newPoint->stream()->dividingIteration;
            next_dividingIteration.iterationNum = dividerIterationNum;  // update iteration num - for new startPoint
            //if (next_dividingIteration.globalStreamIndex == 0 && next_dividingIteration.iterationNum == 3)
            //{
            //    draw.AddEdgeBold(newPoint->point.point, newPoint->point_direction, 5, Color3d(1, 0, 0), "newPoint");
            //}

            bool isNewPointVertex = (newPoint->point.Type == MeshPointType::onVertex);
            bool continueDividing = true;
            if (isNewPointVertex)
            {
                if (utils::stdvector::exists(newPoint->divider->StopVertexIds, newPoint->point.vid_eid_fid))
                {
                    continueDividing = false;
                }
            }
            if (continueDividing)
            {
                addAnchorAndStartPoints2(newPos, isNewPointVertex, segmentOpposite, dividerOpposite, next_dividingIteration, &newPoint->point_direction, &newPoint->point_directionNormal);
            }
            //D dist = utils::point::DistToPoint(dividerOpposite->additional_anchorPoints.back().point, newPoint->point.point);
            //draw.AddPoint(segmentOpposite->GetPointAtLength(newPos), Color3d(1, 0, 0), to_string(segment->id) + "  dist=" + to_string(dist));
            //draw.AddPoint(segmentOpposite->GetPointAtLength(newPos), Color3d(1, 0, 0), to_string(segment->id) + "  dist=" + to_string(dist));
            //draw.AddPoint(segment->GetPointAtLength(newPoint->atLengthOriginal), Color3d(0, 1, 0), "newPoint->atLengthOriginal");
        }
    }

    //
    // populate 'dividersNeedsToRedivide'
    //
    dividersNeedsToRedivide.clear();
    dividersNeedsToRedivide.reserve(dividersAll.size());
    for (int i = 0; i < dividersAll.size(); i++)
    {
        DividerDynamicIteration* d = &dividersAll[i];
        if (d->isNeedToRedivide)
        {
            dividersNeedsToRedivide.push_back(d);
        }
    }

    //
    // return true only if we have more meshes to divide
    //
    return dividersNeedsToRedivide.size() > 0;
}

void DividerIterator::DefineJoinPoints(vector<vector<EndStreamPointOnSegment>>& newSegmentPoints, vector<vector<int>>& streams_visited_meshids)
{
    extern bool IsOmpEnabled;

    //
    // Group newpoints for connections 
    //
    vector<vector<pair<D, EndStreamPointOnSegment*>>> points_on_connnection;
    int maxConnectionIndex = 0;
    for (auto& pp : newSegmentPoints) for (auto& p : pp)
    {
        if (maxConnectionIndex < p.connectionIndex) maxConnectionIndex = p.connectionIndex;
    }
    points_on_connnection.resize(maxConnectionIndex + 1);
    for (auto& pp : newSegmentPoints) for (auto& p : pp)
    {
        points_on_connnection[p.connectionIndex].push_back({ p.atLengthRevertedToFirstSegment, &p });
    }

    //
    // Sort points on connection
    //
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int connectionIndex = 0; connectionIndex < points_on_connnection.size(); connectionIndex++)
    {
        sort(points_on_connnection[connectionIndex].begin(), points_on_connnection[connectionIndex].end());
    }

    // DEBUG trace points on connections
    if (options.DebugEnabled)
    {
        for (int connectionIndex = 0; connectionIndex < points_on_connnection.size(); connectionIndex++)
        {
            cout << "connectionIndex=" << connectionIndex << endl;
            for (auto& p : points_on_connnection[connectionIndex])
            {
                cout << "   atLength=" << p.second->atLengthRevertedToFirstSegment << "   stream.Index=" << p.second->stream()->Index << "   stream.GlobalIndex=" << p.second->globalStreamIndex << endl;
            }
        }
    }
}

