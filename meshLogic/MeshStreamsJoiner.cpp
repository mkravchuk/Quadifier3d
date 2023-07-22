#include "stdafx.h"
#include "MeshStreamsJoiner.h"
#include "Divider.h"
#include "MeshStreams.h"
#include "MeshStreamsAdjuster.h"

MeshLogicOptions_MeshStreamsJoiner& options = meshLogicOptions.Joiner;

MeshStreamsJoiner::MeshStreamsJoiner(Divider& divider, D _meshSize)
    : mesh(divider.mesh), solver(divider.solver), draw(divider.draw),
    startStreamPoints(divider.startPoints), anchorPoints(divider.anchorPoints), streams(divider.streams), meshSize(_meshSize),
    StreamsAdjuster_callsCount_directMethod(0), StreamsAdjuster_callsCount_iterativeMethod(0)
{
}

void MeshStreamsJoiner::JoinStreams()
{
    if (!options.Enabled) return;

    if (options.debug_TestPerformanceCall1000Times)
    {
        bool saveDebugEnabled = options.DebugEnabled;
        options.DebugEnabled = false;
        JoinStreams1000times();
        options.DebugEnabled = saveDebugEnabled;
    }
    else
    {
        JoinStreams1time();
    }
}
void MeshStreamsJoiner::JoinStreams1000times()
{
    vector<StreamStartPoint> saved_startStreamPoints = startStreamPoints;
    vector<StreamAnchorPoint> saved_anchorPoints = anchorPoints;
    vector<vector<StreamPoint>> saved_Points;
    for (const MeshStream& s : streams.streams)
    {
        saved_Points.push_back(s.Points);
    }
    for (int i = 0; i < 1000; i++)
    {
        startStreamPoints = saved_startStreamPoints;
        anchorPoints = saved_anchorPoints;
        for (int k = 0; k < saved_Points.size(); k++)
        {
            streams.streams[k].SetNewPoints(saved_Points[k]);
        }
        JoinStreams1time();
    }
}
void MeshStreamsJoiner::JoinStreams1time()
{
    if (!options.Enabled) return;
    if (startStreamPoints.size() != streams.streams.size())
    {
        cout << "!!!wrong   MeshStreamsJoiner::JoinStreams  -  startStreamPoints.size() != streams.streams.size()" << endl;
        return;
    }
    if (startStreamPoints.size() == 0) return;
    if (options.DebugEnabled) cout << "Joining streams at mesh#" << mesh.id << endl;


    //
    // Find possible connections of stream points to anchor points
    //
    vector<StartPointToAnchorPossibleConnection> possibleConnections;
    possibleConnections.reserve(startStreamPoints.size()*anchorPoints.size());
    for (auto& sp : startStreamPoints)
    {
        //if (options.DebugEnabled && options.debug_debugStreamId != -1 && options.debug_debugStreamId != sp.Index) continue;
        streams[sp.Index].Init_lengthToNextPoint();
        if (FindPossibleConnections(streams, sp, anchorPoints, possibleConnections))
        {
            //if (options.DebugEnabled && options.debug_debugStreamId != -1)
            //{
            //    sp.canBeJoined = true;
            //    sp.joinedAtIndex = possibleConnections.back().closestPoint.index;
            //}
        }
    }

    //if (options.DebugEnabled && options.debug_debugStreamId != -1)
    //{
    //    return;
    //} 

    //
    // Find stream connections - streams that connects each other
    //
    vector<StreamPointsConnection> connections;
    FindConnections(startStreamPoints, possibleConnections, connections);

    //
    // Calculate connection point to which two streams should strive
    //
    for (auto& c : connections)
    {
        SetConnectionPoint(streams, c);
    }

    //
    // Adjust streams to join at same point
    //
    for (auto& c : connections)
    {
        //if (c.Index == 2 && mesh.id==6)
        //{
        //    int temp = 0;
        //}
        AdjustConnectedStreams(streams, c);
    }

    if (meshLogicOptions.StreamAdjuster.DebugEnabled)
    {
        return;
    }

    //
    // Merge two streams into one
    //
    for (auto& c : connections)
    {
        MergeConnectedStreams(streams, c);
    }
}

D MeshStreamsJoiner::AlternativePath::GetMinSortWeight()
{
    D min = 0;
    assert(alternativeConnection.size() != 0);
    if (alternativeConnection.size() != 0)
    {
        min = alternativeConnection.front()->sortWeight;
        for (auto& a : alternativeConnection)
        {
            if (a->sortWeight < min) min = a->sortWeight;
        }
    }
    return min;
}

D MeshStreamsJoiner::AlternativePath::GetMaxSortWeight()
{
    D max = 0;
    assert(alternativeConnection.size() != 0);
    if (alternativeConnection.size() != 0)
    {
        max = alternativeConnection.front()->sortWeight;
        for (auto& a : alternativeConnection)
        {
            if (a->sortWeight > max) max = a->sortWeight;
        }
    }
    return max;
}

bool MeshStreamsJoiner::AlternativePath::ExpandAlternativeConnection(vector<AlternativePath*> connection_to_alternativePath)
{
    for (int i = 0; i < alternativeConnection.size(); i++)
    {
        StreamPointsConnection* c = alternativeConnection[i];
        AlternativePath* sub_alternativePath = connection_to_alternativePath[c->Index];
        if (sub_alternativePath != nullptr)
        {
            utils::stdvector::remove_at(alternativeConnection, i);
            alternativeConnection.insert(alternativeConnection.begin() + i, sub_alternativePath->alternativeConnection.begin(), sub_alternativePath->alternativeConnection.end());
            return true;
        }
    }
    return false;
}

bool MeshStreamsJoiner::AlternativePath::Exists(int connectionIndex, bool serachInNonExpandable)
{
    for (auto& c : (serachInNonExpandable ? alternativeConnection_nonExpanded : alternativeConnection))
    {
        if (c->Index == connectionIndex)
        {
            return true;
        }
    }
    return false;
}


bool MeshStreamsJoiner::FindAlternativePath(D maxDist, const StreamPointsConnection& c, map<int, vector<StreamPointsConnection*>>& map_anchorid_connection,
    bool debug, int debugStreamId,
    vector<StreamPointsConnection*>& alternativePath, vector<int>& aids, D& maxDistFromAidsToStream)
{
    D const COS_75 = utils::angle::DegreesToCos(75);

    //if (c.aid_min_max == pair<int, int>(17, 18) && c.connection_pointsIndex1 > 100)
    //{
    //    cout << "FindConnections_OptimizeConnections_FindAlternativePath  c.aid_min_max={" << c.aid_min_max.first << "," << c.aid_min_max.second << "}" << endl;
    //    debug = true;
    //}
    //if (c.aid_min_max == pair<int, int>(5, 5))
    //{
    //    cout << "FindConnections_OptimizeConnections_FindAlternativePath  c.aid_min_max={" << c.aid_min_max.first << "," << c.aid_min_max.second << "}" << endl;
    //    debug = true;
    //}
    alternativePath.clear();
    aids.clear();
    maxDistFromAidsToStream = 0;
    if (debugStreamId == -1) debugStreamId = c.pc1.sp.Index; // in non debug mode choise first stream
    const StartPointToAnchorPossibleConnection& pc1 = c.pc1.sp.Index == debugStreamId ? c.pc1 : c.pc2;
    const StartPointToAnchorPossibleConnection& pc2 = c.pc1.sp.Index == debugStreamId ? c.pc2 : c.pc1;

    // merge streams to avoid wrong distances
    MeshStream mergedStream = pc1.stream;
    {
        //TODO: optimize performance of simple-merging 2 streams 
        int connection_pointsIndex1 = c.connection_pointsIndex1;
        int connection_pointsIndex2 = c.connection_pointsIndex2;
        if (pc1.Index != c.pc1.sp.Index) swap(connection_pointsIndex1, connection_pointsIndex2);
        //utils::stdvector::remove_at(mergedStream.Points, connection_pointsIndex1+1, )
        mergedStream.Points.resize(connection_pointsIndex1 + 1, pc1.stream[0]);
        vector<StreamPoint> appendPoints = pc2.stream.Points;
        appendPoints.resize(connection_pointsIndex2 + 1, pc2.stream[0]);
        std::reverse(appendPoints.begin(), appendPoints.end());
        utils::stdvector::append(mergedStream.Points, appendPoints);
        mergedStream.Set_IsInited_lengthToNextPoint(false);
        mergedStream.Init_lengthToNextPoint();
    }

    if (debug)
    {
        for (int i = 0; i < mergedStream.size(); i++)
        {
            draw.AddPoint(mergedStream[i].point, Color3d(1, 0, 0), to_string(i));
        }
    }
    int start_aid = pc1.sp.StreamAnchorPointId;
    int end_aid = pc2.sp.StreamAnchorPointId;
    int prev_aid = end_aid;
    int prev_ci_index = c.Index;
    int current_aid = -1;
    int currentPointIndex = 0;
    D currentMaxDist = 0;
    //int maxPointIndex = pc1.closestPoint.index + 1;
    int maxPointIndex = mergedStream.size() + 1;
    if (debug)
    {
        cout << endl << "findAlternativePath  c.Index=" << c.Index << "   start_aid=" << start_aid << "   end_aid=" << end_aid << "   max poinIndex=" << maxPointIndex << endl;
        draw.AddPoint(pc1.sp.point, Color3d(1, 0, 0));
        draw.AddLabel(pc1.sp.point, "  aid=" + to_string(pc1.sp.StreamAnchorPointId), Color3d(1, 0, 0), 2);
        draw.AddPoint(pc2.sp.point, Color3d(1, 0, 0));
        draw.AddLabel(pc2.sp.point, "  aid=" + to_string(pc2.sp.StreamAnchorPointId), Color3d(1, 0, 0), 2);
    }
    aids.push_back(start_aid);


    while (current_aid != end_aid)
    {
        // find best next connection
        StreamPointsConnection* best_ci = nullptr;
        D best_nextConnection_dist = 0;
        int best_nextConnection_pointIndex = 0;
        D best_nextConnection_progressLength = 0;
        const StartPointToAnchorPossibleConnection* best_nextConnection = nullptr;
        D best_nextConnection_closestPoint_length = 0;
        P3 best_nextConnection_pointOnPoints(0, 0, 0);
        if (current_aid == -1) current_aid = start_aid;
        for (StreamPointsConnection* ci : map_anchorid_connection[current_aid]) // for each connection registered for this anchor
        {
            if (ci->pc1.sp.StreamAnchorPointId == ci->pc1.ap.id) continue; // skip cyclic streams
            if (ci->Index == prev_ci_index) continue; //  skip going back to previous anchor point
            //if (pc1.Index == ci->Index) continue; //  skip starting possibleConnection
            const StartPointToAnchorPossibleConnection* nextConnection = (ci->pc1.sp.StreamAnchorPointId == current_aid) ? &ci->pc1 : &ci->pc2;
            D cos_stream_nextConnection = 0;
            for (int pi = 0; pi < 5; pi++)
            {                
                if (currentPointIndex + pi >= mergedStream.size() - 1) break; // dirToNextPointNormalized is not correct to use in last point
                cos_stream_nextConnection = max(cos_stream_nextConnection, utils::vector::Cos(nextConnection->stream[0].dirToNextPointNormalized, mergedStream[currentPointIndex + pi].dirToNextPointNormalized, true));
            }
            if (cos_stream_nextConnection < COS_75) continue; // skip going opposite direction


            P3 nextConnection_startPoint = nextConnection->sp.point;
            P3 nextConnection_endPoint = nextConnection->ap.point;
            P3 nextConnection_pointOnPoints;
            int pointOnPointsIndex = -1;
            D dist = mergedStream.GetClosestDist_Linear(currentPointIndex, nextConnection_endPoint, nextConnection_pointOnPoints, pointOnPointsIndex, false, false); // last parameter must be 'false'
            if (pointOnPointsIndex == -1) continue; // fail if not found 
            if (pointOnPointsIndex < currentPointIndex) continue;// fail if found backward but not forward
            if (nextConnection->ap.id == end_aid) dist = 0; // if we reach target - set dist to minimum
            if (pointOnPointsIndex > maxPointIndex) continue; // fail if we out of target length 
            D progressLength = mergedStream.GetTotalLength(currentPointIndex, pointOnPointsIndex);
            if (min(progressLength, nextConnection->closestPoint.length) < 0.5*max(progressLength, nextConnection->closestPoint.length)) continue; // fail if progress legnth differs a lot from connection length - so we going in wrong direction
            if (best_ci == nullptr || dist < best_nextConnection_dist)
            {
                best_ci = ci;
                best_nextConnection_dist = dist;
                best_nextConnection_pointIndex = pointOnPointsIndex;
                best_nextConnection_progressLength = progressLength;
                best_nextConnection = nextConnection;
                best_nextConnection_closestPoint_length = nextConnection->closestPoint.length;
                best_nextConnection_pointOnPoints = nextConnection_pointOnPoints;
            }
        }
        // fail if not found next connection
        if (best_ci == nullptr)
        {
            if (debug) cout << "  break because 'spcBest == nullptr'" << endl;
            break;
        }
        // fail if dist is bigger from allowed
        if (currentMaxDist > max(maxDist, best_nextConnection_progressLength / 10)) // maxDist or 10% of progress length
        {
            if (debug) cout << "  break because 'currentMaxDist > maxDist'" << endl;
            break;
        }
        currentPointIndex = best_nextConnection_pointIndex;
        currentMaxDist = max(currentMaxDist, best_nextConnection_dist);
        prev_aid = current_aid;
        prev_ci_index = best_ci->Index;
        current_aid = best_nextConnection->ap.id;
        alternativePath.push_back(best_ci);
        aids.push_back(current_aid);
        if (debug)
        {
            cout << "  prev_aid=" << prev_aid << "  current_aid=" << current_aid << "   currentPointIndex=" << currentPointIndex << "   currentMaxDist=" << currentMaxDist << "   progressLength=" << best_nextConnection_progressLength << "   nextConnection->closestPoint.length=" << best_nextConnection_closestPoint_length << endl;
            draw.AddLabel(best_nextConnection->ap.point, "  aid=" + to_string(best_nextConnection->ap.id), Color3d(1, 0, 0), 2);
        }
    }
    if (current_aid == end_aid && aids.size() > 2)
    {

        if (debug) cout << "  found   " << utils::stdvector::toString(aids) << endl;
        maxDistFromAidsToStream = currentMaxDist;
        return true;
    }
    else
    {
        if (debug) cout << "  not found" << endl;
        alternativePath.clear();
        aids.clear();
        return false;
    }
};

void MeshStreamsJoiner::FindAlternativePaths(D maxDist, const vector<StartPointToAnchorPossibleConnection>& possibleConnections, vector<StreamPointsConnection>& connections_all,
    vector<AlternativePath>& alternativePaths)
{
    //
    // create map<anchorid, connection>
    //
    map<int, vector<StreamPointsConnection*>> map_anchorid_connection;
    for (StreamPointsConnection& c : connections_all)
    {
        int aids[2] = { c.pc1.sp.StreamAnchorPointId, c.pc2.sp.StreamAnchorPointId };
        for (int aid : aids)
        {
            auto f = map_anchorid_connection.find(aid);
            if (f == map_anchorid_connection.end())
            {
                map_anchorid_connection[aid] = vector<StreamPointsConnection*>();
                f = map_anchorid_connection.find(aid);
            }
            f->second.push_back(&c);
        }
    }

    //
    // find all alternative paths
    //
    alternativePaths.clear();
    alternativePaths.reserve(connections_all.size() * 10);
    map<pair<int, int>, int> map_aidminmax_AlternativePathIndex;
    //vector<StreamPointsConnection*> alternativePathConnections;
    //alternativePathConnections.reserve(100);
    vector<int> aids;
    aids.reserve(100);
    int debugStreamId = options.DebugEnabled ? options.debug_alternativePath_debugStreamIndex : -1;
    for (StreamPointsConnection& c : connections_all)
    {
        if (debugStreamId != -1 && (debugStreamId != c.pc1.sp.Index && debugStreamId != c.pc2.sp.Index)) continue;
        bool debug = debugStreamId != -1 && (debugStreamId == c.pc1.sp.Index || debugStreamId == c.pc2.sp.Index);
        alternativePaths.push_back(AlternativePath(alternativePaths.size(), c));
        AlternativePath& path = alternativePaths.back();
        D maxDistFromAidsToStream = 0;
        if (FindAlternativePath(maxDist, c, map_anchorid_connection, debug, debugStreamId, path.alternativeConnection, aids, maxDistFromAidsToStream))
        {
            //path.aids = aids;
            //path.start_aid = aids.front();
            //path.end_aid = aids.back();
            //path.aid_min_max = { min(path.start_aid, path.end_aid), max(path.start_aid, path.end_aid) };
            path.maxDistFromAidsToStream = maxDistFromAidsToStream;
            path.alternativeConnection_nonExpanded = path.alternativeConnection;
            map_aidminmax_AlternativePathIndex[path.originConnection.aid_min_max] = path.Index;
        }
        else
        {
            alternativePaths.pop_back(); // remove previously allocated memory for path - since we fail to find it
        }
    }
}

void MeshStreamsJoiner::ExpandAlternativePaths(const vector<StreamPointsConnection>& connections_all, vector<AlternativePath>& alternativePaths)
{
    // restore origin, non expanded path
    for (AlternativePath& alternativePath : alternativePaths)
    {
        if (alternativePath.isRemovedBecauseItIsUseless) continue;
        alternativePath.alternativeConnection = alternativePath.alternativeConnection_nonExpanded;
    }

    // create map
    vector<AlternativePath*> connection_to_alternativePath;
    connection_to_alternativePath.resize(connections_all.size());
    for (AlternativePath& alternativePath : alternativePaths)
    {
        if (alternativePath.isRemovedBecauseItIsUseless) continue;
        connection_to_alternativePath[alternativePath.originConnection.Index] = &alternativePath;
    }

    // expand until no more expansions
    int expanded_alternativePath_count = 1;
    while (expanded_alternativePath_count > 0)
    {
        expanded_alternativePath_count = 0;
        for (AlternativePath& alternativePath : alternativePaths)
        {
            if (alternativePath.isRemovedBecauseItIsUseless) continue;
            //bool isDebugExpandPath = false;
            //if (alternativePath.originConnection.aid_min_max == pair<int, int>(7, 9))
            //{
            //    cout << "alternativePath.originConnection.Index = " << alternativePath.originConnection.Index << endl;
            //    isDebugExpandPath = true;
            //}

            if (alternativePath.ExpandAlternativeConnection(connection_to_alternativePath))
            {
                expanded_alternativePath_count++;
            }

            //if (isDebugExpandPath)
            //{
            //    cout << endl;
            //}
        }
    }
}


void MeshStreamsJoiner::FindAllConnections(D maxDist, const vector<StreamStartPoint>& startStreamPoints, const vector<StartPointToAnchorPossibleConnection>& possibleConnections, vector<StreamPointsConnection>& connections_all)
{
    const D max_lengths_diff_percent = options.max_lengthsDiff_percent;
    D const COS_90 = utils::angle::DegreesToCos(90);

    //
    // set relations anchor to possible connection
    //
    map<int, vector<int>> map_anchorId_to_possibleConnection;
    for (int i = 0; i < possibleConnections.size(); i++)
    {
        const StartPointToAnchorPossibleConnection& pc = possibleConnections[i];
        map_anchorId_to_possibleConnection[pc.sp.StreamAnchorPointId].push_back(i);
    }

    //
    // build connections
    //
    connections_all.clear();
    connections_all.reserve(possibleConnections.size());
    for (auto& m : map_anchorId_to_possibleConnection) // for each stream that starts from specific anchor
    {
        vector<int>& pc_indexes = m.second;
        for (auto& pc_index : pc_indexes) // iterate possible connections indexes
        {
            const StartPointToAnchorPossibleConnection& pc1 = possibleConnections[pc_index];
            vector<int>& pc_indexes2 = map_anchorId_to_possibleConnection[pc1.ap.id]; // get all streams that goes from anchor point of our stream
            for (auto& pc_index2 : pc_indexes2)
            {
                const StartPointToAnchorPossibleConnection& pc2 = possibleConnections[pc_index2];
                if (pc1.sp.StreamAnchorPointId == pc2.ap.id  //if stream point is pointing to anchor point
                    && pc1.sp.Index < pc2.sp.Index) //protection from D registering pair of stream-stream
                {
                    auto& stream1 = streams[pc1.sp.Index];
                    auto& stream2 = streams[pc2.sp.Index];
                    bool acceptConnection = true; // by default lets assume it is acceptable
                    string failReason = "";
                    D connectionDistAtMid = 0;
                    int connection_pointsIndex1 = pc1.closestPoint.index;
                    int connection_pointsIndex2 = pc2.closestPoint.index;
                    bool debug_1_stream = (options.debug_debugStreamIndex == pc1.sp.Index || options.debug_debugStreamIndex == pc2.sp.Index);
                    bool debug_2_streams = options.debug_connection && ((pc1.sp.Index == options.debug_connection_streamIndex1 && pc2.sp.Index == options.debug_connection_streamIndex2) || (pc1.sp.Index == options.debug_connection_streamIndex2 && pc2.sp.Index == options.debug_connection_streamIndex1));
                    bool debug = options.DebugEnabled && (debug_1_stream || debug_2_streams);
                    bool debugEveryPoint = debug && options.debug_everyPointOfStream;
                    D lengths_diff_percent = 1 - min(pc1.closestPoint.length, pc2.closestPoint.length) / max(pc1.closestPoint.length, pc2.closestPoint.length);

                    //cout << pc1.sp.ID << " - " << pc2.sp.ID << endl;
                    if (debug)
                    {
                        if (debugEveryPoint) cout << endl;
                        cout << endl << "StreamPoints::GetConnection  streams #" << pc1.sp.Index << " #" << pc2.sp.Index << endl;
                    }


                    if (pc1.sp.haveContract)
                    {
                        // update connection indexes 'connectionIndex_points1', 'connectionIndex_points2'
                        bool connectedAtMid = MeshStream::GetConnection(draw, mesh, streams[pc1.sp.Index], streams[pc2.sp.Index], connectionDistAtMid, connection_pointsIndex1, connection_pointsIndex2, pc1.closestPoint.index, pc2.closestPoint.index, options.max_connectionAngle_Directions, options.max_connectionAngle_Normals, debug, debugEveryPoint, max(pc1.closestPoint.length, pc2.closestPoint.length));
                        if (connectedAtMid) // if contractors connects properly - put them in the list before others.  
                        {
                            acceptConnection = true; // contractors always accepted
                        }
                        else
                        {
                            acceptConnection = true; // we dont fail contractors, since contract - means manual connection - lets trust to this decision
                        }
                    }
                    else
                    {
                        // check dists
                        if (acceptConnection)
                        {
                            if ((!pc1.closestPoint.isDistAcceptable || !pc2.closestPoint.isDistAcceptable) // if some of 2 dists are not acceptable  (if dist from stream1 to anchor 2 is not acceptable, or if dist from stream2 to anchor 1 is not acceptable)
                                && (!pc1.closestPoint.isCyclic && !pc2.closestPoint.isCyclic)) // and none of them is cyclic  (cyclic streams must be connected somehow - so ignore dists in favor of closing cyclic stream)
                            {
                                acceptConnection = false;
                                if (debug) failReason = "distance from stream to anchor is to big: " + to_string(pc1.closestPoint.dist) + "    " + to_string(pc2.closestPoint.dist);
                            }
                        }

                        // check angles at starts of streams
                        if (acceptConnection)
                        {
                            D dot12 = utils::vector::Cos(stream1.Points[pc1.closestPoint.index].dirToNextPointNormalized, stream2.Points[0].dirToNextPointNormalized);
                            D dot21 = utils::vector::Cos(stream2.Points[pc2.closestPoint.index].dirToNextPointNormalized, stream1.Points[0].dirToNextPointNormalized);
                            if (dot12 > -COS_90 || dot21 > -COS_90)
                            {
                                acceptConnection = false;
                                if (debug) failReason = "connection angles are to big:   " + to_string(utils::angle::RadiansToDegrees(acos(dot12))) + "    " + to_string(utils::angle::RadiansToDegrees(acos(dot21)));
                            }
                        }

                        // check connection at mid
                        if (acceptConnection)
                        {
                            //debug = (pc1.sp.Index == 1 && pc2.sp.Index == 19);
                            bool connectedAtMid = MeshStream::GetConnection(draw, mesh, streams[pc1.sp.Index], streams[pc2.sp.Index], connectionDistAtMid, connection_pointsIndex1, connection_pointsIndex2, pc1.closestPoint.index, pc2.closestPoint.index, options.max_connectionAngle_Directions, options.max_connectionAngle_Normals, debug, debugEveryPoint, max(pc1.closestPoint.length, pc2.closestPoint.length));
                            if (!connectedAtMid)
                            {
                                acceptConnection = false;
                                if (debug) failReason = "streams not connected at mid";
                            }
                        }

                        // check connections lengths - they must be almost same
                        if (acceptConnection && lengths_diff_percent > max_lengths_diff_percent)
                        {
                            acceptConnection = false;
                            if (debug) failReason = "connections lengths are different: lengths_diff_percent=" + to_string(lengths_diff_percent) + "  and maximum allowed " + to_string(max_lengths_diff_percent);
                        }

                        // check dist in middle v0
                        //if (acceptConnection && connectionDist > maxDist)
                        //{
                        //    acceptConnection = false;
                        //    if (debug) failReason = "connection dist is to big: connectionDist=" + to_string(connectionDist) + "  and maximum allowed maxDist=" + to_string(maxDist);
                        //}

                        // check dist in middle v1 - more flexible: gives a chance to join streams even for very small meshsize
                        if (acceptConnection && connectionDistAtMid > max(maxDist * 3, pc1.closestPoint.dist + pc2.closestPoint.dist))
                        {
                            acceptConnection = false;
                            if (debug) failReason = "connection dist is to big: connectionDistAtMid=" + to_string(connectionDistAtMid) + "  and maximum allowed maxDist*3=" + to_string(maxDist * 3);
                        }
                    }

                    // DEBUG show connection info
                    if (debug)
                    {
                        cout << "   connection:   stream#" << pc1.sp.Index << ".point#" << connection_pointsIndex1 << "   stream#" << pc2.sp.Index << ".point#" << connection_pointsIndex2 << "    dist=" << connectionDistAtMid << "  lengths_diff_percent=" << lengths_diff_percent;
                        if (acceptConnection)
                        {
                            cout << "   accepted" << endl << endl;
                        }
                        else
                        {
                            cout << "   failed: " << failReason << endl << endl;
                        }
                        if (debugEveryPoint)
                        {
                            D colorInc = 0;
                            for (int spid : {pc1.sp.Index, pc2.sp.Index})
                            {
                                Color3d color = Color3d(0.7 + colorInc, 0, 0);
                                colorInc += 0.3;
                                for (int i = 0; i < streams[spid].size() - 1; i++)
                                {
                                    draw.AddEdge(streams[spid][i].point, streams[spid][i + 1].point, color);
                                    draw.AddPoint(streams[spid][i].point, color, "stream#" + to_string(spid) + ".point#" + to_string(i));
                                }
                            }
                        }
                        else
                        {
                            if (acceptConnection && debug_1_stream)
                            {
                                Color3d color = Color3d(0.7, 0, 0);
                                int spid = options.debug_debugStreamIndex;
                                for (int i = 0; i < streams[spid].size() - 1; i++)
                                {
                                    draw.AddEdge(streams[spid][i].point, streams[spid][i + 1].point, color);
                                    draw.AddPoint(streams[spid][i].point, color, "stream#" + to_string(spid) + ".point#" + to_string(i));
                                }
                            }
                        }
                    }

                    // two streams must connect at mid at some dist (angle between them must be acceptable), otherwise they not meet each other at any point and streams connot be joined
                    if (acceptConnection)
                    {
                        connections_all.push_back(StreamPointsConnection(connections_all.size(), pc1, pc2, connectionDistAtMid, connection_pointsIndex1, connection_pointsIndex2));
                    }
                }
            }
        }
    }


}

void MeshStreamsJoiner::SortConnections(const vector<StreamPointsConnection>& connections_all, vector<unsigned int>& sorted_indexes)
{
    bool sort_by_length_for_very_small_distances = options.sort_by_length_for_very_small_distances;
    D mesh_avg_edge_length = mesh.avg_edge_length;
    auto sortConnectioins = [&connections_all, sort_by_length_for_very_small_distances, mesh_avg_edge_length](unsigned int i1, unsigned int i2)
    {
        const StreamPointsConnection& c1 = connections_all[i1];
        const StreamPointsConnection& c2 = connections_all[i2];

        // give priority to contractors
        if (c1.haveContract || c2.haveContract)
        {
            if (c1.haveContract && !c2.haveContract) return true;
            if (!c1.haveContract && c2.haveContract) return false;
            // otherwise they have same priority
        }


        // v1 - best angle
        //return c1.connectionDist / c1.travel_length < c2.connectionDist / c2.travel_length;

        //v2 - priority to 'connectionDist' - for small distances lets choise shortest length
        if (sort_by_length_for_very_small_distances
            && (c1.travel_length > c2.travel_length*1.3 || c2.travel_length > c1.travel_length*1.3)) // length must differs at least for 30%, so we can say for shure that smaller length goes first - otherwise our prediction will be a random choise
        {
            if (c1.connectionDistAtMid < mesh_avg_edge_length && c2.connectionDistAtMid < mesh_avg_edge_length)
            {
                return c1.travel_length < c2.travel_length;
            }
        }


        // give priority to cyclic streams
        //        if (c1.hasCyclicStream || c2.hasCyclicStream)
        //        {
        //            int c1cycl = c1.hasCyclicStream ? 0 : 1;
        //            int c2cycl = c2.hasCyclicStream ? 0 : 1;
        //            if (c1cycl != c2cycl) // cyclic streams goes first
        //            {
        //                return c1cycl < c2cycl;
        //            }
        ////            return c1.travel_length < c2.travel_length; // if both connection are cyclic, then connection with shortest connection length goes first
        //        }

        //return c1.travel_length < c2.travel_length;
        // for normal distances lets choise stream that are closest at connection point
        //return c1.travel_length*c1.connectionDistAtMid < c2.travel_length*c2.connectionDistAtMid;
        //return c1.connectionDistAtMid/c1.travel_length < c2.connectionDistAtMid/ c2.travel_length;
        return c1.sortWeight < c2.sortWeight;
    };

    sorted_indexes = utils::stdvector::sort_indexes_custom(connections_all.size(), sortConnectioins);
}

bool MeshStreamsJoiner::FindPossibleConnections(const MeshStreams& streams, StreamStartPoint& sp, const vector<StreamAnchorPoint>& anchors, vector<StartPointToAnchorPossibleConnection>& possibleConnections)
{
    const MeshStream& stream = streams[sp.Index];
    if (stream.IsMerged) return false;
    if (stream.IsDeleted) return false;
    assert(stream.IsInited_lengthToNextPoint());
    if (stream.size() == 0) return false;
    if (anchors.size() == 0) return false;
    D meshSize_relative = options.maxDist_is_minimum_of_mesh_avg_size ? max(meshSize, mesh.avg_edge_length*1.5) : meshSize;
    D maxDist = meshSize_relative * (options.DebugEnabled ? options.debug_meshSize_multipliyer : options.maxDist_meshSize_multipliyer);


    //
    // Init dists to anchors
    //
    vector<ClosestPointFromStreamToAnchor> closests(anchors.size(), ClosestPointFromStreamToAnchor());
    for (int i = 0; i < anchors.size(); i++)
    {
        ClosestPointFromStreamToAnchor& c = closests[i];
        const StreamAnchorPoint& a = anchors[i];
        if (sp.haveContract && sp.contract != a.contract)continue;
        //t.dists_to_anchors_from_start_point = utils::point::DistToPoint(sp.point, anchors[i].point);

        c.isCyclic = !stream.isEndsAtBorder() && stream.isCyclic;

        //v1 - quick but not always correct
        //c.dist = points.GetClosestDist_Quick(a.point, c.pointOnStream, c.index);
        //if (c.index == 0)
        //{
        //    c.dist = points.GetClosestDist_Quick(a.point, c.pointOnStream, c.index, points.size() - 2, points.size() - 2);
        //}
        //v2 - slow but always correct
        if (sp.Index == 90 && a.vid_eid_fid == 183)
        {
            int temp = 0;
        }
        c.dist = stream.GetClosestDist_Linear(0, a.point, c.pointOnStream, c.index, false, false); //  also inits 'c.index'

        // set isDistAcceptable
        c.isDistAcceptable = false;   // will be calculated in next 'for' 
                                      // cyclic streams can connect to theirs start anchor
        if (stream.isCyclic && sp.StreamAnchorPointId == a.id) // testing stream with anchor from which it starts
        {
            c.isDistAcceptable = true;
            // if distance not inited - set distance to the last point of stream
            if (c.index == -1)
            {
                c.index = stream.size() - 1;
                c.pointOnStream = stream.Points.back().point;
            }
            if (options.DebugEnabled && meshLogicOptions.MeshStream.show_ExtendStream_info_in_console)
            {
                cout << "c.isStreamCyclic && stream.isCyclic && sp.StreamAnchorPointId == a.id    sp.Index=" << sp.Index << endl;
            }
        }


        //if (stream.StreamAnchorPointId == 11)
        //{
        //    draw.AddEdge(a.point, c.pointOnStream, Color3d(1, 0, 0), "c.index="+to_string(c.index));
        //}
        if (sp.haveContract)
        {
            c.dist = 0;
        }

        if (c.index != -1)
        {
            c.length = stream.GetTotalLength(0, c.index);
            c.length += utils::point::DistToPoint(stream[c.index].point, c.pointOnStream);
        }
    }

    //DEBUG - show closet point to anchor for lowest dist
    //if (options.DebugEnabled && options.debug_debugStreamId == sp.ID)
    //{
    //    // sort connections
    //    vector<unsigned int> sorted_indexes = utils::stdvector::sort_indexes_custom(closests.size(), [&closests, maxDist](unsigned int i1, unsigned int i2)
    //    {
    //        const ClosestPointFromStreamToAnchor& c1 = closests[i1];
    //        const ClosestPointFromStreamToAnchor& c2 = closests[i2];

    //        if (c1.dist > maxDist  && c2.dist > maxDist) return false;
    //        if (c1.dist > maxDist) return true;
    //        if (c2.dist > maxDist) return false;
    //        return c1.length < c2.length;
    //    });
    //    ClosestPointFromStreamToAnchor& c = closests[sorted_indexes[0]];
    //    for (int i = 0; i < c.index; i++)
    //    {
    //        draw.AddEdge(points[i].point, points[i + 1].point, Color3d(0, 0.7, 0));
    //    }
    //    draw.AddEdge(points[c.index].point, c.pointOnStream, Color3d(0, 0.7, 0), "dist=" + to_string(c.dist) + "   maxAllowedDist=" + to_string(maxDist) + "   length=" + to_string(c.length));
    //}


    const D max_connectionArea_percent = options.max_connectionArea_percent;
    const D min_lengthToDist_percent = options.min_lengthToDist_percent;
    DD maxConnectionArea = mesh.Area*max_connectionArea_percent;
    int added_count = 0;
    if (options.DebugEnabled && options.debug_debugStreamIndex == sp.Index)
    {
        cout << ("Possible connections for stream#" + to_string(sp.Index)) << "   (max allowed dist=" << maxDist << ")" << endl;
    }
    for (int i = 0; i < anchors.size(); i++)
    {
        ClosestPointFromStreamToAnchor& c = closests[i];
        const StreamAnchorPoint& a = anchors[i];
        if (sp.haveContract && sp.contract != a.contract)continue;

        if (c.index == -1) continue;
        if (a.dividerIterator_iterationNum != stream.dividingIteration.iterationNum) continue; // allow joining only with same dividingIteration

        // accept allmost all connections, since we dont want to miss cyclic stream - it must be connected to any stream. final filtering we will do in 'FindConnections'
        DD connectionArea = c.length*c.dist;
        bool accept = sp.haveContract // contractors must be connected to each other
            || (
            (c.length > c.dist * min_lengthToDist_percent)  // stream and anchor points is not of form of a quad
                && (connectionArea < maxConnectionArea) // connection area must be not to big
                );

        // detect if distance is acceptable - will be used in method 'FindConnections'
        D c_dist_notnull = c.dist < 0.000000001f ? 0.000000001f : c.dist;
        D maxDistDynamicMultiplyer = c.length / (10 * c_dist_notnull); // each 10 times length longest from dist 'maxDistDynamicDelta' grows by 'maxDist'
        c.isDistAcceptable = c.isDistAcceptable                      // take into account calculation for cyclic streams in previous loop above
            || sp.haveContract                                             // contractors must be connected to each other
            || (c.dist < maxDist                                           // better to snap stream to anchor since there is hardly can be placed meshSize in between
                || c.length > 20 * (c.dist - maxDist)                 // length is much longer from dist
                                                                      //|| c.length*maxDist >(10 * c.dist) * c.dist      // length is much longer from dist
                || c.dist < maxDist*maxDistDynamicMultiplyer) // distance between lines hardly noticable in comparision to their lengths
            ;


        if (accept)
        {
            added_count++;
            possibleConnections.push_back(StartPointToAnchorPossibleConnection(streams, possibleConnections.size(), sp, a, c));
        }

        if (options.DebugEnabled && options.debug_debugStreamIndex == sp.Index)
        {
            string text = "  aid#" + to_string(a.id) + "   index=" + to_string(c.index) + ",   length=" + to_string(c.length) + ",   dist=" + to_string(c.dist);

            Color3d color = accept ? Color3d(0, 1, 0) : Color3d(0.4, 0.4, 0.4);
            draw.AddLabel(a.point, text, color);
            for (int pi = 0; pi < c.index; pi++)
            {
                draw.AddEdge(stream[pi].point, stream[pi + 1].point, Color3d(0, 0.7, 0));
            }
            draw.AddEdge(stream[c.index].point, c.pointOnStream, Color3d(0, 0.7, 0));
            draw.AddEdge(c.pointOnStream, a.point, color);

            cout << text << (accept ? "   accept" : "") << endl;
        }
    }

    if (sp.haveContract && added_count != 1)
    {
        cout << "! wrong   MeshStreamsJoiner::FindPossibleConnections  failed to find contract  mesh.id="<< mesh.id << "  sp.id=" << sp.id << "  ap.id=" << sp.StreamAnchorPointId <<endl;
    }

    return added_count > 0;
}

void MeshStreamsJoiner::FindConnections(const vector<StreamStartPoint>& startStreamPoints, const vector<StartPointToAnchorPossibleConnection>& possibleConnections, vector<StreamPointsConnection>& connections)
{
    D meshSize_relative = options.maxDist_is_minimum_of_mesh_avg_size ? max(meshSize, mesh.avg_edge_length*1.5) : meshSize;
    const D maxDist = meshSize_relative * (options.DebugEnabled ? options.debug_meshSize_multipliyer : options.maxDist_meshSize_multipliyer);

    //
    // Find all connections
    //
    vector<StreamPointsConnection> connections_all;
    FindAllConnections(maxDist, startStreamPoints, possibleConnections, connections_all);

    //
    // Find alternative paths for all connections
    //
    vector<AlternativePath> alternativePaths;
    if (options.optimizeConnections)
    {
        FindAlternativePaths(maxDist, possibleConnections, connections_all, alternativePaths);
    }

    //
    // Create map streamIndex to connectionIndex
    //
    CompactVectorVector<StreamPointsConnection*> map_streamIndex_connections;
    int maxStreamIndex = 0;
    for (StreamPointsConnection& c : connections_all) maxStreamIndex = max(maxStreamIndex, max(c.pc1.sp.Index, c.pc2.sp.Index));
    map_streamIndex_connections.resizeBegin(maxStreamIndex + 1);
    for (StreamPointsConnection& c : connections_all)
    {
        map_streamIndex_connections.size(c.pc1.sp.Index)++;
        map_streamIndex_connections.size(c.pc2.sp.Index)++;
    }
    map_streamIndex_connections.resizeEnd();
    for (StreamPointsConnection& c : connections_all)
    {
        map_streamIndex_connections.add(c.pc1.sp.Index, &c);
        map_streamIndex_connections.add(c.pc2.sp.Index, &c);
    }

    CompactVectorVector<AlternativePath*> map_connectionIndex_alternativePaths;

    //
    // Iterate until removed all unproper alternative paths
    //
    int iterationNum = -1;
    bool needToResort = true;
    while (needToResort)
    {
        iterationNum++;
        needToResort = false;

        //
        // set sort weights
        //
        for (StreamPointsConnection& c : connections_all)
        {
            //c.sortWeight = c.connectionDistAtMid;
            //c.sortWeight = c.travel_length*c.connectionDistAtMid;
            c.sortWeight = min(min(c.pc1.closestPoint.dist, c.pc2.closestPoint.dist), c.connectionDistAtMid);
        }

        //
        // Expand alternative paths
        //    
        if (options.optimizeConnections)
        {
            // expand alternative paths that has sub-alternative-path
            ExpandAlternativePaths(connections_all, alternativePaths);

            // correct weight to bring forward alternative paths
            for (AlternativePath& alternativePath : alternativePaths)
            {
                if (alternativePath.isRemovedBecauseItIsUseless) continue;
                //if (alternativePath.originConnection.aid_min_max == pair<int,int>(17, 18))
                //{
                //    cout << "connection {" << alternativePath.originConnection.aid_min_max.first<<","<< alternativePath.originConnection.aid_min_max.second<<"}" << endl;
                //}
                D min_sortWeight = alternativePath.GetMinSortWeight();
                D max_sortWeight = alternativePath.GetMaxSortWeight();
                //alternativePath.originConnection.sortWeight = min_sortWeight + (max_sortWeight - min_sortWeight) / 1000;
                alternativePath.originConnection.sortWeight = max(alternativePath.originConnection.sortWeight, max_sortWeight) + (max_sortWeight - min_sortWeight) / 1000;
            }

            // Create map connectionIndex to alternativePaths
            map_connectionIndex_alternativePaths.resizeBegin(connections_all.size());
            for (AlternativePath& a : alternativePaths)
            {
                for (StreamPointsConnection* ac : a.alternativeConnection_nonExpanded)
                {
                    map_connectionIndex_alternativePaths.size(ac->Index)++;
                }
            }
            map_connectionIndex_alternativePaths.resizeEnd();
            for (AlternativePath& a : alternativePaths)
            {
                for (StreamPointsConnection* ac : a.alternativeConnection_nonExpanded)
                {
                    map_connectionIndex_alternativePaths.add(ac->Index, &a);
                }
            }
        }



        //
        // Get only the best connections
        //
        connections.clear();
        map<int, bool> used;
        // sort connections
        vector<unsigned int> sorted_indexes;
        SortConnections(connections_all, sorted_indexes);

        //DEBUG - show all non used connections
        //cout << "iteration#" << iterationNum << endl;
        //for (int i : sorted_indexes)
        //{
        //    const StreamPointsConnection& c = connections_all[i];
        //    if (map_streamId_to_connectionsCount[c.pc1.sp.ID] == 0) continue;
        //    if (map_streamId_to_connectionsCount[c.pc2.sp.ID] == 0) continue;
        //    cout << "     connection #" << c.pc1.sp.ID << "#" << c.pc2.sp.ID << "     map_streamId_to_connectionsCount " << map_streamId_to_connectionsCount[c.pc1.sp.ID] << " " << map_streamId_to_connectionsCount[c.pc2.sp.ID] << endl;
        //}

        //DEBUG - show all connections for debuged stream
        //if (options.DebugEnabled && options.debug_debugStreamId != -1)
        //{
        //    for (int i : sorted_indexes)
        //    {
        //        const StreamPointsConnection& c = connections_all[i];
        //        if (c.pc1.sp.Index == options.debug_debugStreamId || c.pc2.sp.Index == options.debug_debugStreamId)
        //        {
        //            cout << "Connection "
        //        }
        //    }
        //}

        // take first not used connection and add to result list
        for (int i : sorted_indexes)
        {
            const StreamPointsConnection& c = connections_all[i];
            if (used.find(c.pc1.sp.Index) != used.end()) continue;
            if (used.find(c.pc2.sp.Index) != used.end()) continue;
            used[c.pc1.sp.Index] = true;
            used[c.pc2.sp.Index] = true;

            // check if this connection makes some alternative path useless
            if (options.optimizeConnections)
            {
                for (int streamIndex : {c.pc1.sp.Index, c.pc2.sp.Index})
                {
                    assert(streamIndex < map_streamIndex_connections.size() && "streamIndex must be valid");
                    for (int ci = 0; ci < map_streamIndex_connections.size(streamIndex); ci++) // check all connections that is created for this streamIndex
                    {
                        StreamPointsConnection* next_connection_of_stream = map_streamIndex_connections(streamIndex, ci);
                        if (next_connection_of_stream->Index == c.Index) continue; // skip validating current connection
                        assert(next_connection_of_stream->Index < map_connectionIndex_alternativePaths.size() && "connection index must be valid");
                        for (int ai = 0; ai < map_connectionIndex_alternativePaths.size(next_connection_of_stream->Index); ai++) // check all alternative paths that uses this connectoin
                        {
                            AlternativePath* alternativePath = map_connectionIndex_alternativePaths(next_connection_of_stream->Index, ai);
                            if (alternativePath->isRemovedBecauseItIsUseless) continue;
                            if (!alternativePath->Exists(c.Index))
                            {
                                alternativePath->alternativeConnection.clear();
                                alternativePath->isRemovedBecauseItIsUseless = true;
                                needToResort = true;
                                if (options.DebugEnabled) cout << " removed alternative path   sindexes={" << alternativePath->originConnection.sindexes_min_max.first << ", " << alternativePath->originConnection.sindexes_min_max.second << "}   aids={" << alternativePath->originConnection.aid_min_max.first << ", " << alternativePath->originConnection.aid_min_max.second << "}" << endl;
                            }
                        }
                    }
                }
                if (needToResort) break;
            }
            //streamId_to_connectionsCount[c.pc1.sp.Index] = 0;
            //streamId_to_connectionsCount[c.pc2.sp.Index] = 0;
            connections.push_back(StreamPointsConnection(connections.size(), c.pc1, c.pc2, c.connectionDistAtMid, c.connection_pointsIndex1, c.connection_pointsIndex2));
        }
    }

    //
    // DEBUG show connection indexes
    //
    if (options.DebugEnabled && options.debug_show_connection_indexes)
    {
        draw.ReserveLabels(connections_all.size());
        for (auto& c : connections_all)
        {
            P3 p1 = c.pc1.stream.Points[c.connection_pointsIndex1].point;
            P3 p2 = c.pc1.stream.Points[c.connection_pointsIndex2].point;
            P3 p = (p1 + p2) / 2;
            draw.AddLabel(p, to_string(c.Index), Color3d(0, 0, 1), 4);
        }
    }

    //
    // DEBUG show anchor ids
    //
    if (options.DebugEnabled && options.debug_show_connection_anchor_ids)
    {
        draw.ReserveLabels(anchorPoints.size());
        for (auto& a : anchorPoints)
        {
            draw.AddLabel(a.point, "  aid=" + to_string(a.id), Color3d(1, 0, 0), 2);
        }
    }

}

void MeshStreamsJoiner::SetConnectionPoint(MeshStreams& streams, StreamPointsConnection& c)
{
    StreamStartPoint& sp1 = c.pc1.sp;
    StreamStartPoint& sp2 = c.pc2.sp;
    MeshStream& s1 = streams[sp1.Index];
    MeshStream& s2 = streams[sp2.Index];

    sp1.canBeJoined = true;
    sp2.canBeJoined = true;

    // set draft version, in case precise version wont be able to define
    sp1.joinedAtIndex = max(min(1, s1.size()-1), c.pc1.closestPoint.index);
    sp2.joinedAtIndex = max(min(1, s2.size()-1), c.pc2.closestPoint.index);

    // set precise connection point
    D connectionDist = 0;
    int connectionIndex_points1 = 0;
    int connectionIndex_points2 = 0;
    //StreamPoints::GetConnection(streams[sp1.ID].Points, streams[sp2.ID].Points, connectionDist, connectionIndex_points1, connectionIndex_points2, c.pc1.closestPoint.index, c.pc2.closestPoint.index);
    connectionDist = c.connectionDistAtMid;
    connectionIndex_points1 = c.connection_pointsIndex1; //calculated in method 'FindConnections'
    connectionIndex_points2 = c.connection_pointsIndex2; //calculated in method 'FindConnections'

    // assign bestIndex to joinedAtIndex
    if (connectionIndex_points1 != 0 || connectionIndex_points2 != 0)
    {
        sp1.joinedAtIndex = connectionIndex_points1;
        sp2.joinedAtIndex = connectionIndex_points2;
    }

    c.connectionPoint = (s1[sp1.joinedAtIndex].point + s2[sp2.joinedAtIndex].point) / 2;

    // project connection point
    int fid1 = s1[sp1.joinedAtIndex].fid;
    int fid2 = s2[sp2.joinedAtIndex].fid;
    const D cos15degrees = utils::angle::DegreesToCos(15);
    if (fid1 != -1 && fid2 != -1)
    {
        if (utils::vector::Cos(mesh.F_normals.row(fid1), mesh.F_normals.row(fid2), true) < cos15degrees) //  if angle between normals > 20 degree
        {
            c.connectionPoint = mesh.ProjectPoint(c.connectionPoint);
        }
    }

    // update joinedAtIndex to more precise (this time closest to connectionPoint)
    P3 pointOnStream1;
    int joinedAtIndex1 = 0;
    D newDist1 = s1.GetClosestDist_Linear(0, c.connectionPoint, pointOnStream1, joinedAtIndex1, false, true);
    P3 pointOnStream2;
    int joinedAtIndex2 = 0;
    D newDist2 = s2.GetClosestDist_Linear(0, c.connectionPoint, pointOnStream2, joinedAtIndex2, false, true);
    D new_connectionDistAtMid = newDist1 + newDist2;
    if (new_connectionDistAtMid < c.connectionDistAtMid)
    {
        sp1.joinedAtIndex = joinedAtIndex1;
        sp2.joinedAtIndex = joinedAtIndex2;
    }

    //DEBUG - show connection point
    if (options.DebugEnabled &&  options.debug_show_connection_points &&  sp1.canBeJoined &&  sp2.canBeJoined)
    {
        cout << "joining streams {#" << sp1.Index << ", #" << sp2.Index << "} at their indexes {" << sp1.joinedAtIndex << ", " << sp2.joinedAtIndex << "}" << endl;

        P3 p1 = streams[sp1.Index][sp1.joinedAtIndex].point;
        P3 p2 = streams[sp2.Index][sp2.joinedAtIndex].point;
        P3 pConnection = c.connectionPoint;
        if (options.debug_connection)
        {
            bool debugTheseOnly2streams = options.debug_connection && ((sp1.Index == options.debug_connection_streamIndex1 && sp2.Index == options.debug_connection_streamIndex2) || (sp1.Index == options.debug_connection_streamIndex2 && sp2.Index == options.debug_connection_streamIndex1));
            if (debugTheseOnly2streams)
            {
                draw.AddPoint(pConnection, Color3d(0, 0, 0), "connectionPoint #" + to_string(sp1.Index) + "#" + to_string(sp2.Index));
                draw.AddEdge(pConnection, p1, Color3d(0, 0, 0));
                draw.AddEdge(pConnection, p2, Color3d(0, 0, 0));
            }
        }
        else
        {
            draw.AddPoint(p1, Color3d(0, 0, 0), "stream#" + to_string(sp1.Index) + "  joinedAtIndex=" + to_string(sp1.joinedAtIndex));
            draw.AddPoint(p2, Color3d(0, 0, 0), "stream#" + to_string(sp2.Index) + "  joinedAtIndex=" + to_string(sp2.joinedAtIndex));
            draw.AddPoint(pConnection, Color3d(0, 0, 0), "connectionPoint #" + to_string(sp1.Index) + "#" + to_string(sp2.Index));
        }
    }
}

void MeshStreamsJoiner::AdjustConnectedStreams(MeshStreams& s, StreamPointsConnection& connection)
{
    // get middle point of streams - connection point
    StreamStartPoint& sp1 = connection.pc1.sp;
    StreamStartPoint& sp2 = connection.pc2.sp;

    P3 connectionPoint = connection.connectionPoint;

    //DEBUG - show connection point
    if (meshLogicOptions.StreamAdjuster.DebugEnabled)
    {
        draw.AddPoint(connectionPoint, Color3d(1, 0, 0), "connectionPoint #" + to_string(sp1.Index) + "#" + to_string(sp2.Index));
    }

    // adjust each stream get as close as possible to connectionPoint
    MeshStreamsAdjuster adjuster(mesh, solver, draw);
    for (int spNum = 1; spNum <= 2; spNum++)
    {
        StreamStartPoint& sp = (spNum == 1) ? connection.pc1.sp : connection.pc2.sp;
        MeshStream& stream = s[sp.Index];
        MeshStream adjusted_stream(stream.Index, -1, -1, stream.draw, stream.mesh, stream.color, -1);
        //if (connection.Index == 2 && mesh.id == 6 && spNum==2)
        //{
        //    int temp = 0;
        //}
        if (adjuster.AdjustStream(connectionPoint, sp, stream, adjusted_stream))
        {
            s[sp.Index].SetNewPoints(adjusted_stream.Points);
        }
        else
        {
            connection.pc1.sp.canBeJoined = false;
            connection.pc2.sp.canBeJoined = false;
            if (!meshLogicOptions.StreamAdjuster.DebugEnabled) return;
        }
    }
    StreamsAdjuster_callsCount_directMethod += adjuster.callsCount_directMethod;
    StreamsAdjuster_callsCount_iterativeMethod += adjuster.callsCount_iterativeMethod;
}

bool MeshStreamsJoiner::MergeConnectedStreams(MeshStreams& s, StreamPointsConnection& connection)
{
    StreamStartPoint& sp1 = connection.pc1.sp;
    StreamStartPoint& sp2 = connection.pc2.sp;
    MeshStream& stream1 = s[sp1.Index];
    MeshStream& stream2 = s[sp2.Index];

    //
    // find common edge or vertex
    //
    int common_i1 = -1;
    int common_i2 = -1;
    if (!MeshStream::FindBestCommonEdgeOrVertexBetween2Streams_InScope(stream1, stream2,
       connection.connection_pointsIndex1, connection.connection_pointsIndex2, common_i1, common_i2, 7))
    {
        MeshStream::FindBestCommonEdgeOrVertexBetween2Streams_AllPoints(stream1, stream2,
            common_i1, common_i2);
    }
    bool found_common_EdgeOrVertex = (common_i1 != -1);

    //
    // find common face (in case we didnt find common edge or vertex)
    //
    int common_i1_byfid = -1;
    int common_i2_byfid = -1;
    if (!found_common_EdgeOrVertex) // find common face only if common edge or vertex failed - speed optimization
    {
        MeshStream::FindBestCommonFaceBetween2Streams(stream1, stream2,
            common_i1_byfid, common_i2_byfid);
    }
    bool found_common_face = (common_i1_byfid != -1);

    //
    // fail if we dint found any common point
    //
    if (!found_common_EdgeOrVertex && !found_common_face)
    {
        cout << "!!! failed to merge connections in meshid=" << s.mesh.id << " since common edge to join 2 streams #" << stream1.Index << ", #" << stream2.Index << " not found" << endl;
        draw.AddPoint(connection.connectionPoint, Color3d(1, 0, 0), "connection point");
        return false;
    }


    // DEBUG show best common points for merging two streams
    //draw.AddPoint(points1[common_i1].point, Color3d(1, 0, 0));
    //draw.AddPoint(points2[common_i2].point, Color3d(1,0,0));
    //draw.AddLabel((points1[common_i1].point + points2[common_i2].point) / 2, "   min_dist = " + to_string(min_dist));

    //
    // check if we will merge common points or not
    //
    // by default we will merge 2 points
    bool merge_common_points = true;
    // not merge points - in case of common face
    if (found_common_face)
    {
        common_i1 = common_i1_byfid;
        common_i2 = common_i2_byfid;
        merge_common_points = false;
        if (options.DebugEnabled)
        {
            int common_fid = stream1[common_i1_byfid].fid;
            cout << "joining at fid=" << common_fid << "    common_i1_byfid=" << common_i1_byfid << "   common_i2_byfid=" << common_i2_byfid << endl;
        }
    }
    // not merge points - in case if merge point is at start of stream - we cant move start point of stream
    if (found_common_EdgeOrVertex)
    {
        if (common_i2 == 0 && common_i1 > 0)
        {
            merge_common_points = false;
            common_i1--;
        }
        else if (common_i1 == 0 && common_i2 > 0)
        {
            merge_common_points = false;
            common_i2--;
        }
    }

    //
    // create new stream of points from stream1 and stream2
    //
    MeshStream mergedPoints(stream1.Index, -1, -1, stream1.draw, stream1.mesh, stream1.color, -1);
    // copy points from stream1
    for (int i = 0; i <= common_i1; i++)// loop including common_i1
    {
        mergedPoints.Add(stream1[i]);
    }
    // add merge point or point from stream2
    if (merge_common_points)
    {
        mergedPoints.Points.back().point = (mergedPoints.Points.back().point + stream2[common_i2].point) / 2; // merged point is avarage of thwo streams points
    }
    else
    {
        //draw.AddPoint(mergedPoints.Points.back().point, Color3d(1, 0, 0), "s1");
        mergedPoints.Add(stream2[common_i2]);
        //draw.AddPoint(mergedPoints.Points.back().point, Color3d(1, 0, 0), "s2");
    }
    //copy points from stream2
    for (int i = common_i2 - 1; i >= 0; i--)// loop exluding common_i2 
    {
        mergedPoints.Add(stream2[i]);
    }

    //
    // store merged streams to first stream
    //
    stream1.SetNewPoints(mergedPoints.Points);
    stream1.IsMerged = true;
    stream1.IsMerged_with_streamIndex = stream2.Index;
    //stream1.dividingIteration = max(sp1.dividingIteration.iterationNum, sp2.dividingIteration.iterationNum);
    if (sp1.dividingIteration.iterationNum > sp2.dividingIteration.iterationNum)
    {
        stream1.dividingIteration = sp1.dividingIteration.iterationNum;
    }
    else
    {
        stream1.dividingIteration = sp2.dividingIteration.iterationNum;
    }

    //
    // remove second stream
    //
    stream2.IsDeleted = true;
    stream2.clear();

    //
    // clear flags
    //
    connection.pc1.sp.canBeJoined = false;
    connection.pc1.sp.canBeJoined = false;

    return true;

}
