#include "stdafx.h"
#include "DividerOptimalConnector.h"
#include "Divider.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshStreams.h"
#include "MeshStreamsJoiner.h"

const MeshLogicOptions_DividerOptimalConnector& options = meshLogicOptions.DividerOptimalConnector;


DividerOptimalConnector::DividerOptimalConnector(ViewerDrawObjects& _draw, SSMeshes& _ss)
    : draw(_draw), ss(_ss)
{

}

void DividerOptimalConnector::Solve()
{
    CalculateInitialDivs();

    //
    // Merge connection conflicts
    //
    JoinConnections();

    //
    // Join conflicted loops
    //
    if (options.JoinConflictedLoops)
    {
        JoinConflictedLoops();
    }


    //
    // Join mesh streams with dividing points on connections
    //
    if (options.JoinStreamsWithDividingPoints)
    {
        JoinStreamsWithDividingPoints();
    }

    //
    // DEBUG
    //
    DrawDebug();
}

void DividerOptimalConnector::CalculateInitialDivs()
{
    for (auto& mesh : ss.meshes)
    {
        for (auto& loop : mesh.quadLoops)
        {
            auto& segment01 = loop.segment01;
            auto& div01 = loop.div01;
            auto& meshSize = ss.meshSize;
            switch (options.DividingType)
            {
                case MeshLogicOptions_DividerOptimalConnector::LoopDividingType::Min:
                {
                    int minDiv = min(int(utils::num::round(segment01[0]->Length3d / meshSize)), int(utils::num::round(segment01[1]->Length3d / meshSize)));
                    if (minDiv == 0) minDiv = 1;
                    div01[0] = minDiv;
                    div01[1] = minDiv;
                }
                break;

                case MeshLogicOptions_DividerOptimalConnector::LoopDividingType::Avarage:
                {
                    int avgDiv = int(utils::num::round((segment01[0]->Length3d + segment01[1]->Length3d) / 2 / meshSize));
                    if (avgDiv == 0) avgDiv = 1;
                    div01[0] = avgDiv;
                    div01[1] = avgDiv;
                }
                break;

                case MeshLogicOptions_DividerOptimalConnector::LoopDividingType::Max:
                {
                    int maxDiv = max(int(utils::num::round(segment01[0]->Length3d / meshSize)), int(utils::num::round(segment01[1]->Length3d / meshSize)));
                    if (maxDiv == 0) maxDiv = 1;
                    div01[0] = maxDiv;
                    div01[1] = maxDiv;
                }
                break;


                case MeshLogicOptions_DividerOptimalConnector::LoopDividingType::_1to1:
                {
                    div01[0] = 1;
                    div01[1] = 1;
                }
                break;

                case MeshLogicOptions_DividerOptimalConnector::LoopDividingType::Independed:
                {
                    div01[0] = max(1, int(utils::num::round(segment01[0]->Length3d / meshSize)));
                    div01[1] = max(1, int(utils::num::round(segment01[1]->Length3d / meshSize)));
                }
                break;
            }
        }
    }
}

void DividerOptimalConnector::JoinConnections()
{
    cout << endl << "JoinConnections" << endl;

    int max_conflicts = options.max_conflicts;
    auto findNextConflict = [&](SSTopologyConnection_Conflict*& conflict)
    {
        conflict = nullptr;
        int best_maxDivBy = -1;
        for (auto& c : ss.allConnections)
        {
            c.conflict.Calculate();
            if (c.conflict.status == SSTopologyConnection_Conflict::Status::Found
                && c.conflict.resolvesTimesCount < max_conflicts) // if we cant resolve conflict since it returns back as cyclic - then skip resolving and let mesh have some gaps - this is better then hanging application in forever-loop
            {
                if (conflict == nullptr || best_maxDivBy > c.conflict.max_divBy)
                {
                    conflict = &c.conflict;
                    best_maxDivBy = c.conflict.max_divBy;
                }
            }
        }
        return conflict != nullptr;
    };

    SSTopologyConnection_Conflict* conflict;
    int iterationNum = 0;
    while (findNextConflict(conflict))
    {
        bool debug_iteration = options.DebugEnabled && options.debug_debug_iteration_num >= iterationNum;
        conflict->ResolveConflict(options.max_conflicts, iterationNum, debug_iteration, draw);
        iterationNum++;
    }
}

void DividerOptimalConnector::JoinConflictedLoops()
{
    cout << endl << "JoinConflictedLoops" << endl;
    ConflictedLoops_Build();
    if (options.JoinConflictedLoops_ReserveSpaceForSmallInBig)
    {
        int reserveSpace_iteration_num = 0;
        while (ConflictedLoops_ReserveSpaceForSmallInBig(reserveSpace_iteration_num))  // reserve space for small conflict loops in big lops - to make 2 loops from 1 loop
        {
            reserveSpace_iteration_num++;
            if (options.debug_ReserveSpaceForSmallInBig_max_iterations_count != -1 && reserveSpace_iteration_num > options.debug_ReserveSpaceForSmallInBig_max_iterations_count)
            {
                cout << "stoping reserveSpace iterations at step defined in debug options." << endl;
                break;
            }

            JoinConnections(); // rebuild conflicts - we have to rebuild all, since we cant predict what loop influence another after reseved space
            ConflictedLoops_Build(); // rebuild conflict loops
        }
    }
}

void DividerOptimalConnector::JoinStreamsWithDividingPoints()
{
    cout << endl << "JoinStreamsWithDividingPoints" << endl;

    D save_max_stream_angle_diff = meshLogicOptions.StreamAdjuster.max_stream_angle_diff; // remember 'max_stream_angle_diff'
    meshLogicOptions.StreamAdjuster.max_stream_angle_diff = 80; // increase max angle diff to be able join very curved joins

    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < ss.meshes.size(); i++)
    {
        auto& mesh = ss.meshes[i];
        // add anchorPoints and streamIntersectionPoints
        mesh.JoinStreamsWithDividingPoints();
        // divide those added points preserving existed streams
        mesh.divider.Divide(false, 0); // 0 means - preserve previous streams - speed optimization
        // clear 'dividingIteration' flag to allow join newly added points with existed streams
        for (auto& s : mesh.divider.streams.streams) s.dividingIteration.iterationNum = 0;
        for (auto& ap : mesh.divider.anchorPoints) ap.dividerIterator_iterationNum = 0;
        for (auto& sp : mesh.divider.startPoints) sp.dividingIteration.iterationNum = 0;
        // join newly added streams with existed streams
        MeshStreamsJoiner joiner(mesh.divider, ss.meshSize);
        joiner.JoinStreams();
    }

    meshLogicOptions.StreamAdjuster.max_stream_angle_diff = save_max_stream_angle_diff;  // restore 'max_stream_angle_diff'
}

void DividerOptimalConnector::ConflictedLoops_Build()
{
    //
    // clear conflictedLoops
    //
    for (SSTopologyConnection& con : ss.allConnections) // for each connection in all meshes
    {
        for (int si = 0; si < con.segmentsCount; si++) // for each segment that is in this connection
        {
            SSTopologySegment& segment = con.segments[si];
            for (SSQuadsLoop_Side* loopSide : segment.loopSides) // for each loop connected to this segment
            {
                loopSide->ReinitConflictJoins(); // reinit data
            }
        }
    }


    //
    // Connect loops between meshes
    //
    conflictedLoops.clear();
    conflictedLoops.reserve(ss.meshes.size());

    auto getOppositeJoin = [](SSQuadsLoop_Side_ConflictJoin* join)
    {
        SSQuadsLoop_Side_ConflictJoin* none = nullptr;

        // get opposite side
        SSQuadsLoop_Side& oppositeSide = join->side.oppositeSide();
        if (oppositeSide.conflictJoins.size() != join->side.conflictJoins.size())
        {
            //cout << "           oppositeJoin.id=" << "   not found!" << endl;
            return none;
        }

        // get coresponding opposite join in opposite side
        //cout << "           oppositeJoin.id   search" << endl;
        int oppositeIndex = oppositeSide.conflictJoins.size() - (join->divIndex + 1);
        //cout << "           oppositeJoin.id   search for oppositeIndex=" << oppositeIndex << endl;
        SSQuadsLoop_Side_ConflictJoin* oppositeJoin = &oppositeSide.conflictJoins[oppositeIndex];
        //cout << "           oppositeJoin.id=" << oppositeJoin->id << endl;
        return oppositeJoin;
    };
    auto getFriendJoin = [](SSQuadsLoop_Side_ConflictJoin* join)
    {
        SSQuadsLoop_Side_ConflictJoin* none = nullptr;
        if (join->side.isTriangleStop) //  triangles doesnt have friends
        {
            //cout << "                   --getFriendJoin          join->side.isTriangleStop" << endl;
            return none;
        }

        // get segment for this join - each join should be defined for segment
        auto segment = join->side.segment()->ssSegment;
        if (segment == nullptr)
        {
            //cout << "                   --getFriendJoin          segment == nullptr" << endl;
            cout << "! wrong:   BuildConflictedLoops::addJoin()   segment == nullptr - each loop should end at topology segment" << endl;
            return none;
        }

        // get joinIndex
        int divIndexInSegment = join->GetDivIndexInSegment(); // get index of join for segment for which it belongs
        if (divIndexInSegment == -1)
        {
            //cout << "                   --getFriendJoin          divIndexInSegment == -1" << endl;
            return none;
        }

        // find friend join on friend mesh
        if (segment->Friend == nullptr)
        {
            //cout << "                   --getFriendJoin          segment->Friend == nullptr" << endl;
            return none;
        }
        SSQuadsLoop_Side_ConflictJoin* friendJoin = segment->Friend->GetJoinByIndex(divIndexInSegment, true);
        //cout << "                   --getFriendJoin          friendJoin found" << endl;
        return friendJoin;
    };
    auto addJoin = [](SSMeshConflictedLoop& conflictedLoop, SSQuadsLoop_Side_ConflictJoin* join)
    {
        if (join->isTaken) return false;

        // add this join
        join->isTaken = true;
        conflictedLoop.joins.push_back(join);
        //cout << "BuildConflictedLoops take:  loop" << join->side.loop->id << ", side" << join->side.sideIndex << ", join" << join->divIndex << endl;
        //cout << "BuildConflictedLoops take:  loop" << join->side.loop->id << endl;
        return true;
    };
    auto addJoinBranch = [&](SSMeshConflictedLoop& conflictedLoop, SSQuadsLoop_Side_ConflictJoin* join, int& addedCount, bool isDebugThisConflictedLoop)
    {
        if (isDebugThisConflictedLoop)
        {
            cout << "Enter  addJoinBranch() method at meshid=" << join->meshid() << endl;
        }
        while (join != nullptr)
        {
            // add current join
            if (!addJoin(conflictedLoop, join)) break;
            addedCount++;
            if (isDebugThisConflictedLoop) cout << "    added loop with length " << join->side.loop->quads.size() << " in meshid" << join->meshid() << endl;

            // find friend join
            SSQuadsLoop_Side_ConflictJoin* friendJoin = getFriendJoin(join);
            if (friendJoin == nullptr) break;
            //if (isDebugThisConflictedLoop) cout << "    friend loop with length " << friendJoin->side.loop->quads.size() << " in meshid" << friendJoin->meshid() << endl;

            // add friend join
            if (!addJoin(conflictedLoop, friendJoin)) break;
            addedCount++;

            // find opposite join for friend join to continue a loop
            join = getOppositeJoin(friendJoin);
        }
    };

    for (SSTopologyConnection& con : ss.allConnections) // for each connection in all meshes
    {
        //cout << "con.Index=" << con.Index << endl;
        if (con.segmentsCount == 2 && con.segments[0].GetSumDivBy() != con.segments[1].GetSumDivBy()) continue; // ignore unresolved conflicts
        for (int si = 0; si < con.segmentsCount; si++) // for each segment (mesh) that is in this connection
        {
            SSTopologySegment& segment = con.segments[si];
            //cout << "   segment.GetSumDivBy()=" << segment.GetSumDivBy() << endl;
            for (SSQuadsLoop_Side* loopSide : segment.loopSides) // for each loop registered in this segment
            {
                for (SSQuadsLoop_Side_ConflictJoin& join : loopSide->conflictJoins) // for each join registered in this loop
                {
                    SSQuadsLoop_Side_ConflictJoin* joinOpposite = join.opposite();
                    if (join.isTaken && (joinOpposite == nullptr || joinOpposite->isTaken)) continue; // skip already taken joins - speed optimization

                    conflictedLoops.push_back(SSMeshConflictedLoop(conflictedLoops.size())); // allocate conflictedLoop to add later in it loopSide's
                    SSMeshConflictedLoop& conflictedLoop = conflictedLoops.back();

                    bool isDebugThisConflictedLoop = options.DebugEnabled && options.debug_show_conflictedLoops && options.debug_debug_conflictedLoop_num != -1 && options.debug_debug_conflictedLoop_num == conflictedLoop.Index;

                    int addedCount = 0;
                    addJoinBranch(conflictedLoop, &join, addedCount, isDebugThisConflictedLoop); //add all connected loops on one side
                    if (joinOpposite != nullptr)
                    {
                        if (conflictedLoop.joins.size() != 0 && conflictedLoop.joins.back() == joinOpposite) // if we have cyclic loop,
                        {
                            conflictedLoop.isCyclic = true;
                            conflictedLoop.joins.insert(conflictedLoop.joins.begin(), joinOpposite); // move last element to first position
                            conflictedLoop.joins.pop_back(); // move last element to first position
                        }
                        else
                        {
                            if (isDebugThisConflictedLoop) cout << "                   --reverse" << endl;
                            std::reverse(conflictedLoop.joins.begin(), conflictedLoop.joins.end()); // reverse joins, since we want add from another side
                            addJoinBranch(conflictedLoop, joinOpposite, addedCount, isDebugThisConflictedLoop); //add all connected loops on another side
                        }
                    }
                    if (addedCount == 0)
                    {
                        conflictedLoops.pop_back(); // if we cant add this join (since it is already taken) - then simply cancel allocation
                    }
                    else
                    {
                        //cout << "     conflictedLoop.joins.size() = " << conflictedLoop.joins.size() << endl;
                    }
                }
            }
        }
    }
}

bool DividerOptimalConnector::ConflictedLoops_ReserveSpaceForSmallInBig(int reserveSpace_iteration_num)
{
    const D ACCEPT_RATION = options.JoinConflictedLoops_ReserveSpaceForSmallInBig_acceptRation;
    bool isDebugThisIteration = options.DebugEnabled && options.debug_ReserveSpaceForSmallInBig_max_iterations_count != -1 && options.debug_ReserveSpaceForSmallInBig_max_iterations_count == reserveSpace_iteration_num;
    bool isDebugConflictedLoopNum = options.DebugEnabled && options.debug_debug_conflictedLoop_num != -1;
    bool isDebug = isDebugThisIteration || isDebugConflictedLoopNum;

    struct LoopMinMax
    {
        SSQuadsLoop* loop;
        bool isDebug;
        LoopMinMax(SSQuadsLoop* _loop, bool _isDebug)
            : loop(_loop), isDebug(_isDebug)
        {
        }
        bool isInited()
        {
            return loop != nullptr;
        }
        bool IsReadyToBeMin()
        {
            if (loop->sides[0].conflictJoins.size() > 1 || loop->sides[1].conflictJoins.size() > 1)
            {
                return false;
            }
            if (loop->reserved_length3d > 0) return false;
            D reserved_length3d = max(loop->reserved_length3d, 0.0000000001); // zero division protection
            D maxSegmentLength = max(loop->segment01[0]->Length3d, loop->segment01[1]->Length3d);
            if (isDebug) cout << "IsReadyToBeMin = " << reserved_length3d / maxSegmentLength << "    reserved_length3d=" << reserved_length3d << "   maxSegmentLength=" << maxSegmentLength << endl;
            return reserved_length3d / maxSegmentLength < 0.5;
        }
        bool IsReadyToBeMax()
        {
            if (loop->sides[0].conflictJoins.size() > 1 || loop->sides[1].conflictJoins.size() > 1)
            {
                return false;
            }
            return true;
        }
        D MaxFreeLengthForReserve()
        {
            return max(loop->segment01[0]->Length3d, loop->segment01[1]->Length3d) - loop->reserved_length3d;
        }
        D MinFreeLengthForReserve()
        {
            return min(loop->segment01[0]->Length3d, loop->segment01[1]->Length3d) - loop->reserved_length3d;
        }
    };

    if (isDebug) cout << endl << endl << "---ConflictedLoops_ReserveSpaceForSmallInBig--- ITERATION# " << reserveSpace_iteration_num << endl;
    LoopMinMax bestMin(nullptr, isDebug);
    LoopMinMax bestMax(nullptr, isDebug);
    D bestRatioMaxToMin = 0;
    SSMeshConflictedLoop* bestConflictedLoop = nullptr;

    for (auto& conflictedLoop : conflictedLoops)
    {
        if (isDebugConflictedLoopNum && options.debug_debug_conflictedLoop_num != conflictedLoop.Index) continue;
        //cout << "conflictedLoop.joins.size() = " << conflictedLoop.joins.size() << endl;
        LoopMinMax min(nullptr, isDebug);
        LoopMinMax max(nullptr, isDebug);
        D minFreeLengthForReserve = FLT_MAX;
        for (int i = 0; i < conflictedLoop.joins.size(); i++)
        {
            SSQuadsLoop_Side_ConflictJoin* join0 = conflictedLoop.joins[i];
            if (i != 0 && conflictedLoop.joins[i - 1]->side.loop == join0->side.loop) continue; // skip already processed loop

            LoopMinMax current(join0->side.loop, isDebug);
            minFreeLengthForReserve = std::min(minFreeLengthForReserve, current.MaxFreeLengthForReserve());
            if (current.IsReadyToBeMin() && (!min.isInited() || current.MaxFreeLengthForReserve() < min.MaxFreeLengthForReserve())) // min of max
            {
                min = current;
            }
            if (current.IsReadyToBeMax() && (!max.isInited() || max.MinFreeLengthForReserve() < current.MinFreeLengthForReserve())) // max of min
            {
                max = current;
            }
        }
        if (!min.isInited() || !max.isInited()) continue; // must find min and max loops
        if (min.loop == max.loop) continue; // must be 2 different loops
        if (minFreeLengthForReserve < min.MaxFreeLengthForReserve()) continue; // we cant devide loops if there are no more space for it
        D ratioMaxToMin = max.MinFreeLengthForReserve() / min.MaxFreeLengthForReserve();
        bool accepted = ratioMaxToMin > ACCEPT_RATION;
        if (!bestMin.isInited() || ratioMaxToMin > bestRatioMaxToMin)
        {
            bestMin = min;
            bestMax = max;
            bestRatioMaxToMin = ratioMaxToMin;
            bestConflictedLoop = &conflictedLoop;
        }
        if (isDebug) cout << "ratioMaxToMin=" << ratioMaxToMin << "   minFreeLengthForReserve=" << minFreeLengthForReserve << "    min.MaxFreeLengthForReserve()=" << min.MaxFreeLengthForReserve() << (accepted ? "   accepted" : "") << endl;
    }

    if (isDebug) cout << "bestRatioMaxToMin = " << bestRatioMaxToMin << endl;

    if (bestRatioMaxToMin > ACCEPT_RATION)
    {
        D bestMinLength = bestMin.MaxFreeLengthForReserve();
        if (isDebug) cout << "bestMinLength = " << bestMinLength << "  bestMin.reserved = " << bestMin.loop->reserved_length3d << endl;
        bestConflictedLoop->debugUpdatedReserveSpaceAtIterationNum = reserveSpace_iteration_num;
        for (int i = 0; i < bestConflictedLoop->joins.size(); i++)
        {
            SSQuadsLoop_Side_ConflictJoin* join0 = bestConflictedLoop->joins[i];
            if (i != 0 && bestConflictedLoop->joins[i - 1]->side.loop == join0->side.loop) continue; // skip already processed loop

            SSQuadsLoop* currentLoop = join0->side.loop;
            LoopMinMax current(currentLoop, currentLoop);
            D currentRationMaxToMin = current.MinFreeLengthForReserve() / bestMinLength;
            if (isDebug) cout << "   currentRationMaxToMin = " << currentRationMaxToMin << "    current.reservedLegnth = " << current.loop->reserved_length3d << endl;
            if (currentRationMaxToMin > ACCEPT_RATION)
            {
                currentLoop->div01[0]++;
                currentLoop->div01[1]++;
                currentLoop->reserved_length3d += bestMinLength;
            }
            else
            {
                currentLoop->reserved_length3d += bestMinLength;
            }
        }
        ConflictedLoops_ReserveSpaceForSmallInBig(reserveSpace_iteration_num); //  recursive call, to reserve space for others
        return true;
    }

    return false;
}

void DividerOptimalConnector::DrawDebug()
{
    //
    // DEBUG - show loops
    //
    if (options.DebugEnabled && options.debug_show_conflictedLoops)
    {
        int addcount = 0;
        for (auto& m : ss.meshes) addcount += m.quadLoops.size();
        draw.ReserveEdges(addcount * 5);
        draw.ReserveLabels(addcount * 2);
        bool isDebugConflictedLoopNum = options.DebugEnabled &&  options.debug_debug_conflictedLoop_num != -1;
        bool isDebugThisIteration = options.DebugEnabled && options.debug_ReserveSpaceForSmallInBig_max_iterations_count != -1;
        int debugConflictedLoopIndex = -1;
        if (isDebugConflictedLoopNum) debugConflictedLoopIndex = options.debug_debug_conflictedLoop_num;
        if (isDebugThisIteration) for (auto& cl : conflictedLoops) if (cl.debugUpdatedReserveSpaceAtIterationNum == options.debug_ReserveSpaceForSmallInBig_max_iterations_count)  debugConflictedLoopIndex = cl.Index;
        for (auto& conflictedLoop : conflictedLoops)
        {
            if (debugConflictedLoopIndex != -1 && debugConflictedLoopIndex != conflictedLoop.Index) continue;
            //cout << "conflictedLoop.joins.size() = " << conflictedLoop.joins.size() << endl;
            for (int i = 0; i < conflictedLoop.joins.size(); i++) // draw edge for [0-1, 2-3, 4-5, ...] - size have to be an even value
            {
                if (i + 1 >= conflictedLoop.joins.size()) break; // out of scope protection
                SSQuadsLoop_Side_ConflictJoin* join0 = conflictedLoop.joins[i];
                SSQuadsLoop_Side_ConflictJoin* join1 = conflictedLoop.joins[i + 1];
                if (join0->side.sideIndex == 1) swap(join0, join1); // force first side to be first

                // skip loop for triangle-quad
                if (join0->side.loop->id != join1->side.loop->id)
                {
                    continue;
                }
                // v0 - draw 1 line
                //P3 p0 = join0->GetMidPoint();
                //P3 p1 = join1->GetMidPoint();
                //draw.AddEdge(p0, p1, Color3d(0, 0, 1));
                //draw.AddLabel((p0 + p1) / 2, "conflictLoop#" + to_string(conflictedLoop.Index), Color3d(0, 0, 1));

                //v1 - draw few lines
                auto& loop = *join0->side.loop;
                auto color = (loop.div01[0] != loop.div01[1]) ? Color3d(1, 0, 0) : Color3d(0, 0, 1);
                for (int qi = 0; qi < loop.quads.size(); qi++)
                {
                    auto& quad = loop.quads[qi];
                    P3 p0 = quad.segment0->PointMid(false);
                    P3 p1 = quad.segment1->PointMid(false);
                    if (qi == 0) p0 = join0->GetMidPoint();
                    if (qi == loop.quads.size() - 1) p1 = join1->GetMidPoint();
                    draw.AddEdge(p0, p1, color);
                    string text = "" + to_string(conflictedLoop.Index) + "(" + to_string(loop.id) + ")";
                    draw.AddLabel((p0 + p1) / 2, text, color);
                    draw.AddLabel(p0, text, color);
                    draw.AddLabel(p1, text, color);
                }
            }
        }
    }



    //
    // DEBUG - show unresolved conflicts
    //
    if (options.DebugEnabled && options.debug_show_unresolved_conflicts)
    {
        auto color = Color3d(1, 0, 0);
        int addcount = 0;
        for (auto& connection : ss.allConnections)
        {
            if (connection.segments[0].GetSumDivBy() != connection.segments[1].GetSumDivBy())
            {
                addcount++;
            }
        }
        draw.ReserveLabels(addcount * 2);
        for (auto& connection : ss.allConnections)
        {
            if (connection.connection.segments.size() < 2) continue;
            if (connection.segments[0].GetSumDivBy() != connection.segments[1].GetSumDivBy())
            {
                string text = connection.GetDivByText();
                draw.AddLabel(connection.connection.middlePoint, text, color, 5);
            }
        }

        for (auto& m : ss.meshes) addcount += m.quadLoops.size();
        draw.ReserveLabels(addcount);
        draw.ReserveEdges(addcount);
        for (auto& mesh : ss.meshes)
        {
            for (auto& loop : mesh.quadLoops)
            {
                if (loop.div01[0] != loop.div01[1])
                {
                    for (auto& quad : loop.quads)
                    {
                        draw.AddEdge(quad.segment0->PointMid(true), quad.segment1->PointMid(true), color);
                    }
                    draw.AddLabel(loop.segment01[0]->PointMid(true), to_string(loop.div01[0]), color);
                    draw.AddLabel(loop.segment01[1]->PointMid(true), to_string(loop.div01[1]), color);
                }
            }
        }
    }

    //
    // DEBUG - show dividing points for joins
    //
    if (options.DebugEnabled && options.debug_show_dividingPointsForJoins)
    {
        for (SSTopologyConnection& connection : ss.allConnections)
        {
            if (connection.segmentsCount == 0) continue;
            vector<SSTopologySegment_DividingPoint> divPoints[2];
            divPoints[0] = connection.segments[0].mesh->GetDividingPointsForJoins(connection.segments[0]);
            if (connection.segmentsCount == 2)
            {
                divPoints[1] = connection.segments[1].mesh->GetDividingPointsForJoins(connection.segments[1]);
            }
            // merge points if possible
            if (divPoints[0].size() == divPoints[1].size())
            {
                for (int i = 0; i < divPoints[0].size(); i++)
                {
                    int i2 = divPoints[0].size() - 1 - i;
                    if (divPoints[1][i2].segmentOrtogonal != nullptr)
                    {
                        divPoints[0][i].segmentOrtogonal = divPoints[1][i2].segmentOrtogonal;
                    }
                }
                divPoints[1].clear(); // we merged points into divPoints[0]
            }

            for (int k = 0; k < connection.segmentsCount; k++)
            {
                if (divPoints[k].size() == 0) continue;
                auto& segment = connection.segments[0].curveSegment;
                for (int i = 1; i < divPoints[k].size() - 1; i++) // excluding first and last points, since theay are start and end of the segment, and we have already points on segments
                {
                    Color3d color = divPoints[k][i].segmentOrtogonal == nullptr ? Color3d(0, 0.5, 0) : Color3d(1, 0, 0);
                    P3 point = segment->GetPointAtLength(divPoints[k][i].pos);
                    draw.AddPoint(point, color);
                }
            }
        }
    }
}
