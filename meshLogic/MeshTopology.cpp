#include "stdafx.h"
#include "MeshTopology.h"
#include "ModelObject.h"
#include "Mesh.h"
#include "MeshSurface.h"

const MeshLogicOptions_MeshesTopology& options = meshLogicOptions.MeshesTopology;

atomic_int TopologyPoint::nextTopologyPointID;

TopologyPoint::TopologyPoint(int index)
    : id(nextTopologyPointID++), Index(index)
{
    int temp = 0;
}
II TopologyPoint::SizeOF() const
{
    II r = sizeof(TopologyPoint);
    r += segments.size() * sizeof(MeshTopologyLoopCurveSegment*);
    return r;
}

void TopologyPoint::Update()
{
    point = P3(0, 0, 0);
    int count = segments.size();
    assert(count > 0);
    if (count == 0) return;
    for (auto& s : segments)
    {
        if (s->isCyclic) continue; // skip cyclic segment since they dont have 'topologyPointStart' and 'topologyPointEnd'
        assert(s->topologyPointStart != nullptr);
        assert(s->topologyPointEnd != nullptr);
        assert(s->topologyPointStart == this || s->topologyPointEnd == this);
        if (s->topologyPointStart == this)
            point += s->pointStart;
        else  //important, since cyclic curves can have same topologyPoint for 'start' and 'end'
            point += s->pointEnd;
    }
    point /= (D)count;
    color = Color();
    if (id == 20)
    {
        int temp = 0;
    }
}



Color3d TopologyPoint::Color()
{
    const Color3d& outherColor = meshLogicOptions.Draw.Mesh.TopologyConnection_Color_Outher;
    const Color3d& innerColor = meshLogicOptions.Draw.Mesh.TopologyConnection_Color_Inner;
    const Color3d& connectedColor = meshLogicOptions.Draw.Mesh.TopologyConnection_Color_Connected;

    bool isPointOnBorder = false;
    for (auto& s : segments)
    {
        if (s->ConnectedTo == nullptr)
        {
            isPointOnBorder = true;
            break;
        }
    }
    return isPointOnBorder
        ? segments[0]->curve.loop.Type == MeshLoopType::Outher ? outherColor : innerColor
        : connectedColor;
}



atomic_int TopologyConnection::nextTopologyConnectionID;

TopologyConnection::TopologyConnection(int index)
    : id(nextTopologyConnectionID++), Index(index)
{
}

bool TopologyConnection::IsSegmentReverted(int segmentIndex) const
{
    if (segments.size() < 2) return false;
    return segments[0].IsOppositeDirections(segments[segmentIndex]);
}

II TopologyConnection::SizeOF() const
{
    II r = sizeof(TopologyConnection);
    r += segments.size() * sizeof(MeshTopologyLoopCurve);
    for (const auto& s : segments)
        r += s.SizeOF();
    return r;
}

void TopologyConnection::Update()
{
    // set 'connectionIndex' for all segments
    for (auto& s : segments)
    {
        s.topologyConnection = this;
        s.IndexInTopologyConnections = Index;
    }

    int count = segments.size();

    assert(count > 0);
    isPointsOptimizedToLine = false;
    if (count == 1)
    {
        // set connection between segments
        segments[0].ConnectedTo = nullptr;// clear connection since no friend found

        // take points of first curve
        points = segments[0].GetPointsForDraw(isPointsOptimizedToLine);
        middlePoint = segments[0].MiddlePoint();
    }
    else
    {
        // set connection between segments
        assert(count == 2 && "currenly supported only connection between 2 curves, manifold connection are not supported");
        segments[0].ConnectedTo = &segments[1];
        segments[1].ConnectedTo = &segments[0];

        // v0
        //// sort curves, that smallest will be first
        //sort(curves.begin(), curves.end(), [](const MeshTopologyLoopCurveSegment& c1, const MeshTopologyLoopCurveSegment& c2)
        //{
        //    return c1.Count < c2.Count;
        //});
        ////vector<unsigned> sorted_indexes = utils::stdvector::sort_indexes_custom(curves.size(), [&](unsigned int i1, unsigned int i2)
        ////{
        ////    const MeshTopologyLoopCurveSegment& c1 = curves[i1];
        ////    const MeshTopologyLoopCurveSegment& c2 = curves[i2];
        ////    return c1.Count < c2.Count;
        ////});
        // v1 - no need in sort, since segments will be always of same length
        // none

        // take points of first curve, since it is smallest after sort
        auto smallestCurve = segments[0];
        points = smallestCurve.GetPointsForDraw(isPointsOptimizedToLine);
        middlePoint = smallestCurve.MiddlePoint();
    }
    color = Color();
}

const MeshTopologyLoopCurveSegment& TopologyConnection::getOppositeSegment(const MeshTopologyLoopCurveSegment& segment) const
{
    for (const auto& s : segments)
    {
        if (s.id != segment.id) return s;
    }
    assert(false && "getOppositeSegment failed");
    return getOppositeSegment(segment); // fail here - 
}

int TopologyConnection::findSegmentIndex(const MeshTopologyLoopCurveSegment& segment) const
{
    for (int i = 0; i < segments.size(); i++)
    {
        const auto& s = segments[i];
        if (s.id == segment.id)
        {
            return i;
        }
    }
    cout << "!!! wrong    TopologyConnection::findSegmentIndex  not found segment" << endl;
    assert(false && "TopologyConnection::findSegmentIndex  not found segment");
    return -1;
}

Color3d TopologyConnection::Color()
{
    const Color3d& outherColor = meshLogicOptions.Draw.Mesh.TopologyConnection_Color_Outher;
    const Color3d& innerColor = meshLogicOptions.Draw.Mesh.TopologyConnection_Color_Inner;
    const Color3d& connectedColor = meshLogicOptions.Draw.Mesh.TopologyConnection_Color_Connected;
    int count = segments.size();
    switch (count)
    {
        case 0:
            return Color3d(0, 0, 0);
        case 1:
            return segments[0].curve.loop.Type == MeshLoopType::Outher ? outherColor : innerColor;
        default:
            return connectedColor;
    }
}



II Topology::SizeOF() const
{
    II r = sizeof(Topology);

    r += Points.size() * sizeof(TopologyPoint);
    for (const auto& p : Points)
        r += p.SizeOF();

    r += Connections.size() * sizeof(TopologyConnection);
    for (const auto& c : Connections)
        r += c.SizeOF();

    r += draw.SizeOF();
    r += last_Update_for_Objects_Ids.size() * sizeof(int);
    return r;
}

void sortcurves_by_count(const vector<MeshTopologyLoopCurveSegment>& curves_unsorted, vector<MeshTopologyLoopCurveSegment>& curves_sorted)
{
    vector<unsigned> sorted_indexes = utils::stdvector::sort_indexes_custom(curves_unsorted.size(), [&](unsigned int i1, unsigned int i2)
    {
        const MeshTopologyLoopCurveSegment& c1 = curves_unsorted[i1];
        const MeshTopologyLoopCurveSegment& c2 = curves_unsorted[i2];
        //if (c1.Count == c2.Count)
        //{
        //    return c1.middlePoint_DistToZeroPow2 < c2.middlePoint_DistToZeroPow2;
        //}
        return c1.PointsCount < c2.PointsCount;
    });
    curves_sorted.clear();
    curves_sorted.reserve(curves_unsorted.size());
    for (auto sortedIndex : sorted_indexes)
    {
        curves_sorted.push_back(curves_unsorted[sortedIndex]);
    }
}

void sortcurves_by_count_and_pseudomid(const vector<MeshTopologyLoopCurveSegment>& curves_unsorted, vector<MeshTopologyLoopCurveSegment>& curves_sorted)
{
    vector<unsigned> sorted_indexes = utils::stdvector::sort_indexes_custom(curves_unsorted.size(), [&](unsigned int i1, unsigned int i2)
    {
        const MeshTopologyLoopCurveSegment& c1 = curves_unsorted[i1];
        const MeshTopologyLoopCurveSegment& c2 = curves_unsorted[i2];
        if (c1.PointsCount == c2.PointsCount)
        {
            return c1.positionInPseudo3dMid < c2.positionInPseudo3dMid;
        }
        return c1.PointsCount < c2.PointsCount;
    });
    curves_sorted.clear();
    curves_sorted.reserve(curves_unsorted.size());
    for (auto sortedIndex : sorted_indexes)
    {
        curves_sorted.push_back(curves_unsorted[sortedIndex]);
    }
}

vector<MeshTopologyLoopCurveSegment> getUnusedCurves(const vector<MeshTopologyLoopCurveSegment>& curves, const Bs& isAlreadyUsed)
{
    vector<MeshTopologyLoopCurveSegment> unusedCurves;
    unusedCurves.reserve(curves.size());
    for (int i = 0; i < curves.size(); i++)
    {
        if (i <= isAlreadyUsed.size() - 1 && isAlreadyUsed(i)) continue;
        const MeshTopologyLoopCurveSegment& c = curves[i];
        unusedCurves.push_back(c);
    }
    return unusedCurves;
}

D getMinDistBetweenStartEnd(const MeshTopologyLoopCurveSegment& c, const MeshTopologyLoopCurveSegment& c2, bool& isreversed_c, bool& isreversed_c2)
{
    D minPointsDistPow2_SS = utils::point::DistToPointPow2(c.pointStart, c2.pointStart);
    D minPointsDistPow2_SE = utils::point::DistToPointPow2(c.pointStart, c2.pointEnd);

    D minPointsDistPow2 = minPointsDistPow2_SS;
    isreversed_c = false;
    isreversed_c2 = false;

    if (minPointsDistPow2_SE < minPointsDistPow2)
    {
        isreversed_c = false;
        isreversed_c2 = true;
        minPointsDistPow2 = minPointsDistPow2_SE;
    }

    if (c.PointsCount != c2.PointsCount) // if length of c and c2 differs - we have to check 2 other cases, where curves contacted at 'c' end (since 'c' is always smaller from 'c2')
    {
        D minPointsDistPow2_ES = utils::point::DistToPointPow2(c.pointEnd, c2.pointStart);
        D minPointsDistPow2_EE = utils::point::DistToPointPow2(c.pointEnd, c2.pointEnd);
        if (minPointsDistPow2_ES < minPointsDistPow2)
        {
            isreversed_c = true;
            isreversed_c2 = false;
            minPointsDistPow2 = minPointsDistPow2_ES;
        }
        if (minPointsDistPow2_EE < minPointsDistPow2)
        {
            isreversed_c = true;
            isreversed_c2 = true;
            minPointsDistPow2 = minPointsDistPow2_EE;
        }
    }
    return minPointsDistPow2;
}

bool isInternalPointsMatch(const MeshTopologyLoopCurveSegment& c, const MeshTopologyLoopCurveSegment& c2, bool isreversed_c, bool isreversed_c2, D tolPow2, D& current_sumOfdistDifferences)
{
    current_sumOfdistDifferences = 0;
    int minCount = min(c.PointsCount, c2.PointsCount); // we check only smaller length from both 'c' and 'c2' - the common length where curves can have same 3d position
    int pi_inc = max(1, minCount / 8); // check maximum 10 points

    //DEBUG - show reversed flags

    // check if this is standard case - in such case we can use simplified algorithm (very small improvement like 2% but still faster)
    if (c.PointsCount == c2.PointsCount && !isreversed_c && isreversed_c2 && !c.curve.isCyclic && !c2.curve.isCyclic)
    {
        //cout << "isreversed_c=" << isreversed_c << "   isreversed_c2=" << isreversed_c2 << endl;
        const P3s& cPoints = c.curve.points;
        int cShift = c.fromIndex;
        const P3s& c2Points = c2.curve.points;
        int c2Shift = c2.fromIndex + c2.PointsCount - 1;
        for (int pi = 0; pi < minCount; /*empty, we increment 'pi' inside loop at end of the loop*/)// check smaller amount of 2 curves ('c' is always smaller from 'c2' since we have sorted array by 'Count' criteria)
        {
            P3 cPoint = cPoints[cShift + pi];
            P3 c2Point = c2Points[c2Shift - pi];
            D distPow2 = utils::point::DistToPointPow2(cPoint, c2Point);
            current_sumOfdistDifferences += distPow2;
            if (distPow2 > tolPow2)
            {
                return false;
            }
            if (pi == minCount - 1) break; // to avoid forever loop thanks to 2 lines below, we have to break it manually when we tested last point
            pi += pi_inc; // increment loop index manually, to be able to check last point 
            if (pi > minCount - 1) pi = minCount - 1; // we have to ensure that last point will be checked
        }
    }
    else
    {
        //cout << "!!!  isreversed_c=" << isreversed_c << "   isreversed_c2=" << isreversed_c2 << endl;
        for (int pi = 0; pi < minCount; /*empty, we increment 'pi' inside loop at end of the loop*/)// check smaller amount of 2 curves ('c' is always smaller from 'c2' since we have sorted array by 'Count' criteria)
        {
            P3 cPoint = c.Point(!isreversed_c ? pi : (c.PointsCount - 1 - pi));
            P3 c2Point = c2.Point(!isreversed_c2 ? pi : (c2.PointsCount - 1 - pi));
            D distPow2 = utils::point::DistToPointPow2(cPoint, c2Point);
            current_sumOfdistDifferences += distPow2;
            if (distPow2 > tolPow2)
            {
                return false;
            }
            if (pi == minCount - 1) break; // to avoid forever loop thanks to 2 lines below, we have to break it manually when we tested last point
            pi += pi_inc; // increment loop index manually, to be able to check last point 
            if (pi > minCount - 1) pi = minCount - 1; // we have to ensure that last point will be checked
        }
    }
    return true;
}

void AddConnection(int i1, int i2, bool isreversed_c2, const vector<MeshTopologyLoopCurveSegment>& curves, vector<MeshTopologyLoopCurveSegment>& curves_added, Bs& isAlreadyUsed, vector<TopologyConnection>& Connections)
{
    if (i2 == -1) return;
    isAlreadyUsed(i1) = true; // mark curve 'c' as used
    isAlreadyUsed(i2) = true; // mark curve 'c2' as used

    const MeshTopologyLoopCurveSegment& c1 = curves[i1];
    const MeshTopologyLoopCurveSegment& c2 = curves[i2];

    Connections.push_back(TopologyConnection(Connections.size()));
    auto& connection = Connections.back();
    connection.segments.push_back(c1);

    if (c1.PointsCount != c2.PointsCount) // if length of c and c2 differs - we have to add segment and rest of curve leave also as segment
    {
        int startIndex = c2.fromIndex;
        int cutIndex = c2.fromIndex + c1.PointsCount - 1;
        int endIndex = c2.toIndex;
        if (isreversed_c2)
        {
            cutIndex = c2.toIndex - c1.PointsCount + 1;
            connection.segments.push_back(MeshTopologyLoopCurveSegment(c2.curve, cutIndex, endIndex)); // add segment of 'c2' to connection
            curves_added.push_back(MeshTopologyLoopCurveSegment(c2.curve, startIndex, cutIndex)); // add rest segment of 'c2' to all available curves for next connections
        }
        else
        {
            connection.segments.push_back(MeshTopologyLoopCurveSegment(c2.curve, startIndex, cutIndex)); // add segment of 'c2' to connection
            curves_added.push_back(MeshTopologyLoopCurveSegment(c2.curve, cutIndex, endIndex)); // add rest segment of 'c2' to all available curves for next connections
        }
    }
    else
    {
        connection.segments.push_back(c2); // add full curve
    }
}


vector<MeshTopologyLoopCurveSegment> findPerfectOrPartialConnections(bool _mustBeOfSameLength, D _tolPercent, const vector<MeshTopologyLoopCurveSegment>& curvesUnsorted, vector<TopologyConnection>& Connections)
{
    // sort curve before main loop
    vector<MeshTopologyLoopCurveSegment> curves;
    sortcurves_by_count(curvesUnsorted, curves);
    bool mustBeOfSameLength = _mustBeOfSameLength;
    D tolPercent = _tolPercent;

    // run main loop
    Bs isAlreadyUsed = Bs::Zero(curves.size());
    int start_curves_size = curves.size(); // we use iMax since size of 'curves' can be extended, but we want to use start value length all the time
    for (int i = 0; i < start_curves_size - 1; i++) // exluding last since we want to take 'i+1'
    {
        if (isAlreadyUsed(i)) continue;
        const MeshTopologyLoopCurveSegment& c = curves[i];
        int c_curve_mesh_id = c.curve.mesh.id;
        int best_i1 = -1;
        int best_i2 = -1;
        bool c2MatchIndex_reversed = false;
        D sumOfdistDifferences = 0;
        D onePercentOfMinLength = c.AvgEdgeLength3d * tolPercent;
        D onePercentOfMinLengthPow2 = onePercentOfMinLength * onePercentOfMinLength;
        for (int i2 = i + 1; i2 < start_curves_size; i2++)
        {
            if (isAlreadyUsed(i2)) continue; // if this curve already taken by some connection - ignore it
            const MeshTopologyLoopCurveSegment& c2 = curves[i2];
            if (mustBeOfSameLength && c.PointsCount != c2.PointsCount) break; // break search if count are already different - we rely on sorted property of 'curves'

            //D onePercentOfMinLength = min(c.Length3d, c2.Length3d)*tolPercent; // not always good
            //D onePercentOfMinLength = min(c.AvgEdgeLength3d, c2.AvgEdgeLength3d)*tolPercent; // more precise, but more sensetive - its ok, since 'inside connection' algorithm will handle anyway, and in this algorithm we have to be very precise
            if (mustBeOfSameLength && (abs(c.Length3d - c2.Length3d) > onePercentOfMinLength)) continue; // if cuves 3d length differs in 1% - ignore it

            // so here: c2 is same length as c, and not used, and from other mesh, and almost same length
            // now compare points - if they very close - curve is perfectly match our curve
            // check if curves match
            // check if curve 'c2' is reversed
            bool isreversed_c = false;
            bool isreversed_c2 = false;
            D minPointsDistPow2 = getMinDistBetweenStartEnd(c, c2, isreversed_c, isreversed_c2);
            if (minPointsDistPow2 > onePercentOfMinLengthPow2) continue;

            // compare points - if they very close - curve is perfectly match our curve
            D current_sumOfdistDifferences = 0;
            bool current_match = isInternalPointsMatch(c, c2, isreversed_c, isreversed_c2, onePercentOfMinLengthPow2, current_sumOfdistDifferences);

            // remember current match
            if (current_match)
            {
                if (c_curve_mesh_id == c2.curve.mesh.id) continue; // if cuve is from same mesh - ignore it

                if (current_sumOfdistDifferences < sumOfdistDifferences || best_i2 == -1) // if new match is closer or not exists
                {
                    best_i1 = i;
                    best_i2 = i2;
                    c2MatchIndex_reversed = isreversed_c2;
                    sumOfdistDifferences = current_sumOfdistDifferences;
                }
            }
        }

        // add connection if found
        AddConnection(best_i1, best_i2, c2MatchIndex_reversed, curves, curves, isAlreadyUsed, Connections);
    }

    // remove used connections
    auto unusedCurves = getUnusedCurves(curves, isAlreadyUsed);

    // return all unused curves(segments)
    return unusedCurves;
}


vector<MeshTopologyLoopCurveSegment> findPerfectConnections_fast(D _tolPercent, const vector<MeshTopologyLoopCurveSegment>& curvesUnsorted, vector<TopologyConnection>& Connections)
{
    vector<MeshTopologyLoopCurveSegment> curvesAdded;
    // sort curve before main loop
    vector<pair<D, int>> sorted_indexes;
    sorted_indexes.reserve(curvesUnsorted.size());
    for (int i = 0; i < curvesUnsorted.size(); i++)
    {
        const MeshTopologyLoopCurveSegment& c = curvesUnsorted[i];
        sorted_indexes.push_back({ c.positionInPseudo3dMid, i });
    }
    // sort all edges by position in pseudo 3d coordinates (to be able later break second loop - speed optimization)
    utils::stdvector::sort(sorted_indexes);

    D tolPercent = _tolPercent;

    // run main loop
    Bs isAlreadyUsed = Bs::Zero(curvesUnsorted.size());
    int start_curves_size = curvesUnsorted.size(); // we use iMax since size of 'curves' can be extended, but we want to use start value length all the time
    for (int i_unsorted = 0; i_unsorted < start_curves_size - 1; i_unsorted++) // exluding last since we want to take 'i+1'
    {
        int i = sorted_indexes[i_unsorted].second;
        if (isAlreadyUsed(i)) continue;
        const MeshTopologyLoopCurveSegment& c = curvesUnsorted[i];
        int best_i1 = -1;
        int best_i2 = -1;
        D best_sumOfdistDifferences = 0;
        D onePercentOfMinLength = c.AvgEdgeLength3d * tolPercent;
        D onePercentOfMinLengthPow2 = onePercentOfMinLength * onePercentOfMinLength;
        for (int i2_unsorted = i_unsorted + 1; i2_unsorted < start_curves_size; i2_unsorted++)
        {
            int i2 = sorted_indexes[i2_unsorted].second;
            if (isAlreadyUsed(i2)) continue; // if this curve already taken by some connection - ignore it
            const MeshTopologyLoopCurveSegment& c2 = curvesUnsorted[i2];
            if (c.PointsCount != c2.PointsCount // break search if count are already different - we rely on sorted property of 'curves'
                || (abs(c.positionInPseudo3dMid - c2.positionInPseudo3dMid) > onePercentOfMinLength)) break;// break search if midpoint are far enought for tol - we rely on sorted property of 'curves'

            //D onePercentOfMinLength = min(c.Length3d, c2.Length3d)*tolPercent; // not always good 
            //D onePercentOfMinLength = min(c.AvgEdgeLength3d, c2.AvgEdgeLength3d)*tolPercent; // more precise, but more sensetive - its ok, since 'inside connection' algorithm will handle anyway, and in this algorithm we have to be very precise
            if (abs(c.Length3d - c2.Length3d) > onePercentOfMinLength) continue; // if cuves 3d length differs in 1% - ignore it

            // check if curve 'c2' is reversed
            bool isreversed_c = false;
            bool isreversed_c2 = false;
            D minPointsDistPow2 = getMinDistBetweenStartEnd(c, c2, isreversed_c, isreversed_c2);
            if (minPointsDistPow2 > onePercentOfMinLengthPow2) continue;

            // compare points - if they very close - curve is perfectly match our curve
            D current_sumOfdistDifferences = 0;
            bool current_match = isInternalPointsMatch(c, c2, false, isreversed_c2, onePercentOfMinLengthPow2, current_sumOfdistDifferences);

            // remember current match
            if (current_match)
            {
                if (c.curve.mesh.id == c2.curve.mesh.id) continue; // if cuve is from same mesh - ignore it

                if (current_sumOfdistDifferences < best_sumOfdistDifferences || best_i2 == -1) // if new match is closer or not exists
                {
                    best_i1 = i;
                    best_i2 = i2;
                    best_sumOfdistDifferences = current_sumOfdistDifferences;
                }
            }
        }

        // add connection if found
        AddConnection(best_i1, best_i2, false, curvesUnsorted, curvesAdded, isAlreadyUsed, Connections);
    }

    // remove used connections
    auto unusedCurves = getUnusedCurves(curvesUnsorted, isAlreadyUsed);

    // return all unused curves(segments)
    return unusedCurves;
}

vector<MeshTopologyLoopCurveSegment> findPartialConnections_fast(D _tolPercent, const vector<MeshTopologyLoopCurveSegment>& curvesUnsorted, vector<TopologyConnection>& Connections)
{
    vector<pair<D, int>> sorted_indexes;
    sorted_indexes.reserve(curvesUnsorted.size() * 2);
    for (int i = 0; i < curvesUnsorted.size(); i++)
    {
        const MeshTopologyLoopCurveSegment& c = curvesUnsorted[i];
        sorted_indexes.push_back({ c.positionInPseudo3dStart2, i }); // is normal
        sorted_indexes.push_back({ c.positionInPseudo3dEnd2, -i }); // is reversed (minus means reversed)
    }
    // sort all edges by position in pseudo 3d coordinates (to be able later break second loop - speed optimization)
    utils::stdvector::sort(sorted_indexes);

    // run main loop
    vector<MeshTopologyLoopCurveSegment> curves_added;
    D tolPercent = _tolPercent;
    Bs isAlreadyUsed = Bs::Zero(curvesUnsorted.size());
    int start_cse_size = sorted_indexes.size();
    for (int i_unsorted = 0; i_unsorted < start_cse_size - 1; i_unsorted++) // exluding last since we want to take 'i+1'
    {
        D pseudo3d1 = sorted_indexes[i_unsorted].first;
        int i1 = sorted_indexes[i_unsorted].second;
        bool isreversed_c = (i1 < 0);
        i1 = abs(i1);
        const MeshTopologyLoopCurveSegment& c = curvesUnsorted[i1];
        if (isAlreadyUsed(i1)) continue;
        int best_i1 = -1;
        int best_i2 = -1;
        bool best_c2MatchIndex_isreversed = false;
        D best_sumOfdistDifferences = 0;
        D onePercentOfMinLength = (isreversed_c ? c.edgeLengthEnd : c.edgeLengthStart)* tolPercent;
        D onePercentOfMinLengthPow2 = onePercentOfMinLength * onePercentOfMinLength;
        for (int i2_unsorted = i_unsorted + 1; i2_unsorted < start_cse_size; i2_unsorted++)
        {
            D pseudo3d2 = sorted_indexes[i2_unsorted].first;
            int i2 = sorted_indexes[i2_unsorted].second;
            bool isreversed_c2 = (i2 < 0);
            i2 = abs(i2);
            if (isAlreadyUsed(i2)) continue; // if this curve already taken by some connection - ignore it
            if (abs(pseudo3d1 - pseudo3d2) > onePercentOfMinLength) break;// break search if start2 or end2 are far enought for tol - we rely on sorted property of 'cse'

            if (curvesUnsorted[i2].PointsCount < curvesUnsorted[i1].PointsCount)
            {
                swap(i1, i2);//shortest segment should by 'se1' and longest 'se2'
                swap(isreversed_c, isreversed_c2);
            }

            // compare points - if they very close - curve is perfectly match our curve
            D current_sumOfdistDifferences = 0;
            bool current_match = isInternalPointsMatch(curvesUnsorted[i1], curvesUnsorted[i2], isreversed_c, isreversed_c2, onePercentOfMinLengthPow2, current_sumOfdistDifferences);

            // remember current match
            if (current_match)
            {
                if (curvesUnsorted[i1].curve.mesh.id == curvesUnsorted[i2].curve.mesh.id) continue; // if cuve is from same mesh - ignore it

                if (current_sumOfdistDifferences < best_sumOfdistDifferences || best_i2 == -1) // if new match is closer or not exists
                {
                    best_i1 = i1;
                    best_i2 = i2;
                    best_c2MatchIndex_isreversed = isreversed_c2;
                    best_sumOfdistDifferences = current_sumOfdistDifferences;
                }
            }
        }

        // add connection if found
        AddConnection(best_i1, best_i2, best_c2MatchIndex_isreversed, curvesUnsorted, curves_added, isAlreadyUsed, Connections);
    }

    // remove used connections
    auto unusedCurves = getUnusedCurves(curvesUnsorted, isAlreadyUsed);

    // add added curves, that was created by removing connected segment from curve
    unusedCurves.reserve(unusedCurves.size() + curves_added.size());
    for (const auto& c : curves_added)
    {
        unusedCurves.push_back(MeshTopologyLoopCurveSegment(c.curve, c.fromIndex, c.toIndex));
    }

    // return all unused curves(segments)
    return unusedCurves;
}


void Topology::AddPerfectConnections(vector<MeshTopologyLoopCurveSegment>& freeCurves)
{
    if (!options.check_PerfectConnections) return;

    int size_before = freeCurves.size();
    if (options.check_PerfectConnections_fast)
        freeCurves = findPerfectConnections_fast(options.tol_PerfectConnections, freeCurves, Connections);
    else
        freeCurves = findPerfectOrPartialConnections(true, options.tol_PerfectConnections, freeCurves, Connections);
    if (options.DebugEnabled)
    {
        cout << "found " << (size_before - freeCurves.size()) / 2 << " perfect connections   (left " << freeCurves.size() << " curves)" << endl;
    }
}

void Topology::AddPartialConnections(vector<MeshTopologyLoopCurveSegment>& freeCurves)
{
    if (!options.check_PartialConnections) return;

    int size_before = freeCurves.size();
    for (auto tolPercent : { options.tol_PartialConnections })
    {
        int currentSizeOfOtherCurves = 0; ;
        while (freeCurves.size() != currentSizeOfOtherCurves && freeCurves.size() != 0)
        {
            currentSizeOfOtherCurves = freeCurves.size();
            if (options.check_PartialConnections_fast)
                freeCurves = findPartialConnections_fast(tolPercent, freeCurves, Connections);
            else
                freeCurves = findPerfectOrPartialConnections(false, tolPercent, freeCurves, Connections);
        }
    }
    if (options.DebugEnabled)
    {
        cout << "found " << (size_before - freeCurves.size()) / 2 << " partial connections   (left " << freeCurves.size() << " curves)" << endl;
    }
}

void Topology::AddInsideConnections(vector<MeshTopologyLoopCurveSegment>& freeCurves)
{
    if (!options.check_InsideConnections) return;

    struct CurveSegmentEdge
    {
        int segmentIndex; // index of segment that holds this edge
        int index;  // index in curve points
        int mesh_id; // curve.mesh.id
        P3 P0; // start point of edge
        P3 PM; // middle point of edge (of P0 and P1)
        P3 P1;// end point of edge
        D Length3d; // length of this edge (pow2 version for better performance)
        D positionInPseudo3d; // position in pseudo 3d coordinates (value is used only for sorting)
    };
    vector<CurveSegmentEdge> edges;
    int edgesConnectedCount = 0;
    D COS_max_angle_between_edges_InsideConnections = cos(options.max_angle_between_edges_InsideConnections);
    map<pair<int, int>, vector<pair<int, int>>>  edgeConnections; //store edges connection for pair: curve-curve2

    //
    // allocate size for all edges
    //
    int edgesTotalSize = 0;
    for (const MeshTopologyLoopCurveSegment& c : freeCurves)
        edgesTotalSize += c.EdgesCount;
    edges.reserve(edgesTotalSize);

    //
    // store all edges in one vector
    //
    vector<pair<D, int>> sorted_indexes_by_positionInPseudo3d;
    sorted_indexes_by_positionInPseudo3d.resize(edgesTotalSize);
    pair<D, int>* p_sorted_indexes_by_positionInPseudo3d = sorted_indexes_by_positionInPseudo3d.data();
    int sorted_indexes_by_positionInPseudo3d__index = 0;
    for (int si = 0; si < freeCurves.size(); si++)
    {
        const MeshTopologyLoopCurveSegment& c = freeCurves[si];
        int mesh_id = c.curve.mesh.id;
        for (int i = 0; i < c.EdgesCount; i++)
        {
            edges.push_back(CurveSegmentEdge());
            CurveSegmentEdge& edge = edges.back();// write directly to vector memory
            edge.segmentIndex = si;
            edge.index = i;
            edge.mesh_id = mesh_id;
            edge.P0 = c.Point(i);
            edge.P1 = c.Point(i + 1);
            edge.PM = (edge.P0 + edge.P1) / 2;
            //edge.Length3d = utils::point::DistToPoint(edge.P0, edge.P1);
            edge.Length3d = c.EdgeLength(i);
            //cout << "c.EdgeLength(i)-utils::point::DistToPoint(edge.P0, edge.P1)=" << c.EdgeLength(i) - utils::point::DistToPoint(edge.P0, edge.P1) << endl;
            edge.positionInPseudo3d = edge.PM(0) + edge.PM(1) + edge.PM(2);
            *p_sorted_indexes_by_positionInPseudo3d = { edge.positionInPseudo3d , sorted_indexes_by_positionInPseudo3d__index };
            p_sorted_indexes_by_positionInPseudo3d++;
            sorted_indexes_by_positionInPseudo3d__index++;
        }
    }

    //
    // sort all edges by position in pseudo 3d coordinates (to be able later break second loop - speed optimization)
    //
    utils::stdvector::sort(sorted_indexes_by_positionInPseudo3d);


    //
    // find best matches for each edge
    //
    struct BestMatch
    {
        D sumOfdistDifferences;
        int e1_segmentIndex;
        int e2_segmentIndex;
        int e1_index;
        int e2_index;
    };
    vector<BestMatch> bestMatches;
    bestMatches.reserve(edges.size());
    for (auto tolPercent : { options.tol_InsideConnections })
    {
        int iMax = edges.size();
        for (int i_unsorted = 0; i_unsorted < iMax - 1; i_unsorted++)   // exluding last since we want to take 'i+1'
        {
            int i = sorted_indexes_by_positionInPseudo3d[i_unsorted].second;
            const CurveSegmentEdge& e = edges[i];
            int best_e2MatchIndex = -1;
            D best_sumOfdistDifferences = 0;
            for (int i2_unsorted = i_unsorted + 1; i2_unsorted < iMax; i2_unsorted++)
            {
                int i2 = sorted_indexes_by_positionInPseudo3d[i2_unsorted].second;
                const CurveSegmentEdge& e2 = edges[i2];
                if (abs(e.positionInPseudo3d - e2.positionInPseudo3d) > e.Length3d) break; // break search if edges are far from each other
                if (e.mesh_id == e2.mesh_id) continue; // if edge is from same mesh - ignore it - have to be after comparing lengths to prevent sloweness for models with a long borders
                D onePercentOfMinLength = min(e.Length3d, e2.Length3d)*tolPercent;
                D onePercentOfMinLengthPow2 = onePercentOfMinLength * onePercentOfMinLength;
                if (abs(e.Length3d - e2.Length3d) > onePercentOfMinLength) continue; // length must be almost same

                // check dists between P0 and P1
                D distP0P0Pow2 = utils::point::DistToPointPow2(e.P0, e2.P0);
                D distP0P1Pow2 = utils::point::DistToPointPow2(e.P0, e2.P1);
                D distP1P1Pow2 = utils::point::DistToPointPow2(e.P1, e2.P1);
                D distP1P0Pow2 = utils::point::DistToPointPow2(e.P1, e2.P0);
                bool ise2reversed = false;
                if (distP0P1Pow2 + distP1P0Pow2 < distP0P0Pow2 + distP1P1Pow2)
                {
                    swap(distP0P0Pow2, distP0P1Pow2);
                    swap(distP1P1Pow2, distP1P0Pow2);
                    ise2reversed = true;
                }
                if (distP0P0Pow2 > onePercentOfMinLengthPow2) continue; // points P0 must be very close
                if (distP1P1Pow2 > onePercentOfMinLengthPow2) continue; // points P0 must be very close

                                                                        // check dists between PM
                D distPMPow2 = utils::point::DistToPointPow2(e.PM, e2.PM);
                if (distPMPow2 > onePercentOfMinLengthPow2) continue; // points P0 must be very close

                                                                      // check angle
                V3 eDir = e.P1 - e.P0;
                V3 e2Dir = e2.P1 - e2.P0;
                if (ise2reversed) e2Dir = -e2Dir;
                if (utils::vector::Cos(eDir, e2Dir) < COS_max_angle_between_edges_InsideConnections) continue;

                bool current_match = true;
                D current_sumOfdistDifferences = distP0P0Pow2 + distPMPow2 + distP1P1Pow2;



                // remember current match
                if (current_match)
                {
                    if (current_sumOfdistDifferences < best_sumOfdistDifferences || best_e2MatchIndex == -1) // if new match is closer or not exists
                    {
                        best_e2MatchIndex = i2;
                        best_sumOfdistDifferences = current_sumOfdistDifferences;
                    }
                }
            }

            // add connection if found
            if (best_e2MatchIndex != -1)
            {
                const CurveSegmentEdge& e2 = edges[best_e2MatchIndex];
                bestMatches.push_back(BestMatch{ best_sumOfdistDifferences,e.segmentIndex,e2.segmentIndex,i,best_e2MatchIndex });
            }
        }
    }

    //
    // add best of the best matches (becuase border curves can pretent on some internal, but internal curve must find their internal friend)
    // but firstly sort them by lowest distances
    //
    vector<unsigned int> sorted_indexes_by_sumOfdistDifferences = utils::stdvector::sort_indexes_custom(bestMatches.size(), [&bestMatches](unsigned int i1, unsigned int i2)
    {
        const BestMatch& a = bestMatches[i1];
        const BestMatch& b = bestMatches[i2];
        return a.sumOfdistDifferences < b.sumOfdistDifferences;
    });
    Bs isAlreadyUsed = Bs::Zero(edges.size());
    for (int i : sorted_indexes_by_sumOfdistDifferences)
    {
        BestMatch& best = bestMatches[i];
        if (isAlreadyUsed(best.e1_index)) continue; // if this edge already taken by some connection - ignore it
        if (isAlreadyUsed(best.e2_index)) continue; // if this edge already taken by some connection - ignore it

        isAlreadyUsed(best.e1_index) = true; // mark edge 'e1' as used
        isAlreadyUsed(best.e2_index) = true; // mark edge 'e2' as used

        edgesConnectedCount += 2; // report that we took 2 edges
        assert(best.e1_segmentIndex != best.e2_segmentIndex);
        if (best.e1_segmentIndex > best.e2_segmentIndex) // make shure that first will be lowest index, and second will highest index
        {
            swap(best.e1_segmentIndex, best.e2_segmentIndex);
            swap(best.e1_index, best.e2_index);
        }
        auto& pairs = edgeConnections[{best.e1_segmentIndex, best.e2_segmentIndex}];
        pairs.push_back({ best.e1_index, best.e2_index });
    }



    //
    // remember what edges are used for each segment
    //
    vector<Bs> otherCurves_isEdgeConnected;
    otherCurves_isEdgeConnected.resize(freeCurves.size());
    for (int i = 0; i < freeCurves.size(); i++)
    {
        const MeshTopologyLoopCurveSegment& c = freeCurves[i];
        otherCurves_isEdgeConnected[i] = Bs::Zero(c.EdgesCount);
    }

    //
    // DEBUG - show pairs
    //
    if (options.DebugEnabled && options.debug_tracePairsInConsole)
    {
        for (auto & m : edgeConnections)
        {
            int segmentIndex1 = m.first.first;
            int segmentIndex2 = m.first.second;
            vector<pair<int, int>>& pairs = m.second;
            // sort pairs(index,index) on each pair(curveSegment,curveSegment)
            vector<unsigned int> sorted_indexes = utils::stdvector::sort_indexes_custom(pairs.size(), [&pairs, &edges](unsigned int i1, unsigned int i2)
            {
                const pair<int, int>& p1 = pairs[i1];
                const pair<int, int>& p2 = pairs[i2];
                return edges[p1.first].index < edges[p2.first].index;// sort by first segment - second segment will be sorted automatically
            });

            cout << "pair segmentIndexes={" << segmentIndex1 << ", " << segmentIndex2 << "}" << endl;
            cout << "      edge.segmentIndex=" << segmentIndex1 << "    edgeindexes=[";
            for (int i = 0; i < pairs.size(); i++)
            {
                const CurveSegmentEdge& e = edges[pairs[sorted_indexes[i]].first];
                if (i != 0) cout << ", ";
                cout << e.index;
                //draw.AddPoint(e.PM, Color3d(1, 0, 0)); doesnt work in this place - must be in 'DrawAll' method
            }
            cout << "]" << endl;
            //for (int i = 0; i < pairs.size(); i++)
            //{
            //    const CurveSegmentEdge& e2 = edges[pairs[sorted_indexes[i]].second];
            //    cout << "   edge.segmentIndex=" << e2.segmentIndex << "   edgeindex=" << e2.index << endl;
            //}
            cout << "      edge.segmentIndex=" << segmentIndex2 << "    edgeindexes=[";
            for (int i = 0; i < pairs.size(); i++)
            {
                const CurveSegmentEdge& e = edges[pairs[sorted_indexes[i]].second];
                if (i != 0) cout << ", ";
                cout << e.index;
                //draw.AddPoint(e.PM, Color3d(1, 0, 0)); doesnt work in this place - must be in 'DrawAll' method
            }
            cout << "]" << endl;

        }
        cout << "inside connections:   total edges=" << edges.size() << "  connected=" << edgesConnectedCount << endl;
    }


    //
    // swap pairs 'first' and 'second' for calling method 'splitPairs' firstly for 'first' and then for 'second'
    //
    auto getSwapedPairs = [](vector<pair<int, int>>& pairs)
    {
        vector<pair<int, int>> res;
        res.reserve(pairs.size());
        for (auto& p : pairs)
        {
            //swap(p.first, p.second);
            res.push_back({ p.second, p.first });
        }
        return res;
    };


    //
    // split pairs if there are need to create 2 connections instead of 1 (very rare case but it must to be checked (happend in test file 'korzyna_small'))
    // spliting is based on 'first' and when need to check on 'second' - use 'getSwapedPairs' method so 'second' will became 'first'
    //
    auto splitPairs = [](const vector<CurveSegmentEdge>& edges, const vector<pair<int, int>>& pairs_unsorted)
    {
        vector<vector<pair<int, int>>> res;
        // sort to get continious array, so we can easely check if there is some break in indexing in between
        vector<unsigned int> sorted_indexes = utils::stdvector::sort_indexes_custom(pairs_unsorted.size(), [&pairs_unsorted, &edges](unsigned int i1, unsigned int i2)
        {
            const pair<int, int>& p1 = pairs_unsorted[i1];
            const pair<int, int>& p2 = pairs_unsorted[i2];
            return edges[p1.first].index < edges[p2.first].index;// sort by first segment - second segment will be sorted automatically
        });

        // form new sorted pairs for easeir coding
        vector<pair<int, int>> pairs;
        pairs.reserve(pairs_unsorted.size());
        for (int i = 0; i < sorted_indexes.size(); i++)
        {
            pairs.push_back(pairs_unsorted[sorted_indexes[i]]);
        }

        if (pairs[pairs.size() - 1].first == pairs[0].first + pairs.size() - 1) // if indexes are 100% continious
        {
            res.push_back(pairs); // there is no cyclic indexing - return 1 pair - same 'pairs' as method parameter
        }
        else
        {
            for (int i = 0; i < pairs.size() - 1; i++) // find break in indexing between all edges
            {
                if (pairs[i].first + 1 != pairs[i + 1].first) // test where is break in indexing
                {
                    if (res.size() > 0) // if there are more then 2 pairs - skip these connections for now (mb later we can handle such not simple case)
                    {
                        res.clear();
                        return res;
                    }
                    res.push_back(vector<pair<int, int>>(pairs.begin(), pairs.begin() + i + 1));
                    res.push_back(vector<pair<int, int>>(pairs.begin() + i + 1, pairs.end()));
                }
            }
            assert(res.size() == 2 && "must be found break in indexing");
        }

        return res;
    };


    //
    // merge edge connections into segment
    //
    for (auto & m : edgeConnections)
    {
        int segmentIndex1 = m.first.first;
        int segmentIndex2 = m.first.second;

        vector<vector<pair<int, int>>> twoPairs;
        twoPairs.push_back(m.second);  // by default we assume that there is no need in breaking pairs apart into 2 pairs

        // try to split pair into 2 pair base on 'first' parameter (if there is case, what is very rary alike 0.01%)
        if (!freeCurves[segmentIndex1].curve.isCyclic)
        {
            twoPairs = splitPairs(edges, m.second);
        }

        // try to split pair into 2 pair base on 'second' parameter if spliting on 'first' failed
        if (twoPairs.size() == 1 && !freeCurves[segmentIndex2].curve.isCyclic) // spliting need only non cyclic curves
        {
            auto swapedPairs = getSwapedPairs(m.second); // swap pairs, so we can check on 'second' parameter
            vector<vector<pair<int, int>>> twoPairsSwaped = splitPairs(edges, swapedPairs); // try to split on already swaped pairs
            if (twoPairsSwaped.size() == 2) // if split is successful
            {
                twoPairs.clear(); // prepare to rewrite pairs
                twoPairs.push_back(getSwapedPairs(twoPairsSwaped[0])); // add 1th pair (swap before, coz it is swaped)
                twoPairs.push_back(getSwapedPairs(twoPairsSwaped[1])); // add 2th pair (swap before, coz it is swaped)
            }
        }

        // iterate 2 or 1 pairs than was abtained by splitting pairs
        for (auto& pairs : twoPairs)
        {
            assert(pairs.size() > 0);

            // sort pairs(index,index) on each pair(curveSegment,curveSegment)
            vector<unsigned> sorted_indexes = utils::stdvector::sort_indexes_custom(pairs.size(), [&pairs, &edges](unsigned int i1, unsigned int i2)
            {
                const pair<int, int>& p1 = pairs[i1];
                const pair<int, int>& p2 = pairs[i2];
                return edges[p1.first].index < edges[p2.first].index;// sort by first segment - second segment will be sorted automatically
            });

            // collect indexes for each pair of segments
            vector<int> indexes1; // stores edge indexes for segment 1
            vector<int> indexes2; // stores edge indexes for segment 2
            indexes1.reserve(pairs.size());
            indexes2.reserve(pairs.size());
            for (int i = 0; i < pairs.size(); i++)
            {
                const CurveSegmentEdge& e1 = edges[pairs[sorted_indexes[i]].first];
                const CurveSegmentEdge& e2 = edges[pairs[sorted_indexes[i]].second];
                indexes1.push_back(e1.index);
                indexes2.push_back(e2.index);
            }

            // translate indexes from segment-indexes to curve-indexes
            for (auto& a : indexes1) a += freeCurves[segmentIndex1].fromIndex;
            for (auto& a : indexes2) a += freeCurves[segmentIndex2].fromIndex;

            // ensure that 'start' and 'end' points are sharp on some of 2 curve segments
            bool startIsSharp = false;
            bool endIsSharp = false;
            auto s1 = MeshTopologyLoopCurveSegment(freeCurves[segmentIndex1].curve, indexes1);
            auto s2 = MeshTopologyLoopCurveSegment(freeCurves[segmentIndex2].curve, indexes2);
            bool s1tos2_isOpposite = s1.IsOppositeDirections(s2);
            if (s2.PointIsSharp(0)) startIsSharp = true;
            if (s2.PointIsSharp(s2.PointsCount - 1)) endIsSharp = true;
            if (s1tos2_isOpposite) swap(startIsSharp, endIsSharp);
            if (s1.PointIsSharp(0)) startIsSharp = true;
            if (s1.PointIsSharp(s1.PointsCount - 1)) endIsSharp = true;
            if (!startIsSharp || !endIsSharp) // start and end of segmen must by sharp in some of 2 curves, otherwise segment is uncomplete
            {
                //draw.AddPoint(s2.MiddlePoint(), Color3d(1, 0, 0), "s2");
                if (options.DebugEnabled && options.debug_tracePairsInConsole) cout << "skiping add segment for curves " << freeCurves[segmentIndex1].curve.id << " and " << freeCurves[segmentIndex2].curve.id << "  since start and end point must by sharp" << endl;
                continue; // segment is uncomplete - skip it
            }

            // ensure distance between start and end points are no to big  
            // - can heppend if there is only 1 edge (very rare case, happend in 'quad_ngon.obj'):
            //   usually there will be many edges, and every edge should be almost close to his friend - but this is local condition
            //   in case we have only 1 edge - we need to apply global close condition
            P3 p1Start = s1.Point(0);
            P3 p1End = s1.Point(s1.PointsCount - 1);            
            P3 p2Start = s2.Point(0);
            P3 p2End = s2.Point(s2.PointsCount - 1);
            if (s1tos2_isOpposite) swap(p2Start, p2End);
            D maxDistPow2 = max(utils::point::DistToPointPow2(p1Start, p2Start), utils::point::DistToPointPow2(p1End, p2End));
            D minLengthPow2 = min(utils::point::DistToPointPow2(p1Start, p1End), utils::point::DistToPointPow2(p2Start, p2End));
            D maxProportion = options.tol_InsideConnectionsDist; 
            D maxProportionPow2 = maxProportion * maxProportion;
            if (maxDistPow2 / minLengthPow2 > maxProportionPow2 && !isnan(maxDistPow2 / minLengthPow2))
            {
                if (options.DebugEnabled && options.debug_tracePairsInConsole) cout << "skiping add segment for curves " << freeCurves[segmentIndex1].curve.id << " and " << freeCurves[segmentIndex2].curve.id << "  since  start or end points are suspicious far" << endl;
                continue; // segments are suspicious far at start or end
            }

            // add connection
            Connections.push_back(TopologyConnection(Connections.size()));
            auto& connection = Connections.back();
            connection.segments.push_back(MeshTopologyLoopCurveSegment(freeCurves[segmentIndex1].curve, s1.fromIndex, s1.toIndex));
            connection.segments.push_back(MeshTopologyLoopCurveSegment(freeCurves[segmentIndex2].curve, s2.fromIndex, s2.toIndex));

            // mark edges as used to remove them from next usages
            for (int i = 0; i < pairs.size(); i++)
            {
                const CurveSegmentEdge& e1 = edges[pairs[sorted_indexes[i]].first];
                const CurveSegmentEdge& e2 = edges[pairs[sorted_indexes[i]].second];
                otherCurves_isEdgeConnected[segmentIndex1](e1.index) = true; // remember what edges are used for each segment
                otherCurves_isEdgeConnected[segmentIndex2](e2.index) = true; // remember what edges are used for each segment
            }

        }
    }

    //
    // add unconnected edges
    //
    vector<MeshTopologyLoopCurveSegment> restCurves;
    for (int si = 0; si < freeCurves.size(); si++)
    {
        const MeshTopologyLoopCurveSegment& c = freeCurves[si];
        int openIndex = -1;
        int count = 0;
        for (int i = 0; i < c.EdgesCount; i++)
        {
            bool isConnected = otherCurves_isEdgeConnected[si](i);
            if (!isConnected)
            {
                if (openIndex == -1)
                {
                    openIndex = i;
                    count = 1;
                }
                else
                {
                    count++;
                }
            }

            if (openIndex != -1 && (i == c.EdgesCount - 1 || isConnected))
            {
                // handle special case where cyclic curve is connected in middle, so 2 parts left unconnected: at start and at end -  in this case start and end parts have to be merged into a single segment
                if (c.isCyclic && openIndex == 0 && i != c.EdgesCount - 1 && count != c.EdgesCount && c.EdgesCount!=0 && !otherCurves_isEdgeConnected[si](c.EdgesCount - 1))
                {
                    int countLast = 0;
                    while (!otherCurves_isEdgeConnected[si](c.EdgesCount - 1 - countLast)) // get edges count for end part
                    {
                        countLast++;
                        otherCurves_isEdgeConnected[si](c.EdgesCount - countLast) = true; // mark edge as used - since we merging start and end
                    }
                    restCurves.push_back(MeshTopologyLoopCurveSegment(c.curve, c.EdgesCount - countLast, c.fromIndex + count)); // add merged end and start as single segment
                }
                else
                {
                    restCurves.push_back(MeshTopologyLoopCurveSegment(c.curve, c.fromIndex + openIndex, c.fromIndex + openIndex + count)); // add unconnected edges as new segment
                }
                openIndex = -1;
            }
        }

    }

    // copy restCurves to otherCurves (we must do it manualy since direct copy doesnt work becuse of 'const reference')
    int otherCurves_size_before = freeCurves.size();
    freeCurves.clear();
    freeCurves.reserve(restCurves.size());
    for (const auto& c : restCurves)
    {
        freeCurves.push_back(MeshTopologyLoopCurveSegment(c.curve, c.fromIndex, c.toIndex));
    }
    if (options.DebugEnabled)
    {
        cout << "found " << (otherCurves_size_before - freeCurves.size()) / 2 << " inside connections   (left " << freeCurves.size() << " curves)" << endl;
    }



}

void Topology::AddConnections(const vector<const Mesh*>& meshes)
{
    Connections.clear();

    //
    // merge all curves into one vector
    //
    vector<MeshTopologyLoopCurveSegment> freeCurves;
    int totalCount = 0;
    for (auto m : meshes)
    {
        totalCount += m->LoopTopology.curves.size();
    }
    freeCurves.reserve(totalCount);
    for (auto m : meshes)
    {
        for (const MeshTopologyLoopCurve& c : m->LoopTopology.curves)
        {
            assert(c.PointsCount > 0 && "curve must be not empty");
            freeCurves.push_back(MeshTopologyLoopCurveSegment(c)); // create segment from each curve (segment covers complete curve length, this is made for 'partially connected curves' case)
        }
    }
    if (freeCurves.size() == 0) return;
    Connections.reserve(freeCurves.size());
    if (options.DebugEnabled)
    {
        cout << "Updating topology on " << freeCurves.size() << " curves" << endl;
    }


    //
    // add connections
    //
    AddPerfectConnections(freeCurves);
    AddPartialConnections(freeCurves);
    AddInsideConnections(freeCurves);


    //
    // add unconnected curves
    //
    for (int i = 0; i < freeCurves.size(); i++) // exluding last since we want to take 'i+1'
    {
        const MeshTopologyLoopCurveSegment& c = freeCurves[i];
        Connections.push_back(TopologyConnection(Connections.size()));
        auto& connection = Connections.back();
        connection.segments.push_back(c);
    }
}

void Topology::SetPrevNext_SegmentInLoop()
{
    // group segments by loop to which they belongs
    map<int, vector<pair<int, MeshTopologyLoopCurveSegment*>>> loop_to_segments; // using 'pair<int, MeshTopologyLoopCurveSegment*>' just to be able laster to sort
    for (auto& c : Connections)
    {
        for (auto& s : c.segments)
        {
            vector<pair<int, MeshTopologyLoopCurveSegment*>>& segments = loop_to_segments[s.curve.loop.id];
            int indexInLoop = s.curve.loopPointIndexes(s.CorrectCyclicIndex(s.fromIndex));
            segments.push_back(pair<int, MeshTopologyLoopCurveSegment*>(indexInLoop, &s));
        }
    }

    // sort segments in each group (in each loop to which they belongs)
    for (auto& m : loop_to_segments)
    {
        vector<pair<int, MeshTopologyLoopCurveSegment*>>& segments = m.second;
        utils::stdvector::sort(segments);
    }

    // set 'prev' and 'next'
    for (auto& m : loop_to_segments)
    {
        vector<pair<int, MeshTopologyLoopCurveSegment*>>& segments = m.second;
        int segmentsCount = segments.size();
        for (int i = 0; i < segmentsCount; i++)
        {
            int iPrev = (i == 0) ? segmentsCount - 1 : i - 1;
            int iNext = (i == segmentsCount - 1) ? 0 : i + 1;
            MeshTopologyLoopCurveSegment* s = segments[i].second;
            MeshTopologyLoopCurveSegment* sPrev = segments[iPrev].second;
            MeshTopologyLoopCurveSegment* sNext = segments[iNext].second;
            s->PrevSegmentInLoop = sPrev;
            s->NextSegmentInLoop = sNext;
        }
    }
}

void setPoint(MeshTopologyLoopCurveSegment& s, TopologyPoint& p, int start0_end1)
{
    if (start0_end1 == 0)
    {
        if (s.topologyPointStart != nullptr) return;
        s.topologyPointStart = &p;
        p.segments.push_back(&s);
        setPoint(*s.PrevSegmentInLoop, p, 1);
        if (s.ConnectedTo != nullptr)
        {
            bool sameDirection = !s.IsOppositeDirections(*s.ConnectedTo);
            setPoint(*s.ConnectedTo, p, sameDirection ? 0 : 1);
        }
    }
    else
    {
        if (s.topologyPointEnd != nullptr) return;
        s.topologyPointEnd = &p;
        p.segments.push_back(&s);
        setPoint(*s.NextSegmentInLoop, p, 0);
        if (s.ConnectedTo != nullptr)
        {
            bool sameDirection = !s.IsOppositeDirections(*s.ConnectedTo);
            setPoint(*s.ConnectedTo, p, sameDirection ? 1 : 0);
        }
    }
};

void Topology::AddPoints()
{
    Points.clear();
    Points.reserve(Connections.size() * 2);// reserve enought space to avoid reallocation memory, since we will point on this memory and in case of reallocation we will get invalid pointer

    for (auto& c : Connections)
    {
        for (auto& s : c.segments)
        {
            if (s.isCyclic) continue; // avoid adding points on cyclic segments

            if (s.topologyPointStart == nullptr)
            {
                assert(Points.capacity() > Points.size());
                Points.push_back(TopologyPoint(Points.size()));
                TopologyPoint& p = Points.back();
                setPoint(s, p, 0);
            }
            if (s.topologyPointEnd == nullptr)
            {
                assert(Points.capacity() > Points.size());
                Points.push_back(TopologyPoint(Points.size()));
                TopologyPoint& p = Points.back();
                setPoint(s, p, 1);
            }
        }
    }

    // update points 
    for (auto& p : Points)
    {
        p.Update();
    }

    // validate assignment
    for (auto& c : Connections)
    {
        for (auto& s : c.segments)
        {
            if (s.isCyclic) continue; // skip cyclic segments since they dont have points
            assert(s.topologyPointStart != nullptr && "topologyPointStart must be initialized in this method");
            assert(s.topologyPointEnd != nullptr && "topologyPointEnd must be initialized in this method");
        }
    }
}

void Topology::Update(const vector<ModelObject*>& activeObjects, const vector<int>& activeObjects_Ids, bool updateOnlyIfObjectsAreChanged)
{
    if (updateOnlyIfObjectsAreChanged && !IsUpdateNeeded(activeObjects_Ids))
    {
        cout << "Reusing last topology since active objects are same" << endl;
        return;
    }

    vector<const Mesh*> meshes;
    meshes.reserve(activeObjects.size());
    for (const auto& o : activeObjects)
    {
        meshes.push_back(&o->srf.mesh);
    }

    Update(meshes);


    // remember for what objects we have done update
    last_Update_for_Objects_Ids = activeObjects_Ids;

}

void Topology::Update(const vector<const Mesh*>& meshes)
{
    // clear previously allocated data
    Clear();


    // check parameters
    if (!options.Enabled) return;
    if (meshes.size() == 0) return;

    // add connections
    AddConnections(meshes);

    // update segments pointers base on connection-information
    for (auto& c : Connections) c.Update();
    SetPrevNext_SegmentInLoop();

    // add points
    AddPoints();

    // update map for fast access in DividerIterator
    for (int connectionIndex = 0; connectionIndex < Connections.size(); connectionIndex++) // for each connection
    {
        TopologyConnection& c = Connections[connectionIndex];
        for (auto& s : c.segments) // for each segment in this connection
        {
            int meshid = s.meshid;
            for (int index = 0; index < s.EdgesCount; index++) // for each edge in this segment
            {
                int loopIndex = s.curve.loopPointIndexes(s.CorrectCyclicIndex(s.fromIndex + index));
                const MeshLoopPoint& p = s.curve.loop.points[loopIndex];
                const MeshLoopEdge& e = s.curve.loop.edges[loopIndex];
                map_meshid_vid_connectionIndex[meshid][p.VertexId] = connectionIndex; // store all vertexes except last one - so keep in mind, if vertex id is first in segment then it is also same fro prev segment - one vertex shared by 2 segments
                map_meshid_eid_connectionIndex[meshid][e.EdgeId] = connectionIndex;
            }
        }
    }
}

void Topology::Clear()
{
    Points.clear();
    Connections.clear();
    draw.Clear();
    map_meshid_vid_connectionIndex.clear();
    map_meshid_eid_connectionIndex.clear();
    last_Update_for_Objects_Ids.clear();
}

bool Topology::IsUpdateNeeded(const vector<int>& activeObjects_Ids)
{
    bool isTopologyRefreshNeeded = !utils::stdvector::same(last_Update_for_Objects_Ids, activeObjects_Ids);
    return isTopologyRefreshNeeded;
}

void Topology::DrawAll()
{
    if (!options.draw.Enabled) return;
    draw.Clear();

    //
    // draw points
    //
    for (const auto& p : Points)
    {
        //DEBUG highlight selected curve
        Color3d color = p.color;
        if (options.DebugEnabled && options.debug_highlighPoint_id == p.id) color = Color3d(1, 0, 0);


        if (options.draw.drawDots)
        {
            draw.AddPoint(p.point, color);
        }

        string label = " ";
        int addedCount = 0;
        if (options.draw.drawDotsIds)
        {
            label += "" + to_string(p.id);
            addedCount++;
        }

        if (options.draw.drawDotsSegmentsIds)
        {
            for (const auto& si : p.segments)
            {
                if (addedCount != 0) label += ", ";
                label += "seg" + to_string(si->id);
                addedCount++;
            }
        }

        if (options.draw.drawDotsCurvesIds)
        {
            for (const auto& si : p.segments)
            {
                if (addedCount != 0) label += ", ";
                label += "crv" + to_string(si->curve.id);
                addedCount++;
            }
        }


        if (addedCount > 0)
        {
            draw.AddLabel(p.point, label, color);
        }
    }


    //
    // draw curves
    //
    if (options.draw.drawSingleCurves || options.draw.drawConnectedCurves)
        for (const auto& c : Connections)
        {
            if (c.segments.size() == 1 && !options.draw.drawSingleCurves) continue;
            if (c.segments.size() > 1 && !options.draw.drawConnectedCurves) continue;


            //DEBUG highlight selected curve
            if (options.DebugEnabled)
            {
                int siIndex = 0;
                for (const auto& si : c.segments)
                {
                    if (si.curve.id == options.debug_highlighCurve_id || c.id == options.debug_highlighConnection_id)
                    {
                        siIndex++;
                        for (int pi = 0; pi < si.PointsCount; pi++)
                        {
                            P3 p = si.Point(pi);
                            string prefix = ((c.id == options.debug_highlighConnection_id) && siIndex > 1) ? "          " : "";
                            draw.AddPoint(p, Color3d(1, 0, 0), prefix + "crv" + to_string(si.curve.id));
                        }
                    }
                }
            }

            //
            // add curve edges
            //
            assert(c.points.rows() > 0 && "connections curve must be not empty");
            P3 p0 = c.points.row(0);
            for (int pi = 1; pi < c.points.rows(); pi++)
            {
                P3 p1 = c.points.row(pi);
                draw.AddEdge(p0, p1, c.color);
                p0 = p1;
            }

            // DEBUG show isPointsOptimizedToLine
            //Color3d color = c.color;
            //if (c.isPointsOptimizedToLine) color = Color3d(0.9, 0.9, 0.9);
            //for (int pi = 1; pi < c.points.rows(); pi++)
            //{
            //    P3 p1 = c.points.row(pi);
            //    draw.AddEdge(p0, p1, color);
            //    p0 = p1;
            //}
            

            //
            // add curve labels
            //
            string label = " ";
            int addedCount = 0;
            if (options.draw.drawConnectionsIds)
            {
                label += "con" + to_string(c.id);
                addedCount++;
            }
            if (options.draw.drawSegmenstIds)
            {
                for (const auto& si : c.segments)
                {
                    if (addedCount != 0) label += ", ";
                    label += "seg" + to_string(si.id);
                    addedCount++;
                }
            }
            if (options.draw.drawCurvesIds)
            {
                for (const auto& si : c.segments)
                {
                    if (addedCount != 0) label += ", ";
                    label += "crv" + to_string(si.curve.id);
                    addedCount++;
                }
            }
            if (options.draw.drawSegmentsPointsCount)
            {
                if (addedCount != 0) label += ", ";
                label += "points" + to_string(c.segments[0].PointsCount);
                addedCount++;
            }

            if (options.draw.drawConnectionsIds || options.draw.drawSegmenstIds || options.draw.drawCurvesIds || options.draw.drawSegmentsPointsCount)
            {
                draw.AddLabel(c.middlePoint, label, c.color);
            }
        }


    //
    // draw highlights
    //
    if (meshLogicOptions.Draw.Mesh.Highlight.Enabled && utils::strings::IsInt(meshLogicOptions.Draw.Mesh.Highlight.id))
    {
        for (int Highlight_id : utils::strings::extractInts(utils::strings::Trim(meshLogicOptions.Draw.Mesh.Highlight.id)))
        {
            auto Highlight_color = Color3d(0, 1, 1);
            switch (meshLogicOptions.Draw.Mesh.Highlight.What)
            {
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::TopologyConnection:
                {
                    for (const auto& c : Connections)
                    {
                        if (c.id == Highlight_id)
                        {
                            draw.AddLabel(c.middlePoint, "con#" + to_string(c.id), Highlight_color, 5);
                            const P3s& points = c.points;
                            for (int i = 0; i < points.rows() - 1; i++)
                            {
                                P3 p0 = points.row(i);
                                P3 p1 = points.row(i + 1);
                                draw.AddEdge(p0, p1, Highlight_color);
                            }
                        }
                    }
                }
                break;
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::TopologySegment:
                {
                    for (const auto& c : Connections)
                    {
                        for (const auto& s : c.segments)
                        {
                            if (s.id == Highlight_id)
                            {
                                draw.AddLabel(s.MiddlePoint(), "seg#" + to_string(s.id), Highlight_color, 5);
                                 bool isPointsOptimizedToLine = false;
                                P3s points = s.GetPointsForDraw(isPointsOptimizedToLine);
                                for (int i = 0; i < points.rows() - 1; i++)
                                {
                                    P3 p0 = points.row(i);
                                    P3 p1 = points.row(i + 1);
                                    draw.AddEdge(p0, p1, Highlight_color);
                                }
                            }
                        }
                    }
                }
                break;
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::TopologyPoint:
                {
                    for (const auto& p : Points)
                    {
                        if (p.id == Highlight_id)
                        {
                            draw.AddPoint(p.point, Highlight_color);
                            draw.AddLabel(p.point, "point#" + to_string(p.id), Highlight_color, 5);
                        }
                    }
                }
                break;
                default:
                    // others halndle in other classes
                    break;
            }
        }
    }
}
