#include "stdafx.h"
#include "Divider.h"
#include "Mesh.h"
#include "MeshLoop.h"
//#include <igl/streamlines.h>
//#include <igl/polyvector_field_matchings.h>
#include "MeshStreams.h"
#include "MeshTopology.h"
#include "MeshSolverNrosy.h"
#include "MeshSurface.h"

const MeshLogicOptions_Divider& options = meshLogicOptions.Divider;




const D rotationDegree = 1;


Divider::Divider(const Topology& _topology, MeshSurface& srf)
    : topology(_topology), mesh(srf.mesh), solver(srf.solver), solverUV(srf.solverUV), draw(srf.draw), startStreamPoints_originalsize(-1), anchorPoints_originalsize(-1), streams(draw, mesh, solver)
{
}

Divider::Divider(const Topology& _topology, const Mesh& _mesh, const MeshSolverNrosy& _solver, ViewerDrawObjects& _draw)
    : topology(_topology), mesh(_mesh), solver(_solver), solverUV(MeshSolverUV(_mesh, _draw)), draw(_draw), startStreamPoints_originalsize(-1), anchorPoints_originalsize(-1), streams(_draw, _mesh, _solver)
{
}

Divider::Divider(const Topology& _topology, const Mesh& _mesh, const MeshSolverUV& _solver, ViewerDrawObjects& _draw)
    : topology(_topology), mesh(_mesh), solver(MeshSolverNrosy(_mesh, _draw)), solverUV(_solver), draw(_draw), startStreamPoints_originalsize(-1), anchorPoints_originalsize(-1), streams(_draw, _mesh, _solver)
{
}

int Divider::getFacesIndex(const vector<VertexToFacesSortedInfo>& faces, vector<D>& facesAngleSegments, D _rotationDegree)
{
    //D fromDegree = -1;//to make sure we wont miss 0 degree
    //for (int fi = 0; fi < faces.size(); fi++)
    //{
    //    D toDegree = fromDegree + faces[fi].angle;
    //    if (fi == faces.size() - 1) toDegree += 2;//to make sure we wont miss 360 degree
    //    if (fromDegree <= rotationDegree && rotationDegree <= toDegree)// when we found face - set results
    //    {
    //        return fi;
    //    }
    //    fromDegree = toDegree;
    //}
    //return 0;

    int faceIndex = 0;
    for (int i = 0; i < faces.size(); i++)
    {
        faceIndex = i;//must be before if - since if statement can be always false
        if (_rotationDegree < facesAngleSegments[i + 1])
        {
            break;
        }
    }
    return faceIndex;
}

V3 Divider::getAngleFacePlaneVector(const vector<VertexToFacesSortedInfo>& faces, vector<D>& facesAngleSegments,
    int vertexIndex, D angleSumm360, const Mesh& mesh, D _rotationDegree)
{
    // v1 - simple but not always correct
    //V3 vertexPlaneTangent = faces[0].RightSideDirection;
    //V3 vertexNormal = mesh.V_normals.row(vertexIndex);
    //return utils::vector::Rotate(vertexPlaneTangent, vertexNormal, rotationDegree);

    // v2 - harder but correct -  detects rotation vector in face plane
    int faceIndex = getFacesIndex(faces, facesAngleSegments, _rotationDegree);

    D rotationDegreeRelativeToFace = _rotationDegree - facesAngleSegments[faceIndex];
    if (rotationDegreeRelativeToFace < 0)
    {
        return faces[faceIndex].RightSideDirection;
    }
    else if (rotationDegreeRelativeToFace > faces[faceIndex].angle)
    {
        return faces[faceIndex].LeftSideDirection;
    }
    else
    {
        V3 vertexPlaneTangent = faces[faceIndex].RightSideDirection;
        V3 faceNormal = mesh.F_normals.row(faces[faceIndex].FaceId);
        return utils::vector::Rotate(vertexPlaneTangent, faceNormal, rotationDegreeRelativeToFace, true);
    }
}

II Divider::SizeOF() const
{
    II r = sizeof(Divider);
    r += vertexesIdInvolvedInTopologySegments.size() * sizeof(int);
    r += startPoints.size() * sizeof(StreamStartPoint);
    r += anchorPoints.size() * sizeof(StreamAnchorPoint);
    r += streams.SizeOF();
    return r;
};



bool Divider::Divide(vector<StreamStartPoint> additional_startStreamPoints, vector<StreamAnchorPoint> additional_anchorPoints, bool autoDetectSharpPointsAndSingularPoints, int preserveStreamsUntilDividingIteration)
{
    if (solver.Result.Field.size() == 0) return false;
    if (solver.Result.Field[0].rows() != mesh.F.rows())
    {
        cout << "!!!wrong   Divider::Divide  -  solver.Result.Field.size() != mesh.F.rows()" << endl;
        return false;
    }

    //
    // Get all stream and anchor points from mesh analyziz
    //
    if (autoDetectSharpPointsAndSingularPoints)
    {
        GetStartStreamDirections(startPoints, anchorPoints, 0);  // original mesh will produce only startPoints and anchors with original dividingIteration = 0 (only additional startPoints and anchors can have dividingIteration!=0)
    }

    auto defineVertexDirection = [&](StreamStartPoint& sp)
    {
        if (sp.Type == MeshPointType::onVertex && sp.fid == -1) //  in case direction of startPoint in not defined yet because it started from vertex - manually define it
        {
            vector<V3> directionsI;
            vector<int> directionsFaceIdsI;
            vector<D> directionsRotatedByDegrees;
            GetPointDirection(sp.vid_eid_fid, 1, directionsI, directionsFaceIdsI, directionsRotatedByDegrees);
            if (directionsI.size() > 0)
            {
                sp.fid = directionsFaceIdsI[0];
                sp.dir = directionsI[0];
                sp.dir_ri = directionsRotatedByDegrees[0];
            }
        }
    };

    // append all additional startStreamPoints and anchorPoints defined outside  (in class DividerIterator)
    for (auto& sp : startPoints)
    {
        defineVertexDirection(sp);
    }
    for (auto& sp : additional_startStreamPoints)
    {
        defineVertexDirection(sp);
        if (sp.fid != -1)
        {
            startPoints.push_back(sp);
        }
    }
    for (const auto& ap : additional_anchorPoints) anchorPoints.push_back(ap);
    for (int i = 0; i < startPoints.size(); i++) startPoints[i].Index = i; //  fix Index, since index must be exactly continious (like 0,1,2,3,...)

    // ClearModifications since this method can be called few times resuing vectors 'startStreamPoints', 'anchorPoints', 'additional_startStreamPoints', 'additional_anchorPoints'
    bool clearContract = false; // dont clear contracts since DidiverIterator will relly on this possibility to change contract of start startStreamPoints and AnchorPoints
    for (auto& sp : startPoints) sp.ClearModifications(true, clearContract);
    for (auto& ap : anchorPoints) ap.ClearModifications(clearContract);


    //
    // Extend streams
    //
    if (preserveStreamsUntilDividingIteration == -1)
    {
        streams.ClearStreams();
        streams.Reserve(startPoints.size());
    }
    for (auto& sp : startPoints)
    {
        if (preserveStreamsUntilDividingIteration != -1 && sp.dividingIteration.iterationNum <= preserveStreamsUntilDividingIteration) continue; // skipp adding stream from startPoint
        streams.Add(sp.id, sp.StreamAnchorPointId, sp.fid, sp.Type, sp.vid_eid_fid, sp.point, sp.dir, Color3d(0, 0, 0), sp.dividingIteration);
    }
    if (streams.streams.size() != startPoints.size() && preserveStreamsUntilDividingIteration == -1)
    {
        assert(streams.streams.size() == startPoints.size() && "streams.streams.size() == startPoints.size()");
        cout << "!!!wrong   Divider.Divide()     streams.streams.size() == startPoints.size()" << endl;
    }
    int isoLinesExtensionsCount = min(options.SingularityExtendCount, mesh.FacesCount);
    streams.ExtendStreams(isoLinesExtensionsCount, true, preserveStreamsUntilDividingIteration);

    return true;
}

bool Divider::Divide(bool autoDetectSharpPointsAndSingularPoints, int preserveStreamsUntilDividingIteration)
{
    vector<StreamStartPoint> additional_startStreamPoints;
    vector<StreamAnchorPoint> additional_anchorPoints;
    return Divide(additional_startStreamPoints, additional_anchorPoints, autoDetectSharpPointsAndSingularPoints, preserveStreamsUntilDividingIteration); // pass empty additional points
}

int Divider::DebugShowStreams_GetAddedEdgesCount() const
{
    int addedEdges = 0;
    for (int i = 0; i < streams.Count(); i++) addedEdges += streams[i].size();
    return addedEdges;
}

void Divider::DebugShowStreams()
{
    //
    //DEBUG show direction iso-lines
    //
    draw.ReserveEdges(DebugShowStreams_GetAddedEdgesCount()); // reserve space for lines

    vector<int> highlightStreamIds;
    auto highligtWhat = meshLogicOptions.Draw.Mesh.Highlight.What;
    bool highligtStream = meshLogicOptions.Draw.Mesh.Highlight.Enabled &&
        (highligtWhat == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIndex
            || meshLogicOptions.Draw.Mesh.Highlight.What == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamId
            || meshLogicOptions.Draw.Mesh.Highlight.What == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIterativeIndex
            || meshLogicOptions.Draw.Mesh.Highlight.What == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIterativeId);
    if (highligtStream)
    {
        highlightStreamIds = utils::strings::extractInts(meshLogicOptions.Draw.Mesh.Highlight.id);
        if (highlightStreamIds.size() == 0) highligtStream = false;
    }
    for (int i = 0; i < streams.Count(); i++)
    {
        const StreamStartPoint& sp = startPoints[i];
        Color3d color = sp.canBeJoined
            ? Color3d(0, 0.7, 0)
            : (streams[i].IsMerged
                ? Color3d(0.7, 0, 0)
                : Color3d(0.7, 0, 0.7));

        MeshStream& stream = streams[i];

        if (highligtStream)
        {
            int highlightId = -1;
            if (highligtWhat == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIndex) highlightId = stream.Index;
            if (highligtWhat == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamId) highlightId = stream.StreamStartPointId;
            if (highligtWhat == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIterativeIndex) highlightId = stream.dividingIteration.globalStreamIndex;
            if (highligtWhat == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIterativeId) highlightId = stream.dividingIteration.streamStartPoint_id__at_first_iteration;
            bool highligtCurrentStream = utils::stdvector::exists(highlightStreamIds, highlightId);
            if (highligtWhat == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamIndex && stream.IsMerged)
            {
                highligtCurrentStream |= utils::stdvector::exists(highlightStreamIds, stream.IsMerged_with_streamIndex);
            }
            if (highligtCurrentStream)
            {
                color = Color3d(0, 1, 1);
            }
        }
        int piMax = stream.size() - 1;// loop excluding last one to be able get 'pi+1'
        if (sp.canBeJoined) piMax = min(piMax, sp.joinedAtIndex + 3);
        stream.Init_lengthToNextPoint();
        stream.DebugShowStreamPoints(color, piMax);
    }
}
void Divider::GetDividingPoints(vector<MeshPoint>& points, vector<int>& dividesBy, vector<bool>& isSingularPoints)
{
    points.clear();
    dividesBy.clear();
    isSingularPoints.clear();
    points.reserve(mesh.LoopsCount * 7); // approximately not more than 7 points on loop
    dividesBy.reserve(points.capacity());
    if (solver.Result.Field.size() == 0) return;


    //
    // Add border points
    //
    vector<int> sharpVertexes;
    for (const MeshLoop& loop : mesh.Loops)
    {
        // show sharp vertexes
        vector<int> vertexesL;
        vector<D> angleChange;
        vector<int> pointIndexes;
        loop.GetSharpPoints(vertexesL, angleChange, pointIndexes);
        for (int i = 0; i < vertexesL.size(); i++)
        {
            int vid = vertexesL[i];
            sharpVertexes.push_back(vid);

            // v1 - old
            auto anglev1 = angleChange[i];
            int dividesCountv1 = static_cast<int>((anglev1 - 40) / 90);
            if (anglev1 <= 100) continue;


            auto dividesCountv2 = dividesCountv1;

            // v2 - take into account constains and that fact that solver can ignore contrains corrections - eventually calculate angle based on solver Field
            //if (dividesCountv1 == 1 || dividesCountv1 == 2)
            {
                int pi = pointIndexes[i];
                int piPrev = loop.edges[pi].IndexPrev;
                auto e = loop.edges[pi];
                auto ePrev = loop.edges[piPrev];
                int fid = loop.edges[pi].FaceId;
                int fidPrev = loop.edges[piPrev].FaceId;
                V3 constrain = solver.Result.Field[1].row(fid);
                V3 constrainPrev = solver.Result.Field[1].row(fidPrev);
                D anglev2;
                D anglev2Full;
                loop.GetAngleFull(constrain, constrainPrev, e, ePrev, anglev2, anglev2Full);
                dividesCountv2 = static_cast<int>((anglev2Full - 40) / 90);// for count 1 make stronger condition

                //cout << "angle v1 = " << anglev1 << "  count " << dividesCountv1 << ",          angle v2 = " << anglev2Full << "  count " << dividesCountv2 << endl;
            }

            if (dividesCountv2 > 0)
            {
                P3 p = mesh.V.row(vid);
                points.push_back(MeshPoint(MeshPointType::onVertex, vid, p));
                dividesBy.push_back(dividesCountv2);
                isSingularPoints.push_back(false);
            }
        }
    }

    //
    // Add singularity points
    //
    for (int i = 0; i < solver.Result.Singularities.size(); ++i)
    {
        bool isSing3 = (solver.Result.Singularities(i) < -0.001);
        bool isSing5 = (solver.Result.Singularities(i) > 0.001);

        if (isSing3 || isSing5)
        {
            int vid = i;
            P3 p = mesh.V.row(vid);
            points.push_back(MeshPoint(MeshPointType::onVertex, vid, p));
            dividesBy.push_back(isSing3 ? 3 : 5);
            isSingularPoints.push_back(true);
        }
    }

    //
    // Add points defined by Topology
    //
    for (int vid : vertexesIdInvolvedInTopologySegments)
    {
        if (utils::stdvector::exists(sharpVertexes, vid)) continue; // ignore already added vids
        sharpVertexes.push_back(vid); // add vid to avoid later adding duplicates values, since list vertexesIdInvolvedInTopologySegments will have them a lot
        points.push_back(MeshPoint(MeshPointType::onVertex, vid, mesh.V.row(vid)));
        dividesBy.push_back(1);
        isSingularPoints.push_back(false);
    }


    //DEBUG show points
    //for (int i = 0; i < vertexes.size(); ++i)
    //{
    //    draw.AddPoint(V.row(vertexes[i]), Color3d(1, 0, 0));
    //    draw.AddLabel(V.row(vertexes[i]), "    +" + to_string(dividesBy[i]), Color4d(1, 0, 0, 1));
    //}
}

void Divider::GetStartStreamDirections(vector<StreamStartPoint>& startStreamDirections, vector<StreamAnchorPoint>& anchorPoints, int dividerIterator_iterationNum)
{
    // reuse vectors if we already called this method
    bool was_called_GetStartStreamDirections = startStreamPoints_originalsize != -1 || anchorPoints_originalsize != -1;
    if (was_called_GetStartStreamDirections)
    {
        while (startStreamDirections.size() > startStreamPoints_originalsize) startStreamDirections.pop_back();
        while (anchorPoints.size() > anchorPoints_originalsize) anchorPoints.pop_back();
        return;
    }


    startStreamDirections.clear();
    anchorPoints.clear();
    startPoints.reserve(20);
    anchorPoints.reserve(20);

    vector<MeshPoint> points;
    vector<int> dividesBy; //  how many lines comes from this points (singularity - 3 or 5, others - 1,2,3)
    vector<bool> isSingularPoints;
    GetDividingPoints(points, dividesBy, isSingularPoints);
    startStreamDirections.reserve(points.size() * 5);
    int count = points.size();
    //cout << "Found " << count << " points" << endl;
    for (int i = 0; i < count; ++i)
    {
        if (options.DebugEnabled && options.Debug_point_index != -1 && options.Debug_point_index != i) continue;

        vector<V3> directionsI;
        vector<int> directionsFaceIdsI;
        vector<D> directionsRotatedByDegrees;
        if (points[i].Type == MeshPointType::onVertex)
        {
            int vid = points[i].vid_eid_fid;
            GetPointDirection(vid, dividesBy[i], directionsI, directionsFaceIdsI, directionsRotatedByDegrees);

            P3 vertex = mesh.V.row(vid);

            if (directionsI.size() > 0)
            {
                anchorPoints.push_back(StreamAnchorPoint(points[i].Type, points[i].vid_eid_fid, vertex
                    , (!isSingularPoints[i])
                    , (isSingularPoints[i])
                    , directionsI.size()
                    , dividerIterator_iterationNum));
            }

            for (int pi = 0; pi < directionsI.size(); pi++)
            {
                D max_change_angle = meshLogicOptions.StreamAdjuster.iterative_calculations__max_change_angle;
                // for singular points limit angle change so we dont intersect next or prev direction for same singularity
                if (isSingularPoints[i])  // if vertex is singular
                {
                    int piPrev = pi - 1;
                    if (piPrev < 0) piPrev = directionsI.size() - 1;
                    max_change_angle = min(max_change_angle, utils::vector::Angle(directionsI[pi], directionsI[piPrev]) / 2);
                    int piNext = pi + 1;
                    if (piNext > directionsI.size() - 1) piNext = 0;
                    max_change_angle = min(max_change_angle, utils::vector::Angle(directionsI[pi], directionsI[piNext]) / 2);
                }
                else
                {
                    // for non-singular points we dont have to implement because non singular points will have more than 25 degrees in advance - usually it will be 180/2==90
                }
                StreamStartPoint p(points[i].Type, points[i].vid_eid_fid, vertex);
                p.Index = startStreamDirections.size();
                p.StreamAnchorPointId = anchorPoints.back().id;
                p.dir = directionsI[pi];
                p.maxAngleChangeForDir = max_change_angle;
                p.fid = directionsFaceIdsI[pi];
                p.dir_ri = directionsRotatedByDegrees[pi];
                p.dividesBy = dividesBy[i];
                p.isBorderPoint = (!isSingularPoints[i]);
                p.isSingularPoint = (isSingularPoints[i]);
                p.canBeJoined = false;
                p.joinedAtIndex = 0;
                p.dividingIteration = dividerIterator_iterationNum;
                p.dividingIteration.meshid__at_first_iteration = mesh.id;
                p.dividingIteration.streamStartPoint_id__at_first_iteration = p.id;
                startStreamDirections.push_back(p);
            }


        }
    }

    startStreamPoints_originalsize = startPoints.size();
    anchorPoints_originalsize = anchorPoints.size();
}


//mesh, vertexIndex, dividesBy, faces, fieldWeight, avarageFieldWeight, isDEBUG_show_results, directions, faceIds
void Divider::GetPointDirection_RetriveResults(const Mesh& mesh, ViewerDrawObjects& draw, int vertexIndex, int dividesBy,
    const vector<VertexToFacesSortedInfo>& faces, const vector<D>& fieldWeight, Ds& avarageFieldWeight, bool isDEBUG_show_results,
    vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees, bool mergeDirection_whenCountIsHigherFrom_dividesBy
)
{
    const bool isDEBUG = options.DebugEnabled;
    P3 vertex = mesh.V.row(vertexIndex);

    //
    // Define rotation
    //
    D angleSumm360 = 0;
    vector<D> facesAngleSegments; //  facesAngleSegments[i] - min face[i] angle, facesAngleSegments[i + 1] - max face[i] angle
    facesAngleSegments.push_back(0);
    for (int i = 0; i < faces.size(); ++i)
    {
        angleSumm360 += faces[i].angle;
        facesAngleSegments.push_back(angleSumm360);
    }
    if (angleSumm360 > 360)
    {
        angleSumm360 = 360;
    }
    bool isRotationCircular = !faces[0].isRightSideBorder;
    int rotationCount = static_cast<int>(round(angleSumm360 / rotationDegree)) + 1; // "+1" to include last degree
    if (rotationCount > 360) rotationCount = 360;


    //
    // convert weight to relative weight - we need relative weight on all 360 weigths
    //
    //D avgmaxWeight = utils::stdvector::max_element(avarageFieldWeight);
    D avgmaxWeight = avarageFieldWeight.maxCoeff();
    if (avgmaxWeight < 0.000001) avgmaxWeight = 0.000001;
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D weight = avarageFieldWeight[ri];
        D realtiveWeight = weight / avgmaxWeight;
        if (isnan(realtiveWeight))
        {
            cout << "!!! error:  isnan(realtiveWeight)" << endl;
        }
        else
        {
            avarageFieldWeight[ri] = realtiveWeight;
        }
    }
    if (isDEBUG)
    {
        //cout << "********* avarageFieldWeight  *********" << endl;
        draw.ReserveEdges(rotationCount);
        for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
        {
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri + 0.5);
            D realtiveWeight = avarageFieldWeight[ri];
            P3 endPoint = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2)*realtiveWeight * 3;
            draw.AddEdge(vertex, endPoint, Color3d(0, 1, 0));
            //cout << " ri = " << ri << "    avarageFieldWeight  " << avarageFieldWeight[ri] << endl;
            //draw.AddLabel(endPoint, to_string(realtiveWeight)); // avarage weight of field vector
        }
    }

    //
    // Get BEST avarage of field weights in diapasons of 10 degrees (local maximus)
    //
    bool foundSmallerAvarage = true;
    vector<bool> smaller(rotationCount, false);
    int riIncMax = min(10, rotationCount);
    while (foundSmallerAvarage)
    {
        foundSmallerAvarage = false;
        for (int ri = 0; ri < rotationCount; ri++)
        {
            if (smaller[ri]) continue;

            // check smaller ri in [-10,+10] degree
            for (int riInc = 1; riInc < riIncMax; riInc++)
            {
                int riPrev = ri - riInc;
                if (riPrev < 0)
                {
                    riPrev = isRotationCircular ? (rotationCount + riPrev) : -1; // set circular value only if we have circular roatation loop
                }
                int riNext = ri + riInc;
                if (riNext > rotationCount - 1)
                {
                    riNext = isRotationCircular ? (riNext - rotationCount) : -1;// set circular value only if we have circular roatation loop
                }
                if (avarageFieldWeight[ri] < 0.1
                    || (riPrev != -1 && avarageFieldWeight[ri] < avarageFieldWeight[riPrev])
                    || (riNext != -1 && avarageFieldWeight[ri] < avarageFieldWeight[riNext]))
                {
                    smaller[ri] = true;
                    foundSmallerAvarage = true;
                    break;
                    //if (avarageFieldWeight[ri] > 0.1) cout << "weights:  ri = " << ri << "  prev = " << avarageFieldWeight[riPrev] << "  test = " << avarageFieldWeight[ri] << "  next = " << avarageFieldWeight[riNext] << endl;
                }
            }
        }
    }

    //
    // Get maximus
    //
    vector<int> riMaximus;
    for (int ri = 0; ri < rotationCount; ri++)
    {
        if (smaller[ri]) continue;

        //dont add directions than are very close to borders directions ( filter those directions that are very close to border direction)
        D rotatedByDegree = rotationDegree * ri;
        //V3 direction = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotatedByDegree);
        bool isVectorVeryCloseToBorders = false;
        if (!isRotationCircular)
        {
            if (rotatedByDegree < 30 || angleSumm360 - rotatedByDegree < 30)
            {
                isVectorVeryCloseToBorders = true;
            }
        }
        if (isVectorVeryCloseToBorders) continue;


        riMaximus.push_back(ri);
    }


    //DEBUG show all non-smaller weights
    if (isDEBUG)
    {
        cout << "********* added  avarageFieldWeight  *********" << endl;
        //cout << "#" << dividesBy << endl;

        for (int ri : riMaximus) // for each ratation of 1 degree around plane normal
        {
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            //cout << "ri = " << ri << endl;
            D realtiveWeight = avarageFieldWeight[ri];
            P3 endPoint = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length * 2)*realtiveWeight;
            P3 endPointLabel = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length * 2.2)*realtiveWeight;
            draw.AddEdge(vertex, endPoint, Color3d(0, 0, 1));
            //draw.AddLabel(endPointLabel, to_string(realtiveWeight) + " avarageFieldWeight"); // weight of field vector
            draw.AddLabel(endPointLabel, " ri " + to_string(ri) + ", avarageFieldWeight " + to_string(realtiveWeight)); // weight of field vector
            cout << " ri = " << ri << "    fieldWeight  " << fieldWeight[ri] << "    avarageFieldWeight  " << avarageFieldWeight[ri] << endl;
        }
        // add angle change delta - how much angle have changed between directions
        for (int i = 0; i < riMaximus.size(); i++)
        {
            int iPrev = i - 1;
            if (iPrev < 0) iPrev = riMaximus.size() - 1;
            int ri = riMaximus[i];
            int rPrev = riMaximus[iPrev];
            if (rPrev > ri) rPrev = rPrev - static_cast<int>(angleSumm360);
            int delta = ri - rPrev;
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            D realtiveWeight = avarageFieldWeight[ri];
            P3 endPointLabel = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length * 2.5)*realtiveWeight;
            draw.AddLabel(endPointLabel, "+" + to_string(delta) + "°,  #" + to_string(i) + ",  ri=" + to_string(ri), Color4d(0, 0, 1, 0.3)); // weight of field vector
        }
    }

    //
    // Merge maximus if count is heigher from dividesBy
    //    
    int maxMergeTriesCount = riMaximus.size();
    for (int tryi = 0; tryi < maxMergeTriesCount; tryi++)
    {
        if (riMaximus.size() > dividesBy && riMaximus.size() >= 2)
        {
            /*if (!mergeDirection_whenCountIsHigherFrom_dividesBy)
            {
                vector<>
                continue;
            }*/
            assert(rotationDegree == 1); //  we dont support other values except 1 - if needed - update code bellow to support it
            int iMax = isRotationCircular ? riMaximus.size() : riMaximus.size() - 1;
            int minRotation_index = 0;
            int minRotation_index2 = 1;
            int minRotation = riMaximus[1] - riMaximus[0];
            for (int i = 1; i < iMax; i++)
            {
                int iNext = i + 1;
                if (iNext >= riMaximus.size())
                {
                    iNext = 0;
                }

                int rotation = riMaximus[iNext] - riMaximus[i];
                if (iNext == 0) rotation = rotationCount + rotation;
                if (rotation < minRotation)
                {
                    minRotation_index = i;
                    minRotation_index2 = iNext;
                    minRotation = rotation;
                }
            }
            //cout << "minRotation = " << minRotation << endl;
            // merge 2 maximus
            //avarageFieldWeight[ri]
            int ri1 = riMaximus[minRotation_index];
            int ri2 = riMaximus[minRotation_index2];
            D wieghtsum = avarageFieldWeight[ri1] + avarageFieldWeight[ri2];
            D ri1_weight = avarageFieldWeight[ri1] / wieghtsum;
            D ri2_weight = avarageFieldWeight[ri2] / wieghtsum;
            if (!mergeDirection_whenCountIsHigherFrom_dividesBy) // take just maximum, and ignore lesser value
            {
                if (avarageFieldWeight[ri1] > avarageFieldWeight[ri2])
                {
                    ri1_weight = 1;
                    ri2_weight = 0;
                }
                else
                {
                    ri1_weight = 0;
                    ri2_weight = 1;
                }
            }

            int avgRotation = static_cast<int>(ri1_weight*ri1 + ri2_weight * ri2);
            if (minRotation_index2 == 0)
            {
                avgRotation = static_cast<int>(ri1_weight*ri1 + ri2_weight * (rotationCount + ri2));
                if (avgRotation > rotationCount - 1)
                {
                    avgRotation -= rotationCount;
                }
                //cout << "   minRotation_index = " << minRotation_index << "    minRotation_index2 = " << minRotation_index2 << "   avgRotation = " << avgRotation << endl;
                if (abs(avgRotation - riMaximus[minRotation_index2]) < abs(avgRotation - riMaximus[minRotation_index])) swap(minRotation_index, minRotation_index2);
            }
            riMaximus[minRotation_index] = avgRotation;
            avarageFieldWeight[avgRotation] = ri1_weight * avarageFieldWeight[ri1] + ri2_weight * avarageFieldWeight[ri2];
            D new_fieldWeight = ri1_weight * fieldWeight[ri1] + ri2_weight * fieldWeight[ri2];
            //fieldWeight[avgRotation] = new_fieldWeight;
            utils::stdvector::remove_at(riMaximus, minRotation_index2);
            if (isDEBUG)
            {
                cout << " merged " << avarageFieldWeight[ri1] << "  and  " << avarageFieldWeight[ri2] << "   new weight " << avarageFieldWeight[avgRotation] << "     new ri=" << avgRotation << endl;
                for (int ri : riMaximus) // for each ratation of 1 degree around plane normal
                {
                    cout << "       new  ri = " << ri << "    fieldWeight  " << new_fieldWeight << "    avarageFieldWeight  " << avarageFieldWeight[ri] << endl;
                }
            }
        }
    }

    //
    // Create new maximum if count is less from dividesBy
    //
    for (int tryi = 0; tryi < riMaximus.size(); tryi++)
    {
        if (riMaximus.size() < dividesBy && dividesBy == 5 && riMaximus.size() == 4)//limit this case only for 1 miss maiximum of singularity with 5 lines
        {
            assert(rotationDegree == 1); //  we dont support other values except 1 - if needed - update code bellow to support it
            int iMax = isRotationCircular ? riMaximus.size() : riMaximus.size() - 1;
            int maxRotation_index = 0;
            int maxRotation_index2 = 1;
            int maxRotation = riMaximus[1] - riMaximus[0];
            for (int i = 1; i < iMax; i++)
            {
                int iNext = i + 1;
                if (iNext >= riMaximus.size())
                {
                    iNext = 0;
                }

                int rotation = riMaximus[iNext] - riMaximus[i];
                if (iNext == 0) rotation = rotationCount + rotation;
                if (rotation > maxRotation)
                {
                    maxRotation_index = i;
                    maxRotation_index2 = iNext;
                    maxRotation = rotation;
                }
            }

            //cout << "minRotation = " << minRotation << endl;
            // create new maximum
            int ri1 = riMaximus[maxRotation_index];
            int ri2 = riMaximus[maxRotation_index2];

            int avgRotation = static_cast<int>((ri1 + ri2) / 2);
            if (maxRotation_index2 == 0)
            {
                avgRotation = static_cast<int>((ri1 + (rotationCount + ri2)) / 2);
                if (avgRotation > rotationCount) avgRotation -= rotationCount;
            }

            int insertIndex = maxRotation_index2;
            if (maxRotation_index2 == 0 && avgRotation > ri1)
            {
                insertIndex = riMaximus.size();
            }
            riMaximus.insert(riMaximus.begin() + insertIndex, avgRotation);
            if (isDEBUG) cout << " insert   ri " << avgRotation << endl;
        }
    }

    //
    // Extract results ( filter those directions that are very close to border direction)
    //    
    directions.clear();
    directionsFaceIds.clear();
    directionsRotatedByDegrees.clear();
    for (int ri : riMaximus) // for each ratation of 1 degree around plane normal
    {
        D rotatedByDegree = rotationDegree * ri;
        V3 direction = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotatedByDegree);

        //DEBUG - show added directions
        //P3 endPoint = vertex + direction.normalized()*(mesh.avg_edge_length*2);
        //draw.AddLabel(endPoint, to_string(ri), Color3d(0, 0, 0.5));

        D fromDegree = -1;//to make sure we wont miss 0 degree
        for (int fi = 0; fi < faces.size(); fi++)
        {
            D toDegree = fromDegree + faces[fi].angle;
            if (fi == faces.size() - 1) toDegree += 2;//to make sure we wont miss 360 degree
            if (fromDegree <= rotatedByDegree && rotatedByDegree <= toDegree)// when we found face - set results
            {
                directions.push_back(direction.normalized());
                directionsFaceIds.push_back(faces[fi].FaceId);
                directionsRotatedByDegrees.push_back(rotatedByDegree);
                break;
            }
            fromDegree = toDegree;
        }
    }
    //DEBUG show results
    if (isDEBUG_show_results)
    {
        for (int ri = 0; ri < directions.size(); ri++) // for each ratation of 1 degree around plane normal
        {
            P3 endPoint = vertex + directions[ri].normalized()*(mesh.avg_edge_length * 5);
            draw.AddEdge(vertex, endPoint, Color3d(0, 0, 0.5));
            //P3 faceCenter = mesh.F_Barycenters.row(directionsFaceIds[ri]);
            draw.AddLabel(endPoint, to_string(directionsFaceIds[ri]), Color3d(0, 0, 0.5));
        }
    }
}

void Divider::GetPointDirection(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees)
{
    auto algorithm = mesh.V_isborder[vertexIndex]
        ? MeshLogicOptions_Divider::AlgorithmType::BestDirFast // a small speed omptimization
        : options.Algorithm;

    //auto algorithm = options.Algorithm;

    switch (algorithm)
    {
        case MeshLogicOptions_Divider::AlgorithmType::AvgDir:
        default:
            // simples and worst
            GetPointDirection_AvgDir(vertexIndex, dividesBy, directions, directionsFaceIds, directionsRotatedByDegrees);
            break;
        case MeshLogicOptions_Divider::AlgorithmType::BestDir:
            // good working but not so precise
            GetPointDirection_BestDir(vertexIndex, dividesBy, directions, directionsFaceIds, directionsRotatedByDegrees);
            break;
        case MeshLogicOptions_Divider::AlgorithmType::BestDirFast:
            // good working but not so precise (improved)
            GetPointDirection_BestDir_fast(vertexIndex, dividesBy, directions, directionsFaceIds, directionsRotatedByDegrees);
            break;
        case MeshLogicOptions_Divider::AlgorithmType::AvgStream:
            // good working
            GetPointDirection_AvgStream(vertexIndex, dividesBy, directions, directionsFaceIds, directionsRotatedByDegrees);
            break;
        case MeshLogicOptions_Divider::AlgorithmType::StreamAngles:
            // good working (improved)
            GetPointDirection_StreamAngles(vertexIndex, dividesBy, directions, directionsFaceIds, directionsRotatedByDegrees);
            break;
    }

}

void Divider::GetPointDirection_AvgDir(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees)
{

    //if (vertexIndex != 60)return;
    //if (vertexIndex != 794 && vertexIndex != 406 && vertexIndex != 679 && vertexIndex != 5266) return;
    const bool isDEBUG = options.DebugEnabled;
    const bool isDEBUG_show_results = false;

    directions.clear();
    directionsFaceIds.clear();
    directionsRotatedByDegrees.clear();

    P3 vertex = mesh.V.row(vertexIndex);
    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    if (!mesh.GetVertexFacesAngleSegments(vertexIndex, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
    {
        return;
    }


    //
    // Get two best nrosy field vectors for each face
    //
    const vector<V3s>& YYY = solver.Result.Field;
    assert(YYY.size() == 4 || (YYY.size() == 2 && solver.Result.IsFieldSymetric));
    constexpr int N = 4;
    vector<D> anglesTemp;
    anglesTemp.resize(N);
    vector<V3> fieldVectors(faces.size() * 2);
    Ds fieldVectorsWeights(faces.size() * 2);
    for (int i = 0; i < faces.size(); ++i)
    {
        // get nrosy field for face
        int fid = faces[i].FaceId;
        P3 faceCenter = mesh.F_Barycenters.row(fid);
        V3 vectorToFaceCenter = faceCenter - vertex;
        //DEBUG show nrosy field for face
        //for (int ni = 0; ni < YYY.size(); ni++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + YYY[ni].row(fid).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // sort them by smallest angle
        for (int ni = 0; ni < YYY.size(); ni++)
        {
            V3 fieldVector = YYY[ni].row(fid).transpose();
            anglesTemp[ni] = utils::vector::Angle(vectorToFaceCenter, fieldVector);
            if (solver.Result.IsFieldSymetric)
            {
                anglesTemp[2 + ni] = 180 - anglesTemp[ni];
            }
        }
        vector<unsigned> sortedIndexes = utils::stdvector::sort_indexes(anglesTemp);
        //DEBUG show 2 field vectors for face
        //for (int i = 0; i < 2; i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(sortedIndexes[i]).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // set weights (base on angle)
        int bestIndex1 = sortedIndexes[0];
        int bestIndex2 = sortedIndexes[1];
        D sumangle = anglesTemp[bestIndex1] + anglesTemp[bestIndex2];
        D weight1 = 1 - anglesTemp[bestIndex1] / sumangle; //  the smallest angle - the heigher weight
        D weight2 = 1 - anglesTemp[bestIndex2] / sumangle; //  the smallest angle - the heigher weight
        V3 dir1 = solver.Result.getField(bestIndex1, fid);
        V3 dir2 = solver.Result.getField(bestIndex2, fid);

        fieldVectors[i * 2 + 0] = dir1.transpose();
        fieldVectors[i * 2 + 1] = dir2.transpose();
        if (options.fieldVectorsWeights_isrelative_to_fielddirection_norm)
        {
            weight1 *= dir1.norm();
            weight2 *= dir2.norm();
        }
        if (options.fieldVectorsWeights_is_fielddirection_norm)
        {
            weight1 = dir1.norm();
            weight2 = dir2.norm();
        }
        fieldVectorsWeights[i * 2 + 0] = isnan(weight1) ? 0 : weight1;
        fieldVectorsWeights[i * 2 + 1] = isnan(weight2) ? 0 : weight2;
    }

    //DEBUG show 2 field vectors for face
    if (isDEBUG)
    {
        draw.ReserveEdges(faces.size() * 3);
        for (int i = 0; i < faces.size(); ++i)
        {
            int fid = faces[i].FaceId;
            P3 faceCenter = mesh.F_Barycenters.row(fid);
            for (int ni = 0; ni < 2; ni++)
            {
                V3 fieldVector = fieldVectors[i * 2 + ni];
                D fieldVectorWeight = fieldVectorsWeights[i * 2 + ni];
                D faceWeight = faces[i].weightBasedOnAngle * faces.size();
                P3 endPoint = faceCenter + fieldVector * (mesh.avg_edge_length / 2)*fieldVectorWeight*faceWeight;
                draw.AddEdge(faceCenter, endPoint, Color3d(1, 0, 0));                  // field vector
                //draw.AddLabel(endPoint, to_string(fieldVectorWeight*faceWeight)); // weight of field vector
            }
            draw.AddEdge(faceCenter, vertex, Color3d(0, 1, 0));
        }

    }







    //
    // Get intensity of field around vertex normal (around vertex on 360 degree (could be less))
    //


    //DEBUG show rotationCount
    // draw.AddLabel(V.row(faces[0].vertexIndexRight), "rotationCount="+to_string(rotationCount));
    D angleMax = N < 4 ? 360.0 / 4 : 360.0 / N; // what is more than 80 degrees - not influence on avarage direction. notice - we use 360 but not angleSumm360
    D angleMaxAllowed = angleMax - angleMax / 1.55; // what is more than 45 degrees - not influence on avarage direction
    //angleMaxAllowed = 300.0 / dividesBy;
    //angleMaxAllowed = 80;
    D angleMaxAllowed_DR_COS = utils::angle::DegreesToCos(angleMaxAllowed);
    D angleMaxAllowed_DR_COS_QUAD = angleMaxAllowed_DR_COS * angleMaxAllowed_DR_COS;
    //if (dividesBy > 4) angleMaxAllowed = 360 / dividesBy - 10; // for singularities with level 5 - max angle will be smaller - approx 60 degree
    vector<D> fieldWeight(rotationCount);
    for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
    {
        D fieldPower360Ri = 0;
        //int addednrosyfieldVectors = 0;
        V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
        vertexPlaneTangentI.normalize();
        for (int fi = 0; fi < faces.size(); fi++) // for each face (contacted with vertex)
        {
            D fieldPower360RiAdd = 0;
            for (int ni = 0; ni < 2; ni++) // for each nrosy-field vector (of face)
            {
                V3 yi = fieldVectors[fi * 2 + ni];

                //v1
                //D angle = utils::vector::Angle(yi, vertexPlaneTangentI);
                //if (angle < angleMaxAllowed) // if nrosy-field vector somehow goes same direction
                //{
                //    D weight = 1 - (angle / angleMaxAllowed); // weight will the bigger the less differnce in directions. max weight is 1, smallest is 0
                //    //cout << "weight = " << weight << endl;
                //    //weight *= fieldVectorsWeights[fi * 2 + ni] * faces.size(); 
                //    weight *= fieldVectorsWeights[fi * 2 + ni];
                //    fieldPower360RiAdd += weight*weight; // add weight - bigger difference in direcition with plane x - smaller weight. if direction same - value will be 1, if differnece is like 80 degree - value will be almost 0
                //}

                //v2 - fast - without using normalize and devide (avoiding zero devision)
                // (v1·v2)^2   >   v1·v1 * v2·v2 * (cos(DegreeToRadians(max)))^2
                D v1_dot_v2 = utils::vector::Dot(yi, vertexPlaneTangentI);
                D v1_dot_v1 = utils::vector::LengthPow2(yi);
                D angle_relative = v1_dot_v2 * v1_dot_v2;
                D max_angle_relative = v1_dot_v1 * angleMaxAllowed_DR_COS_QUAD;
                if (v1_dot_v2 > 0 && angle_relative > max_angle_relative)// if nrosy-field vector somehow goes same direction (if angle < 90, and angle < max)
                {
                    //D angle = utils::vector::Angle(yi, vertexPlaneTangentI);
                    D d = v1_dot_v2 / sqrt(v1_dot_v1);
                    if (d > 1.0)d = 1.0;
                    if (d < -1.0)d = -1.0;
                    D angle = utils::angle::RadiansToDegrees(acos(d));
                    //cout << "angle = " << angle << "    angle2 = " << angle2 << endl;
                    D weight = 1 - angle / angleMaxAllowed; // weight will the bigger the less differnce in directions. max weight is 1, smallest is 0
                    //weight *= fieldVectorsWeights[fi * 2 + ni] * faces.size(); 
                    //cout << "weight = " << weight << endl << endl;
                    weight *= fieldVectorsWeights[fi * 2 + ni];
                    fieldPower360RiAdd += weight * weight; // add weight - bigger difference in direcition with plane x - smaller weight. if direction same - value will be 1, if differnece is like 80 degree - value will be almost 0
                }
                //DEBUG - test vs v1
                //bool testv1 = angle < angleMaxAllowed;
                //bool testv2 = angle_relative > max_angle_relative;
                //if (testv1 != testv2)
                //{
                //    int temp = 0;
                //}
            }
            fieldPower360Ri += fieldPower360RiAdd;// best choise
            //fieldPower360Ri += fieldPower360RiAdd*faces[fi].weightBasedOnAngle;
            //fieldPower360Ri += fieldPower360RiAdd/faces[fi].weightBasedOnAngle;
        }
        fieldWeight[ri] = fieldPower360Ri;
    }
    // convert weight to relative weight - we need relative weight on all 360 weigths
    D maxWeight = utils::stdvector::max_element(fieldWeight);
    if (maxWeight < 0.000001) maxWeight = 0.000001;
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D weight = fieldWeight[ri];
        D realtiveWeight = weight / maxWeight;
        fieldWeight[ri] = realtiveWeight;
    }
    //DEBUG show avarageNrosyfieldVector
    if (isDEBUG)
    {
        //cout << "********* fieldWeight  *********" << endl;
        draw.ReserveEdges(rotationCount);
        for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
        {
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            D realtiveWeight = fieldWeight[ri];
            Color3d color = Color3d(0.6 + 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight); // from silver to red
            draw.AddEdge(vertex, vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2)*realtiveWeight, color);
            //cout << " ri = " << ri << "    fieldWeight  " << fieldWeight[ri] << endl;
        }

    }


    //
    // Get avarage of field weights in 30 degrees (local maximus)
    //    
    //int avarageDiapasonLength = static_cast<int>(angleMaxAllowed / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //cout << "angleMaxAllowed = " << angleMaxAllowed << endl;
    int avarageDiapasonLength = static_cast<int>(20 / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //int avarageDiapasonLength = 50 / rotationDegree; // if rotationDegree == 1 - then 32 degrees
    Ds avarageFieldWeight;
    avarageFieldWeight.resize(rotationCount);
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D avarage = fieldWeight[ri]; // this is middle of our internal loop - others values will be added in next for cycle
        int avarageCount = 1;
        for (int i = 1; i < avarageDiapasonLength / 2; i++)
        {
            D innerWeight = 1 - (static_cast<D>(i - 1) / (avarageDiapasonLength / 2)); //this weight will decrease noise in mean direction - direction will be mutch more on highest value
            int riPrev = ri - i;
            if (riPrev < 0)
            {
                riPrev = isRotationCircular ? (rotationCount + riPrev) : -1; // set circular value only if we have circular roatation loop
            }
            int riNext = ri + i;
            if (riNext > rotationCount - 1)
            {
                riNext = isRotationCircular ? (riNext - rotationCount) : -1;// set circular value only if we have circular roatation loop
            }
            if (riPrev != -1)
            {
                avarage += fieldWeight[riPrev] * innerWeight;
                avarageCount++;
            }
            if (riNext != -1)
            {
                avarage += fieldWeight[riNext] * innerWeight;
                avarageCount++;
            }
        }
        //DEBUG
        //if (ri > 350 || ri < 10)
        //{
        //    cout << "ri = " << ri << "    avarageCount = " << avarageCount << endl;
        //}
        avarageFieldWeight[ri] = (avarageCount != 0)
            ? avarage / avarageCount
            : 0;
    }
    if (isnan(avarageFieldWeight[0]))
    {
        cout << "!!! error:  isnan(avarageFieldWeight[0])" << endl;
    }
    GetPointDirection_RetriveResults(mesh, draw, vertexIndex, dividesBy, faces, fieldWeight, avarageFieldWeight, isDEBUG_show_results, directions, directionsFaceIds, directionsRotatedByDegrees);
}


void Divider::GetPointDirection_BestDir(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees)
{

    //if (vertexIndex != 60)return;
    //if (vertexIndex != 794 && vertexIndex != 406 && vertexIndex != 679 && vertexIndex != 5266) return;
    const bool isDEBUG = options.DebugEnabled;
    const bool isDEBUG_show_results = false;

    directions.clear();
    directionsFaceIds.clear();
    directionsRotatedByDegrees.clear();

    P3 vertex = mesh.V.row(vertexIndex);
    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    if (!mesh.GetVertexFacesAngleSegments(vertexIndex, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
    {
        return;
    }


    //
    // Get two best nrosy field vectors for each face
    //
    int N = 4;
    vector<D> anglesTemp;
    anglesTemp.resize(N);
    vector<V3> fieldVectors(faces.size() * 2);
    vector<D> fieldVectorsWeights(faces.size() * 2);
    for (int i = 0; i < faces.size(); ++i)
    {
        const vector<V3s>& YYY = solver.Result.Field;
        // get nrosy field for face
        int fid = faces[i].FaceId;
        P3 faceCenter = mesh.F_Barycenters.row(fid);
        V3 vectorToFaceCenter = faceCenter - vertex;
        //DEBUG show nrosy field for face
        //for (int i = 0; i < Y.rows(); i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(i).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // sort them by smallest angle
        assert(YYY.size() == 4 || (YYY.size() == 2 && solver.Result.IsFieldSymetric));
        for (int ni = 0; ni < YYY.size(); ni++)
        {
            V3 fieldVector = YYY[ni].row(fid).transpose();
            anglesTemp[ni] = utils::vector::Angle(vectorToFaceCenter, fieldVector);
            if (solver.Result.IsFieldSymetric)
            {
                anglesTemp[2 + ni] = utils::vector::Angle(vectorToFaceCenter, -fieldVector);
            }

        }
        vector<unsigned> sortedIndexes = utils::stdvector::sort_indexes(anglesTemp);
        //DEBUG show 2 field vectors for face
        //for (int i = 0; i < 2; i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(sortedIndexes[i]).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // set best 2 direction
        int bestIndex1 = sortedIndexes[0];
        int bestIndex2 = sortedIndexes[1];
        V3 dir1 = solver.Result.getField(bestIndex1, fid);
        V3 dir2 = solver.Result.getField(bestIndex2, fid);
        if (utils::vector::AngleFull(dir2, dir1, mesh.F_normals.row(fid)) > 180)
        {
            // swap dir1 and dir2
            V3 savedir1 = dir1;
            dir1 = dir2;
            dir2 = savedir1;
        }
        fieldVectors[i * 2 + 0] = dir1.transpose();
        fieldVectors[i * 2 + 1] = dir2.transpose();
        // set weights (base on angle)
        D weight1 = 1;
        D weight2 = 1;
        if (options.fieldVectorsWeights_isrelative_to_fielddirection_norm)
        {
            weight1 *= dir1.norm();
            weight2 *= dir2.norm();
        }
        if (options.fieldVectorsWeights_is_fielddirection_norm)
        {
            weight1 = dir1.norm();
            weight2 = dir2.norm();
        }
        fieldVectorsWeights[i * 2 + 0] = weight1;
        fieldVectorsWeights[i * 2 + 1] = weight2;
    }
    //DEBUG show 2 field vectors for face
    if (isDEBUG)
    {
        draw.ReserveEdges(faces.size() * 3);
        for (int i = 0; i < faces.size(); ++i)
        {
            int fid = faces[i].FaceId;
            P3 faceCenter = mesh.F_Barycenters.row(fid);
            for (int ni = 0; ni < 2; ni++)
            {
                V3 fieldVector = fieldVectors[i * 2 + ni];
                D fieldVectorWeight = fieldVectorsWeights[i * 2 + ni];
                D faceWeight = faces[i].weightBasedOnAngle * faces.size();
                P3 endPoint = faceCenter + fieldVector * (mesh.avg_edge_length / 2)*fieldVectorWeight*faceWeight;
                draw.AddEdge(faceCenter, endPoint, Color3d(1, 0, 0));                  // field vector
                //draw.AddLabel(endPoint, to_string(fieldVectorWeight*faceWeight)); // weight of field vector
                //draw.AddLabel(endPoint, "dir" + to_string(ni + 1)); // weight of field vector 
            }
            //draw.AddEdge(faceCenter, vertex, Color3d(0, 1, 0));
        }

    }







    //
    // Get intensity of field around vertex normal (around vertex on 360 degree (could be less))
    //
    vector<int> crossFiedDirectionsCount(faces.size(), 0);
    vector<D> crossFiedDirectionsWeightsSum(faces.size(), 0);
    vector<D> crossFiedDirectionsAnglesSum(faces.size(), 0);
    vector<D> fieldWeight(rotationCount + 1, 0);
    D angleSum = 0;
    int fieldWeightAddedCount = 0;
    //cout << endl << endl;
    for (int i = 0; i < faces.size(); ++i)
    {
        auto& f = faces[i];
        V3 fieldVectorLeft = fieldVectors[i * 2 + 0];
        V3 fieldVectorRight = fieldVectors[i * 2 + 1];
        V3 fNormal = mesh.F_normals.row(f.FaceId);
        D angleRightFull = utils::vector::AngleFull(f.RightSideDirection, fieldVectorRight, fNormal);
        D angleLeftFull = utils::vector::AngleFull(fieldVectorLeft, f.LeftSideDirection, fNormal);
        //cout << "i = " << i << "   angleRightFull = " << angleRightFull << "   angleLeftFull = " << angleLeftFull << endl;
        if (angleRightFull < 180 && angleRightFull < f.angle)
        {
            int index = static_cast<int>(round(angleSum + angleRightFull));
            if (index < 0) index = 0;
            if (index > rotationCount - 1) index = rotationCount - 1;
            fieldWeight[index] = fieldVectorRight.norm();
            fieldWeightAddedCount++;
            //cout << "dividesBy = " << dividesBy << "     fieldWeight[" << index << "] = " << fieldWeight[index] << endl;
            //V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*index);
            //P3 endPoint = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2);
            //draw.AddEdge(vertex, endPoint, Color3d(0.5, 0.5, 0.5));
            //draw.AddLabel(endPoint, "angleRightFull " +to_string(angleRightFull) + "    index "+ to_string(index)); // avarage weight of field vector
        }

        if (angleLeftFull < 180 && angleLeftFull < f.angle)
        {
            int index = static_cast<int>(round(angleSum + f.angle - angleLeftFull));
            if (index < 0) index = 0;
            if (index > rotationCount - 1) index = rotationCount - 1;
            fieldWeight[index] = fieldVectorLeft.norm();
            fieldWeightAddedCount++;
            //cout << "dividesBy = " << dividesBy << "     fieldWeight[" << index << "] = " << fieldWeight[index] << endl;
            //V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*index);
            //P3 endPoint = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2);
            //draw.AddEdge(vertex, endPoint, Color3d(0.5, 0.5, 0.5));
            //draw.AddLabel(endPoint, "angleLeftFull " + to_string(angleLeftFull) + "    index " + to_string(index)); // avarage weight of field vector
        }

        //if ((angleRightFull > 180 || (!isRotationCircular && i == 0))
        //    && angleLeftFull > 180 || (!isRotationCircular && i == faces.size() - 1))
        //{
        //    crossFiedDirectionsCount[i]++;
        //    D sumRight = (360 - angleRightFull) > 45 ? 0 : (45 - (360 - angleRightFull)) / 45;
        //    crossFiedDirectionsWeightsSum[i] += sumRight;
        //    crossFiedDirectionsAnglesSum[i] += 360 - angleRightFull;
        //    //cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << i << "] = " << crossFiedDirectionsCount[i] << endl;
        //    D sumLeft = (360 - angleLeftFull) > 45 ? 0 : (45 - (360 - angleLeftFull)) / 45;
        //    if (i == faces.size() - 1)
        //    {
        //        if (isRotationCircular)
        //        {
        //            crossFiedDirectionsCount[0]++;
        //            crossFiedDirectionsWeightsSum[0] += sumLeft;
        //            crossFiedDirectionsAnglesSum[0] += 360 - sumLeft;
        //            //cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << 0 << "] = " << crossFiedDirectionsCount[0] << endl;
        //        }
        //    }
        //    else
        //    {
        //        crossFiedDirectionsCount[i + 1]++;
        //        crossFiedDirectionsWeightsSum[i + 1] += sumLeft;
        //        crossFiedDirectionsAnglesSum[i + 1] += 360 - sumLeft;
        //        //cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << (i+1) << "] = " << crossFiedDirectionsCount[i + 1] << endl;
        //    }
        //}

        if (angleRightFull > 180 || (!isRotationCircular && i == 0))
        {
            crossFiedDirectionsCount[i]++;
            D sumRight = (360 - angleRightFull) > 45 ? 0 : (45 - (360 - angleRightFull)) / 45;
            crossFiedDirectionsWeightsSum[i] += sumRight;
            crossFiedDirectionsAnglesSum[i] += 360 - angleRightFull;
            if (isDEBUG) cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << i << "] = " << crossFiedDirectionsCount[i] << endl;
        }
        if (angleLeftFull > 180 || (!isRotationCircular && i == faces.size() - 1))
        {
            D sumLeft = (360 - angleLeftFull) > 45 ? 0 : (45 - (360 - angleLeftFull)) / 45;
            if (i == faces.size() - 1)
            {
                if (isRotationCircular)
                {
                    crossFiedDirectionsCount[0]++;
                    crossFiedDirectionsWeightsSum[0] += sumLeft;
                    crossFiedDirectionsAnglesSum[0] += 360 - angleLeftFull;
                    if (isDEBUG) cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << 0 << "] = " << crossFiedDirectionsCount[0] << endl;
                }
            }
            else
            {
                crossFiedDirectionsCount[i + 1]++;
                crossFiedDirectionsWeightsSum[i + 1] += sumLeft;
                crossFiedDirectionsAnglesSum[i + 1] += 360 - angleLeftFull;
                if (isDEBUG) cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << (i + 1) << "] = " << crossFiedDirectionsCount[i + 1] << endl;
            }
        }

        angleSum += f.angle;
    }


    angleSum = 0;
    for (int i = 0; i < faces.size(); ++i)
    {
        auto& f = faces[i];
        if (crossFiedDirectionsCount[i] == 2 && crossFiedDirectionsAnglesSum[i] < 45)
        {
            int index = static_cast<int>(round(angleSum));
            if (index < 0) index = 0;
            if (index > rotationCount - 1) index = rotationCount - 1;
            //fieldWeight[index] = crossFiedDirectionsWeightsSum[i] / crossFiedDirectionsCount[i];
            fieldWeight[index] = (45 - crossFiedDirectionsAnglesSum[i]) / 45;
        }
        angleSum += f.angle;
    }

    // convert weight to relative weight - we need relative weight on all 360 weigths
    D maxWeight = utils::stdvector::max_element(fieldWeight);
    if (maxWeight < 0.000001) maxWeight = 0.000001;
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D weight = fieldWeight[ri];
        D realtiveWeight = weight / maxWeight;
        fieldWeight[ri] = realtiveWeight;
    }
    //DEBUG show avarageNrosyfieldVector
    if (isDEBUG)
    {
        //cout << "********* fieldWeight  *********" << endl;
        draw.ReserveEdges(rotationCount);
        for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
        {
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            D realtiveWeight = fieldWeight[ri];
            Color3d color = Color3d(0.6 + 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight); // from silver to red
            draw.AddEdge(vertex, vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2)*realtiveWeight, color);
            //cout << " ri = " << ri << "    fieldWeight  " << fieldWeight[ri] << endl;
        }

    }


    //
    // Get avarage of field weights in 30 degrees (local maximus)
    //    
    //int avarageDiapasonLength = static_cast<int>(angleMaxAllowed / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //cout << "angleMaxAllowed = " << angleMaxAllowed << endl;
    //int avarageDiapasonLength = static_cast<int>(20 / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //int avarageDiapasonLength = 50 / rotationDegree; // if rotationDegree == 1 - then 32 degrees
    Ds avarageFieldWeight;
    avarageFieldWeight.setConstant(rotationCount, 0);
    for (int ri = 0; ri < rotationCount; ri++)
    {
        avarageFieldWeight[ri] = fieldWeight[ri];
        //D avarage = fieldWeight[ri]; // this is middle of our internal loop - others values will be added in next for cycle
        //int avarageCount = 1;
        //for (int i = 1; i < avarageDiapasonLength / 2; i++)
        //{
        //    D innerWeight = 1 - (static_cast<D>(i - 1) / (avarageDiapasonLength / 2)); //this weight will decrease noise in mean direction - direction will be mutch more on highest value
        //    int riPrev = ri - i;
        //    if (riPrev < 0)
        //    {
        //        riPrev = isRotationCircular ? (rotationCount + riPrev) : -1; // set circular value only if we have circular roatation loop
        //    }
        //    int riNext = ri + i;
        //    if (riNext > rotationCount - 1)
        //    {
        //        riNext = isRotationCircular ? (riNext - rotationCount) : -1;// set circular value only if we have circular roatation loop
        //    }
        //    if (riPrev != -1)
        //    {
        //        avarage += fieldWeight[riPrev] * innerWeight;
        //        avarageCount++;
        //    }
        //    if (riNext != -1)
        //    {
        //        avarage += fieldWeight[riNext] * innerWeight;
        //        avarageCount++;
        //    }
        //}
        ////DEBUG
        ////if (ri > 350 || ri < 10)
        ////{
        ////    cout << "ri = " << ri << "    avarageCount = " << avarageCount << endl;
        ////}
        //avarageFieldWeight[ri] = (avarageCount != 0)
        //    ? avarage / avarageCount
        //    : 0;
    }

    GetPointDirection_RetriveResults(mesh, draw, vertexIndex, dividesBy, faces, fieldWeight, avarageFieldWeight, isDEBUG_show_results, directions, directionsFaceIds, directionsRotatedByDegrees, false);
}

void Divider::GetPointDirection_BestDir_fast(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees)
{
    //if (vertexIndex != 60)return;
    //if (vertexIndex != 794 && vertexIndex != 406 && vertexIndex != 679 && vertexIndex != 5266) return;
    const bool isDEBUG = options.DebugEnabled;
    const bool isDEBUG_show_results = false;

    directions.clear();
    directionsFaceIds.clear();
    directionsRotatedByDegrees.clear();

    P3 vertex = mesh.V.row(vertexIndex);
    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    if (!mesh.GetVertexFacesAngleSegments(vertexIndex, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
    {
        return;
    }


    //
    // Get two best nrosy field vectors for each face
    //
    const vector<V3s>& YYY = solver.Result.Field;
    assert(YYY.size() == 4 || (YYY.size() == 2 && solver.Result.IsFieldSymetric));
    int N = 4;
    int YYY_size = YYY.size();
    pair<D, int> angles_cos[4];
    vector<V3> fieldVectors(faces.size() * 2);
    vector<D> fieldVectorsWeights(faces.size() * 2);
    for (int i = 0; i < faces.size(); ++i)
    {
        // get nrosy field for face
        int fid = faces[i].FaceId;
        P3 faceCenter = mesh.F_Barycenters.row(fid);
        V3 vectorToFaceCenter = faceCenter - vertex;
        //DEBUG show nrosy field for face
        //for (int i = 0; i < Y.rows(); i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(i).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // sort them by smallest angle
        for (int ni = 0; ni < YYY_size; ni++)
        {
            V3 fieldVector = YYY[ni].row(fid);
            D cos = -utils::vector::Cos(vectorToFaceCenter, fieldVector, true);
            angles_cos[ni] = { cos, ni };
            if (solver.Result.IsFieldSymetric)
            {
                angles_cos[2 + ni] = { -cos, 2 + ni };
            }
        }
        sort(angles_cos, angles_cos + N);

        //DEBUG show 2 field vectors for face
        //for (int ni = 0; ni < 2; ni++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + solver.Result.getField(angles_cos[ni].second, fid).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // set best 2 direction
        int bestIndex1 = angles_cos[0].second;
        int bestIndex2 = angles_cos[1].second;
        V3 dir1 = solver.Result.getField(bestIndex1, fid);
        V3 dir2 = solver.Result.getField(bestIndex2, fid);

        //TODO can be optimized if needed 
        //v0 - simple
        bool swapDir1Dir2 = (utils::vector::AngleFull(dir2, dir1, mesh.F_normals.row(fid)) > 180);
        //v1 - fast
        //bool swapDir1Dir2_v2 = (bestIndex1 < bestIndex2);
        //if (swapDir1Dir2_v2) swap(bestIndex1, bestIndex2);
        //if (bestIndex1 - bestIndex2 >= 3) swapDir1Dir2_v2 = false;
        //cout << "swapDir1Dir2 = " << swapDir1Dir2 << "   swapDir1Dir2_v2=" << swapDir1Dir2_v2 << endl;

        if (swapDir1Dir2)
        {
            // swap dir1 and dir2
            V3 savedir1 = dir1;
            dir1 = dir2;
            dir2 = savedir1;
        }
        fieldVectors[i * 2 + 0] = dir1.transpose();
        fieldVectors[i * 2 + 1] = dir2.transpose();
        // set weights (base on angle)
        D weight1 = 1;
        D weight2 = 1;
        if (options.fieldVectorsWeights_isrelative_to_fielddirection_norm)
        {
            weight1 *= dir1.norm();
            weight2 *= dir2.norm();
        }
        if (options.fieldVectorsWeights_is_fielddirection_norm)
        {
            weight1 = dir1.norm();
            weight2 = dir2.norm();
        }
        fieldVectorsWeights[i * 2 + 0] = weight1;
        fieldVectorsWeights[i * 2 + 1] = weight2;
    }
    //DEBUG show 2 field vectors for face
    if (isDEBUG)
    {
        draw.ReserveEdges(faces.size() * 3);
        for (int i = 0; i < faces.size(); ++i)
        {
            int fid = faces[i].FaceId;
            P3 faceCenter = mesh.F_Barycenters.row(fid);
            for (int ni = 0; ni < 2; ni++)
            {
                V3 fieldVector = fieldVectors[i * 2 + ni];
                D fieldVectorWeight = fieldVectorsWeights[i * 2 + ni];
                D faceWeight = faces[i].weightBasedOnAngle * faces.size();
                P3 endPoint = faceCenter + fieldVector * (mesh.avg_edge_length / 2)*fieldVectorWeight*faceWeight;
                draw.AddEdge(faceCenter, endPoint, Color3d(1, 0, 0));                  // field vector
                //draw.AddLabel(endPoint, to_string(fieldVectorWeight*faceWeight)); // weight of field vector
                //draw.AddLabel(endPoint, "dir" + to_string(ni + 1)); // weight of field vector 
            }
            //draw.AddEdge(faceCenter, vertex, Color3d(0, 1, 0));
        }

    }







    //
    // Get intensity of field around vertex normal (around vertex on 360 degree (could be less))
    //
    vector<int> crossFiedDirectionsCount(faces.size(), 0);
    vector<D> crossFiedDirectionsWeightsSum(faces.size(), 0);
    vector<D> crossFiedDirectionsAnglesSum(faces.size(), 0);
    vector<D> fieldWeight(rotationCount + 1, 0);
    D angleSum = 0;
    int fieldWeightAddedCount = 0;
    //cout << endl << endl;
    for (int i = 0; i < faces.size(); ++i)
    {
        auto& f = faces[i];
        V3 fieldVectorLeft = fieldVectors[i * 2 + 0];
        V3 fieldVectorRight = fieldVectors[i * 2 + 1];
        V3 fNormal = mesh.F_normals.row(f.FaceId);
        D angleRightFull = utils::vector::AngleFull(f.RightSideDirection, fieldVectorRight, fNormal);
        D angleLeftFull = utils::vector::AngleFull(fieldVectorLeft, f.LeftSideDirection, fNormal);
        //cout << "i = " << i << "   angleRightFull = " << angleRightFull << "   angleLeftFull = " << angleLeftFull << endl;
        if (angleRightFull < 180 && angleRightFull < f.angle)
        {
            int index = static_cast<int>(round(angleSum + angleRightFull));
            if (index < 0) index = 0;
            if (index > rotationCount - 1) index = rotationCount - 1;
            fieldWeight[index] = fieldVectorRight.norm();
            fieldWeightAddedCount++;
            //cout << "dividesBy = " << dividesBy << "     fieldWeight[" << index << "] = " << fieldWeight[index] << endl;
            //V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*index);
            //P3 endPoint = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2);
            //draw.AddEdge(vertex, endPoint, Color3d(0.5, 0.5, 0.5));
            //draw.AddLabel(endPoint, "angleRightFull " +to_string(angleRightFull) + "    index "+ to_string(index)); // avarage weight of field vector
        }

        if (angleLeftFull < 180 && angleLeftFull < f.angle)
        {
            int index = static_cast<int>(round(angleSum + f.angle - angleLeftFull));
            if (index < 0) index = 0;
            if (index > rotationCount - 1) index = rotationCount - 1;
            fieldWeight[index] = fieldVectorLeft.norm();
            fieldWeightAddedCount++;
            //cout << "dividesBy = " << dividesBy << "     fieldWeight[" << index << "] = " << fieldWeight[index] << endl;
            //V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*index);
            //P3 endPoint = vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2);
            //draw.AddEdge(vertex, endPoint, Color3d(0.5, 0.5, 0.5));
            //draw.AddLabel(endPoint, "angleLeftFull " + to_string(angleLeftFull) + "    index " + to_string(index)); // avarage weight of field vector
        }

        //if ((angleRightFull > 180 || (!isRotationCircular && i == 0))
        //    && angleLeftFull > 180 || (!isRotationCircular && i == faces.size() - 1))
        //{
        //    crossFiedDirectionsCount[i]++;
        //    D sumRight = (360 - angleRightFull) > 45 ? 0 : (45 - (360 - angleRightFull)) / 45;
        //    crossFiedDirectionsWeightsSum[i] += sumRight;
        //    crossFiedDirectionsAnglesSum[i] += 360 - angleRightFull;
        //    //cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << i << "] = " << crossFiedDirectionsCount[i] << endl;
        //    D sumLeft = (360 - angleLeftFull) > 45 ? 0 : (45 - (360 - angleLeftFull)) / 45;
        //    if (i == faces.size() - 1)
        //    {
        //        if (isRotationCircular)
        //        {
        //            crossFiedDirectionsCount[0]++;
        //            crossFiedDirectionsWeightsSum[0] += sumLeft;
        //            crossFiedDirectionsAnglesSum[0] += 360 - sumLeft;
        //            //cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << 0 << "] = " << crossFiedDirectionsCount[0] << endl;
        //        }
        //    }
        //    else
        //    {
        //        crossFiedDirectionsCount[i + 1]++;
        //        crossFiedDirectionsWeightsSum[i + 1] += sumLeft;
        //        crossFiedDirectionsAnglesSum[i + 1] += 360 - sumLeft;
        //        //cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << (i+1) << "] = " << crossFiedDirectionsCount[i + 1] << endl;
        //    }
        //}

        if (angleRightFull > 180 || (!isRotationCircular && i == 0))
        {
            crossFiedDirectionsCount[i]++;
            D sumRight = (360 - angleRightFull) > 45 ? 0 : (45 - (360 - angleRightFull)) / 45;
            crossFiedDirectionsWeightsSum[i] += sumRight;
            crossFiedDirectionsAnglesSum[i] += 360 - angleRightFull;
            //if (isDEBUG) cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << i << "] = " << crossFiedDirectionsCount[i] << endl;
        }
        if (angleLeftFull > 180 || (!isRotationCircular && i == faces.size() - 1))
        {
            D sumLeft = (360 - angleLeftFull) > 45 ? 0 : (45 - (360 - angleLeftFull)) / 45;
            if (i == faces.size() - 1)
            {
                if (isRotationCircular)
                {
                    crossFiedDirectionsCount[0]++;
                    crossFiedDirectionsWeightsSum[0] += sumLeft;
                    crossFiedDirectionsAnglesSum[0] += 360 - angleLeftFull;
                    //if (isDEBUG) cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << 0 << "] = " << crossFiedDirectionsCount[0] << endl;
                }
            }
            else
            {
                crossFiedDirectionsCount[i + 1]++;
                crossFiedDirectionsWeightsSum[i + 1] += sumLeft;
                crossFiedDirectionsAnglesSum[i + 1] += 360 - angleLeftFull;
                //if (isDEBUG) cout << "dividesBy = " << dividesBy << "     crossFiedDirectionsCount[" << (i + 1) << "] = " << crossFiedDirectionsCount[i + 1] << endl;
            }
        }

        angleSum += f.angle;
    }


    angleSum = 0;
    for (int i = 0; i < faces.size(); ++i)
    {
        auto& f = faces[i];
        if (crossFiedDirectionsCount[i] == 2 && crossFiedDirectionsAnglesSum[i] < 45)
        {
            int index = static_cast<int>(round(angleSum));
            if (index < 0) index = 0;
            if (index > rotationCount - 1) index = rotationCount - 1;
            //fieldWeight[index] = crossFiedDirectionsWeightsSum[i] / crossFiedDirectionsCount[i];
            fieldWeight[index] = (45 - crossFiedDirectionsAnglesSum[i]) / 45;
        }
        angleSum += f.angle;
    }

    // convert weight to relative weight - we need relative weight on all 360 weigths
    D maxWeight = utils::stdvector::max_element(fieldWeight);
    if (maxWeight < 0.000001) maxWeight = 0.000001;
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D weight = fieldWeight[ri];
        D realtiveWeight = weight / maxWeight;
        fieldWeight[ri] = realtiveWeight;
    }
    //DEBUG show avarageNrosyfieldVector
    if (isDEBUG)
    {
        //cout << "********* fieldWeight  *********" << endl;
        draw.ReserveEdges(rotationCount);
        for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
        {
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            D realtiveWeight = fieldWeight[ri];
            Color3d color = Color3d(0.6 + 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight); // from silver to red
            draw.AddEdge(vertex, vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2)*realtiveWeight, color);
            //cout << " ri = " << ri << "    fieldWeight  " << fieldWeight[ri] << endl;
        }

    }


    //
    // Get avarage of field weights in 30 degrees (local maximus)
    //    
    //int avarageDiapasonLength = static_cast<int>(angleMaxAllowed / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //cout << "angleMaxAllowed = " << angleMaxAllowed << endl;
    //int avarageDiapasonLength = static_cast<int>(20 / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //int avarageDiapasonLength = 50 / rotationDegree; // if rotationDegree == 1 - then 32 degrees
    Ds avarageFieldWeight;
    avarageFieldWeight.setConstant(rotationCount, 0);
    for (int ri = 0; ri < rotationCount; ri++)
    {
        avarageFieldWeight[ri] = fieldWeight[ri];
        //D avarage = fieldWeight[ri]; // this is middle of our internal loop - others values will be added in next for cycle
        //int avarageCount = 1;
        //for (int i = 1; i < avarageDiapasonLength / 2; i++)
        //{
        //    D innerWeight = 1 - (static_cast<D>(i - 1) / (avarageDiapasonLength / 2)); //this weight will decrease noise in mean direction - direction will be mutch more on highest value
        //    int riPrev = ri - i;
        //    if (riPrev < 0)
        //    {
        //        riPrev = isRotationCircular ? (rotationCount + riPrev) : -1; // set circular value only if we have circular roatation loop
        //    }
        //    int riNext = ri + i;
        //    if (riNext > rotationCount - 1)
        //    {
        //        riNext = isRotationCircular ? (riNext - rotationCount) : -1;// set circular value only if we have circular roatation loop
        //    }
        //    if (riPrev != -1)
        //    {
        //        avarage += fieldWeight[riPrev] * innerWeight;
        //        avarageCount++;
        //    }
        //    if (riNext != -1)
        //    {
        //        avarage += fieldWeight[riNext] * innerWeight;
        //        avarageCount++;
        //    }
        //}
        ////DEBUG
        ////if (ri > 350 || ri < 10)
        ////{
        ////    cout << "ri = " << ri << "    avarageCount = " << avarageCount << endl;
        ////}
        //avarageFieldWeight[ri] = (avarageCount != 0)
        //    ? avarage / avarageCount
        //    : 0;
    }

    GetPointDirection_RetriveResults(mesh, draw, vertexIndex, dividesBy, faces, fieldWeight, avarageFieldWeight, isDEBUG_show_results, directions, directionsFaceIds, directionsRotatedByDegrees, false);
}

void Divider::GetPointDirection_AvgStream(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees)
{

    //if (vertexIndex != 60)return;
    //if (vertexIndex != 794 && vertexIndex != 406 && vertexIndex != 679 && vertexIndex != 5266) return;
    const bool isDEBUG = options.DebugEnabled;
    const bool isDEBUG_show_results = false;

    directions.clear();
    directionsFaceIds.clear();
    directionsRotatedByDegrees.clear();

    P3 vertex = mesh.V.row(vertexIndex);

    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    if (!mesh.GetVertexFacesAngleSegments(vertexIndex, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
    {
        return;
    }

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

    auto angle45Baddnes = [](D angle)
    {
        const D MAX_ANGLE = 45;
        if (angle > 180) angle = 360 - angle;
        D percent = angle > MAX_ANGLE ? 0 : (MAX_ANGLE - angle) / MAX_ANGLE;
        //percent = percent*percent;
        return 1 - percent;
    };

    auto sumAngle45Baddnes = [&angle45Baddnes, &isDEBUG](D& anglesSum, D angle)
    {
        if (isnan(angle))
        {
            if (isDEBUG)
            {
                cout << "!!!! angle == NAN" << endl;
            }
            return;
        }
        anglesSum += angle45Baddnes(angle);
        //if (anglesSum < angle) anglesSum = angle;
    };

    //
    // Get two best nrosy field vectors for each face
    //
    int N = 4;
    vector<D> anglesTemp;
    anglesTemp.resize(N);
    vector<V3> fieldVectors(faces.size() * 2);
    vector<D> fieldVectorsWeights(faces.size() * 2);
    for (int fi = 0; fi < faces.size(); ++fi)
    {
        const vector<V3s>& YYY = solver.Result.Field;
        // get nrosy field for face
        int fid = faces[fi].FaceId;
        P3 faceCenter = mesh.F_Barycenters.row(fid);
        V3 vectorToFaceCenter = faceCenter - vertex;
        //DEBUG show nrosy field for face
        //for (int i = 0; i < Y.rows(); i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(i).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // sort them by smallest angle
        assert(YYY.size() == 4 || (YYY.size() == 2 && solver.Result.IsFieldSymetric));
        for (int ni = 0; ni < YYY.size(); ni++)
        {
            V3 fieldVector = YYY[ni].row(fid).transpose();
            anglesTemp[ni] = utils::vector::Angle(vectorToFaceCenter, fieldVector);
            if (solver.Result.IsFieldSymetric)
            {
                anglesTemp[2 + ni] = utils::vector::Angle(vectorToFaceCenter, -fieldVector);
            }

        }
        vector<unsigned> sortedIndexes = utils::stdvector::sort_indexes(anglesTemp);
        //DEBUG show 2 field vectors for face
        //for (int i = 0; i < 2; i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(sortedIndexes[i]).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // set best 2 direction
        int bestIndex1 = sortedIndexes[0];
        int bestIndex2 = sortedIndexes[1];
        V3 dir1 = solver.Result.getField(bestIndex1, fid);
        V3 dir2 = solver.Result.getField(bestIndex2, fid);
        if (utils::vector::AngleFull(dir2, dir1, mesh.F_normals.row(fid)) > 180)
        {
            // swap dir1 and dir2
            V3 temp = dir1;
            dir1 = dir2;
            dir2 = temp;
        }
        fieldVectors[fi * 2 + 0] = dir1.transpose();
        fieldVectors[fi * 2 + 1] = dir2.transpose();
        // set weights (base on angle)
        D weight1 = 1;
        D weight2 = 1;
        if (options.fieldVectorsWeights_isrelative_to_fielddirection_norm)
        {
            weight1 *= dir1.norm();
            weight2 *= dir2.norm();
        }
        if (options.fieldVectorsWeights_is_fielddirection_norm)
        {
            weight1 = dir1.norm();
            weight2 = dir2.norm();
        }
        fieldVectorsWeights[fi * 2 + 0] = weight1;
        fieldVectorsWeights[fi * 2 + 1] = weight2;
    }
    //DEBUG show 2 field vectors for face
    if (isDEBUG)
    {
        draw.ReserveEdges(faces.size() * 3);
        for (int i = 0; i < faces.size(); ++i)
        {
            int fid = faces[i].FaceId;
            P3 faceCenter = mesh.F_Barycenters.row(fid);
            for (int ni = 0; ni < 2; ni++)
            {
                V3 fieldVector = fieldVectors[i * 2 + ni];
                D fieldVectorWeight = fieldVectorsWeights[i * 2 + ni];
                D faceWeight = faces[i].weightBasedOnAngle * faces.size();
                P3 endPoint = faceCenter + fieldVector * (mesh.avg_edge_length / 2)*fieldVectorWeight*faceWeight;
                draw.AddEdge(faceCenter, endPoint, Color3d(1, 0, 0));                  // field vector
                                                                                       //draw.AddLabel(endPoint, to_string(fieldVectorWeight*faceWeight)); // weight of field vector
                                                                                       //draw.AddLabel(endPoint, "dir" + to_string(ni + 1)); // weight of field vector 
            }
            //draw.AddEdge(faceCenter, vertex, Color3d(0, 1, 0));
        }

    }


    //
    // Get intensity of field around vertex normal (around vertex on 360 degree (could be less))
    //
    vector<D> fieldWeight(rotationCount);
    vector<int> fieldWeight_faceids_first(rotationCount);
    vector<int> fieldWeight_faceids0(rotationCount);
    vector<int> fieldWeight_faceids1(rotationCount);
    vector<V3> fieldWeight_vectors(rotationCount);
    vector<V3> fieldWeight_vectors_first(rotationCount);
    // use streams for each rotationCount
    MeshStreams s(draw, mesh, solver); // match_with_curl = true works 30% faster and have same results
    s.Reserve(rotationCount);
    for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
    {
        V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
        vertexPlaneTangentI.normalize();
        int faceIndex = faceIndexFromRI(ri);
        int fid = faces[faceIndex].FaceId;
        s.Add(-1, -1, fid, MeshPointType::onVertex, vertexIndex, vertex, vertexPlaneTangentI, Color3d(0.7, 0, 0), -1);
        fieldWeight_faceids_first[ri] = fid;
        fieldWeight_faceids0[ri] = fid;
        fieldWeight_faceids1[ri] = fid;
        fieldWeight[ri] = 0; //  at start weight(summ of angles) is zero
    }
    s.ExtendStreams(1, false); // extend first time to get second point - in such way we will have first line of stream
    // init vector and faceid for first triangle
    for (int ri = 0; ri < rotationCount; ri++)
    {
        if (!s.streamers[ri].finished)
        {
            fieldWeight_vectors_first[ri] = s.streamers[ri].current_end_point - s.streamers[ri].current_start_point;
            fieldWeight_vectors[ri] = fieldWeight_vectors_first[ri];
            fieldWeight_faceids1[ri] = s.streamers[ri].current_fid;
            // add weight based on angle between first triangle directions
            // its not always valid since near to singularity directions sometimes are so screwed
            //int fi = faceIndexFromRI(ri);
            //V3 dir0 = fieldVectors[fi * 2 + 0];
            //V3 dir1 = fieldVectors[fi * 2 + 1];
            //D angle0 = utils::vector::Angle(dir0, fieldWeight_vectors[ri]);
            //D angle1 = utils::vector::Angle(dir1, fieldWeight_vectors[ri]);
            //D angle = min(angle0, angle1);
            //sumAngle45Baddnes(fieldWeight[ri], angle);
        }
    }
    // now extend streams and calculate weight based on stream curvature
    int isoLinesExtensionsCount = options.isoLines_ExtensionsCount;
    int approxExtensionsCount = max(20, min(static_cast<int>(mesh.FacesCount / 20), 1000));
    draw.ReserveEdges(approxExtensionsCount * 5); // reserve space for lines - approximately of what we will add
    for (int extendi = 0; extendi < isoLinesExtensionsCount; extendi++)
    {
        if (s.ExtendStreams(1, false) == 0) break;
        for (int ri = 0; ri < rotationCount; ri++)
        {
            const MeshStreamer& si = s.streamers[ri];
            if (!si.finished)
            {
                V3 currectVector = si.current_end_point - si.current_start_point;
                V3 prev_vector = fieldWeight_vectors[ri];
                int prev_faceid = fieldWeight_faceids0[ri];
                if (options.use_always_first_vector_as_prev)
                {
                    prev_vector = fieldWeight_vectors_first[ri];
                    prev_faceid = fieldWeight_faceids_first[ri];
                }
                V3 currectVectorTranslated = utils::vector::Translate(currectVector, mesh.F_normals.row(fieldWeight_faceids1[ri]), mesh.F_normals.row(prev_faceid), true);
                D angle = utils::vector::Angle(currectVectorTranslated, prev_vector);
                fieldWeight_vectors[ri] = currectVector;
                sumAngle45Baddnes(fieldWeight[ri], angle);
                //fieldWeight[ri] = max(angle, fieldWeight[ri]);
                //draw.AddEdge(s.start_points.row(ri), s.start_points.row(ri) + currectVector, s.colors.row(ri));
                //draw.AddLabel(s.start_points.row(ri), to_string(fieldWeight_faceids0[ri]) + " - "+ to_string(fieldWeight_faceids1[ri]));
                if (isDEBUG && options.Debug_isoLines_show)
                {
                    draw.AddEdge(si.current_start_point, si.current_end_point, Color3d(0.7, 0.7, 0.7));
                    //draw.AddLabel(s.start_points.row(ri), to_string(fieldWeight_faceids[ri]));
                    //draw.AddLabel(s.start_points.row(ri), to_string(angle));
                    //DEBUG - show numbers 
                    //draw.AddEdge(s.start_points.row(i), "  " + to_string(i));
                    //draw.AddPoint(s.start_points.row(i), Color3d(0,0,1));
                    //if (extendi > 1000)
                    //{
                    //    draw.AddLabel(s.start_points.row(i), "AAA");
                    //    break;
                    //}
                }
                fieldWeight_faceids0[ri] = fieldWeight_faceids1[ri];
                fieldWeight_faceids1[ri] = si.current_fid;
            }
        }
    }
    // add weight for those stream that reach surface border at some edge
    for (int ri = 0; ri < rotationCount; ri++)
    {
        if (s.streamers[ri].finished)
        {
            int fid = fieldWeight_faceids0[ri];// since stream is not finished we should take start faceid from first list#0
            V3 constrain_opposite = -solver.Result.Field[1].row(fid); // - so lets take direction from result
            D angle = utils::vector::Angle(constrain_opposite, fieldWeight_vectors[ri]);
            sumAngle45Baddnes(fieldWeight[ri], angle);
            //draw.AddLabel(s.end_points.row(ri), to_string(fid));
        }
    }
    // convert weight to relative weight - we need relative weight on all 360 weigths
    D maxWeight = utils::stdvector::max_element(fieldWeight);
    if (maxWeight < 0.000001) maxWeight = 0.000001;
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D weight = fieldWeight[ri];
        if (isnan(weight))
        {
            cout << "!!!! fieldWeight[" << ri << "] == NAN" << endl;
        }
        D realtiveWeight = (maxWeight - weight) / maxWeight;
        fieldWeight[ri] = realtiveWeight;
    }
    //DEBUG show avarageNrosyfieldVector
    if (isDEBUG)
    {
        //cout << "********* fieldWeight  *********" << endl;
        draw.ReserveEdges(rotationCount);
        for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
        {
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            D realtiveWeight = fieldWeight[ri];
            Color3d color = Color3d(0.6 + 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight); // from silver to red
            draw.AddEdge(vertex, vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2)*realtiveWeight, color);
            //cout << " ri = " << ri << "    fieldWeight  " << fieldWeight[ri] << endl;
        }

    }



    //
    // Get avarage of field weights in 30 degrees (local maximus)
    //    
    //int avarageDiapasonLength = static_cast<int>(angleMaxAllowed / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //cout << "angleMaxAllowed = " << angleMaxAllowed << endl;
    int avarageDiapasonLength = static_cast<int>(35 / rotationDegree); // if rotationDegree == 1 - then 32 degrees
                                                                       //int avarageDiapasonLength = 50 / rotationDegree; // if rotationDegree == 1 - then 32 degrees
    Ds avarageFieldWeight;
    avarageFieldWeight.resize(rotationCount);
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D avarage = fieldWeight[ri]; // this is middle of our internal loop - others values will be added in next for cycle
        int avarageCount = 1;
        for (int i = 1; i < avarageDiapasonLength / 2; i++)
        {
            D innerWeight = 1 - (static_cast<D>(i - 1) / (avarageDiapasonLength / 2)); //this weight will decrease noise in mean direction - direction will be mutch more on highest value
            int riPrev = ri - i;
            if (riPrev < 0)
            {
                riPrev = isRotationCircular ? (rotationCount + riPrev) : -1; // set circular value only if we have circular roatation loop
            }
            int riNext = ri + i;
            if (riNext > rotationCount - 1)
            {
                riNext = isRotationCircular ? (riNext - rotationCount) : -1;// set circular value only if we have circular roatation loop
            }
            if (riPrev != -1)
            {
                avarage += fieldWeight[riPrev] * innerWeight;
                avarageCount++;
            }
            if (riNext != -1)
            {
                avarage += fieldWeight[riNext] * innerWeight;
                avarageCount++;
            }
        }
        //DEBUG
        //if (ri > 350 || ri < 10)
        //{
        //    cout << "ri = " << ri << "    avarageCount = " << avarageCount << endl;
        //}
        avarageFieldWeight[ri] = (avarageCount != 0)
            ? avarage / avarageCount
            : 0;
    }

    GetPointDirection_RetriveResults(mesh, draw, vertexIndex, dividesBy, faces, fieldWeight, avarageFieldWeight, isDEBUG_show_results, directions, directionsFaceIds, directionsRotatedByDegrees, false);

}

void Divider::GetPointDirection_StreamAngles(int vertexIndex, int dividesBy, vector<V3>& directions, vector<int>& directionsFaceIds, vector<D>& directionsRotatedByDegrees)
{

    //if (vertexIndex != 60)return;
    //if (vertexIndex != 794 && vertexIndex != 406 && vertexIndex != 679 && vertexIndex != 5266) return;
    const bool isDEBUG = options.DebugEnabled;
    const bool isDEBUG_show_results = false;

    directions.clear();
    directionsFaceIds.clear();
    directionsRotatedByDegrees.clear();

    P3 vertex = mesh.V.row(vertexIndex);

    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    if (!mesh.GetVertexFacesAngleSegments(vertexIndex, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
    {
        return;
    }

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

    auto angle45Baddnes = [](D angle)
    {
        const D MAX_ANGLE = 45;
        if (angle > 180)
        {
            angle = 360 - angle;
        }
        if (angle < 0)
        {
            angle = -angle;
        }
        D percent = angle > MAX_ANGLE ? 0 : (MAX_ANGLE - angle) / MAX_ANGLE;
        //percent = percent*percent;
        return 1 - percent;
    };

    auto sumAngle45Baddnes = [&angle45Baddnes, &isDEBUG](D& anglesSum, D angle)
    {
        if (isnan(angle))
        {
            if (isDEBUG)
            {
                cout << "!!!! angle == NAN" << endl;
            }
            return;
        }
        anglesSum += angle45Baddnes(angle);
        //if (anglesSum < angle) anglesSum = angle;
    };

    //
    // Get two best nrosy field vectors for each face
    //
    int N = 4;
    vector<D> anglesTemp;
    anglesTemp.resize(N);
    vector<V3> fieldVectors(faces.size() * 2);
    vector<D> fieldVectorsWeights(faces.size() * 2);
    for (int fi = 0; fi < faces.size(); ++fi)
    {
        const vector<V3s>& YYY = solver.Result.Field;
        // get nrosy field for face
        int fid = faces[fi].FaceId;
        P3 faceCenter = mesh.F_Barycenters.row(fid);
        V3 vectorToFaceCenter = faceCenter - vertex;
        //DEBUG show nrosy field for face
        //for (int i = 0; i < Y.rows(); i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(i).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // sort them by smallest angle
        assert(YYY.size() == 4 || (YYY.size() == 2 && solver.Result.IsFieldSymetric));
        for (int ni = 0; ni < YYY.size(); ni++)
        {
            V3 fieldVector = YYY[ni].row(fid).transpose();
            anglesTemp[ni] = utils::vector::Angle(vectorToFaceCenter, fieldVector); //TODO fieldVector seems to be normalized + use cos instead of angle 
            if (solver.Result.IsFieldSymetric)
            {
                //anglesTemp[2 + ni] = utils::vector::Angle(vectorToFaceCenter, -fieldVector);
                anglesTemp[2 + ni] = 180 - anglesTemp[ni];
            }
        }
        //cout << "anglesTemp={" << anglesTemp[0] << "," << anglesTemp[1] << "," << anglesTemp[2] << "," << anglesTemp[3] <<"}"<< endl;
        vector<unsigned> sortedIndexes = utils::stdvector::sort_indexes(anglesTemp);
        //DEBUG show 2 field vectors for face
        //for (int i = 0; i < 2; i++)
        //{
        //    draw.AddEdge(faceCenter, faceCenter + Y.row(sortedIndexes[i]).transpose()*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
        //}

        // set best 2 direction
        int bestIndex1 = sortedIndexes[0];
        int bestIndex2 = sortedIndexes[1];
        V3 dir1 = solver.Result.getField(bestIndex1, fid);
        V3 dir2 = solver.Result.getField(bestIndex2, fid);
        if (utils::vector::AngleFull(dir2, dir1, mesh.F_normals.row(fid)) > 180) //TODO dir1 and dir2 seems to be normalized + use cos instead of angle 
        {
            // swap dir1 and dir2
            V3 savedir1 = dir1;
            dir1 = dir2;
            dir2 = savedir1;
        }
        //cout << "dir1=" << dir1 << "    dir2=" << dir2 << endl;
        fieldVectors[fi * 2 + 0] = dir1.transpose();
        fieldVectors[fi * 2 + 1] = dir2.transpose();
        // set weights (base on angle)
        D weight1 = 1;
        D weight2 = 1;
        if (options.fieldVectorsWeights_isrelative_to_fielddirection_norm)
        {
            weight1 *= dir1.norm();
            weight2 *= dir2.norm();
        }
        if (options.fieldVectorsWeights_is_fielddirection_norm)
        {
            weight1 = dir1.norm();
            weight2 = dir2.norm();
        }
        fieldVectorsWeights[fi * 2 + 0] = weight1;
        fieldVectorsWeights[fi * 2 + 1] = weight2;
        //cout << "weight1=" << weight1 << "    weight2=" << weight2 << endl;
    }
    //DEBUG show 2 field vectors for face
    if (isDEBUG)
    {
        draw.ReserveEdges(faces.size() * 3);
        for (int i = 0; i < faces.size(); ++i)
        {
            int fid = faces[i].FaceId;
            P3 faceCenter = mesh.F_Barycenters.row(fid);
            for (int ni = 0; ni < 2; ni++)
            {
                V3 fieldVector = fieldVectors[i * 2 + ni];
                D fieldVectorWeight = fieldVectorsWeights[i * 2 + ni];
                D faceWeight = faces[i].weightBasedOnAngle * faces.size();
                P3 endPoint = faceCenter + fieldVector * (mesh.avg_edge_length / 2)*fieldVectorWeight*faceWeight;
                draw.AddEdge(faceCenter, endPoint, Color3d(1, 0, 0));                  // field vector
                                                                                       //draw.AddLabel(endPoint, to_string(fieldVectorWeight*faceWeight)); // weight of field vector
                                                                                       //draw.AddLabel(endPoint, "dir" + to_string(ni + 1)); // weight of field vector 
            }
            //draw.AddEdge(faceCenter, vertex, Color3d(0, 1, 0));
        }

    }


    //
    // Get intensity of field around vertex normal (around vertex on 360 degree (could be less))
    //
    vector<D> fieldWeight(rotationCount);
    vector<D> fieldWeight_PositiveNegativeAnglesMaxSum(rotationCount);
    MeshStreams s(draw, mesh, solver); // use streams for each rotationCount, match_with_curl = true works 30% faster and have same results
    s.Reserve(rotationCount);
    for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
    {
        V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
        vertexPlaneTangentI.normalize();
        int faceIndex = faceIndexFromRI(ri);
        int fid = faces[faceIndex].FaceId;
        s.Add(-1, -1, fid, MeshPointType::onVertex, vertexIndex, vertex, vertexPlaneTangentI, Color3d(0.7, 0, 0), -1);
        fieldWeight[ri] = 0; //  at start weight(summ of angles) is zero
        fieldWeight_PositiveNegativeAnglesMaxSum[ri] = 0;
    }
    int isoLinesExtensionsCount = options.isoLines_ExtensionsCount;
    int approxExtensionsCount = max(20, min(mesh.FacesCount / 20, 1000));
    if (isDEBUG && options.Debug_isoLines_show) draw.ReserveEdges(approxExtensionsCount * 5); // reserve space for lines - approximately of what we will add
    s.ExtendStreams(isoLinesExtensionsCount); //extend streams
    // calculate weight based on stream curvature
    for (int ri = 0; ri < rotationCount; ri++)
    {
        if (isDEBUG && options.Debug_ri_index != -1 && options.Debug_ri_index != ri) continue;
        const MeshStreamer& si = s.streamers[ri];
        const MeshStream& points = s[ri];
        if (points.size() < 2) continue;
        int first_fid = points[0].fid;
        V3 first_vector = points[1].point - points[0].point;
        V3 first_normal = mesh.F_normals.row(first_fid);
        int prev_fid = first_fid;
        V3 prev_vector = first_vector;
        V3 prev_normal = first_normal;
        int positiveAnglesCount = 0;
        D positiveAngleMax = 0;
        int negativeAnglesCount = 0;
        D negativeAngleMax = 0;
        int iMax = points.size() - 2;
        if (si.finished) iMax++;// include point on border
        for (int i = 1; i <= iMax; i++)
        {
            bool isCurrentVector_BorderConstrain = (i == iMax) && (si.finished);
            int current_fid = !isCurrentVector_BorderConstrain
                ? points[i].fid
                : points[i - 1].fid; //  for point on border we will take prev fid, since current will be -1
            V3 currect_vector = !isCurrentVector_BorderConstrain
                ? (points[i + 1].point - points[i].point).eval()
                : -solver.Result.Field[1].row(current_fid).transpose();
            //int current_fid = points[i].fid;
            //V3 currect_vector = points[i + 1].point - points[i].point;
            V3 currect_normal = mesh.F_normals.row(current_fid);

            D angle_with_prev = utils::vector::AngleFull(prev_vector, utils::vector::Translate(currect_vector, currect_normal, prev_normal, true), prev_normal);
            D angle_with_first = utils::vector::AngleFull(first_vector, utils::vector::Translate(currect_vector, currect_normal, first_normal, true), first_normal);
            if (angle_with_prev > 180)
            {
                angle_with_prev = angle_with_prev - 360;
            }
            if (angle_with_prev > 0)
            {
                positiveAnglesCount++;
                positiveAngleMax = max(positiveAngleMax, angle_with_prev);
            }
            else
            {
                negativeAnglesCount++;
                negativeAngleMax = min(negativeAngleMax, angle_with_prev);
            }
            sumAngle45Baddnes(fieldWeight[ri], options.use_always_first_vector_as_prev ? angle_with_first : angle_with_prev);
            //fieldWeight[ri] = max(angle, fieldWeight[ri]);
            //draw.AddEdge(s.start_points.row(ri), s.start_points.row(ri) + currectVector, s.colors.row(ri));
            //draw.AddLabel(s.start_points.row(ri), to_string(fieldWeight_faceids0[ri]) + " - "+ to_string(fieldWeight_faceids1[ri]));
            if (isDEBUG && options.Debug_isoLines_show)
            {
                draw.AddEdge(points[i].point, points[i + 1].point, Color3d(0.7, 0.7, 0.7));
                //draw.AddLabel(s.start_points.row(ri), to_string(fieldWeight_faceids[ri]));
                //draw.AddLabel(s.start_points.row(ri), to_string(angle));
                //DEBUG - show numbers 
                //draw.AddEdge(s.start_points.row(i), "  " + to_string(i));
                //draw.AddPoint(s.start_points.row(i), Color3d(0,0,1));
                //if (extendi > 1000)
                //{
                //    draw.AddLabel(s.start_points.row(i), "AAA");
                //    break;
                //}
            }
            if (isDEBUG &&  options.Debug_ri_index != -1 && options.Debug_ri_index == ri)
            {
                draw.AddEdge(points[i].point, points[i - 1].point, Color3d(0.2, 0.2, 0.2));
                draw.AddPoint(points[i].point, Color3d(0.2, 0.2, 0.2), "  " + to_string(angle_with_prev));
            }

            prev_fid = current_fid;
            prev_vector = currect_vector;
            prev_normal = currect_normal;

        }
        if (isDEBUG && options.Debug_ri_index != -1)
        {
            //cout << "anglesCount = [positive " << positiveAnglesCount << ", negative " << negativeAnglesCount << "]" << "    anglesMax = [positive " << positiveAngleMax << ", negative " << negativeAngleMax << "]" << endl;
            cout << "anglesCount = [positive " << positiveAnglesCount << ", negative " << negativeAnglesCount << "]" << endl;
            cout << "anglesMax   = [positive " << positiveAngleMax << ", negative " << negativeAngleMax << "]" << endl;
        }
        // v1
        fieldWeight_PositiveNegativeAnglesMaxSum[ri] = positiveAngleMax + (-negativeAngleMax);
        // v2
        //fieldWeight_PositiveNegativeAnglesMaxSum[ri] = 0;
        //if (positiveAnglesCount > 0) fieldWeight_PositiveNegativeAnglesMaxSum[ri] += positiveAngleMax / positiveAnglesCount;
        //if (negativeAnglesCount > 0) fieldWeight_PositiveNegativeAnglesMaxSum[ri] += (-negativeAngleMax / negativeAnglesCount);
    }
    if (isDEBUG && options.Debug_ri_index != -1)
    {
        return;
    }
    //DEBUG - show non zero weights
    //for (int ri = 0; ri < rotationCount; ri++)
    //{
    //    if (fieldWeight[ri] > 0)
    //    {
    //        cout << "ri = " << ri <<"   weight = "<<fieldWeight[ri]<< endl;
    //    }
    //} 
    // convert weight to relative weight - we need relative weight on all 360 weigths
    for (int i = 0; i < 2; i++)
    {
        vector<D>& fieldWeightREL = (i == 0) ? fieldWeight : fieldWeight_PositiveNegativeAnglesMaxSum;
        D maxWeight = utils::stdvector::max_element(fieldWeightREL);
        if (maxWeight < 0.000001) maxWeight = 0.000001;
        for (int ri = 0; ri < rotationCount; ri++)
        {
            D weight = fieldWeightREL[ri];
            if (isnan(weight))
            {
                cout << "!!!! fieldWeight[" << ri << "] == NAN" << endl;
            }
            D realtiveWeight = (maxWeight - weight) / maxWeight;
            if (weight < 0) realtiveWeight = 0;
            fieldWeightREL[ri] = realtiveWeight;
        }
    }
    //DEBUG show avarageNrosyfieldVector
    if (isDEBUG)
    {
        //cout << "********* fieldWeight  *********" << endl;
        draw.ReserveEdges(rotationCount);
        for (int ri = 0; ri < rotationCount; ri++) // for each ratation of 1 degree around plane normal
        {
            //vector<D>& fieldWeightREL = fieldWeight;
            vector<D>& fieldWeightREL = fieldWeight_PositiveNegativeAnglesMaxSum;
            V3 vertexPlaneTangentI = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotationDegree*ri);
            D realtiveWeight = fieldWeightREL[ri];
            Color3d color = Color3d(0.6 + 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight, 0.6 - 0.4*realtiveWeight); // from silver to red
            draw.AddEdge(vertex, vertex + vertexPlaneTangentI.normalized()*(mesh.avg_edge_length / 2)*realtiveWeight, color);
            //cout << " ri = " << ri << "    fieldWeightREL  " << fieldWeightREL[ri] << endl;
        }

    }



    //
    // Get avarage of field weights in 30 degrees (local maximus)
    //    
    //int avarageDiapasonLength = static_cast<int>(angleMaxAllowed / rotationDegree); // if rotationDegree == 1 - then 32 degrees
    //cout << "angleMaxAllowed = " << angleMaxAllowed << endl;
    int avarageDiapasonLength = static_cast<int>(35 / rotationDegree); // if rotationDegree == 1 - then 32 degrees
                                                                       //int avarageDiapasonLength = 50 / rotationDegree; // if rotationDegree == 1 - then 32 degrees
    Ds avarageFieldWeight;
    avarageFieldWeight.resize(rotationCount);
    for (int ri = 0; ri < rotationCount; ri++)
    {
        D avarage = fieldWeight[ri]; // this is middle of our internal loop - others values will be added in next for cycle
        int avarageCount = 1;
        for (int i = 1; i < avarageDiapasonLength / 2; i++)
        {
            D innerWeight = 1 - (static_cast<D>(i - 1) / (avarageDiapasonLength / 2)); //this weight will decrease noise in mean direction - direction will be mutch more on highest value
            int riPrev = ri - i;
            if (riPrev < 0)
            {
                riPrev = isRotationCircular ? (rotationCount + riPrev) : -1; // set circular value only if we have circular roatation loop
            }
            int riNext = ri + i;
            if (riNext > rotationCount - 1)
            {
                riNext = isRotationCircular ? (riNext - rotationCount) : -1;// set circular value only if we have circular roatation loop
            }
            if (riPrev != -1)
            {
                avarage += fieldWeight[riPrev] * innerWeight;
                avarageCount++;
            }
            if (riNext != -1)
            {
                avarage += fieldWeight[riNext] * innerWeight;
                avarageCount++;
            }
        }
        //DEBUG
        //if (ri > 350 || ri < 10)
        //{
        //    cout << "ri = " << ri << "    avarageCount = " << avarageCount << endl;
        //}
        avarageFieldWeight[ri] = (avarageCount != 0)
            ? avarage / avarageCount
            : 0;

        //avarageFieldWeight[ri] = fieldWeight[ri];
    }


    //
    // Get results based on 'avarageFieldWeight'
    //
    GetPointDirection_RetriveResults(mesh, draw, vertexIndex, dividesBy, faces, fieldWeight, avarageFieldWeight, isDEBUG_show_results, directions, directionsFaceIds, directionsRotatedByDegrees, false);

    //
    // Improve results base on 'fieldWeight_PositiveNegativeAnglesMaxSum'
    //
    if (options.improve_stream_angles && isRotationCircular)
    {
        int diretcionsCount = directions.size();
        for (int pi = 0; pi < diretcionsCount; pi++)
        {
            D max_change_angle = 25;
            int piPrev = pi - 1;
            if (piPrev < 0) piPrev = diretcionsCount - 1;
            int piNext = pi + 1;
            if (piNext > diretcionsCount - 1) piNext = 0;

            int riStart = static_cast<int>(round(directionsRotatedByDegrees[pi] / rotationDegree));
            int riDecr = static_cast<int>((min(max_change_angle, utils::vector::Angle(directions[pi], directions[piPrev]) / 3)) / rotationDegree);
            int riIncr = static_cast<int>((min(max_change_angle, utils::vector::Angle(directions[pi], directions[piNext]) / 3)) / rotationDegree);
            int riMin = riStart - riDecr;
            int riMax = riStart + riIncr;

            int riBest = riStart;

            for (int ri = riStart - 1; ri >= riMin; ri--)
            {
                int ri360 = ri;
                if (ri360 < 0)
                {
                    ri360 = rotationCount + ri360;
                }
                if (fieldWeight_PositiveNegativeAnglesMaxSum[ri360] > fieldWeight_PositiveNegativeAnglesMaxSum[riBest])
                {
                    riBest = ri360;
                }
                else
                {
                    break;
                }
            }
            for (int ri = riStart + 1; ri <= riMax; ri++)
            {
                int ri360 = ri;
                if (ri360 > rotationCount - 1)
                {
                    ri360 = ri360 - rotationCount;
                }
                if (fieldWeight_PositiveNegativeAnglesMaxSum[ri360] > fieldWeight_PositiveNegativeAnglesMaxSum[riBest])
                {
                    riBest = ri360;
                }
                else
                {
                    break;
                }
            }
            if (riStart != riBest)
            {
                D rotatedByDegree = rotationDegree * riBest;
                V3 direction = getAngleFacePlaneVector(faces, facesAngleSegments, vertexIndex, angleSumm360, mesh, rotatedByDegree);
                directionsRotatedByDegrees[pi] = rotatedByDegree;
                directions[pi] = direction.normalized();
                directionsFaceIds[pi] = faces[getFacesIndex(faces, facesAngleSegments, rotatedByDegree)].FaceId;
            }
        }
    }

}

StreamStartPoint* Divider::FindStartPoint(int id)
{
    for (auto& sp : startPoints) if (sp.id == id) return &sp;
    cout << "!!!   Divider::FindStartPoint()  failed for id=" << id << endl;
    assert(false && "Divider::FindStartPoint()  failed");
    return nullptr;
}

StreamAnchorPoint* Divider::FindAnchorPoint(int id)
{
    for (auto& ap : anchorPoints) if (ap.id == id) return &ap;
    cout << "!!!   Divider::FindAnchorPoint()  failed for id=" << id << endl;
    assert(false && "Divider::FindAnchorPoint()  failed");
    return nullptr;
}

void Divider::SignContract(StreamStartPoint& sp0, StreamAnchorPoint& ap0, StreamStartPoint& sp1, StreamAnchorPoint& ap1)
{
    //sp0.haveContract = true;
    //sp0.contract = { ap1.id, 0 };
    //ap0.haveContract = true;
    //ap0.contract = { ap0.id, 0 };

    //sp1.haveContract = true;
    //sp1.contract = { ap0.id, 0 };
    //ap1.haveContract = true;
    //ap1.contract = { ap1.id, 0 };

    sp0.SignContract(ap1); // force connect sp0 to ap1
    sp1.SignContract(ap0); // force connect sp1 to ap0
}
