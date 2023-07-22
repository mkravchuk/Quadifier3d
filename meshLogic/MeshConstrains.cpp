#include "stdafx.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "FaceConstrain.h"
#include "MeshConstrains.h"
#include <igl/null.h>

const MeshLogicOptions_Constrains& options = meshLogicOptions.Constrains;


MeshConstrains::MeshConstrains(const Mesh& _mesh, ViewerDrawObjects& _draw)
    : mesh(_mesh), draw(_draw), Type(MeshLogicOptions_Constrains::MeshConstrainsType::AllEdgesWithCorrectionInCorners)
{
}


II MeshConstrains::SizeOF() const
{
    return sizeof(MeshConstrains)
        + Constrains.capacity() * sizeof(FaceConstrain);
}

void MeshConstrains::GetConstrains(const vector<Mesh>& meshes, MeshLogicOptions_Constrains::MeshConstrainsType type, vector<MeshConstrains>& constrains)
{
    // create empty contrain classes in single threading
    constrains.clear();
    constrains.reserve(meshes.size());
    ViewerDrawObjects draw;
    for (int i = 0; i < meshes.size(); i++)
    {
        MeshConstrains con(meshes[i], draw); //pospound call Init to make it in multitreading
        constrains.push_back(con);
    }

    // initialize class in mutitreading
    extern bool IsOmpEnabled;
    #pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int i = 0; i < constrains.size(); i++)
    {
        constrains[i].Init(type);
    }
}

void MeshConstrains::Init(MeshLogicOptions_Constrains::MeshConstrainsType type)
{
    Type = type;
    // prepare Contrains - clear and reserve space for better performance
    Constrains.clear();
    int surfaceLoopEdgesCount = 0;
    for (const MeshLoop& loop : mesh.Loops)
    {
        surfaceLoopEdgesCount += loop.Length;
    }
    Constrains.reserve(surfaceLoopEdgesCount);


    switch (Type)
    {
        case MeshLogicOptions_Constrains::MeshConstrainsType::AllEdgesWithCorrectionInCorners:
            Init_AllEdgesWithCorrectionInCorners();
            break;
        case MeshLogicOptions_Constrains::MeshConstrainsType::QuadOnly:
            Init_QuadOnly();
            break;
        default:
            cout << endl << endl << "!!! Exception:   Unknown SurfaceConstrainType !!!" << endl << endl << endl;
            assert(false);
            break;
    }
}

void MeshConstrains::Clear()
{
    Constrains.clear();
}

void MeshConstrains::Init_AllEdgesWithCorrectionInCorners()
{
    int nexId = 0;

    // reserve space for loop constrains
    int countOfEdgesInAllLoops = 0;
    for (int i = 0; i < mesh.Loops.size(); i++)
    {
        countOfEdgesInAllLoops += mesh.Loops[i].edges.size();
    }
    Constrains.reserve(countOfEdgesInAllLoops);

    // add loop constrains
    vector<V3> edgeConstraints;
    vector<V3> edgeConstraintsCorrected;
    vector<D> correctionAngles;
    for (const MeshLoop& loop : mesh.Loops)
    {
        GetBorderConstrains(loop, edgeConstraints, edgeConstraintsCorrected, correctionAngles);
        for (int i = 0; i < loop.edges.size(); i++)
        {
            const MeshLoopEdge& e = loop.edges[i];
            FaceConstrain fc(nexId, e.FaceId, e.EdgeId);
            fc.DirectionX = e.directionAlongEdge / e.Length;
            fc.DirectionY = edgeConstraints[i];
            fc.DirectionY_Corrected = edgeConstraintsCorrected[i];
            fc.DirectionY_Corrected_Angle = correctionAngles[i];
            Constrains.push_back(fc);
            nexId++;
        }
    }

    // add sharp edges constrains - for organic meshes - betta version of algorithm - need to be improved to escape random sharpness
    AddContrainsInSharpEdges(nexId);
}


struct SharpPointAngles
{
    bool accepted;
    D angle;
    int index_points;
    int stopLength;

    int countAngleCloserTo0;
    int countAngleCloserTo90;
    D stopAngleAvarageSum;
    bool isAngleStraightTo0;
    bool isAngleStraightTo90;
    D sumAngleChangePrev;
    D sumAngleChangeNext;

    SharpPointAngles(bool _accepted, D _angle, int _index_points, int _stopLength)
        : accepted(_accepted), angle(_angle), index_points(_index_points), stopLength(_stopLength),
        countAngleCloserTo0(0), countAngleCloserTo90(0), stopAngleAvarageSum(0), isAngleStraightTo0(false), isAngleStraightTo90(false), sumAngleChangePrev(0), sumAngleChangeNext(0)
    {
    }
};

void MeshConstrains::CorrectConstrainesInAngles(const MeshLoop& loop, vector<V3>& constrains, vector<D>& correctionAngles) const
{
    const bool isDEBUG = options.CorrectInAngles_Debug;
    const D CORRECT_PERCENT = options.CorrectInAngles_Percent;
    const bool CORRECT_PERCENT_ISDYNAMIC = options.CorrectInAngles_Percent_IsDynamic;
    const bool CORRECT_PERCENT_ISPROGRESSIVE = options.CorrectInAngles_Percent_isProgressive;
    const bool CORRECT_straightTo90_angleX2 = options.CorrectInAngles_straightTo90_angleX2;
    const bool FORCE_4side_to_straightTo90 = options.CorrectInAngles_force_4side_to_straightTo90;
    const int SKIPP_LAST_EDGES_COUNT = options.CorrectInAngles_skipp_last_edges_count;
    const bool CORRECT_ForceTo0_90to0 = options.CorrectInAngles_ForceTo0_90to0;


    const P3s& V = mesh.V;
    const V3s& F_normals = mesh.F_normals;

    int Length = loop.Length;
    const vector<MeshLoopEdge>& edges = loop.edges;
    const vector<MeshLoopPoint>& points = loop.points;
    vector<D> constrainCorrectionAngle(Length, 0);
    vector<int> constrainCorrectionAngleCount(Length, 0);

    int sharpPointsCount = 0;
    for (int i = 0; i < Length; i++)
    {
        if (points[i].IsSharp)
        {
            sharpPointsCount++;
        }
    }

    vector<SharpPointAngles> shaprs;

    for (int i = 0; i < Length; i++)
    {
        //if (points[i].AngleBetweenEdgesFull > 180) continue;

        if (points[i].IsSharp)
        {
            //if (points[i].AngleBetweenEdgesFull > 27
            //if (points[i].EdgeIdBackward != 9945 && points[i].EdgeIdBackward != 9876) continue;

            //
            // Init
            //

            int iPrev = i - 1;
            if (iPrev < 0) iPrev = Length - 1;
            int iNext = i;
            D angle = points[i].AngleBetweenEdges;
            if (angle > 90) angle = 180 - angle;
            //V3 avgNormal = (edges[iPrev].directionToFace + edges[iNext].directionToFace) / 2;
            if (isDEBUG) cout << endl << "--- Sharp edge  " << points[i].EdgeIdBackward << "-" << points[i].EdgeIdForward << " --- " << angle << "    " << endl;

            shaprs.push_back(SharpPointAngles(false, angle, i, 0));
            //if (90 - angle < 2) continue;

            //
            // Find set of edges
            //
            int stopiPrev = iPrev;
            int stopiNext = iNext;
            D stopAngle = angle;
            D angleI = angle;
            int stopLength = 0;
            D sumAngleChangePrev = 0;
            D sumAngleChangeNext = 0;
            D angleChangePrev = 0;
            D angleChangeNext = 0;
            if (isDEBUG) cout << "#" << stopLength << "  angle " << angleI << "  edges  " << edges[iPrev].EdgeId << "-" << edges[iNext].EdgeId << endl;
            int countAngleCloserTo0 = 0;
            int countAngleCloserTo90 = 0;
            D stopAngleAvarageSum = 0;
            while (angleI > 0 && angleI < 90)
            {
                sumAngleChangePrev += angleChangePrev;
                sumAngleChangeNext += angleChangeNext;
                stopiPrev = iPrev;
                stopiNext = iNext;
                stopAngle = angleI;
                if (stopAngle < 50)
                    countAngleCloserTo0++;
                else
                    countAngleCloserTo90++;
                stopAngleAvarageSum += stopAngle;
                stopLength++;

                // stop if we reach sharp point
                int iNextNext = iNext + 1;
                if (iNextNext > Length - 1) iNextNext = 0;
                if (points[iPrev].IsSharp || points[iNextNext].IsSharp)
                {
                    break;
                }

                int iPrevPoint = iPrev;
                iPrev--;
                if (iPrev < 0) iPrev = Length - 1;
                iNext++;
                if (iNext > Length - 1) iNext = 0;
                // fail if we back to start index
                if (iPrev == i || iPrevPoint == i || iNext == i)
                {
                    stopLength = 0;
                    break;
                }

                angleChangePrev = points[iPrevPoint].AngleBetweenEdges;
                //cout << "anglePrevChange = " << anglePrevChange << endl;
                if (points[iPrevPoint].AngleBetweenEdgesFull > 180) angleChangePrev = -angleChangePrev;
                angleChangeNext = points[iNext].AngleBetweenEdges;
                //cout << "angleNextChange = " << angleNextChange << endl;
                if (points[iNext].AngleBetweenEdgesFull > 180) angleChangeNext = -angleChangeNext;

                //stopAngle = utils::vector::Angle(edges[iPrev].directionToFace, edges[iNext].directionToFace);
                if (points[i].AngleBetweenEdgesFull < 90)
                    angleI -= angleChangePrev + angleChangeNext;
                else
                    angleI += angleChangePrev + angleChangeNext;

                if (isDEBUG) cout << "#" << stopLength << "  angle " << angleI << "  edges  " << edges[iPrev].EdgeId << "-" << edges[iNext].EdgeId << endl;
            }
            if (stopLength < 0) stopLength = 0;
            sumAngleChangePrev = abs(sumAngleChangePrev);
            sumAngleChangeNext = abs(sumAngleChangeNext);
            D sumAngleChangeTotal = sumAngleChangePrev + sumAngleChangeNext;

            //
            // Check set of found edges
            //
            if (stopLength == 0) continue;
            D stopAngleAvarage = stopAngleAvarageSum / stopLength;
            //bool isAngleStraightTo0 = (angle < 50 && stopAngle < 50);
            //bool isAngleStraightTo90 = (angle > 90 - 50 && stopAngle > 90 - 50);
            //if (isAngleStraightTo90) isAngleStraightTo0 = false;
            //bool isAngleStraightTo0 = (countAngleCloserTo0 > countAngleCloserTo90);
            bool isAngleStraightTo0 = (angle < 65 && stopAngleAvarage < 50) || (countAngleCloserTo0 > countAngleCloserTo90 * 5);
            bool isAngleStraightTo90 = !isAngleStraightTo0;

            // if we have 4 sides - make all sharp corners 90 degree
            if (FORCE_4side_to_straightTo90 && sharpPointsCount == 4)
            {
                isAngleStraightTo0 = false;
                isAngleStraightTo90 = true;
            }
            else
            {
                if (isAngleStraightTo90 && angle > stopAngle) // && points[i].AngleBetweenEdgesFull < 120
                {
                    isAngleStraightTo90 = false;
                    if (CORRECT_ForceTo0_90to0)
                    {
                        if (stopAngle < 15 && sharpPointsCount == 2) isAngleStraightTo0 = true;
                    }
                }
            }

            D angleChange = isAngleStraightTo90
                ? 90 - min(stopAngle, angle)
                : max(stopAngle, angle);

            bool accept = (angleChange > 5) && (isAngleStraightTo90 || isAngleStraightTo0);
            //DEBUG
            if (isDEBUG)
            {
                cout << "stopLength " << stopLength << "  stopAngle " << stopAngle << "  angleChange " << angleChange << "  edges  " << edges[stopiPrev].EdgeId << "-" << edges[stopiNext].EdgeId << " ---" << endl;
                cout << "countAngleCloserTo0 = " << countAngleCloserTo0 << "    countAngleCloserTo90 = " << countAngleCloserTo90 << "   stopAngleAvarage = " << stopAngleAvarage << endl;
                cout << "sumAngleChangePrev " << sumAngleChangePrev << "  sumAngleChangeNext " << sumAngleChangeNext << "  sumAngleChangeTotal  " << sumAngleChangeTotal << endl;

                if (accept)
                {
                    cout << "accept:";
                    cout << "   angle change " << (int)round(angleChange);
                    cout << "   straight to "; cout << (isAngleStraightTo0 ? "0" : "90");
                    cout << endl;
                    cout << "sumAngleChangeTotal " << (int)round(sumAngleChangeTotal) << "   sumAngleChangePrev " << (int)round(sumAngleChangePrev) << "   sumAngleChangeNext " << (int)round(sumAngleChangeNext) << endl;
                    P3 P1 = V.row(points[stopiPrev].VertexId);
                    int pointstopiNext = stopiNext + 1;
                    if (pointstopiNext > Length - 1) pointstopiNext = 0;
                    P3 P2 = V.row(points[pointstopiNext].VertexId);
                    draw.AddEdge(P1, P2, Color3d(1, 0, 0));
                    //draw.AddLabel((P1 + P2) / 2, isAngleStraightTo180 ? "180" : "90", Color4d(1, 0, 0, 1));
                    draw.AddLabel((P1 + P2) / 2, "   stopAngleAvarage " + to_string(static_cast<int>(round(stopAngleAvarage))) + "   angle change " + to_string(static_cast<int>(round(angleChange))) + "    " + (isAngleStraightTo0 ? "isAngleStraightTo0" : "isAngleStraightTo90"), Color3d(1, 0, 0));
                }
                else
                {
                    cout << "fail" << endl;
                }
            }

            SharpPointAngles& s = shaprs.back();
            if (accept)
            {
                s.accepted = true;
                s.stopLength = stopLength;
                s.countAngleCloserTo0 = countAngleCloserTo0;
                s.countAngleCloserTo90 = countAngleCloserTo90;
                s.stopAngleAvarageSum = stopAngleAvarageSum;
                s.isAngleStraightTo0 = isAngleStraightTo0;
                s.isAngleStraightTo90 = isAngleStraightTo90;
                s.sumAngleChangePrev = sumAngleChangePrev;
                s.sumAngleChangeNext = sumAngleChangeNext;
            }
            else
            {
                s.accepted = false;
            }

        }
    }

    //
    // Adjust sharp angles
    //
    //if (isDEBUG)cout << endl << "---Adjust sharp angles---" << endl << endl;
    if (shaprs.size() > 2)
        for (int i = 0; i < shaprs.size(); i++)
        {
            int iNext = i + 1;
            if (iNext > shaprs.size() - 1) iNext = 0;
            if (iNext == i) continue;

            //DEBUG
            //SharpPointAngles& s = shaprs[i];
            //const MeshLoopPoint& p = points[s.index_points];
            //const V3& point3d = V.row(p.VertexId);
            //draw.AddLabel(point3d, (s.isAngleStraightTo0 ? "#" + to_string(i) + "  isAngleStraightTo0" : "isAngleStraightTo90"));

            if (shaprs[i].accepted && shaprs[iNext].accepted)
            {
                SharpPointAngles* s1 = &shaprs[i]; // use pointer to be able reassign reference to s2 if needed
                SharpPointAngles* s2 = &shaprs[iNext];// use pointer to be able reassign  reference to s1 if needed

                if (s1->isAngleStraightTo90 && s2->isAngleStraightTo0)
                {
                    s2 = &shaprs[i];
                    s1 = &shaprs[iNext];
                }

                if (s1->isAngleStraightTo0 && s2->isAngleStraightTo90)
                {
                    const MeshLoopPoint& p1 = points[s1->index_points];
                    const MeshLoopPoint& p2 = points[s2->index_points];
                    if (p1.AngleBetweenEdgesFull < 70 && p2.AngleBetweenEdgesFull > 110 && p2.AngleBetweenEdgesFull < 160)
                    {
                        D stopAngleAvarage = (s1->stopAngleAvarageSum + s2->stopAngleAvarageSum) / (s1->stopLength + s2->stopLength);
                        if (stopAngleAvarage < 40)
                        {
                            s2->isAngleStraightTo0 = true;
                            s2->isAngleStraightTo90 = false;
                        }
                        else
                        {
                            s1->isAngleStraightTo0 = false;
                            s1->isAngleStraightTo90 = true;

                        }
                        //DEBUG - show tested sharp points
                        P3 point3d1 = V.row(p1.VertexId);
                        P3 point3d2 = V.row(p2.VertexId);
                        P3 point3dmid = (point3d1 + point3d2) / 2;
                        if (isDEBUG)
                        {
                            draw.AddLabel(point3d1, (s1->isAngleStraightTo0 ? "s1:isAngleStraightTo0" : "s1:isAngleStraightTo90"));
                            draw.AddLabel(point3d2, (s2->isAngleStraightTo0 ? "s2:isAngleStraightTo0" : "s2:isAngleStraightTo90"));
                            draw.AddLabel(point3dmid, to_string(stopAngleAvarage));
                        }
                    }
                }
            }
        }


    //
    // Generate constrains correction angles
    //
    for (int is = 0; is < shaprs.size(); is++)
    {
        auto& s = shaprs[is];
        int isPrev = is - 1;
        if (isPrev < 0) isPrev = shaprs.size() - 1;
        auto& sPrev = shaprs[isPrev];
        int isNext = is + 1;
        if (isNext > shaprs.size() - 1) isNext = 0;
        auto& sNext = shaprs[isNext];
        if (!s.accepted) continue;
        int i = s.index_points;
        int stopLength = s.stopLength;
        bool isAngleStraightTo0 = s.isAngleStraightTo0;
        bool isAngleStraightTo90 = s.isAngleStraightTo90;
        D sumAngleChangePrev = s.sumAngleChangePrev;
        D sumAngleChangeNext = s.sumAngleChangeNext;


        int iPrev = i - 1;
        if (iPrev < 0) iPrev = Length - 1;
        int iNext = i;
        D angle = points[iNext].AngleBetweenEdges;
        D sign = isAngleStraightTo90 ? 1 : -1;  // clockwise: sign '+',  counter-clockwise: sign '-'
        if (angle > 90)
        {
            angle = 180 - angle;
            sign = -sign;
        }
        if (points[iNext].AngleBetweenEdgesFull > 180) sign = -sign;
        if (loop.Type == MeshLoopType::Inner)  sign = -sign; // inner loop goes always opposite direction rom outher loop

        bool angleCorrection_mult2 = isAngleStraightTo90 && CORRECT_straightTo90_angleX2 && (points[iNext].AngleBetweenEdges > 90); // mult alngle correction for very sharp angles

        D angleI = angle;
        stopLength -= SKIPP_LAST_EDGES_COUNT;
        if (stopLength <= 0) stopLength = 1;// do at least one correction
        for (int n = 0; n < stopLength; n++)
        {
            //
            // calculate angle correction angle
            //
            D angleCorrection = isAngleStraightTo0
                ? angleI
                : (90 - angleI);

            if (angleCorrection_mult2 && n < 3)
            {
                angleCorrection *= 2;
            }

            D MIN_DOUBLE = 0.0000001;
            if (sumAngleChangePrev < MIN_DOUBLE) sumAngleChangePrev = MIN_DOUBLE; // null devision protection
            if (sumAngleChangeNext < MIN_DOUBLE) sumAngleChangeNext = MIN_DOUBLE; // null devision protection
            D sumAngleChangeTotal = sumAngleChangePrev + sumAngleChangeNext; // null devision protection

            //v1
            //D percentPrev = sumAngleChangePrev / sumAngleChangeTotal;
            //D percentNext = sumAngleChangeNext / sumAngleChangeTotal;
            //v2
            D percentPrev = CORRECT_PERCENT;
            D percentNext = 1 - CORRECT_PERCENT;
            if (sumAngleChangePrev < sumAngleChangeNext)
            {
                percentPrev = 1 - CORRECT_PERCENT;
                percentNext = CORRECT_PERCENT;
            }

            // trying to solve issue for example:
            // TestFile = TestFiles::korzyna_small;
            // TestFileDensity = TestFilesDensity::high;
            // TestFileSrfIds = 106;
            if (CORRECT_PERCENT_ISDYNAMIC && isAngleStraightTo0 && points[s.index_points].AngleBetweenEdgesFull > 100 && points[s.index_points].AngleBetweenEdgesFull < 200)
            {
                D prevAngleDiff = static_cast<int>(points[sPrev.index_points].AngleBetweenEdgesFull) % 90;
                if (prevAngleDiff < 0.00001) prevAngleDiff = 0.00001; // devision by zero protection
                D nextAngleDiff = static_cast<int>(points[sNext.index_points].AngleBetweenEdgesFull) % 90;
                if (nextAngleDiff < 0.00001) nextAngleDiff = 0.00001; // devision by zero protection
                D angleDiffSumm = prevAngleDiff + nextAngleDiff;
                percentPrev = (angleDiffSumm - prevAngleDiff) / angleDiffSumm;
                percentNext = (angleDiffSumm - nextAngleDiff) / angleDiffSumm;
                percentPrev = min(max(0.30, percentPrev), 0.70);
                percentNext = min(max(0.39, percentNext), 0.70);
            }

            //D percentLength = CORRECT_PERCENT_PROGRESSIVE
            //    ? 1.0*(stopLength - n) / stopLength
            //    : 1;

            D stopLengthdiv2 = (1.0*stopLength) / 2;
            D percentLength = CORRECT_PERCENT_ISPROGRESSIVE
                ? (n < stopLengthdiv2)
                ? 1
                : 1.0*((stopLength - stopLengthdiv2) - (n - stopLengthdiv2)) / (stopLength - stopLengthdiv2)
                : 1;
            if (percentLength > 1 || percentLength < 0)
            {
                cout << "percentLength = " << percentLength << endl;
            }

            constrainCorrectionAngle[iPrev] += angleCorrection * percentPrev*percentLength*sign;  // for next edges angle correction is counter-clockwise - sign '-'
            constrainCorrectionAngleCount[iPrev] += 1;
            constrainCorrectionAngle[iNext] += -angleCorrection * percentNext*percentLength*sign; // for next edges angle correction is clockwise - sign '+'
            constrainCorrectionAngleCount[iNext] += 1;

            //if (iPrev == 64) cout << "DEBUG     edge " << iPrev << "   angle" << -angleCorrection*percentPrev;
            //if (iNext == 64) cout << "DEBUG     edge " << iNext << "   angle" << angleCorrection*percentPrev;

            //
            // iterate next edge
            //
            int iPrevPoint = iPrev;
            iPrev--;
            if (iPrev < 0) iPrev = Length - 1;
            iNext++;
            if (iNext > Length - 1) iNext = 0;
            D anglePrevChange = points[iPrevPoint].AngleBetweenEdges;
            //cout << "anglePrevChange = " << anglePrevChange << endl;
            if (points[iPrevPoint].AngleBetweenEdgesFull > 180) anglePrevChange = -anglePrevChange;
            D angleNextChange = points[iNext].AngleBetweenEdges;
            //cout << "angleNextChange = " << angleNextChange << endl;
            if (points[iNext].AngleBetweenEdgesFull > 180) angleNextChange = -angleNextChange;

            //stopAngle = utils::vector::Angle(edges[iPrev].directionToFace, edges[iNext].directionToFace);
            if (points[i].AngleBetweenEdgesFull < 90)
                angleI -= anglePrevChange + angleNextChange;
            else
                angleI += anglePrevChange + angleNextChange;

        }
    }

    //
    // Return results
    //
    int totalCount = 0;
    for (int i = 0; i < Length; i++)
    {
        int count = constrainCorrectionAngleCount[i];
        totalCount += count;
        if (count == 0) continue; // no angle changes appliyed to contrain

        V3 normal = F_normals.row(edges[i].FaceId);
        V3 constrain = constrains[i];
        //D correctionAngle = CORRECT_PERCENT*(constrainCorrectionAngle[i] / count);
        D correctionAngle = (constrainCorrectionAngle[i] / count);
        //if (isDEBUG) cout << "edge " << edges[i].EdgeId << "   angle correction = " << correctionAngle << endl;
        V3 constrainCorrected = utils::vector::Rotate(constrain, normal, -correctionAngle, true);
        constrainCorrected.normalize();

        // DEBUG show corrected constrain        
        //P3 edgeMidPoint = (mesh.V.row(edges[i].StartVertexId) + mesh.V.row(edges[i].EndVertexId)) / 2;
        //model.draw.AddEdge(edgeMidPoint, edgeMidPoint + constrainCorrected*mesh.avg_edge_length*4, Color3d(1, 0, 0));


        constrains[i] = constrainCorrected;
        correctionAngles[i] = correctionAngle;
    }
    //cout << "totalCount=" << totalCount << endl;
}

void MeshConstrains::GetBorderConstrains(const MeshLoop& loop, vector<V3>& constrains, vector<V3>& constrainsCorrected, vector<D>& correctionAngles) const
{
    constrains.resize(loop.Length);
    for (int i = 0; i < loop.Length; i++)
    {
        constrains[i] = loop.edges[i].directionToFace;

        //DEBUG
        //extern Model model;
        //if (edges[i].FaceId == 1618)
        //{
        //    int edgeId = edges[i].EdgeId;
        //    P3 faceCentroid = mesh.F_Barycenters.row(edges[i].FaceId);
        //    P3 closestEdgePoint;
        //    //V3 directionToFace = utils::mesh::GetEdgeNormalToFace(edgeId, edges[i].FaceId , closestEdgePoint);
        //    //V3 normalOfEdgeToFace = edges[i].directionToFace;

        //    const P3s& V = mesh.V;
        //    const I2s& EV = mesh.EV;
        //    P3 P1 = V.row(EV(edgeId, 0));
        //    P3 P2 = V.row(EV(edgeId, 1));
        //    // get closet point to line in 3d - http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
        //    V3 u = P2 - P1;
        //    V3 PQ = faceCentroid - P1;
        //    V3 w2 = PQ - u*(PQ.dot(u) / u.squaredNorm());
        //    closestEdgePoint = faceCentroid - w2;
        //    V3 normalOfEdgeToFace = (faceCentroid - closestEdgePoint).normalized();

        //    model.draw.AddPoint(P1, Color3d(0, 1, 0));
        //    model.draw.AddPoint(P2, Color3d(0, 1, 0));
        //    model.draw.AddPoint(faceCentroid, Color3d(1, 0, 0));
        //    model.draw.AddLabel((P1+P2)/2, to_string(edgeId), Color4d(1, 0, 0, 1));

        //    //model.draw.AddPoint(closestEdgePoint, Color3d(0, 1, 0));
        //    //model.draw.AddPoint(closestEdgePoint + normalOfEdgeToFace, Color3d(1, 0, 0));
        //}
    }

    constrainsCorrected.clear();
    constrainsCorrected.reserve(loop.Length);
    for (int i = 0; i < loop.Length; i++)
    {
        constrainsCorrected.push_back(constrains[i]);
    }

    correctionAngles.resize(loop.Length);
    if (options.CorrectInAngles) CorrectConstrainesInAngles(loop, constrainsCorrected, correctionAngles);
}

void MeshConstrains::Init_QuadOnly()
{

}


void MeshConstrains::AddContrainsInSharpEdges(int nexId)
{
    if (!options.AddContrainsInSharpEdges) return;

    D sharpAngle_COS = utils::angle::DegreesToCos(options.AddContrainsInSharpEdges_sharpAngle);
    for (int eid = 0; eid < mesh.EF.rows(); eid++)
    {
        if (mesh.E_isborder[eid]) continue; // only for non border edges - since for border edges we have another algorithm
        int fid0 = mesh.EF(eid, 0);
        int fid1 = mesh.EF(eid, 1);
        if (fid0 == -1 || fid1 == -1) continue;

        V3 normal0 = mesh.F_normals.row(fid0);
        V3 normal1 = mesh.F_normals.row(fid1);
        D angleCos = utils::vector::Cos(normal0, normal1, true);
        //D angle = utils::vector::Angle(normal0, normal1, true);
        bool isSharp = angleCos < sharpAngle_COS;
        if (isSharp)
        {
            for (int fid : {fid0, fid1})
            {
                V3 directionToFace = mesh.EdgeNormalToFace(eid, fid);
                FaceConstrain fc(nexId, fid, eid);
                fc.DirectionX = mesh.EdgeDirectionForFace(eid, fid) / mesh.E_Length(eid);
                fc.DirectionY = directionToFace;
                fc.DirectionY_Corrected = directionToFace;
                fc.DirectionY_Corrected_Angle = 0;
                Constrains.push_back(fc);
                nexId++;
            }
        }
    }
}
