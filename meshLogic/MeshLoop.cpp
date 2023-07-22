#include "stdafx.h"
#include "MeshLoop.h"
#include "Mesh.h"
#include "Model.h"

MeshLoopEdge::MeshLoopEdge(const MeshLoop& _loop, int _edgeId, int _faceId, int _startVertexId, int _endVertexId)
    : EdgeId(_edgeId), FaceId(_faceId), StartVertexId(_startVertexId), EndVertexId(_endVertexId), Index(0), IndexPrev(0), IndexNext(0)
{
}
  
atomic_int MeshLoop::nextMeshLoopID;


MeshLoop::MeshLoop(const Mesh& _mesh)
    : id(nextMeshLoopID++), Length(0), mesh(_mesh), Type(MeshLoopType::Outher), Loop3dLength(0), DistFromCentroidToZero(0)
{

}
II MeshLoop::SizeOF() const
{
    return  sizeof(MeshLoop)
        + edges.size() * sizeof(MeshLoopEdge)
        + points.size() * sizeof(MeshLoopPoint);
}
void MeshLoop::GetLoops(const Mesh& mesh, vector<MeshLoop>& sortedloops, bool initLoops)
{
    //GetLoops_slow(mesh, sortedloops, initLoops);
    GetLoops_fast(mesh, sortedloops, initLoops);
}
  

void MeshLoop::GetLoops_slow(const Mesh& mesh, vector<MeshLoop>& sortedloops, bool initLoops)
{
    vector<MeshLoop> loops;
    loops.clear();
    loops.reserve(10000);

    const auto& EF = mesh.EF;
    const auto& EV = mesh.EV;
    const auto& F = mesh.F;

    vector<int> nakedEdgesIds;
    for (int eid = 0; eid < EF.rows(); ++eid) // For every border edge 
    {
        if (mesh.E_isborder[eid])
        {
            nakedEdgesIds.push_back(eid);
        }
    }
    int bordersCount = nakedEdgesIds.size();
    if (bordersCount == 0)
    {
        sortedloops.clear();
        return;
    }

    bool needToStartANewLoop = true;
    int loopStartVertexId = 0;
    int loopEndVertexId = 0;
    int addedEdgesCount = 0;


    //
    // create linked list of unused naked edges - this will help to indentify if we can use an edge or already not
    //
    int usedCheckedCount = 0;
    vector<bool> used(bordersCount);//slow for huge models - need in every time loop throught the list - very slow
    std::fill(used.begin(), used.end(), false);

    //
    // Collect all loops
    //
    for (int temp = 0; temp < bordersCount; ++temp) // iterate all border edges
    {
        // start a new loop (add only one edge - start edge of the loop)
        if (needToStartANewLoop && addedEdgesCount < bordersCount)
        {
            loops.push_back(MeshLoop(mesh));
            int firstNotUsedBorderEdgeIndex = 0;
            for (int ui = 0; ui < used.size(); ++ui)
            {
                usedCheckedCount++;
                if (!used[ui])
                {
                    firstNotUsedBorderEdgeIndex = ui;
                    break;
                }
            }
            int eid = nakedEdgesIds[firstNotUsedBorderEdgeIndex];
            loopStartVertexId = EV(eid, 0);
            loopEndVertexId = EV(eid, 1);
            int fid = EF(eid, 0);
            if (fid == -1) fid = EF(eid, 1);
            // check if we have to exchange start and end loop vertextes - this is important for loop direction
            for (int n = 0; n < 3; n++)
            {
                if (F(fid, n) == loopStartVertexId
                    && F(fid, (n+1) % 3) != loopEndVertexId)
                {
                    std::swap(loopStartVertexId, loopEndVertexId);
                }
            }

            const MeshLoopEdge le(loops.back(), eid, fid, loopStartVertexId, loopEndVertexId);
            loops.back().edges.reserve(10000); // reserve a lot of space for edges
            loops.back().edges.push_back(le);
            addedEdgesCount++;
            used[firstNotUsedBorderEdgeIndex] = true;
            needToStartANewLoop = false;
        }

        // find next edge of new loop - for this we have to check from all border edges if some edge is a friend
        // for every border-edge find a friend (connected border-edge) 
        MeshLoop& currentNewLoop = loops.back();
        for (int i = 0; i < bordersCount; ++i) // find edge friend base on prev Vertex - 'loopEndVertexId'
        {
            usedCheckedCount++;
            if (used[i]) continue;
            int eid = nakedEdgesIds[i];
            int v1 = EV(eid, 0);
            int v2 = EV(eid, 1);
            if (v1 != loopEndVertexId && v2 != loopEndVertexId) continue;
            if (v2 == loopEndVertexId)
            {
                std::swap(v1, v2);
            }
            int faceId = EF(eid, 0);
            if (faceId == -1) faceId = EF(eid, 1);
            const MeshLoopEdge le(currentNewLoop, eid, faceId, v1, v2);
            currentNewLoop.edges.push_back(le);
            addedEdgesCount++;
            used[i] = true;
            loopEndVertexId = v2;
            // if we found closed loop - then we have to start a new loop
            if (loopEndVertexId == loopStartVertexId)
            {
                needToStartANewLoop = true;
            }
            break; // we foud a next border-edge
        }
        // free unused space allocated for speeding up 'adding new edges' by method 'loops.back().edges.reserve(10000)'
        currentNewLoop.edges.shrink_to_fit();
    }
    //cout << "usedCheckedCount = " << usedCheckedCount << "      bordersCount = " << bordersCount << "      loops.count = " << loops.size()<< endl;

    if (!initLoops) return;
    //
    // Init loops (also sets LoopType)
    //
    extern bool IsOmpEnabled;
#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int i = 0; i < loops.size(); i++)
    {
        loops[i].Init();
    }


    //
    // Sort loops - first come outher loops, then closer to P3(0,0,0) - this methrics taken to make order invariant for different mesh density
    //
    vector<unsigned int> loops_sorted_indexes = utils::stdvector::sort_indexes_custom(loops.size(), [&loops](unsigned int i1, unsigned int i2)
    {
        const MeshLoop& loop1 = loops[i1];
        const MeshLoop& loop2 = loops[i2];
        if (loop1.Type != loop2.Type)
        {
            return static_cast<int>(loop1.Type) < static_cast<int>(loop2.Type);
        }
        return utils::vector::LengthPow2(loop1.Centroid - P3(0,0,0)) < utils::vector::LengthPow2(loop2.Centroid - P3(0, 0, 0));
    });
    //copy sorted loops to output result list
    sortedloops.clear();
    sortedloops.reserve(loops.size());
    for (auto index : loops_sorted_indexes)
    {
        sortedloops.push_back(MeshLoop(mesh));
        MeshLoop& dest = sortedloops.back();
        MeshLoop& src = loops[index];
        dest.edges = src.edges;
        dest.points = src.points;
        dest.Type = src.Type;
        dest.Length = src.Length;
        dest.Loop3dLength = src.Loop3dLength;
        dest.Centroid = src.Centroid;
        dest.DistFromCentroidToZero = src.DistFromCentroidToZero;

    }

    //cout << "***DEBUG***  Result of sorting loops: ";
    //for (const auto& l : sortedloops)
    //{
    //    cout << (l.Type == MeshLoopType::Inner ? "inner, " : "outher, ");
    //}
    //cout << endl;
}

void MeshLoop::GetLoops_fast(const Mesh& mesh, vector<MeshLoop>& sortedloops, bool initLoops)
{
    vector<MeshLoop> loops;
    loops.clear();
    loops.reserve(10000);

    const auto& EF = mesh.EF;
    const auto& EV = mesh.EV;
    const auto& F = mesh.F;
    const P3s& V = mesh.V;


    vector<int> nakedEdgesIds;
    nakedEdgesIds.reserve(1000);
    for (int eid = 0; eid < EF.rows(); ++eid) // For every border edge 
    {
        if (mesh.E_isborder[eid])
        {
            nakedEdgesIds.push_back(eid);
        }
    }
    int nakedEdgesCount = nakedEdgesIds.size();
    if (nakedEdgesCount < 3) // at least 1 naked traingle must be in a loop
    {
        sortedloops.clear();
        return;
    }

    //
    // construct map from vertex to naked edges connected to this vertex
    //
    const int VtoE_CAPACITY = 2; // we can process only 2 naked edges for 1 vertex - if there are more we must ingore others, so there will some nakededges without loop
    MatrixX3i VtoE;
    VtoE.setZero(V.rows(), 1 + VtoE_CAPACITY); // counter,vertex1,vertex2
    for (int i = 0; i < nakedEdgesCount; ++i)
    {
        int ei = nakedEdgesIds[i];
        for (int vi = 0; vi < 2; vi++) // for first and second vertex of naked edge
        {
            int vid = EV(ei, vi);
            VtoE(vid, 0) += 1; // increment counter - so we will know where store next ei. If counter will higher allowed - we will ignore those loops - so counter higher allowed will be a signal to ignore the loop
            if (VtoE(vid, 0) <= VtoE_CAPACITY) // we can process only 2 naked edges for 1 vertex
            {
                VtoE(vid, VtoE(vid, 0)) = ei;
            }
            else
            {
                // so if we have more than 2 naked edges for 1 vertex - what we can do ? - just ignore
                // our loop detection algorithm can work only if 1 vertex has exactly 2 naked edges
                // so just ingore other naked edges - we will have some naked edges unconnected - we can't do anything about it
            }
        }
    }
    // validate VtoE - show a warning message for invalid connections
    vector<int> badVertexesCount_vid;
    vector<int> badVertexesCount_count;
    for (int vid = 0; vid < VtoE.rows(); vid++)
    {
        int countOfNakededgesConnectedToVertex = VtoE(vid, 0);
        if (countOfNakededgesConnectedToVertex > VtoE_CAPACITY)
        {
            badVertexesCount_vid.push_back(vid);
            badVertexesCount_count.push_back(countOfNakededgesConnectedToVertex);
        }
    }
    if (badVertexesCount_vid.size() > 0)
    {
        cout << "!!!  GetLoops_fast: vertexes " << utils::stdvector::toString(badVertexesCount_vid) << " has " << utils::stdvector::toString(badVertexesCount_count) << " naked edges and max allowed is " << VtoE_CAPACITY << "   "<< mesh.GetMeshIdStr() << endl;
        
    }


    //
    // create linked list of unused naked edges - this will help to indentify if we can use an edge or already not
    //
    int usedCheckedCount = 0;
    vector<bool> used(nakedEdgesCount);//slow for huge models - need in every time loop throught the list - very slow
    std::fill(used.begin(), used.end(), false);
    Is EtoUsedIndex;
    EtoUsedIndex.setZero(EV.rows());
    for (int i = 0; i < nakedEdgesCount; i++)
    {
        int eid = nakedEdgesIds[i];
        EtoUsedIndex(eid) = i;
    }

    //
    // create fast used linked list - to avoid search for first non used index (lenear seach bool false value can take a lost of time for model with many surfaces)
    //
    MatrixX2i usedFastCheck; //prev unused, next unused
    usedFastCheck.setZero(used.size(), 2);
    int usedLastIndex = used.size() - 1;
    usedFastCheck(0, 0) = usedLastIndex; // first prev unused - last index
    usedFastCheck(0, 1) = 1; // first next unused - second index
    for (int i = 1; i < usedLastIndex; i++) // skip first an last
    {
        usedFastCheck(i, 0) = i - 1;
        usedFastCheck(i, 1) = i + 1;
    }
    usedFastCheck(usedLastIndex, 0) = usedLastIndex - 1; // last prev unused - pre-last index
    usedFastCheck(usedLastIndex, 1) = 0; // last next unused - first index
    int usedFastCheck_firstUnusedIndex = 0; // this variable will store frist 'used[i]==false' index
    auto setUsedAtIndex = [&usedFastCheck, &used, &usedFastCheck_firstUnusedIndex](int index)
    {
        assert(used[index] == false);
        used[index] = true;
        //if removed index is the lowest unused index - reassign it to next first
        if (usedFastCheck_firstUnusedIndex == index)
        {
            usedFastCheck_firstUnusedIndex = usedFastCheck(usedFastCheck_firstUnusedIndex, 1);// replace with next unused index
        }
        int prev = usedFastCheck(index, 0); // prev connected used item to this index
        int next = usedFastCheck(index, 1); // next connected used item to this index
        assert(prev != -1);
        assert(next != -1);
        usedFastCheck(prev, 1) = next; // set connection between prev and next, before remocing current
        usedFastCheck(next, 0) = prev;  // set connection between prev and next, before remocing current
        usedFastCheck(index, 0) = -1; // remove relation - like used[i] = false
        usedFastCheck(index, 1) = -1;// remove relation - like used[i] = false
    };

    //
    // Collect all loops
    //

    int loopStartVertexId = 0;
    int loopEndVertexId = 0;
    int addedEdgesCount = 0;
    while (addedEdgesCount < nakedEdgesCount)
    {
        //Start a new loop - we assume that mesh want have vertexes with more than 2 naked edges connected to
        //v1 - simple search - slow
        //int firstNotUsedBorderEdgeIndex = 0;
        //for (int ui = 0; ui < used.size(); ++ui)
        //{
        //    usedCheckedCount++;
        //    if (!used[ui])
        //    {
        //        firstNotUsedBorderEdgeIndex = ui;
        //        break;
        //    }
        //}
        //assert(usedFastCheck_firstUnusedIndex == firstNotUsedBorderEdgeIndex);
        //v2 - using linked list - fast
        int firstNotUsedBorderEdgeIndex = usedFastCheck_firstUnusedIndex;

        int loopStartEdgeId = nakedEdgesIds[firstNotUsedBorderEdgeIndex];
        loopStartVertexId = EV(loopStartEdgeId, 0);
        loopEndVertexId = EV(loopStartEdgeId, 1);
        int fid = EF(loopStartEdgeId, 0);
        if (fid == -1) fid = EF(loopStartEdgeId, 1);
        // check if we have to exchange start and end loop vertextes - this is important for loop direction
        for (int n = 0; n < 3; n++)
        {
            if (F(fid, n) == loopStartVertexId
                && F(fid, (n + 1) % 3) != loopEndVertexId)
            {
                std::swap(loopStartVertexId, loopEndVertexId);
            }
        }

        loops.push_back(MeshLoop(mesh));
        MeshLoop& currentNewLoop = loops.back();
        const MeshLoopEdge leStart(currentNewLoop, loopStartEdgeId, fid, loopStartVertexId, loopEndVertexId);
        currentNewLoop.edges.reserve(10000); // reserve a lot of space for edges to avoid realocating memory for huge models, for small we dont notice a space resevation
        currentNewLoop.edges.push_back(leStart);
        addedEdgesCount++;
        setUsedAtIndex(firstNotUsedBorderEdgeIndex);

        // continues loop from start edge
        // find next edge of new loop 
        int loopEndEdgeId = loopStartEdgeId;
        if (VtoE(loopStartVertexId, 0) == VtoE_CAPACITY) // ignore invalid connections, and thus ignore all edges connected to this connection (FOR START VERTEX)
        {
            while (addedEdgesCount < nakedEdgesCount) // continue until we checked all edges
            {
                int countOfNakededgesConnectedToVertex = VtoE(loopEndVertexId, 0);

                // ignore invalid connections, and thus ignore all edges connected to this connection (FOR NEXT END VERTEX)
                if (countOfNakededgesConnectedToVertex != VtoE_CAPACITY)
                {
                    break;
                }

                // find edge friend base on prev Vertex - 'loopEndVertexId' and prev edge 'loopEndEdgeId'
                int nextEdgeId = VtoE(loopEndVertexId, 1);//assume next edge if first edge in map, if it same as our edge - take second 
                if (nextEdgeId == loopEndEdgeId) nextEdgeId = VtoE(loopEndVertexId, 2);

                // if next edge is already used - loop is invalid (this can happend if vertex has 3 naked edges)
                if (used[EtoUsedIndex[nextEdgeId]])
                {
                    break;
                }

                int v1 = EV(nextEdgeId, 0);
                int v2 = EV(nextEdgeId, 1);
                if (v2 == loopEndVertexId)
                {
                    std::swap(v1, v2);
                }
                int faceId = EF(nextEdgeId, 0);
                if (faceId == -1) faceId = EF(nextEdgeId, 1);
                const MeshLoopEdge le(currentNewLoop, nextEdgeId, faceId, v1, v2);
                currentNewLoop.edges.push_back(le);
                addedEdgesCount++;
                setUsedAtIndex(EtoUsedIndex[nextEdgeId]);
                loopEndVertexId = v2;
                loopEndEdgeId = nextEdgeId;
                // if this is end of the loop - stop search
                if (loopEndVertexId == loopStartVertexId)
                {
                    break;
                }
            }
            // free unused space allocated for speeding up 'adding new edges' by method 'loops.back().edges.reserve(10000)'
            currentNewLoop.edges.shrink_to_fit();
        }

        //discard new loop if it is not finished (this can happend if vertex has 3 naked edges)        
        if (loopEndVertexId != loopStartVertexId)
        {
            #if DEBUG
            cout << "!!!  error: Loop is unfinished and will be ignored. Ignored " << currentNewLoop.edges.size() << " naked edges." << "   meshid=" << mesh.id << endl;
            #endif
            loops.pop_back(); // ignore invalid loop (for example if vertex has 3 naked edges - this cannot be threated as loop - loop must have 2 naked edges for 1 vertex)
        }
    }
    //cout << "usedCheckedCount = " << usedCheckedCount << "      nakedEdgesCount = " << nakedEdgesCount << "      loops.count = " << loops.size() << endl;

    for (int i = 0; i < loops.size(); i++)
    {
        loops[i].Length = loops[i].edges.size();
    }
      
      
    if (!initLoops)
    {
        sortedloops.clear();
        sortedloops.reserve(loops.size());
        for (int i = 0; i < loops.size(); i++)
        {
            sortedloops.push_back(MeshLoop(mesh));
            MeshLoop& dest = sortedloops.back();
            MeshLoop& src = loops[i];
            dest.edges = src.edges;
            dest.Length = src.Length;
        }
        return;
    }

    //
    // Init loops (also sets LoopType)
    //
    if (loops.size() > 1)
    {
        extern bool IsOmpEnabled;
        #pragma omp parallel for schedule(static) if(IsOmpEnabled)
        for (int i = 0; i < loops.size(); i++)
        {
            loops[i].Init();
        }
    }
    else
    {
        if (loops.size() != 0)
        {
            loops[0].Init();
        }
    }


    //
    // Sort loops - first come outher loops, then closer to P3(0,0,0) - this methrics taken to make order invariant for different mesh density
    //
    vector<unsigned int> loops_sorted_indexes = utils::stdvector::sort_indexes_custom(loops.size(), [&loops](unsigned int i1, unsigned int i2)
    {
        const MeshLoop& loop1 = loops[i1];
        const MeshLoop& loop2 = loops[i2];
        if (loop1.Type != loop2.Type)
        {
            return static_cast<int>(loop1.Type) < static_cast<int>(loop2.Type);
        }
        return utils::vector::LengthPow2(loop1.Centroid - P3(0, 0, 0)) < utils::vector::LengthPow2(loop2.Centroid - P3(0, 0, 0));
    });
    //copy sorted loops to output result list
    sortedloops.clear();
    sortedloops.reserve(loops.size());
    for (auto index : loops_sorted_indexes)
    {
        sortedloops.push_back(MeshLoop(mesh));
        MeshLoop& dest = sortedloops.back();
        MeshLoop& src = loops[index];
        dest.edges = src.edges;
        dest.points = src.points;
        dest.Type = src.Type;
        dest.Length = src.Length;
        dest.Loop3dLength = src.Loop3dLength;
        dest.Centroid = src.Centroid;
        dest.DistFromCentroidToZero = src.DistFromCentroidToZero;

    }

    //cout << "***DEBUG***  Result of sorting loops: ";
    //for (const auto& l : sortedloops)
    //{
    //    cout << (l.Type == MeshLoopType::Inner ? "inner, " : "outher, ");
    //}
    //cout << endl;
}


void MeshLoop::GetAngleFull(V3 directionToFace, V3 directionToFacePrev, const MeshLoopEdge& e, const MeshLoopEdge& ePrev,
    D& AngleBetweenEdges, D& AngleBetweenEdgesFull) const
{
    
    // correct angle for cases when faces are not in same plane (using transformation matrix)
    V3 normal1 = mesh.F_normals.row(ePrev.FaceId);
    V3 normal2 = mesh.F_normals.row(e.FaceId);

    //if (e.Index == 0) cout << endl << endl;
    //cout << "#" << e.Index << "";
    //cout << " normal1:" <<normal1(0) << "," << normal1(1) << "," << normal1(2);
    //cout << " normal2:" << normal2(0) << "," << normal2(1) << "," << normal2(2);
    //cout << " dir:" << directionToFace(0) << "," << directionToFace(1) << "," << directionToFace(2);
    //D facesRotationAngle = utils::vector::Angle(normal1, normal2, true);
    //if (facesRotationAngle > 0.00001) // we can calculate cross of normals only if there is some angle between normals
    //{
        //v1
        //V3 facesRotationAxis = normal1.cross(normal2);
        //V3 ePrev_directionAlongEdge_Rotated = utils::vector::Rotate(ePrev.directionAlongEdge, facesRotationAxis, facesRotationAngle);
        //v2
        //V3 ePrev_directionAlongEdge_Rotated = utils::vector::Translate(ePrev.directionAlongEdge, normal1, normal2);
        ////V3 ePrev_directionAlongEdge_Rotated2 = utils::vector::Rotate(ePrev.directionAlongEdge, facesRotationAxis, -facesRotationAngle);
        //D AngleBetweenEdges1 = utils::vector::Angle(e.directionAlongEdge, ePrev_directionAlongEdge_Rotated);
        ////D AngleBetweenEdges2 = utils::vector::Angle(e.directionAlongEdge, ePrev_directionAlongEdge_Rotated2);
        //AngleBetweenEdges = AngleBetweenEdges1;
        //v3
        V3 directionToFacePrev_Rotated = utils::vector::Translate(directionToFacePrev, normal1, normal2, true);
        AngleBetweenEdges = utils::vector::Angle(directionToFace, directionToFacePrev_Rotated);
        if (isnan(AngleBetweenEdges))
        {
            cout << "! wrong   MeshLoop::GetAngleFull()   AngleBetweenEdges is NAN   mesh.id=" << mesh.id << endl;
        }
        //cout << " dirRot:" << directionToFacePrev_Rotated(0) << "," << directionToFacePrev_Rotated(1) << "," << directionToFacePrev_Rotated(2);
        //cout << " AngleBetweenEdges=" << AngleBetweenEdges << endl;
        //DEBUG
        ////if (e.EdgeId + ePrev.EdgeId == 322 + 323)
        //{
            //cout << endl << "#" << e.EdgeId << "-#" << ePrev.EdgeId << endl;
            //cout << "AngleBetweenEdges  " << AngleBetweenEdges << endl;
            //cout << "facesRotationAngle   " << facesRotationAngle << endl;
            //cout << "AngleBetweenEdges   " << AngleBetweenEdges1 << endl;
            //cout << "AngleBetweenEdges2   " << AngleBetweenEdges2 << endl;
        //}
    //}
    //else
    //{
    //    // Calculate angle between edges
    //    // angle between direction to faces (correct only if faces are in same plane)
    //    AngleBetweenEdges = utils::vector::Angle(directionToFace, directionToFacePrev);// to ignore angle between faces - we have to take angle between normal to faces - this is the real border angle 
    //}

    if (abs(AngleBetweenEdges) < 0.0000000001) AngleBetweenEdges = 0;// truncate to 0 for better debuging

    D angle = 180 - AngleBetweenEdges; //  we need to take angle between edges, instead of angle between change vector direction of edge

    //v1 - using angle
    //D angleBetweenNormalAndPrev = utils::vector::Angle(ePrev.directionAlongEdge, directionToFace);
    //if (angleBetweenNormalAndPrev < 90) angle = 360 - angle;
    //v2 - using the fact that Dot product is possitive if angle is less from 90
    if (utils::vector::Dot(ePrev.directionAlongEdge, directionToFace) > 0) angle = 360 - angle;

    //cout << "loopAngle = " << loopAngles[i] << ",    agleBetweenFarestSides = " << agleBetweenFarestSides << ",    angleBetweenNormalAndPrev = " << angleBetweenNormalAndPrev << endl;
    AngleBetweenEdgesFull = angle;
}

void MeshLoop::GetLoopFaceIds(vector<int>& faceids) const
{
    faceids.clear();
    faceids.reserve(edges.size());
    for (auto e : edges)
    {
        faceids.push_back(e.FaceId);
    }
}

void MeshLoop::Init()
{
    Length = edges.size();

    //
    // Set indexes for later easy use of loop edge
    //
    for (int i = 0; i < Length; i++)
    {
        MeshLoopEdge& e = edges[i];

        e.Index = i;

        int iPrev = i - 1;
        if (iPrev < 0) iPrev = Length - 1;
        e.IndexPrev = iPrev;

        int iNext = i + 1;
        if (iNext > Length - 1) iNext = 0;
        e.IndexNext = iNext;

        //Precalculate vectors for each edge
        e.directionAlongEdge = mesh.V.row(e.EndVertexId) - mesh.V.row(e.StartVertexId);
        
        //e.Length = utils::vector::Length(e.directionAlongEdge);
        e.Length = mesh.E_Length(e.EdgeId); // we have already calculated all edges lenghts

        //Precalculate vectors to face
        e.directionToFace = mesh.EdgeNormalToFace(e.EdgeId, e.FaceId);

        int temp = 0;
    }


    //
    // Populate MeshLoopPoint base on MeshLoopEdge
    //
    points.resize(Length);
    const V3s& F_normals = mesh.F_normals;
    //cout << endl << endl << endl << "=== TESTING  MeshLoop - angles between edges === " << endl << endl;
    for (int i = 0; i < Length; i++)
    {
        const MeshLoopEdge& e = edges[i];
        const MeshLoopEdge& ePrev = edges[e.IndexPrev];
        MeshLoopPoint& p = points[i];
        p.Index = i;
        p.VertexId = e.StartVertexId;
        p.EdgeIdForward = e.EdgeId;
        p.EdgeIdBackward = ePrev.EdgeId;
        p.IsSharp = false;
        p.probably_IsSharp = false;
        GetAngleFull(e.directionToFace, ePrev.directionToFace, e, ePrev, p.AngleBetweenEdges, p.AngleBetweenEdgesFull);
    }
    //cout << endl << endl << endl << "=================================";


    //
    // Set loop type
    //
    Type = MeshLoopType::Outher;
    Centroid = P3(0, 0, 0);
    if (Length > 0)
    {
        D distEdges = 0;
        //D distFaceCentroids = 0;
        D angleSumm = 0;
        for (int i = 0; i < Length; i++)
        {
            const MeshLoopEdge& e = edges[i];
            const MeshLoopEdge& eNext = edges[e.IndexNext];
            //const P3& pEdge = mesh.V.row(e.StartVertexId);
            //const P3& pEdgeNext = mesh.V.row(eNext.StartVertexId);
            distEdges += e.Length;
            //distFaceCentroids += utils::point::Length(mesh.F_Barycenters.row(e.FaceId), mesh.F_Barycenters.row(eNext.FaceId));
            Centroid += mesh.V.row(points[i].VertexId);
            angleSumm += points[i].AngleBetweenEdgesFull - 180;
        }
        Centroid = Centroid / Length;
        //DistFromCentroidToZero = utils::vector::Length(Centroid);
        Loop3dLength = distEdges;
        //v1 - sometimes incorrect
        //if (distFaceCentroids > distEdges) 
        //v2 - always correct
        if (angleSumm > 0)
        {
            Type = MeshLoopType::Inner;
        }
        //cout << "Length = " << Length << "   angleSumm = " << angleSumm << "   distEdges = " << distEdges << "   distFaceCentroids = " << distFaceCentroids << "   LoopType = " << (Type == MeshLoopType::Outher ? "outher" : "inner") << endl;
    }



    //
    // Add sharpness
    //
    vector<int> pointIndexes;
    GetSharpPoints_Indexes(pointIndexes);    
    //if (pointIndexes.size() > 10)cout << "pointIndexes.count = " << pointIndexes.size() << endl;
    for (int i = 0; i < pointIndexes.size(); i++)
    {
        MeshLoopPoint& p = points[pointIndexes[i]]; // edge indexes is same for points
        D angleChange = abs(p.AngleBetweenEdgesFull - 180);
        p.IsSharp = angleChange > 40;
        p.probably_IsSharp = angleChange > 15;
        //if (pointIndexes.size() > 10)
        //{
        //    cout << "p.AngleBetweenEdgesFull = " << p.AngleBetweenEdgesFull<<"    p.AngleBetweenEdges = " << p.AngleBetweenEdges << endl;
        //}
    }
}

void MeshLoop::GetSharpPoints(vector<int>& vertexes, vector<D>& angleChange, vector<int>& pointIndexes) const
{
    vertexes.clear();
    angleChange.clear();
    pointIndexes.clear();
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i].IsSharp)
        {
            pointIndexes.push_back(i);
        }
    }
    vertexes.resize(pointIndexes.size());
    angleChange.resize(pointIndexes.size());
    for (int i = 0; i < pointIndexes.size(); i++)
    {
        int pi = pointIndexes[i];
        vertexes[i] = points[pi].VertexId;
        angleChange[i] = points[pi].AngleBetweenEdgesFull;
    }
}










bool MeshLoop::angleSuddenlyHasChanged(const int DEEP, const int prevsnexts[], const vector<D>& loopAngles) const
{ 
    const bool TEST = false;
    const P3s& V = mesh.V;

    return false; // - for now this method is disabled, since it makes sometimes wrong results

    //int loopAnglesTestIndex = prevsnexts[0];

    //D summChange = 0;
    //D maxChange = 0;
    //for (int ideep = 1; ideep < DEEP + 1; ideep++) //  skip first item (skip tested vertex for summ and avg)
    //{
    //    D angle = loopAngles[prevsnexts[ideep]];
    //    if (angle > 35) return false; // find sharness on mostly smooth curvature
    //    summChange += angle;
    //    maxChange = max(maxChange, angle);
    //}
    //D avgChange = summChange / DEEP;
    //D maxDeviation = maxChange - avgChange; if (maxDeviation < 0.000001) maxDeviation = 0.000001;
    //D endDeviation = loopAngles[loopAnglesTestIndex] - avgChange;
    //D deviationTimes = endDeviation / maxDeviation;

    //// angle deviation is much different from tested angle
    //bool res = (deviationTimes >= 4 && endDeviation >= 30)
    //    || (deviationTimes >= 30 && endDeviation >= 5)
    //    || (deviationTimes >= 300 && endDeviation >= 1)
    //    || (deviationTimes >= 333000);

    //// DEBUG - show angles and deviations for passed test
    //if (TEST && res)
    //{
    //    //cout << "angle = " << loopAngles[loopAnglesTestIndex] << "   deviationTimes = " << deviationTimes << ",    endDeviation = " << endDeviation << endl;
    //    cout << "#" << loopAnglesTestIndex << "   angle = " << utils::angle::ToString(points[loopAnglesTestIndex].AngleBetweenEdgesFull) << "   deviationTimes = " << deviationTimes << ",    endDeviation = " << endDeviation << endl;
    //    //mesh.VertexToPointOutOfFaces(
    //    extern Model model;
    //    for (int ideep = 1; ideep < DEEP + 1; ideep++) //  skip first item (skip tested vertex for summ and avg)
    //    {
    //        D angle = loopAngles[prevsnexts[ideep]];
    //        cout << "   loopAngles[prevsnexts[ii]] = " << loopAngles[prevsnexts[ideep]] << endl;
    //        model.draw.AddEdge(V.row(points[prevsnexts[ideep - 1]].VertexId), V.row(points[prevsnexts[ideep]].VertexId), Color3d(1, 0, 0));
    //    }
    //}
    //return res;
};



void MeshLoop::GetSharpPoints_Indexes(vector<int>& pointIndexes) const
{
    extern Model model;
    const bool TEST = false;
    const P3s& V = mesh.V;
    const I2s& EV = mesh.EV;
    const vector<MeshLoopPoint>& _points = points;
    const vector<MeshLoopEdge>& _edges = edges;

    const int DEEP = 5;

    if (TEST)
    {
        cout << endl;
    }

    //Precalculate angles between edges for each edge at starting point of source edges and end point of prev edge    
    vector<D> loopAngles(edges.size()); // angles at start point
    if (TEST)model.draw.ReserveLabels(edges.size() * 2);
    for (int i = 0; i < edges.size(); i++)
    {

        D angle = points[i].AngleBetweenEdges;

        // filter noise - when prev or next angle is compensating current angle
        //if (angle < 30)
        //{
        //    int iPrev = i - 1;
        //    if (iPrev < 0) iPrev = Length + iPrev;
        //    int iNext = i + 1;
        //    if (iNext > Length - 1) iNext -= Length;
        //    D angleFull = points[i].AngleBetweenEdgesFull;
        //    D angleFullPrev = points[iPrev].AngleBetweenEdgesFull;
        //    D angleFullNext = points[iNext].AngleBetweenEdgesFull;

        //    D angleCompensated = angle;
        //    //if ((anglePrev < 180 && angleI > 180 && angleNext < 180)
        //    //    || (anglePrev > 180 && angleI < 180 && angleNext > 180))
        //    if ((angleFull > 180 && (angleFullPrev < 180 || angleFullNext < 180))
        //        || (angleFull < 180 && (angleFullPrev > 180 || angleFullNext > 180)))
        //    {
        //        angleCompensated = min(angleCompensated, abs(360 - (angleFull + angleFullPrev)) / 2);
        //        angleCompensated = min(angleCompensated, abs(360 - (angleFull + angleFullNext)) / 2);
        //        if (angleCompensated < angle)
        //        {
        //            angle = angleCompensated;
        //        }
        //    }
        //}
        //
        //// filter noise - very small angles
        //if (angle < 1) angle = 0; 

        loopAngles[i] = angle;
        if (TEST) model.draw.AddLabel(mesh.VertexToPointOutOfFaces(_points[i].VertexId, -1), "  #" + to_string(i), Color3d(1, 0, 0));
        if (TEST) model.draw.AddLabel(mesh.VertexToPointOutOfFaces(_points[i].VertexId, -2), to_string(angle), Color3d(1, 0, 0));
    }




    int prevs[DEEP + 1]; // indexes to Loops
    int nexts[DEEP + 1];
    for (int i = 0; i < edges.size(); i++)
    {
        const MeshLoopEdge& edge = edges[i];

        // First get list of edges to test
        for (int ideep = 0; ideep < DEEP + 1; ideep++) // loop DEEP+1 counts (including starting edge)
        {
            int iPrev = i - ideep;
            if (iPrev < 0) iPrev = Length + iPrev;
            prevs[ideep] = iPrev;
            int iNext = i + ideep;
            if (iNext > Length - 1) iNext -= Length;
            nexts[ideep] = iNext;
        }
        //if (points[i].VertexId == 369)
        //{
        //    cout << "loopAnglesTestIndex = " << 369 << endl;
        //}
        // Test always first vertex on edge - since loop is circular all vertexes will be tested
        if (angleSuddenlyHasChanged(DEEP, prevs, loopAngles)
            || angleSuddenlyHasChanged(DEEP, nexts, loopAngles)
            // angle change is to big  (test model: Lock_door_4, low)
            || abs(_points[i].AngleBetweenEdgesFull - 180) > 10
            )
        {
            // additional test to skipp noise - angle between last prev and last next must be higher that 20 degrees
            /*if (points[i].AngleBetweenEdges <= 20)
            {
                auto iloopPrevLast = prevs[DEEP];
                auto iloopNextLast = nexts[DEEP];
                D agleBetweenFarestSides = utils::vector::Angle(edges[iloopPrevLast].directionAlongEdge, edges[iloopNextLast].directionAlongEdge);
                if (agleBetweenFarestSides < 20) continue;
            }*/

            pointIndexes.push_back(i);

            //cout << "angle = " << angle << ":  loopAngle = " << loopAngles[i] << ",    agleBetweenFarestSides = " << agleBetweenFarestSides << ",    angleBetweenNormalAndPrev = " << angleBetweenNormalAndPrev << endl;
        }
    }
}




















