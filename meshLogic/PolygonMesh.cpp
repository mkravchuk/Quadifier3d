#include "stdafx.h"
#include "PolygonMesh.h"
#include "Mesh.h"
#include "ViewerDrawObjects.h"
#include <ctime>
#include <mutex>

//******************************************************************
// PolygonMesh_MergeVertexesInfo
//******************************************************************


PolygonMesh_VertexesMergeInfo::PolygonMesh_VertexesMergeInfo(const PolygonMesh& _polygonMesh)
    :polygonMesh(_polygonMesh), V(_polygonMesh.V), VertexesCountGlobal(0), VertexesCountLocal(0)
{

}

II PolygonMesh_VertexesMergeInfo::SizeOF() const
{
    II r = sizeof(PolygonMesh_VertexesMergeInfo);
    r += V_IndexesInFile_GlobalOrLocal.size() * sizeof(int);
    return r;
}


void PolygonMesh_VertexesMergeInfo::MarkVertexAsGlobal(int vid, int globalIndex)
{
    // validate
    assert(V.rows() > 0 && "V must be populated");
    // allocate if not allocated
    if (V_IndexesInFile_GlobalOrLocal.size() != V.rows())
    {
        V_IndexesInFile_GlobalOrLocal = Is::Zero(V.rows());
    }
    // mark vertex as global
    V_IndexesInFile_GlobalOrLocal(vid) = globalIndex;
}

void PolygonMesh_VertexesMergeInfo::UpdateGlobalLocalIndexesCount()
{
    if (V_IndexesInFile_GlobalOrLocal.size() == 0)
    {
        VertexesCountGlobal = 0;
        VertexesCountLocal = V.rows();
    }
    else if (VertexesCountGlobal + VertexesCountLocal != V.rows())
    {
        assert(V.rows() == V_IndexesInFile_GlobalOrLocal.size() && "V and V_IndexesInFile_GlobalOrLocal must have same size");
        VertexesCountGlobal = 0;
        VertexesCountLocal = 0;
        for (int vid = 0; vid < V_IndexesInFile_GlobalOrLocal.size(); vid++)
        {
            int& index = V_IndexesInFile_GlobalOrLocal[vid];
            if (index < 0)  // if vertex is global
            {
                // global index
                VertexesCountGlobal++;
            }
            else
            {
                // local index
                VertexesCountLocal++;
                // set local index
                index = VertexesCountLocal;  // starting from 1 - this is our assumption:  global and local starts from 1 to recognize them from 0
            }
        }
    }
}


//******************************************************************
// PolygonMesh
//******************************************************************


PolygonMesh::PolygonMesh(int _groupid, string _groupname)
    : groupid(_groupid), groupname(_groupname), VMergeInfo(*this)
{

}

II PolygonMesh::SizeOF() const
{
    II r = sizeof(PolygonMesh);
    r += V.SizeOF();
    r += VMergeInfo.SizeOF();
    r += ngons.SizeOF();
    return r;
}

int PolygonMesh::GetVertexesCountLocal() const
{
    int vertexesCountLocal = VMergeInfo.V_IndexesInFile_GlobalOrLocal.size() != 0
        ? VMergeInfo.VertexesCountLocal
        : V.rows();
    return vertexesCountLocal;
}

void PolygonMesh::Draw(ViewerDrawObjects& draw, bool reserveSpace) const
{
    if (reserveSpace)
    {
        draw.ReserveEdges(ngons.size() * 5); // reserve space with advance
    }

    map<pair<int, int>, bool> edgeAdded;
    for (int fid = 0; fid < ngons.size(); fid++)
    {
        int n = ngons.size(fid);
        for (int k = 0; k < n; k++)
        {
            int vid0 = ngons(fid, k);
            int vid1 = ngons(fid, (k + 1) % n);
            if (vid1 < vid0) swap(vid0, vid1);
            if (edgeAdded.find({ vid0, vid1 }) == edgeAdded.end()) // avoid adding edge 2 times  (for each triangle/quad)
            {
                edgeAdded[{ vid0, vid1 }] = true;
                P3 v0 = V.row(vid0);
                P3 v1 = V.row(vid1);
                draw.AddEdge(v0, v1, Color3d(0, 0, 1));
            }
        }
    }
}

void convertNgonToTriangles(const P3s& V_file, int* ints, int intsCount, I3s& F, int& F_writeIndex)
{
    struct NgonPoint
    {
        int vid;
        P3 point;
        NgonPoint* prev;
        NgonPoint* next;

        V3 directionToNextPoint; // normalized vector
        V3 normalAtThisPoint; // normal at this point
        D angleAtThisPoint;

        void Update_directionToNextPoint()
        {
            directionToNextPoint = next->point - point;
            directionToNextPoint.normalize();
        }
        void Update_angleAtThisPoint()
        {
            angleAtThisPoint = utils::vector::AngleFull(directionToNextPoint, -prev->directionToNextPoint, normalAtThisPoint, true);
        }
        void DeleteThisPointFromChain()
        {
            prev->next = next;
            next->prev = prev;
            prev->Update_directionToNextPoint();
            prev->Update_angleAtThisPoint();
            next->Update_angleAtThisPoint();
        }
    };

    //
    // init 'vid', 'point', 'prev', 'next'
    //
    int size = intsCount; // set alias
    std::vector<NgonPoint> points;
    points.resize(size);
    for (int i = 0; i < size; i++)
    {
        NgonPoint& ngon = points[i];
        ngon.vid = ints[i];
        ngon.point = V_file.row(ngon.vid);

        int iPrev = i - 1;
        if (iPrev < 0) iPrev = size - 1;
        int iNext = i + 1;
        if (iNext > size - 1) iNext = 0;
        ngon.prev = &points[iPrev];
        ngon.next = &points[iNext];
    }
    // DEBUG trace obj file for this ngon
    //cout << endl << endl << "#OBJ file for ngon" << endl;
    //for (int i = 0; i < intsCount; i++) cout << "v "<<points[i].point(0) << " " << points[i].point(1) << " "<<points[i].point(2) <<endl;
    //cout << "f";
    //for (int i = 0; i < intsCount; i++) cout << " " << (i+1);//obj file indexes starts from 1
    //cout << endl << endl << endl;

    //
    // init 'directionToNextPoint'
    //
    for (int i = 0; i < size; i++)
    {
        points[i].Update_directionToNextPoint();
        //cout << (i == 0 ? "\n" : "") << "#" << i << "   directionToNextPoint.norm()=" << ngons[i].directionToNextPoint.norm() << endl;
    }

    //
    // set naive normal 'normalAtThisPoint' for each point
    //
    for (int i = 0; i < size; i++)
    {
        NgonPoint& p = points[i];
        V3 normal = utils::vector::Cross(p.prev->directionToNextPoint, p.directionToNextPoint);
        if (!isnan(normal(0)))
        {
            D len = utils::vector::Length(normal);
            if (len < 0.01) // if vectors almost parallel - return NAN normal - dont trust suspicious very close to zero normals
            {
                normal = V3(NAN, NAN, NAN);
            }
            else
            {
                normal /= len;
            }
        }
        p.normalAtThisPoint = normal;
        //cout << (i == 0 ? "\n" : "") << "#" << i << "   p.normalAtThisPoint.norm()=" << p.normalAtThisPoint.norm() << "   normal length="<< len << endl;
    }

    //
    // get first trustable normal
    //
    V3 normal0 = V3(NAN, NAN, NAN);
    for (int i = 0; i < size; i++)
    {
        normal0 = points[i].normalAtThisPoint;
        if (!isnan(normal0(0))) break;
    }

    //
    // sum angles
    //
    DD angleSum = 0;
    for (int i = 0; i < size; i++)
    {
        NgonPoint& p = points[i];
        D angle = utils::vector::AngleFull(p.directionToNextPoint, p.next->directionToNextPoint, normal0, true);
        if (angle > 180) angle = angle - 360;
        angleSum += angle;
        //cout << (i == 0 ? "\n" : "") << "#" << i << "   angle=" << angle << endl;
    }
    //cout << "angleSum=" << angleSum << endl;

    //
    // angleSum should be almost 360 if we have choisen correct normal - otherwise flip it
    //
    if (angleSum < 0)
    {
        normal0 = -normal0; // flip normal
    }

    //
    // correct normals direction 
    //
    for (int i = 0; i < size; i++)
    {
        NgonPoint& p = points[i];
        if (isnan(p.normalAtThisPoint(0)))
        {
            p.normalAtThisPoint = normal0;
        }
        else
        {
            if (!utils::vector::SameDirectionIfTheyParallel(p.normalAtThisPoint, normal0))
            {
                p.normalAtThisPoint = -p.normalAtThisPoint;
            }
        }
        //cout << (i==0 ? "\n": "")<<"#" << i << "   isNormalSameDirectionAsNormalAt0index=" << p.isNormalSameDirectionAsNormalAt0index<<"    dot="<<utils::vector::Dot(normal0, p.normalAtThisPoint) << endl;
    }


    // init 'angleAtThisPoint'
    for (int i = 0; i < size; i++)
    {
        NgonPoint& p = points[i];
        p.Update_angleAtThisPoint();
        //cout << (i == 0 ? "\n" : "") << "#" << i << "   angleAtThisPoint=" << p.angleAtThisPoint << endl;
    }

    // addTriangle method
    int ngonPointCountInChain = size;
    auto addTriangle = [&](NgonPoint* p)
    {
        int vid0 = p->prev->vid;
        int vid1 = p->vid;
        int vid2 = p->next->vid;
        //cout << "adding triangle [" << vid0 << "," << vid1 << "," << vid2 << "]" << endl;
        F(F_writeIndex, 0) = vid0;
        F(F_writeIndex, 1) = vid1;
        F(F_writeIndex, 2) = vid2;
        F_writeIndex++;
        p->DeleteThisPointFromChain();
        ngonPointCountInChain--;
    };

    // add best triangle from ngon
    //cout << endl << endl << "adding best triangle from ngon" << endl;
    NgonPoint* pFirst = &points[0];
    while (ngonPointCountInChain >= 3)
    {
        NgonPoint* bestTriangle_Point = nullptr;
        D bestTriangle_bestValue = 0;
        NgonPoint* pi = pFirst;
        for (int i = 0; i < ngonPointCountInChain; i++)
        {
            if (pi->angleAtThisPoint < 170)
            {
                D dist = utils::point::DistToPointPow2(pi->prev->point, pi->next->point);
                if (bestTriangle_Point == nullptr || dist < bestTriangle_bestValue)
                {
                    bestTriangle_Point = pi;
                    bestTriangle_bestValue = dist;
                }
            }
            pi = pi->next;
        }
        if (bestTriangle_Point == nullptr)
        {
            for (int i = 0; i < ngonPointCountInChain; i++)
            {
                //v0
                D angleSumm = pi->prev->angleAtThisPoint + pi->next->angleAtThisPoint - pi->angleAtThisPoint;
                //v1
                //D angleSumm = 0;
                //NgonPoint* pi2 = pi;
                //for (int i2 = 0; i2 < ngonPointCountInChain; i2++)
                //{
                //    D angle = (pi2 == pi->prev || pi2 == pi->next)  ? pi2->angleAtThisPoint : -pi2->angleAtThisPoint;
                //    angleSumm += angle;
                //    pi2 = pi2->next;
                //}
                if (bestTriangle_Point == nullptr || angleSumm > bestTriangle_bestValue)
                {
                    bestTriangle_Point = pi;
                    bestTriangle_bestValue = angleSumm;
                }
                pi = pi->next;
            }
        }
        addTriangle(bestTriangle_Point);
        if (bestTriangle_Point == pFirst)
        {
            pFirst = bestTriangle_Point->next;
        }

        // DEBUG show angles after changes
        //pi = pFirst;
        //for (int i = 0; i < ngonPointCountInChain; i++)
        //{
        //    cout << "vid#" << pi->vid << "   angleAtThisPoint=" << pi->angleAtThisPoint << endl;
        //    pi = pi->next;
        //}
        //cout << endl;
    }
}


void PolygonMesh::ConvertNgonsToF(I3s& F)
{
    //
    // allocate F
    //
    int trianglesCount = 0;
    for (int i = 0; i < ngons.size(); i++)
    {
        int size = ngons.size(i);
        trianglesCount += size - 2;
    }
    F.resize(trianglesCount, 3);

    //
    // convert all ngons to triangles and write to F
    //
    int F_writeIndex = 0;
    for (int i = 0; i < ngons.size(); i++)
    {
        int* ints = ngons.pointer(i);
        int size = ngons.size(i);
        if (size == 3)
        {
            F(F_writeIndex, 0) = ints[0];
            F(F_writeIndex, 1) = ints[1];
            F(F_writeIndex, 2) = ints[2];
            F_writeIndex++;
            continue;
        }
        if (size == 4)
        {
            F(F_writeIndex, 0) = ints[0];
            F(F_writeIndex, 1) = ints[1];
            F(F_writeIndex, 2) = ints[2];
            F_writeIndex++;
            F(F_writeIndex, 0) = ints[0];
            F(F_writeIndex, 1) = ints[2];
            F(F_writeIndex, 2) = ints[3];
            F_writeIndex++;
            continue;
        }
        convertNgonToTriangles(V, ints, size, F, F_writeIndex);
    }
}

void PolygonMesh::ConvertToMesh(Mesh& mesh)
{
    I3s F;
    ConvertNgonsToF(F);

    // 
    // create mesh from V, F
    //
    mesh.CreateFromVF(V, F, mesh, false, true, true, true, false);
    mesh.FileName += "g " + groupname;
}

int PolygonMesh::MergeVertexes(D tol)
{
    //
    // Merge vertixes
    //
    int vertixesMergedCount = 0;
    vector<int> map_vid_linkedvid;
    vector<int> map_vid_newvid;
    if (V.rows() > ngons.size())
    {
        vector<pair<D, int>> pseudo_distances;
        pseudo_distances.reserve(V.rows());
        for (int i = 0; i < V.rows(); i++)
        {
            P3 p = V.row(i);
            pseudo_distances.push_back(pair<D, int>(p.sum(), i));
        }
        utils::stdvector::sort(pseudo_distances);

        map_vid_linkedvid.resize(V.rows());
        iota(map_vid_linkedvid.begin(), map_vid_linkedvid.end(), 0);
        for (int i = 0; i < V.rows() - 1; i++)
        {
            auto info0 = pseudo_distances[i];
            D dist0 = info0.first;
            int index0 = info0.second;
            if (map_vid_linkedvid[index0] != index0) continue; // skip points that was already merged
            P3 p0 = V.row(index0);
            for (int iNext = i + 1; iNext < V.rows(); iNext++)
            {
                auto info1 = pseudo_distances[iNext];
                D dist1 = info1.first;
                int index1 = info1.second;
                if (abs(dist0 - dist1) > tol) break;
                P3 p1 = V.row(index1);
                D distPow2 = utils::point::DistToPointPow2(p0, p1);
                if (distPow2 < tol)
                {
                    vertixesMergedCount++;
                    //cout << "index " << index1 << "  replaced to " << map_vid_linkedvid[index0] << "   from " << mesh.V.row(index0) << "  to" << mesh.V.row(index1) << endl;
                    map_vid_linkedvid[index1] = map_vid_linkedvid[index0];
                }
            }
        }
    }

    if (vertixesMergedCount == 0) return 0;

    // collapse multiple linking to single one
    for (int i = 0; i < V.rows(); i++)
    {
        int k = i;
        while (map_vid_linkedvid[k] != k)
        {
            k = map_vid_linkedvid[k];
        }
        map_vid_linkedvid[i] = k;
    }

    // remove merged points from V and update indexes
    map_vid_newvid.resize(V.rows());
    auto Vcopy = V;
    V.resize(V.rows() - vertixesMergedCount, 3);
    int newIndex = 0;
    for (int i = 0; i < Vcopy.rows(); i++)
    {
        if (map_vid_linkedvid[i] == i)
        {
            V.row(newIndex) = Vcopy.row(i); // copy vertex data from old index to new one
            map_vid_newvid[i] = newIndex;  // move old index to new index 
            newIndex++;
        }
    }

    //
    // Update faces
    //
    int* pvid = ngons.DataPOINTER();
    int* pvidEnd = pvid + ngons.DataSize();
    while (pvid < pvidEnd)
    {
        int vid = *pvid;
        vid = map_vid_linkedvid[vid];  // get vid to wich this vid was merged
        vid = map_vid_newvid[vid]; // now get new index of vid, beacause we removed some vertexes
        *pvid = vid;
        pvid++;
    }

    return vertixesMergedCount;
}

void PolygonMesh::Heal(bool IsOmpEnabled)
{
    if (!meshLogicOptions.Mesh.Heal) return;

    std::mutex mtx;
    vector<int> deleted_indexes;
    if (meshLogicOptions.MeshHeal.PolygonMesh_DeleteZeroTriangles)
    {
        // get indexes that have to be deleted
        #pragma omp parallel for schedule(static) if(IsOmpEnabled)
        for (int i = 0; i < ngons.size(); i++)
        {
            I size = ngons.size(i);
            int* pStart = ngons.pointer(i);
            int* pEnd = pStart + size;


            I deleted = 0;
            int* p = pStart;
            if (p < pEnd)
            {
                int vid = *p;
                while (p < pEnd)
                {
                    int* pNext = p + 1;
                    if (pNext == pEnd) pNext = pStart;
                    int vidNext = *pNext;
                    // remove vertex from polygon if it is duplicated
                    if (vid == vidNext)
                    {
                        if (size > 3 && pNext != pStart)
                        {
                            while (p < pEnd - 1)
                            {
                                *p = *(p + 1);
                                p++;
                            }
                        }
                        pEnd--;
                        deleted++;
                    }
                    p++;
                    vid = vidNext;
                }
            }

            if (size - deleted < 3)
            {
                //cout << "PolygonMesh::Heal   DeleteZeroTriangles   i=" << i << "   size=" << size << "   deleted=" << deleted << endl;
                mtx.lock();
                deleted_indexes.push_back(i);
                mtx.unlock();
            }
        }
    }

    if (meshLogicOptions.MeshHeal.PolygonMesh_DeleteUnattachedVertexes)
    {
        Bs vs = Bs::Constant(V.rows(), false);
        int* pStart = ngons.DataPOINTER();
        int* pEnd = pStart + ngons.DataSize();
        int* p = pStart;
        while (p < pEnd)
        {
            vs(*p) = true;
            p++;
        }
        if (!vs.all(true))
        {
            //cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!            !vs.all(true)            !!!!!!!!!! " << endl;
            Is indexes;
            vs.ToIndexes(indexes);
            Is old_index_to_new_indexes = Is::Constant(V.rows(), -1);
            for (int i = 0; i < indexes.size(); i++)
            {
                old_index_to_new_indexes[indexes[i]] = i;
            }
            // change ngons vids
            p = pStart;
            while (p < pEnd)
            {
                *p = old_index_to_new_indexes[*p];
                p++;
            }
            // change V
            for (int i = 0; i < old_index_to_new_indexes.size(); i++)
            {
                if (old_index_to_new_indexes[i] == -1) continue;
                V.row(old_index_to_new_indexes[i]) = V.row(i);
            }
            V.conservativeResize(indexes.size(), 3);
        }

    }


    // remove ngons that should be deleted
    ngons.remove(deleted_indexes);
}


//******************************************************************
// PolygonMeshes
//******************************************************************


void PolygonMeshes::Add(vector<const PolygonMesh*> _meshes)
{
    utils::stdvector::append(meshes, _meshes);
}

int PolygonMeshes::GetVertexesCount() const
{
    int localCount = 0;
    for (const auto& m : meshes)
    {
        localCount += m->GetVertexesCountLocal();
    }
    return localCount + GetCommonVertexesCount();
}

int PolygonMeshes::GetNgonsCount() const
{
    int count = 0;
    for (const auto& m : meshes) count += m->ngons.size();
    return count;
}

int PolygonMeshes::GetGroupsCount() const
{
    int prev_groupid = -1;
    int count = 0;
    for (const auto& m : meshes)
    {
        if (m->groupid != prev_groupid)
        {
            count++;
            prev_groupid = m->groupid;
        }
    }
    return count;
}



int PolygonMeshes::GetCommonVertexesCount() const
{
    vector<int> maxGlobalIndexes(meshes.size(), 0);
    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        const auto& m = meshes[i];
        int maxGlobalIndex = m->VMergeInfo.V_IndexesInFile_GlobalOrLocal.size() > 0 ? m->VMergeInfo.V_IndexesInFile_GlobalOrLocal.minCoeff() : 0;
        if (maxGlobalIndex < 0)
        {
            // global index found
            maxGlobalIndexes[i] = -maxGlobalIndex;
        }
        else
        {
            // global index not found
            maxGlobalIndexes[i] = 0;
        }
    }

    int maxGlobalIndex = utils::stdvector::max_element(maxGlobalIndexes);
    return maxGlobalIndex;
}

void PolygonMeshes::GetCommonVertexes(P3s& V_common) const
{
    int commonVertexesCount = GetCommonVertexesCount();
    V_common.resize(commonVertexesCount, 3);

    if (commonVertexesCount > 0)
    {
        extern bool IsOmpEnabled;
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < meshes.size(); i++)
        {
            const auto& m = meshes[i];
            for (int vid = 0; vid < m->VMergeInfo.V_IndexesInFile_GlobalOrLocal.size(); vid++)
            {
                int index = m->VMergeInfo.V_IndexesInFile_GlobalOrLocal[vid];
                if (index < 0) // if vertex is global
                {
                    int vid_common = (-index) - 1; //  since global and local indexes starts from -1 and 1
                    V_common.row(vid_common) = m->V.row(vid);
                }
            }
        }
    }
}
int PolygonMeshes::Draw_GetAddedEdgesCount() const
{
    int ngonsSize = 0;
    for (auto& p : meshes)
    {
        ngonsSize += p->ngons.size();
    }
    return ngonsSize * 5;
};
void PolygonMeshes::Draw(ViewerDrawObjects& draw) const
{
    draw.ReserveEdges(Draw_GetAddedEdgesCount()); // reserve space in advance

    for (auto& p : meshes)
    {
        p->Draw(draw, false);
    }
}




