#include "stdafx.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshTopologyLoop.h"
#include "MeshPart.h"
#include <igl/serialize.h>
#include "PolygonMesh.h"
#include "MeshFile.h"
#include "Model.h"


MeshLogicOptions_Mesh& options = meshLogicOptions.Mesh;
Elapsed_Mesh& elapsed = elapsedTimers.Mesh;

string MeshPoint::vid_eid_fid_toString() const
{
    return ((Type == MeshPointType::onVertex) ? "vid=" : "eid=") + to_string(vid_eid_fid);
}
bool MeshPoint::isOnBorder(const Mesh& mesh) const
{
    return  ((Type == MeshPointType::onVertex && mesh.V_isborder[vid_eid_fid])
        || (Type == MeshPointType::onEdge && mesh.E_isborder[vid_eid_fid]));
}

void MeshVInfo::Update(const P3s& V)
{
    Vmin = P3(0, 0, 0);
    Vmax = P3(0, 0, 0);
    Vcenter = P3(0, 0, 0);
    VcenterProjected = P3(0, 0, 0);
    VcenterProjectedIndside = P3(0, 0, 0);
    vcenterProjectedIndside_isset = false;
    if (V.rows() > 0)
    {
        #ifdef USE_EIGEN
        Vmin = V.colwise().minCoeff();
        Vmax = V.colwise().maxCoeff();
        Vcenter = V.colwise().sum() / V.rows();
        #else
        int rows = V.rows();
        Vmin = V.row(0);
        Vmax = V.row(0);

        DD sum0 = 0;
        DD sum1 = 0;
        DD sum2 = 0;
        for (int i = 0; i < rows; i++)
        {
            P3 v = V.row(i);
            Vmin = utils::sse::min(Vmin, v);
            Vmax = utils::sse::max(Vmax, v);
            sum0 += v(0);
            sum1 += v(1);
            sum2 += v(2);
        }
        Vcenter = P3(sum0 / rows, sum1 / rows, sum2 / rows);
        #endif
        VcenterProjected = V.row(utils::mesh::GetClosestPointIndex(V, Vcenter));
        VcenterProjectedIndside = VcenterProjected;
    }

}
void MeshVInfo::Init_VcenterProjectedIndside(const Mesh& mesh)// center of all points projected to closer point + moved a little inside mesh
{
    if (!vcenterProjectedIndside_isset)
    {
        std::vector<int> closestPointsIndexes;
        utils::mesh::GetClosestPointIndexes(mesh.V, Vcenter, closestPointsIndexes, 30);
        P3s closestpoints;
        closestpoints.resize(closestPointsIndexes.size(), 3);
        for (int i = 0; i < closestPointsIndexes.size(); i++)
        {
            closestpoints.row(i) = mesh.V.row(closestPointsIndexes[i]);
        }
        #ifdef USE_EIGEN
        P3 Vcenter2 = closestpoints.colwise().sum() / closestpoints.rows();
        #else
        int rows = closestpoints.rows();
        DD sum0 = 0;
        DD sum1 = 0;
        DD sum2 = 0;
        for (int i = 0; i < rows; i++)
        {
            P3 v = closestpoints.row(i);
            sum0 += v(0);
            sum1 += v(1);
            sum2 += v(2);
        }
        P3 Vcenter2 = P3(sum0 / rows, sum1 / rows, sum2 / rows);
        #endif
        utils::mesh::GetClosestPointIndexes(closestpoints, Vcenter2, closestPointsIndexes, 3);
        //cout << "Init_VcenterProjectedIndside    mesh.id=" << mesh.id << "   vids=" << closestPointsIndexes[0] << "," << closestPointsIndexes[1] << "," << closestPointsIndexes[2] << endl;
        VcenterProjectedIndside = (closestpoints.row(closestPointsIndexes[0]) + closestpoints.row(closestPointsIndexes[1]) + closestpoints.row(closestPointsIndexes[2])) / 3;
        vcenterProjectedIndside_isset = true;
    }
}

void MeshVInfo::InitMesh_VcenterProjectedIndside_IfRequiredByDraw(const Mesh& mesh)// center of all points projected to closer point + moved a little inside mesh
{
    if (meshLogicOptions.Draw.Mesh.Id_Mesh || meshLogicOptions.Draw.Mesh.Obj_id || meshLogicOptions.Draw.Mesh.Obj_name)
    {
        Init_VcenterProjectedIndside(mesh);
    }
}

atomic_int Mesh::nextMeshID;


void MeshNormalCorrections::Clear()
{
    isInited = false;
    V.resize(0, 3);
    V_normals.resize(0, 3);
    V_correcteIndexes.resize(0);
    Fcorrections.resize(0, 4);
}

Mesh::Mesh()
    : id(nextMeshID++), originalMeshId(-1), Area(0), avg_edge_length(0), min_edge_length(0), max_edge_length(0),
    EdgesCount(0), FacesCount(0), VertexCount(0), LoopsCount(0), PartsCount(0), FileName("")
{

}

II Mesh::SizeOF() const
{
    //
    // Vertex
    //
    II r = sizeof(Mesh);
    r += V.size() * sizeof(D);
    r += V_isborder.SizeOF();
    r += VF.SizeOF();
    r += VFi.SizeOF();
    r += VE.SizeOF();
    r += V_normals.size() * sizeof(D);


    //
    // Face
    //
    r += F.size() * sizeof(int);
    r += F_isborder.SizeOF();
    //r += F_normals.size() * sizeof(D);  - no need since its just a link to F_Y
    r += F_Areas.size() * sizeof(D);
    r += F_Barycenters.size() * sizeof(D);
    r += F_X.size() * sizeof(D);
    r += F_Y.size() * sizeof(D);
    r += F_Z.size() * sizeof(D);
    r += FE.size() * sizeof(int);
    r += TTT.size() * sizeof(int);
    r += FF.size() * sizeof(int);
    r += FFi.size() * sizeof(int);

    //
    // Edge
    //
    r += EV.size() * sizeof(int);
    r += EF.size() * sizeof(int);
    r += EFi.size() * sizeof(int);
    r += E_isborder.SizeOF();
    r += E_Length.size() * sizeof(D);
    r += E_Angles.size() * sizeof(D);

    //
    // Frame topology (computek)
    //
    r += K.size() * sizeof(D);
    r += Kcos.size() * sizeof(D);
    r += Ksin.size() * sizeof(D);


    //
    // Loops
    //
    r += Loops.capacity() * sizeof(MeshLoop);
    for (const auto& loop : Loops) r += loop.SizeOF();
    r += Parts.capacity() * sizeof(MeshPart);
    for (const auto& parts : Parts) r += parts.SizeOF();
    r += LoopTopology.SizeOF();

    return r;
}

namespace igl
{
    namespace serialization
    {
        IGL_INLINE void serialization(bool s, Mesh& obj, std::vector<char>& buffer)
        {
            // Mesh information
            SERIALIZE_MEMBER(V);
            SERIALIZE_MEMBER(F);

            // Edge Topology
            SERIALIZE_MEMBER(EV);
            SERIALIZE_MEMBER(EF);
            SERIALIZE_MEMBER(EFi);
            SERIALIZE_MEMBER(FE);
            SERIALIZE_MEMBER(E_isborder);
            //SERIALIZE_MEMBER(F_isborder);

            // Face Topology
            SERIALIZE_MEMBER(TTT);
        }

        template<>
        IGL_INLINE void serialize(const Mesh& obj, std::vector<char>& buffer)
        {
            serialization(true, const_cast<Mesh&>(obj), buffer);
        }

        template<>
        IGL_INLINE void deserialize(Mesh& obj, const std::vector<char>& buffer)
        {
            serialization(false, obj, const_cast<std::vector<char>&>(buffer));
        }
    }
}
void Mesh::SplitBorderTriangleWith2BorderEdges()
{
    constexpr bool isDebug = false;
    vector<P3> addedPoints;         // V
    vector<I3> addedFaces;  // F
    vector<I2> addedEF;      // EF
    vector<I3> addedFE;      // FE
    vector<int> outdatedFaces; //  here we will be stored faces that already splited, so we can any more work with them until next self call to this method SplitNakedTriangleWith2NakedEdge
    vector<int> outdatedEdges; //  here we will be stored faces that already splited, so we can any more work with them until next self call to this method SplitNakedTriangleWith2NakedEdge
    addedPoints.reserve(128);
    addedFaces.reserve(128);
    addedEF.reserve(128);
    addedFE.reserve(128);
    outdatedFaces.reserve(128);
    outdatedEdges.reserve(128);
    Vector<bool> ignoreE;      // flags to ignore edge - needed to prevent error when accessing modified EF or FE

    bool needs_to_update_edge_topology = true; // allows algorithm to recall this method in case we need to update edge_topology. we will try to avoid calling to edge_topology if possible, since recalculating topology is not cheap
    while (needs_to_update_edge_topology)
    {
        needs_to_update_edge_topology = false;
        addedPoints.clear();
        addedFaces.clear();
        addedEF.clear();
        addedFE.clear();
        outdatedFaces.clear();
        outdatedEdges.clear();
        ignoreE.setConstant(EF.rows(), false);

        int splitFaces[2];
        for (int eid = 0; eid < EF.rows(); ++eid) // For every border edge 
        {
            if (!E_isborder[eid]) continue;
            if (ignoreE(eid)) continue;

            int fid = EF(eid, 0);
            if (fid == -1) fid = EF(eid, 1);

            //
            // protection of using outdated topology relation
            //
            if (utils::stdvector::exists(outdatedFaces, fid))
            {
                needs_to_update_edge_topology = true;
                continue;
            }

            //
            // check if face has 2 naked edges
            //
            int nakedEdgesCount = 0;
            for (int k = 0; k < 3; k++)
            {
                auto eid_k = FE(fid, k);
                int edge_fid0 = EF(eid_k, 0);
                int edge_fid1 = EF(eid_k, 1);
                bool isEdgeNaked = (edge_fid0 == -1 || edge_fid1 == -1);
                if (isEdgeNaked)
                {
                    nakedEdgesCount++;
                }
            }
            if (nakedEdgesCount != 2) continue;


            //
            // get common edge
            //
            int fid_friend = -1;
            int common_edge_eid = -1;
            int common_edge_k = -1;
            for (int k = 0; k < 3; k++)
            {
                auto eid_k = FE(fid, k);
                int edge_fid0 = EF(eid_k, 0);
                int edge_fid1 = EF(eid_k, 1);
                bool isEdgeNaked = (edge_fid0 == -1 || edge_fid1 == -1);
                if (!isEdgeNaked)
                {
                    common_edge_eid = eid_k;
                    common_edge_k = k;
                    fid_friend = edge_fid0;
                    if (fid_friend == fid)  fid_friend = edge_fid1;
                }
            }
            if (fid_friend == -1) continue;

            if (isDebug)
            {
                cout << endl << endl << "eid=" << eid << "    fid=" << fid << "    fid_friend=" << fid_friend << "    common_edge_eid=" << common_edge_eid << endl;
            }

            //
            // protection of using outdated topology relation
            //
            if (ignoreE(common_edge_eid) // non border edge is setted to ignore when EF for it is updated and EF conatins a fid that is still not exists in F - will be updated after main loop is finished
                || utils::stdvector::exists(outdatedFaces, fid)
                || utils::stdvector::exists(outdatedFaces, fid_friend)
                || utils::stdvector::exists(outdatedEdges, common_edge_eid))
            {
                if (isDebug)
                {
                    if (ignoreE(common_edge_eid))  cout << "  ignored eid " << common_edge_eid;
                    if (utils::stdvector::exists(outdatedFaces, fid)) cout << "  outdated fid " << fid;
                    if (utils::stdvector::exists(outdatedFaces, fid_friend)) cout << "  outdated fid_friend " << fid_friend;
                    if (utils::stdvector::exists(outdatedEdges, common_edge_eid)) cout << "  outdated common_edge_eid " << common_edge_eid;
                    cout << "   -  skip edge=" << eid << "  from splitting" << endl;
                }
                needs_to_update_edge_topology = true;
                continue;
            }
            // mark this faces as outdated - if we will in future in need to split them - we will recall our method to update topology
            outdatedFaces.push_back(fid);
            outdatedFaces.push_back(fid_friend);
            outdatedEdges.push_back(common_edge_eid);
            ignoreE(common_edge_eid) = true;

            //
            // allocate new point - mid point of common edge
            //
            int new_vid = V.rows() + addedPoints.size();
            int common_edge_vid1 = F(fid, common_edge_k);
            int common_edge_vid2 = F(fid, (common_edge_k + 1) % 3);
            P3 newPoint = (V.row(common_edge_vid1) + V.row(common_edge_vid2)) / 2;
            addedPoints.push_back(newPoint); // put new point to the cache list - we will add all points at once to avoid redundant memory reallocations

            if (isDebug)
            {
                cout << "   new_vid=" << new_vid << "    common_edge_vid1=" << common_edge_vid1 << "    common_edge_vid2=" << common_edge_vid2 << endl;
            }
            //
            // split 2 faces to four faces at middle point of common edge
            //
            splitFaces[0] = fid;
            splitFaces[1] = fid_friend;
            //cout << "spliting faces:  " << faceId << ", " << friendFaceId  << "  on common commonEdgeId " << commonEdgeId  << endl;
            pair<int, int> new_fids[8]; // pair<newfid, oldfid>  should be 4 but we reserve 8 to avoid any problems
            I3 new_fs[8]; // should be 4 but we reserve 8 to avoid any problems
            int new_fids_count = 0;
            I2 all_EVs[8 + 4];
            int all_edges_ids[8 + 4];
            int all_edges_count = 0;
            I3 old_vids[2]; // save F.row(fid) and F.row(fid_friend) because we will rewrite this information
            old_vids[0] = F.row(splitFaces[0]);
            old_vids[1] = F.row(splitFaces[1]);
            for (int fid_split : splitFaces)
            {
                // commonEdgeId, newPointIndex
                I3 vids = F.row(fid_split);
                int splitCount = 0;
                // iterate 3 times on each vertex, and 1 time will be discarded as common edge - so in result we will get 2 new faces
                for (int k = 0; k < 3; k++)
                {
                    int vid1 = vids(k);
                    int vid2 = vids((k + 1) % 3);
                    // skip common edge - we wil split it
                    if ((vid1 == common_edge_vid1 && vid2 == common_edge_vid2) || (vid1 == common_edge_vid2 && vid2 == common_edge_vid1))
                    {
                        continue;
                    }

                    all_EVs[all_edges_count] = I2(min(vid1, vid2), max(vid1, vid2));
                    all_edges_ids[all_edges_count] = FE(fid_split, k);
                    all_edges_count++;

                    splitCount++;
                    I3 newFace(vid1, vid2, new_vid);
                    // for the first new face we will reuse same index from F, and for second new face we have to append to F
                    if (splitCount == 1)
                    {
                        F.row(fid_split) = newFace; // reuse same index
                        new_fids[new_fids_count] = { fid_split, fid_split };
                    }
                    else
                    {
                        new_fids[new_fids_count] = { F.rows() + addedFaces.size(), fid_split };
                        addedFaces.push_back(newFace); // put new face to the cache list - we will add all faces at once to avoid redundant memory reallocations
                    }
                    if (isDebug)
                    {
                        cout << "   new_fid=" << new_fids[new_fids_count].first << "    newFace=" << to_string(newFace) << ((splitCount == 1) ? "  (reused)" : "") << endl;
                    }
                    new_fs[new_fids_count] = newFace;
                    new_fids_count++;
                }
                assert(splitCount == 2 && "spliting face should produce exactly 2 faces");
            }
            assert(new_fids_count == 4 && "spliting edge should produce exactly 4 faces");

            //
            // detect new edges: EV, EF
            //
            I2 new_EVs[8]; // EV,  should be 4 but we reserve 8 to avoid any problems
            I2 new_EFs[8]; // EF,  should be 4 but we reserve 8 to avoid any problems
            int new_edges_ids[8]; // should be 4 but we reserve 8 to avoid any problems
            int new_edges_count = 0;
            auto search_new_edge_index = [&](const I2& new_edge)
            {
                for (int i = 0; i < new_edges_count; i++)
                {
                    if (new_EVs[i] == new_edge)
                    {
                        return i;
                    }
                }
                return -1;
            };
            auto search_friend_face = [&](int search_fid, const I2& new_edge)
            {
                for (int i = 0; i < new_fids_count; i++)
                {
                    int friend_fid = new_fids[i].first;
                    if (search_fid == friend_fid) continue;
                    I3 vids = new_fs[i];
                    for (int k = 0; k < 3; k++)
                    {
                        int vid1 = vids(k);
                        int vid2 = vids((k + 1) % 3);
                        if (vid2 < vid1) swap(vid1, vid2);
                        I2 friend_edge(vid1, vid2);
                        if (friend_edge == new_edge)
                        {
                            return friend_fid;
                        }
                    }
                }
                assert(false && "never should reach this line - each new face will have a friend");
                return -1;
            };
            for (int i = 0; i < new_fids_count; i++)
            {
                int fid1 = new_fids[i].first;
                I3 vids = new_fs[i];
                for (int k = 0; k < 3; k++)
                {
                    int vid1 = vids(k);
                    int vid2 = vids((k + 1) % 3);
                    if (vid1 != new_vid && vid2 != new_vid) continue;
                    if (vid2 < vid1) swap(vid1, vid2);
                    I2 new_edge(vid1, vid2);
                    int new_edge_index = search_new_edge_index(new_edge);
                    if (new_edge_index == -1)
                    {
                        new_EVs[new_edges_count] = new_edge;
                        int new_edge_eid = (new_edges_count == 0)
                            ? common_edge_eid // reuse edge index that was common
                            : EF.rows() + addedEF.size();
                        int fid2 = search_friend_face(fid1, new_edge);
                        I2 new_EF(fid1, fid2);
                        if (new_edge_eid < EF.rows())
                        {
                            EF.row(new_edge_eid) = new_EF;
                        }
                        else
                        {
                            addedEF.push_back(new_EF);
                        }
                        if (isDebug)
                        {
                            cout << "   EF:   new_edge_eid=" << new_edge_eid
                                << "    new_EF=" << to_string(new_EF)
                                << ((new_edge_eid < EF.rows()) ? "  (reused)" : "")
                                << endl;
                        }

                        new_edges_ids[new_edges_count] = new_edge_eid;
                        new_EFs[new_edges_count] = new_EF;
                        new_edges_count++;

                        all_EVs[all_edges_count] = new_edge;
                        all_edges_ids[all_edges_count] = new_edge_eid;
                        all_edges_count++;
                    }
                }
            }
            //cout << "new_edges_count=" << new_edges_count << endl;
            assert(new_edges_count == 4 && "splitting should produce 4 new edges");

            // ignore edge in next for statement
            for (int i = 0; i < all_edges_count; i++)
            {
                int edge_eid = all_edges_ids[i];
                if (edge_eid < EF.rows()) // if edge not added but already existed - then ignore it
                {
                    ignoreE(edge_eid) = true;
                }
            }

            //
            //  update old edges to connect to new faces
            //
            auto got_old_edge_id = [&](const I2& edge)
            {
                for (int i = 0; i < 2; i++)
                {
                    I3 vids = old_vids[i];
                    int fid_old = splitFaces[i];
                    for (int k = 0; k < 3; k++)
                    {
                        int vid1 = vids(k);
                        int vid2 = vids((k + 1) % 3);
                        if (vid2 < vid1) swap(vid1, vid2);
                        I2 old_edge(vid1, vid2);
                        if (edge == old_edge)
                        {
                            int old_edge_eid = FE(fid_old, k);
                            return old_edge_eid;
                        }
                    }
                }

                return -1;
            };
            for (int i = 0; i < new_fids_count; i++)
            {
                int new_fid = new_fids[i].first;
                int old_fid = new_fids[i].second;
                if (new_fid == old_fid) continue;
                I3 vids = new_fs[i];
                for (int k = 0; k < 3; k++)
                {
                    int vid1 = vids(k);
                    int vid2 = vids((k + 1) % 3);
                    if (vid1 == new_vid || vid2 == new_vid) continue;
                    if (vid2 < vid1) swap(vid1, vid2);
                    I2 edge(vid1, vid2);
                    int old_eid = got_old_edge_id(edge);
                    I2 old_fids = EF.row(old_eid);
                    if (old_fids(0) == old_fid)
                    {
                        EF(old_eid, 0) = new_fid;
                    }
                    if (old_fids(1) == old_fid)
                    {
                        EF(old_eid, 1) = new_fid;
                    }
                    ignoreE(old_eid) = true; //  we must ingore this edge as a common edge until main loop finished and F will updated
                    if (isDebug)
                    {
                        I2 new__fids = EF.row(old_eid);
                        cout << "   EF:  old_eid=" << old_eid << "    old=" << to_string(old_fids) << "    new=" << to_string(new__fids) << endl;
                    }
                }
            }


            //
            // detect FE
            //
            auto search_all_edge_index = [&](const I2& edge)
            {
                for (int i = 0; i < all_edges_count; i++)
                {
                    if (all_EVs[i] == edge)
                    {
                        return i;
                    }
                }
                assert(false && "can't find edge - search edge must exists in a list 'all_EVs'");
                return -1;
            };
            auto get_FE = [&](int search_fid)
            {

                for (int i = 0; i < new_fids_count; i++)
                {
                    if (search_fid == new_fids[i].first)
                    {
                        I3 new_fe;
                        I3 vids = new_fs[i];
                        for (int k = 0; k < 3; k++)
                        {
                            int vid1 = vids(k);
                            int vid2 = vids((k + 1) % 3);
                            if (vid2 < vid1) swap(vid1, vid2);
                            I2 edge(vid1, vid2);
                            int edge_index = search_all_edge_index(edge);
                            new_fe(k) = all_edges_ids[edge_index];
                        }
                        return new_fe;
                    }
                }
                assert(false && "never should reach this line - each face will have a 3 edges and all edges should be added to 'all_edges_ids'");
                return I3(0, 0, 0);
            };
            for (int i = 0; i < new_fids_count; i++)
            {
                int new_fid = new_fids[i].first;
                I3 new_FE = get_FE(new_fid);
                if (new_fid < F.rows())
                {
                    FE.row(new_fid) = new_FE; // reuse
                }
                else
                {
                    addedFE.push_back(new_FE);
                }
                if (isDebug)
                {
                    cout << "   FE:   new_fid=" << new_fid << "    new_FE=" << to_string(new_FE) << ((new_fid < F.rows()) ? "  (reused)" : "") << endl;
                }
            }

        }

        // add V
        {
            int vindex = V.rows();
            V.conservativeResize(V.rows() + addedPoints.size(), V.cols());
            for (auto p : addedPoints)
            {
                V.row(vindex) = p;
                vindex++;
            }
        }
        //add F
        {
            int findex = F.rows();
            F.conservativeResize(F.rows() + addedFaces.size(), F.cols());
            F_isborder.conservativeResize(F.rows()); // F size is already updated so use without adding addedFaces.size()
            for (auto f : addedFaces)
            {
                F.row(findex) = f;
                F_isborder(findex) = true;
                findex++;
            }
        }
        // add EF
        {
            int efindex = EF.rows();
            EF.conservativeResize(EF.rows() + addedEF.size(), EF.cols());
            E_isborder.conservativeResize(EF.rows());
            for (auto ef : addedEF)
            {
                EF.row(efindex) = ef;
                E_isborder(efindex) = false;
                efindex++;
            }
        }
        // add FE
        {
            assert(addedFE.size() == addedFaces.size() && "amount of added F and FE must be the same");
            int feindex = FE.rows();
            FE.conservativeResize(FE.rows() + addedFE.size(), FE.cols());
            for (auto fe : addedFE)
            {
                FE.row(feindex) = fe;
                feindex++;
            }
        }
        //Update_F_isborder(); //- we done it manually to increase speed
    }
}

void Mesh::Update_F_isborder()
{
    // v0
    F_isborder.resize(F.rows());
    for (int fid = 0; fid < F.rows(); fid++)
    {
        F_isborder[fid] = E_isborder[FE(fid, 0)] || E_isborder[FE(fid, 1)] || E_isborder[FE(fid, 2)];
    }
}

void Mesh::OptimizeVertexIndexesForGPU_old(bool updateMeshTopology_correctFacesEdgeIndexes)
{
    bool debug = false;
    if (debug) cout << "===OptimizeVertexIndexesForGPU===OLD version" << endl;

    //utils::mesh::triangle_triangle_adjacency(F, TTT, FF, FFi);
    //utils::mesh::vertex_triangle_adjacency(V, F, VF, VFi);
    //utils::mesh::vertex_edge_adjacency(V, F, VF, VFi, FE, VE);
    //Bs V_isborder = is_border_vertex(); // create copy to change localy
    Bs E_isborder = this->E_isborder; // create copy to change localy
    Bs F_isborder = this->F_isborder; // create copy to change localy
    Bs F_isProcessed = Bs::Constant(F.rows(), false); // if we added face - mark it

    int processedFacesCount = 0;
    int unprocessedFacesCount = F.rows();

    auto findStartBorderFace = [&](int& start_fid, int& start_eid, int& start_eid_k)
    {
        for (int fid = 0; fid < F.rows(); fid++)
        {
            if ((fid & (boolBitsCount - 1)) == 0) // every 32-th face check at once 32 faces - and skip them if in next 32 faces we will not find border face - speed optimzation - 5% improvement
            {
                int boolBits_isborder = F_isborder.data()[fid / boolBitsCount];
                int boolBits_isProcessed = F_isProcessed.data()[fid / boolBitsCount];
                int boolBits = boolBits_isborder & (~boolBits_isProcessed); // F_isborder[fid] && !F_isProcessed[fid]   -  The bitwise NOT operator in C++ is the tilde character ~
                if (boolBits == 0)
                {
                    fid += (boolBitsCount - 1);
                    continue;
                }
            }
            if (F_isborder[fid] && !F_isProcessed[fid])
            {
                start_fid = fid;
                for (int k = 0; k < 3; k++)
                {
                    start_eid = FE(fid, k);
                    if (E_isborder(start_eid))
                    {
                        start_eid_k = k;
                        return true;
                        break;
                    }
                }
            }
        }

        if (unprocessedFacesCount > 0) // if model is solid
        {
            for (int fid = 0; fid < F.rows(); fid++)
            {
                if (!F_isProcessed(fid))
                {
                    start_fid = fid;
                    start_eid_k = 0;
                    start_eid = FE(fid, start_eid_k);
                    return true;
                }
            }
        }
        return false;
    };
    auto findNextBorderFace = [&](int& fid, int& eid, int& eid_k)
    {
        for (int nextk = 1; nextk < 3; nextk++) // test only 2 next edges starting from current
        {
            int next_eid_k = (eid_k + nextk) % 3;
            int next_eid = FE(fid, next_eid_k);
            if (E_isborder[next_eid])
            {
                int fid1 = EF(next_eid, 0);
                int fid2 = EF(next_eid, 1);
                int next_fid = fid1 == fid ? fid2 : fid1;
                if (next_fid != -1 && !F_isProcessed[next_fid])
                {
                    fid = next_fid;
                    eid = next_eid;
                    for (int k = 0; k < 3; k++)
                    {
                        if (FE(fid, k) == eid)
                        {
                            eid_k = k;
                            break;
                        }
                    }
                    return true;
                }
            }
        }
        return false;
    };


    //
    // Create new F
    //
    bool success = true;
    I3s Fnew(F.rows(), F.cols());
    Is V_newIndex = Is::Constant(V.rows(), -1);
    int next_write_Vindex = 0;

    int groupNum = 0;
    while (unprocessedFacesCount > 0)
    {
        // get border face
        int start_fid;
        int start_eid;
        int start_eid_k;
        bool found = findStartBorderFace(start_fid, start_eid, start_eid_k);
        assert(found);
        // DEBUG
        groupNum++;
        if (debug) cout << "group#" << groupNum << "   start_fid=" << start_fid << "   start_eid=" << start_eid << endl;

        int fid = start_fid;
        int eid = start_eid;
        int eid_k = start_eid_k;
        auto save_processedFacesCount = processedFacesCount;
        do
        {
            if (debug) cout << "    fid=" << fid << "   eid=" << eid << endl;

            // add face to new mesh
            I3 f = F.row(fid);
            I3 fnew;
            for (int k = 0; k < 3; k++)
            {
                int vid_k = (eid_k + k) % 3;
                int vid = f(vid_k);
                int vidnew = V_newIndex(vid);
                if (vidnew == -1)
                {
                    V_newIndex(vid) = next_write_Vindex;
                    vidnew = next_write_Vindex;
                    next_write_Vindex++;
                }
                fnew(k) = vidnew;
            }
            Fnew.row(processedFacesCount) = fnew;

            // update counters
            unprocessedFacesCount--;
            processedFacesCount++;

            // update border flags
            for (int k = 0; k < 3; k++)
            {
                int eidk = FE(fid, k);
                E_isborder(eidk) = true; //  add edges to search
                int fid1 = EF(eidk, 0);
                int fid2 = EF(eidk, 1);
                F_isborder[fid1] = true; // add face to search
                if (fid2 != -1) F_isborder[fid2] = true; // add face to search
            }
            F_isProcessed[fid] = true; // remove face from search
            if (debug)
            {
                extern Model model;
                auto center_face = (V.row(F(fid, 0)) + V.row(F(fid, 1)) + V.row(F(fid, 2))) / 3;
                //model.draw.AddLabel(center_face, "" + to_string(groupNum), Color3d((groupNum & 7) == 1, (groupNum & 7) == 2, (groupNum & 7) == 3));
                model.draw.AddLabel(center_face, "" + to_string(processedFacesCount - save_processedFacesCount), Color3d((groupNum & 7) == 1, (groupNum & 7) == 2, (groupNum & 7) == 3));
                //auto center_edge = (V.row(EV(eid, 0)) + V.row(EV(eid, 1))) / 2;
                //model.draw.AddLabel(center_edge, "" + to_string(processedFacesCount - save_processedFacesCount));
            }
        } while (unprocessedFacesCount > 0 && findNextBorderFace(fid, eid, eid_k));
        cout << "processed " << processedFacesCount - save_processedFacesCount << " faces" << endl;
        debug = false; // show only first group
    }

    //
    // Create new V
    //
    P3s Vnew(V.rows(), V.cols());
    for (int i = 0; i < V.rows(); i++)
    {
        int newIndex = V_newIndex(i);
        if (newIndex == -1)
        {
            success = false;
            cout << "!   error in Mesh::OptimizeVertexIndexesForGPU():  index " << i << " is not initialized" << endl;
            assert(newIndex != -1 && "index is not initialized");
            continue;
        }
        Vnew.row(newIndex) = V.row(i);
    }

    //
    // Update V and F if we succeed
    //
    if (unprocessedFacesCount > 0)
    {
        success = false;
    }
    if (success)
    {
        V = Vnew;
        F = Fnew;
        if (debug) cout << "success !!!" << endl;
    }
    else
    {
        if (debug) cout << "failed !!!" << endl;
    }

    //
    // Update topology after we changed V and F
    //
    if (debug) cout << "=================================" << endl;
}

void Mesh::OptimizeVertexIndexesForGPU(bool updateMeshTopology_correctFacesEdgeIndexes)
{
    const bool debug = false;
    if (debug) cout << "===OptimizeVertexIndexesForGPU===" << endl;
    I2* pEF = (I2*)EF.data();
    register int* pFE = FE.data();
    I3* pF = (I3*)F.data();

    //utils::mesh::triangle_triangle_adjacency(F, TTT, TT, TTi); // create relations only for this method, because after changing 
    //utils::mesh::vertex_triangle_adjacency(V, F, VF, VFi);
    //utils::mesh::vertex_edge_adjacency(V, F, VF, VFi, FE, VE);
    //Bs V_isborder = is_border_vertex(); // create copy to change localy
    Bs E_isborder = this->E_isborder; // create copy to change localy
    Bs F_isborder = this->F_isborder; // create copy to change localy
    Bs F_isProcessed = Bs::Constant(F.rows(), false); // if we added face - mark it


    Is border_fids;
    F_isborder.ToIndexes(border_fids, F.rows() * 3); // allocate enought space for all algorithm life
    int* border_fids_from = border_fids.data();
    int* border_fids_to = border_fids_from + F_isborder.countOfValues(true);
    //int border_fids_ifrom = 0;
    //int border_fids_ito = 0 + F_isborder.countOfValues(true);

    int processedFacesCount = 0;
    int unprocessedFacesCount = F.rows();

    auto findStartBorderFace = [&](int& start_fid, int& start_eid, int& start_eid_k)
    {
        // v0 - iterate over all faces
        //for (int fid = 0; fid < F.rows(); fid++)
        //{
        //    if ((fid & (boolBitsCount - 1)) == 0) // every 32-th face check at once 32 faces - and skip them if in next 32 faces we will not find border face - speed optimzation - 5% improvement
        //    {
        //        int boolBits_isborder = F_isborder.data()[fid / boolBitsCount];
        //        int boolBits_isProcessed = F_isProcessed.data()[fid / boolBitsCount];
        //        int boolBits = boolBits_isborder & (~boolBits_isProcessed); // F_isborder[fid] && !F_isProcessed[fid]   -  The bitwise NOT operator in C++ is the tilde character ~
        //        if (boolBits == 0)
        //        {
        //            fid += (boolBitsCount - 1);
        //            continue;
        //        }
        //    }
        //    if (F_isborder[fid] && !F_isProcessed[fid])
        //    {
        //        start_fid = fid;
        //        for (int k = 0; k < 3; k++)
        //        {
        //            start_eid = FE(fid, k);
        //            if (E_isborder(start_eid))
        //            {
        //                start_eid_k = k;
        //                return true;
        //                break;
        //            }
        //        }
        //    }
        //}

        // v1 - use firstly added face-borders to get more consistance indexing
        while (border_fids_from < border_fids_to)
        {
            int fid = *border_fids_from;
            border_fids_from++;
            //while (border_fids_ifrom < border_fids_ito)
            //{
            //    int fid = border_fids[border_fids_ifrom];
            //    border_fids_ifrom++;
            if (!F_isProcessed[fid])
            {
                assert(F_isborder[fid]);
                start_fid = fid;
                for (int k = 0; k < 3; k++)
                {
                    I3 fe = FE.row(fid);
                    start_eid = fe(k);
                    if (E_isborder(start_eid))
                    {
                        start_eid_k = k;
                        return true;
                        break;
                    }
                }
            }
        }


        if (unprocessedFacesCount > 0) // if model is solid
        {
            for (int fid = 0; fid < F.rows(); fid++)
            {
                if (!F_isProcessed(fid))
                {
                    start_fid = fid;
                    start_eid_k = 0;
                    start_eid = FE(fid, start_eid_k);
                    return true;
                }
            }
        }
        return false;
    };



    //
    // Create new F
    //
    bool success = true;
    I3s Fnew(F.rows(), F.cols());
    Is V_newIndex = Is::Constant(V.rows(), -1);
    int next_write_Vindex = 0;

    int groupNum = 0;
    while (unprocessedFacesCount > 0)
    {
        // get border face
        int start_fid;
        int start_eid;
        int start_eid_k;
        bool found = findStartBorderFace(start_fid, start_eid, start_eid_k);
        assert(found);
        // DEBUG
        groupNum++;
        if (debug) cout << "group#" << groupNum << "   start_fid=" << start_fid << "   start_eid=" << start_eid << endl;

        int fid = start_fid;
        int eid = start_eid;
        int eid_k = start_eid_k;
        auto save_processedFacesCount = processedFacesCount;
        do
        {
            if (debug) cout << "    fid=" << fid << "   eid=" << eid << endl;

            // add face to new mesh
            //I3 f = F.row(fid);
            I3 f = pF[fid];
            I3 fnew;
            for (int k = 0; k < 3; k++)
            {
                int vid_k = (eid_k + k) % 3;
                int vid = f(vid_k);
                int vidnew = V_newIndex(vid);
                if (vidnew == -1)
                {
                    V_newIndex(vid) = next_write_Vindex;
                    vidnew = next_write_Vindex;
                    next_write_Vindex++;
                }
                fnew(k) = vidnew;
            }
            Fnew.row(processedFacesCount) = fnew;

            // update counters
            unprocessedFacesCount--;
            processedFacesCount++;

            // update border flags
            I3 fe = ((I3*)pFE)[fid];
            int fid_friends[3] = { -1,-1,-1 }; // stores cache of EF.row(k_eid)
            bool fe_isborder[3] = { false, false, false };  // stores cache of E_isborder(k_eid)
            //I3 fid_friends_fe[3] = { I3 (-1,-1,-1),I3(-1,-1,-1),I3(-1,-1,-1) }; // stores ahead of time load of FE.row(fid_friend);
            //if (id == 19 && fid == 6)
            //{
            //    D temp = 0;
            //}
            //I3 tt = TT.row(fid);
            //I3 tti = TTi.row(fid);
            for (int k = 0; k < 3; k++)
            {
                if (eid_k == k) continue;
                //int k_eid = FE(fid, k);
                int k_eid = fe(k);
                //v0 - using EF
                //I2 ef = EF.row(k_eid);
                auto ef = pEF[k_eid];
                int fid0 = ef(0);
                int fid1 = ef(1);
                int fid_friend = (fid != fid0) ? fid0 : fid1;
                //v1 - using TT
                //int fid_friend = tt(k);
                if (fid_friend != -1 && !F_isProcessed[fid_friend])
                {
                    E_isborder(k_eid) = true; //  add edges to search
                    fe_isborder[k] = true; // cache value
                    fid_friends[k] = fid_friend; // cache value
                    //fid_friends_fe[k] = FE.row(fid_friend);
                    F_isborder[fid_friend] = true; // add face to search
                    *border_fids_to = fid_friend;
                    border_fids_to++;
                    //border_fids[border_fids_ito] = fid_friend;
                    //border_fids_ito++;
                }
            }
            #ifdef DEBUG
            if (fid_friends[0] != -1 && fid_friends[0] == fid_friends[1])
            {
                cout << "!wrong    Mesh::OptimizeVertexIndexesForGPU   FE is corruped:  fid_friends[0]!=-1 && fid_friends[0] == fid_friends[1]" << endl;
                //assert(false && "!wrong    Mesh::OptimizeVertexIndexesForGPU   FE is corruped:  fid_friends[0]!=-1 && fid_friends[0] == fid_friends[1]");
            }
            #endif

            F_isProcessed[fid] = true; // remove face from search
            F_isborder[fid] = false;
            E_isborder(eid) = false;

            if (debug)
            {
                extern Model model;
                auto center_face = (V.row(F(fid, 0)) + V.row(F(fid, 1)) + V.row(F(fid, 2))) / 3;
                model.draw.AddLabel(center_face, "" + to_string(groupNum), Color3d((groupNum & 7) == 1, (groupNum & 7) == 2, (groupNum & 7) == 3));
                //model.draw.AddLabel(center_face, "" + to_string(processedFacesCount - save_processedFacesCount), Color3d((groupNum & 7) == 1, (groupNum & 7) == 2, (groupNum & 7) == 3));
                //auto center_edge = (V.row(EV(eid, 0)) + V.row(EV(eid, 1))) / 2;
                //model.draw.AddLabel(center_edge, "" + to_string(processedFacesCount - save_processedFacesCount));
            }

            //findNextBorderFace ( same as method 'findNextBorderFace', but unrolled
            if (unprocessedFacesCount > 0)
            {
                bool foundNextBorderFace = false;
                for (int nextk = 1; nextk < 3; nextk++) // test only 2 next edges starting from current
                {
                    int next_eid_k = (eid_k + nextk) % 3;
                    int next_eid = fe(next_eid_k);
                    //if (E_isborder[next_eid])
                    if (fe_isborder[next_eid_k])
                    {
                        //v1 - using cached value of fid friends
                        int next_fid = fid_friends[next_eid_k];

                        //v0 - using EF
                        #ifdef DEBUG
                        I2 ef = EF.row(next_eid);
                        int fid0 = ef(0);
                        int fid1 = ef(1);
                        int next_fid2 = (fid != fid0) ? fid0 : fid1;
                        if (next_fid != next_fid2)
                        {
                            cout << "next_fid != next_fid2" << endl;
                            assert(false, "next_fid != next_fid2");
                        }
                        #endif

                        if (next_fid != -1)
                        {
                            //assert(!F_isProcessed[next_fid]);
                            fid = next_fid;
                            eid = next_eid;
                            auto fe_next = ((I3*)pFE)[next_fid];
                            // v0
                            //int k = -1;
                            //if (fe_next[0] == eid) 
                            //{
                            //    k = 0;
                            //}
                            //else  if (fe_next[1] == eid)
                            //{
                            //    k = 1;
                            //}
                            //else if (fe_next[2] == eid)
                            //{
                            //    k = 2;
                            //}
                            //v1 - faster since we dont use if statement
                            int k = (fe_next[1] == eid) * 1 + (fe_next[2] == eid) * 2; // some edge must mutch - so we suppose by default that mutch edge index is 0
                            //v2 - sse - works slower - we dont use if statement and there is minimum loads from memory and instructions
                            //auto mm_eid = _mm_set1_epi32(eid);
                            //auto mm_fe_12 = _mm_loadu_si64((__m128i const*)(pFE+ next_fid*3+1)); // load only 2 ints, since we need to compare only FE(next_fid,1) and FE(next_fid,2)
                            //auto mm_cmp = _mm_cmpeq_epi32(mm_fe_12, mm_eid);
                            //int bitsSetted = __popcnt(mm_cmp.m128i_i32[0]) + __popcnt(mm_cmp.m128i_i32[1]) * 2;
                            //int k = bitsSetted / 32;

                            #ifdef DEBUG
                            int k_check = -1 + (fe_next[0] == eid) * 1 + (fe_next[1] == eid) * 2 + (fe_next[2] == eid) * 3;
                            if (k_check == -1)
                            {
                                cout << "next_eid_k not found: k == -1" << endl;
                                //assert(false, "k == -1");
                                continue;
                            }
                            if (k != k_check)
                            {
                                cout << "k != k_check" << endl;
                                continue;
                            }
                            #endif
                            eid_k = k;
                            foundNextBorderFace = true;
                            break;
                        }
                    }
                }
                if (!foundNextBorderFace) break; // break do{}while(unprocessedFacesCount > 0)
            }


        } while (unprocessedFacesCount > 0);
        if (debug)
        {
            cout << "processed " << processedFacesCount - save_processedFacesCount << " faces" << endl;
            //debug = false; // show only first group
        }
    }

    //
    // Create new V
    //
    P3s Vnew(V.rows(), V.cols());
    for (int i = 0; i < V.rows(); i++)
    {
        int newIndex = V_newIndex(i);
        if (newIndex == -1)
        {
            success = false;
            cout << "!   error in Mesh::OptimizeVertexIndexesForGPU():  index " << i << " is not initialized" << endl;
            assert(newIndex != -1 && "index is not initialized");
            continue;
        }
        Vnew.row(newIndex) = V.row(i);
    }

    //
    // Update V and F if we succeed
    //
    if (unprocessedFacesCount > 0)
    {
        success = false;
    }
    if (success)
    {
        //V = Vnew;
        //F = Fnew;
        swap(V, Vnew);
        swap(F, Fnew);
        if (debug) cout << "success !!!" << endl;
    }
    else
    {
        if (debug) cout << "failed !!!" << endl;
    }

    //
    // Update topology after we changed V and F
    //
    if (debug) cout << "=================================" << endl;
}

void Mesh::GetNakedEdgesAndFaces(const I2s& EF, const I3s& FE, const Bs& E_isborder, vector<int>& nakedEdges, vector<int>&  nakedFaces)
{
    for (int eid = 0; eid < EF.rows(); ++eid) // For every border edge 
    {
        if (E_isborder[eid])
        {
            int faceId = EF(eid, 0);
            if (faceId == -1) faceId = EF(eid, 1);
            // add faces that have only border 1 edge (for 2 border edges we need to think...)
            int nakedEdgesCount = 0;
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                auto eid2 = FE(faceId, edgeIndex);
                if (EF(eid2, 0) == -1 || EF(eid2, 1) == -1) nakedEdgesCount++;
            }
            if (nakedEdgesCount == 1)
            {
                // if (faceId == 60){  //DEBUG - limit to only 1 triangle
                nakedEdges.push_back(eid);
                nakedFaces.push_back(faceId);
                //                break;} //DEBUG - limit to only 1 triangle
            }
            //else 
            //{
            //    cout << "!!! warning:  nakedEdgesCount != 1   -  nakedEdgesCount==" << nakedEdgesCount << endl;
            //    //assert(nakedEdgesCount == 1);
            //}
        }
    }
}

void Mesh::CorrectFacesEdgeIndexes()
{
    // Select boundary faces
    vector<int> nakedEdges;
    vector<int> nakedFaces;
    GetNakedEdgesAndFaces(EF, FE, E_isborder, nakedEdges, nakedFaces);
    //cout << "done." << endl;

    bool Fchanged = false;
    for (int nakedId = 0; nakedId < nakedEdges.size(); ++nakedId)
    {
        auto fid = nakedFaces[nakedId];
        auto eid = nakedEdges[nakedId];
        auto ev1 = EV(eid, 0);
        auto ev2 = EV(eid, 1);
        auto fv1 = F.row(fid)(0);
        auto fv2 = F.row(fid)(1);
        auto fv3 = F.row(fid)(2);
        int antiForeverLoopFlag = 0;
        while (!((ev1 == fv1 && ev2 == fv2) || (ev2 == fv1 && ev1 == fv2)) && antiForeverLoopFlag < 3) // shift vertexes until naked edge will be first
        {
            //F.row(faceId) << fv2, fv3, fv1;
            F(fid, 0) = fv2;
            F(fid, 1) = fv3;
            F(fid, 2) = fv1;
            fv1 = F(fid, 0);
            fv2 = F(fid, 1);
            fv3 = F(fid, 2);
            antiForeverLoopFlag++;
            Fchanged = true;
            //cout << "CorrectFacesEdgeIndexes - corrected  fid=" << fid << endl;
        }
    }
    // update mesh topology if we changed something in F
    if (Fchanged)
    {
        //cout << endl << endl << endl << "CorrectFacesEdgeIndexes - corrected " << endl << endl << endl;
        UpdateMeshTopology(false, false, true);
    }
}



vector<VertexToFacesSortedInfo> Mesh::VertexToFacesSorted(int vertexIndex, const P3s& m_V) const
{

    //const vector<int>& faces = VertexToFaces(vertexIndex);
    int facesCount = VF.size(vertexIndex);
    vector<VertexToFacesSortedInfo> res(facesCount);
    P3 vertex = m_V.row(vertexIndex);
    int startFace = 0;

    for (int i = 0; i < facesCount; ++i)
    {
        VertexToFacesSortedInfo& f = res[i];
        f.vertexIndex = vertexIndex;
        int fid = VF(vertexIndex, i);
        f.FaceId = fid;
        for (int k = 0; k < 3; k++)
        {
            if (F(fid, k) == vertexIndex)
            {
                int kLeft = k == 0 ? 2 : k - 1;
                int kRight = k == 2 ? 0 : k + 1;
                f.vertexIndexLeft = F(fid, kLeft);
                f.vertexIndexRight = F(fid, kRight);
                P3 pLeft = m_V.row(f.vertexIndexLeft);
                P3 pRight = m_V.row(f.vertexIndexRight);
                f.LeftSideDirection = pLeft - vertex;
                f.RightSideDirection = pRight - vertex;
                break;
            }
        }
        f.angle = utils::vector::Angle(f.LeftSideDirection, f.RightSideDirection);

        // check if left or right side is border
        f.isLeftSideBorder = false;
        f.isRightSideBorder = false;
        for (int ei = 0; ei < 3; ei++)
        {
            int eid = FE(fid, ei);
            int vi1 = EV(eid, 0);
            int vi2 = EV(eid, 1);
            if ((vi1 == vertexIndex && vi2 == f.vertexIndexLeft) || (vi2 == vertexIndex && vi1 == f.vertexIndexLeft))
            {
                f.LeftSideEdgeId = eid;
                if (E_isborder[eid])
                {
                    f.isLeftSideBorder = true;
                }
            }
            if ((vi1 == vertexIndex && vi2 == f.vertexIndexRight) || (vi2 == vertexIndex && vi1 == f.vertexIndexRight))
            {
                f.RightSideEdgeId = eid;
                if (E_isborder[eid])
                {
                    f.isRightSideBorder = true;
                    startFace = i; // always conter-clockwise
                }
            }
        }
    }

    // Sort faces  - first face with right side border edge
    vector<VertexToFacesSortedInfo> resSorted(facesCount);
    int currentFace = startFace;
    for (int temp = 0; temp < facesCount; ++temp)
    {
        resSorted[temp] = res[currentFace];
        // find next face that connects his right side to left side of current face
        for (int nexti = 0; nexti < res.size(); nexti++)
        {
            if (res[nexti].RightSideEdgeId == res[currentFace].LeftSideEdgeId)
            {
                currentFace = nexti;
                break;
            }
        }
    }

    // Set weights base on angle
    D angleSumm = 0;
    for (int i = 0; i < resSorted.size(); ++i)
    {
        angleSumm += resSorted[i].angle;
    }
    for (int i = 0; i < resSorted.size(); ++i)
    {
        resSorted[i].weightBasedOnAngle = resSorted[i].angle / angleSumm; //  the biggest angle - the heigher weight
    }

    return resSorted;
}
V3 Mesh::EdgeNormalToFace(int eid, int faceId) const
{
    P3 faceCentroid = F_Barycenters.row(faceId);
    return EdgeNormalToPoint(eid, faceCentroid);
}

V3 Mesh::NormalAtPoint(const MeshPoint& point) const
{
    switch (point.Type)
    {
        case MeshPointType::onVertex:
            return V_normals.row(point.vid_eid_fid);
        case MeshPointType::onEdge:
            if (E_isborder(point.vid_eid_fid))
            {
                return F_normals.row(EF(point.vid_eid_fid, 0)); // normal of single connected face
            }
            else
            {
                V3 normal = (F_normals.row(EF(point.vid_eid_fid, 0)) + F_normals.row(EF(point.vid_eid_fid, 1))) / 2; // avarage normal of 2 connected faces
                return normal.normalized();
            }
        case MeshPointType::onFace:
            return F_normals.row(point.vid_eid_fid);
        default:
            return V3(0, 0, 0);// will never enter this line
    }
}

V3 Mesh::EdgeNormalToPoint(int eid, const P3& p) const
{
    P3 p1 = V.row(EV(eid, 0));
    P3 p2 = V.row(EV(eid, 1));
    P3 closestEdgePoint = utils::vector::ClosestPoint_ToVector(p1, p2 - p1, E_Length(eid), p);
    V3 normalOfEdgeToFace = (p - closestEdgePoint);
    normalOfEdgeToFace /= utils::vector::Length(normalOfEdgeToFace);
    return normalOfEdgeToFace;
}
D Mesh::EdgeDistToPoint(int eid, const P3& p) const
{
    P3 p1 = V.row(EV(eid, 0));
    P3 p2 = V.row(EV(eid, 1));
    return utils::vector::DistFromLineToPoint(p1, p2, p);
}

P3 Mesh::EdgeMiddlePoint(int eid) const
{
    P3 p1 = V.row(EV(eid, 0));
    P3 p2 = V.row(EV(eid, 1));
    return (p1 + p2) / 2;
}
P3 Mesh::EdgePointAtPercent(int eid, D percent) const
{
    P3 p1 = V.row(EV(eid, 0));
    P3 p2 = V.row(EV(eid, 1));
    return p1 + (p2 - p1)*percent;
}


V3 Mesh::EdgeDirectionForFace(int eid, int fid) const
{
    int vid0 = EV(eid, 0);
    int vid1 = EV(eid, 1);
    int fvids[3] = { F(fid, 0), F(fid, 1), F(fid, 2) };
    if (fvids[0] == vid1 && fvids[1] == vid0
        || fvids[1] == vid1 && fvids[2] == vid0
        || fvids[2] == vid1 && fvids[0] == vid0)
    {
        swap(vid0, vid1);

    }

    P3 v0 = V.row(vid0);
    P3 v1 = V.row(vid1);
    return v1 - v0;
}


int Mesh::GetClosestVertexIdFromDirectionInsideFace(int fid, const P3& startPoint, const V3& direction) const
{
    int vids[3] = { F(fid, 0), F(fid, 1), F(fid, 2) }; // all vertex ids of face
    // shift start point to avoid wrong angles if startpoint is same as some vertex
    P3 barycenter = F_Barycenters.row(fid);
    P3 startPointFixed = startPoint + (barycenter - startPoint) / 100;
    P3 v0 = V.row(vids[0]);
    P3 v1 = V.row(vids[1]);
    P3 v2 = V.row(vids[2]);
    V3 dirToVertexes[3] = { v0 - startPointFixed, v1 - startPointFixed, v2 - startPointFixed };
    // get best angle - so we can choise best vertex
    D angles[3] = { utils::vector::Angle(dirToVertexes[0], direction), utils::vector::Angle(dirToVertexes[1], direction), utils::vector::Angle(dirToVertexes[2], direction) };
    int minAngleIndex = 0;
    if (angles[1] < angles[minAngleIndex]) minAngleIndex = 1;
    if (angles[2] < angles[minAngleIndex]) minAngleIndex = 2;
    //DEBUG show  vertex angles
    //extern Model model;
    //model.draw.AddLabel(V.row(vids[0]), "      Vertex #0, angle " + to_string(angles[0]));
    //model.draw.AddLabel(V.row(vids[1]), "      Vertex #1,  angle " + to_string(angles[1]));
    //model.draw.AddLabel(V.row(vids[2]), "      Vertex #2,  angle " + to_string(angles[2]));
    return vids[minAngleIndex];
}

int Mesh::GetClosestVertexIdFromPoint(int fid, const P3& p, D& distPow2) const
{
    I3 vids = F.row(fid);
    P3 ps[3] = { V.row(vids[0]), V.row(vids[1]), V.row(vids[2]) }; // all vertex points
    D distsPow2[3] = { (ps[0] - p).normPow2(), (ps[1] - p).normPow2(), (ps[2] - p).normPow2() }; // all distances from point to vertexes
    int minDistIndex = 0;
    if (distsPow2[1] < distsPow2[minDistIndex]) minDistIndex = 1;
    if (distsPow2[2] < distsPow2[minDistIndex]) minDistIndex = 2;
    distPow2 = distsPow2[minDistIndex];
    return vids[minDistIndex];
}

int Mesh::VertexIndexFromVertexId(int fid, int vertexId) const
{
    if (F(fid, 0) == vertexId) return 0;
    if (F(fid, 1) == vertexId) return 1;
    if (F(fid, 2) == vertexId) return 2;
    return -1;
}

bool Mesh::IsDirectionInsideFace(int fid, int direction_start_from_vertexId, const V3& direction, D& angleDiff_PositiveInside_NegativeOutside) const
{
    //TODO use cos instead of angles - speed improvement
    int vertexIndex = VertexIndexFromVertexId(fid, direction_start_from_vertexId);
    int vidRight = F(fid, (vertexIndex + 1) % 3);
    int vidLeft = F(fid, (vertexIndex + 2) % 3);
    V3 leftEdge = V.row(vidLeft) - V.row(direction_start_from_vertexId);
    V3 rightEdge = V.row(vidRight) - V.row(direction_start_from_vertexId);
    V3 directionToFaceCenter = F_Barycenters.row(fid) - V.row(direction_start_from_vertexId);
    D leftEdgeAngle = utils::vector::Angle(direction, leftEdge);
    D rightEdgeAngle = utils::vector::Angle(direction, rightEdge);
    V3 closestEdge = leftEdgeAngle < rightEdgeAngle ? leftEdge : rightEdge;
    D closestAngle = leftEdgeAngle < rightEdgeAngle ? leftEdgeAngle : rightEdgeAngle;
    D biggestAngle = leftEdgeAngle > rightEdgeAngle ? leftEdgeAngle : rightEdgeAngle;

    //DEBUG show  edges and angles
    //extern Model model;
    //model.draw.AddPoint(V.row(vidLeft), Color3d(1, 0, 0));
    //model.draw.AddLabel(V.row(vidLeft), "     Left     with angle " + utils::angle::ToString(leftEdgeAngle), Color3d(1, 0, 0));
    //model.draw.AddPoint(V.row(vidRight), Color3d(1, 0, 0));
    //model.draw.AddLabel(V.row(vidRight), "     Right     with angle " + utils::angle::ToString(rightEdgeAngle), Color3d(1, 0, 0));
    //v1 - works 99%
    //D angleInternal = utils::vector::Angle(directionToFaceCenter, closestEdge);
    //D angleExternal = utils::vector::Angle(directionToFaceCenter, direction);
    //D sign = angleInternal < angleExternal ? -1 : 1;
    //v2 - works 99.99%
    //D angleBetweenEdges = utils::vector::Angle(leftEdge, rightEdge);
    //D angleDiff = leftEdgeAngle + rightEdgeAngle - angleBetweenEdges;
    //D sign = angleDiff < 0.0001 ? 1 : -1;
    //v3
    D angleBetweenEdges = utils::vector::Angle(leftEdge, rightEdge);
    D angleDiff = leftEdgeAngle + rightEdgeAngle - angleBetweenEdges;
    D sign = (leftEdgeAngle < angleBetweenEdges && rightEdgeAngle < angleBetweenEdges) ? 1 : -1;

    angleDiff_PositiveInside_NegativeOutside = sign * closestAngle;
    if (fid == 95576)
    {
        int temp = 0;
    }
    return angleDiff_PositiveInside_NegativeOutside >= 0;
}
bool Mesh::IsDirectionInsideFace(int fid, int direction_start_from_vertexId, const V3& direction) const
{
    D angleDiff;
    return IsDirectionInsideFace(fid, direction_start_from_vertexId, direction, angleDiff);
}
bool Mesh::IsPointInsideFace(int fid, const P3& p) const
{
    cout << endl << endl << "!!! Method   Mesh.IsPointInsideFace(fid, p)   not tested - please test it before use!!!" << endl << endl << endl;;
    D distPow2;
    int vid = GetClosestVertexIdFromPoint(fid, p, distPow2);
    V3 dir = p - V.row(vid).transpose();
    return IsDirectionInsideFace(fid, vid, dir);
}

V3 Mesh::VertexToDirectionOutOfFaces(int vertexIndex) const
{
    int facesCount = VF.size(vertexIndex);
    assert(facesCount != 0);
    V3 v(0, 0, 0);
    int addeddirections = 0;
    for (int i = 0; i < facesCount; ++i)
    {
        int fid = VF(vertexIndex, i);
        for (int ei = 0; ei < 3; ei++)
        {
            int  eid = FE(fid, ei);
            if (E_isborder[eid] && (EV(eid, 0) == vertexIndex || EV(eid, 1) == vertexIndex))
            {
                V3 directionToFace = EdgeNormalToFace(eid, fid);
                //directionToFace.normalized(); no need in normalization since method returns already normalized vector
                v = v + directionToFace;
                addeddirections++;
                //DEBUG
                //extern Model model;
                //P3 edgeMiddlePoint = (V.row(EV(eid, 0)) + V.row(EV(eid, 1))) / 2;
                //model.draw.AddEdge(edgeMiddlePoint, edgeMiddlePoint + directionToFace*avg_edge_length/2, Color3d(1, 0, 0));
                //}
            }
        }
    }
    v = -v / addeddirections;
    v.normalize();
    V3 res = v * avg_edge_length;

    //DEBUG
    //if (addeddirections != 2)
    //{
    //    cout << "addeddirections != 2   for  vertex   " << vertexIndex << endl;
    //}
    //if (vertexIndex == 16)
    //{
    //    cout << "v.norm()   " << v.norm() << endl;
    //}
    //extern Model model;
    //P3 p = V.row(vertexIndex);
    //model.draw.AddEdge(p, p + res, Color3d(1, 0, 0));

    return res;
}

P3 Mesh::VertexToPointOutOfFaces(int vertexIndex, D multiplier) const
{
    P3 res = V.row(vertexIndex).transpose() + VertexToDirectionOutOfFaces(vertexIndex) * multiplier;
    return res;
}






void Mesh::Load(const P3s& _V, const I3s& _F, bool logProgressToConsole,
    bool do_splitNakedTriangleWith2NakedEdge, bool do_correctFacesEdgeIndexes, bool do_optimizeVertexIndexesForGPU)
{
    V = _V;
    F = _F;

    // disable splitting if we not support it
    if (!meshLogicOptions.Mesh.SplitNakedTriangleWith2NakedEdge)
    {
        do_splitNakedTriangleWith2NakedEdge = false;
    }
    // disable optimization if we not support it
    if (!meshLogicOptions.Mesh.OptimizeVertexIndexesForGPU)
    {
        do_optimizeVertexIndexesForGPU = false;
    }



    //logProgressToConsole = true;
    //cout << endl << "Mesh::Load" << endl<< GetMeshIdStr() << endl << endl;

    bool is_removed_manifold_edges = false;

    // create temporal partial topology if one of these method is enabled
    if (do_splitNakedTriangleWith2NakedEdge || do_optimizeVertexIndexesForGPU)
    {
        if (logProgressToConsole) cout << "Detecting topological relations for edges (partial) ..."; // partial means 'calculate_EV_EFi=false'
        Timer timer_UpdateMeshTopology;
        UpdateMeshTopology(true, do_correctFacesEdgeIndexes && !do_optimizeVertexIndexesForGPU, false); // (false means partial) must be called after SplitNakedTriangleWith2NakedEdge and CorrectFacesEdgeIndexes since they change V and F
        is_removed_manifold_edges = true;
        timer_UpdateMeshTopology.stop(elapsed.Load_UpdateMeshTopology);
        if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_UpdateMeshTopology << endl; else cout << endl;
    }

    //auto FEbefore = FE; //DEBUG - created temporaly for debugging
    //auto EFbefore = EF; //DEBUG - created temporaly for debugging

    if (do_splitNakedTriangleWith2NakedEdge)
    {
        if (logProgressToConsole) cout << "Split faces with 2 naked edges ...";
        Timer timer_splitNakedTriangleWith2NakedEdge;
        SplitBorderTriangleWith2BorderEdges(); // will change V,F,EF,FE,E_isborder,F_isborder
        timer_splitNakedTriangleWith2NakedEdge.stop(elapsed.Load_SplitNakedTriangleWith2NakedEdge);
        if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_splitNakedTriangleWith2NakedEdge << endl; else cout << endl;
    }

    if (do_optimizeVertexIndexesForGPU)
    {
        if (logProgressToConsole) cout << "Optimize vertex indexes for GPU draw ...";
        Timer timer_optimizeVertexIndexesForGPU;
        OptimizeVertexIndexesForGPU(do_correctFacesEdgeIndexes); // will change V and F
        timer_optimizeVertexIndexesForGPU.stop(elapsed.Load_OptimizeVertexIndexesForGPU);
        if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_optimizeVertexIndexesForGPU << endl; else cout << endl;
    }

    // if topology is not created or is created partially - then recreate if fully  (this can happend if do_splitNakedTriangleWith2NakedEdge is enabled but do_optimizeVertexIndexesForGPU is disabled)
    if (EV.rows() == 0)
    {
        if (logProgressToConsole) cout << "Detecting topological relations for edges (full) ...";// full means 'calculate_EV_EFi=true'
        Timer timer_UpdateMeshTopology2;
        UpdateMeshTopology(!is_removed_manifold_edges, do_correctFacesEdgeIndexes, true); // (false means full) must be called after SplitNakedTriangleWith2NakedEdge and CorrectFacesEdgeIndexes since they change V and F
        timer_UpdateMeshTopology2.stop(elapsed.Load_UpdateMeshTopology);
        if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_UpdateMeshTopology2 << endl; else cout << endl;
    }





    // obsoleted - will be handled in UpdateMeshTopology->triangle_triangle_adjacency_preprocess()
    //if (do_correctFacesEdgeIndexes)  
    //{
    //    if (logProgressToConsole) cout << "Correcting edge indexes inside faces ...";
    //    Timer timer_CorrectFacesEdgeIndexes;
    //    CorrectFacesEdgeIndexes(); // will change F
    //    timer_CorrectFacesEdgeIndexes.stop(elapsed.Load_CorrectFacesEdgeIndexes);
    //    if (logProgressToConsole) cout << " done in " << timer_CorrectFacesEdgeIndexes << endl;
    //}

    FacesCount = F.rows();
    VertexCount = V.rows();
    EdgesCount = EF.rows();
}

void Mesh::CreateFromVF(const P3s& _V, const I3s& _F, Mesh& m, bool logProgressToConsole, bool do_splitNakedTriangleWith2NakedEdge, bool do_correctFacesEdgeIndexes, bool do_optimizeVertexIndexesForGPU, bool refreshNonCachableData)
{
    m.FileName = "";
    m.Load(_V, _F, logProgressToConsole, do_splitNakedTriangleWith2NakedEdge, do_correctFacesEdgeIndexes, do_optimizeVertexIndexesForGPU);
    if (refreshNonCachableData)
    {
        m.RefreshNonCachableData(logProgressToConsole);
    }
}
void Mesh::SaveToFile_Obj(string filename) const
{
    cout << endl << "method 'Mesh::SaveToFile_Obj' is NOT IMPLEMENTED" << endl << endl;
}
void Mesh::SaveToFile_Q3dMesh(string filename) const
{
    cout << "saving meshid=" << id << " to binary mesh file: " << filename << "... ";
    Timer timer;
    string meshFilename = (utils::file::ExtractExtension(filename) == "q3dmesh")
        ? filename
        : filename + ".q3dmesh";
    igl::serialize(*this, "Q3dMesh", meshFilename.c_str());
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer << endl; else cout << endl;
}

void Mesh::UpdateMeshTopology(bool remove_manifold_edges, bool correctFacesEdgeIndexes, bool calculate_EV_EFi)
{
    //clear TT and TTi because we going to change TTT and they depend on it
    FF.resize(0, 3);
    FFi.resize(0, 3);

    //SSE works slower - so it is commented
    if (utils::cpu::isSupportedSSE4 && options.SSEEnabled && options.sse__triangle_triangle_adjacency_preprocess)
        //utils::mesh::triangle_triangle_adjacency_preprocess_sse4(F, V, TTT, correctFacesEdgeIndexes);
        utils::mesh::triangle_triangle_adjacency_preprocess_sse(F, V, TTT, correctFacesEdgeIndexes); // faster from sse4 because have intergrated sorting inplace
    else
        utils::mesh::triangle_triangle_adjacency_preprocess_fast(F, V, TTT, correctFacesEdgeIndexes);

    int removedFacesCount = 0;
    if (remove_manifold_edges)
    {
        utils::mesh::remove_manifold_edges(F, TTT, removedFacesCount);
    }
    utils::mesh::edge_topology(V, F, TTT, FE, EF, E_isborder, EV, EFi, calculate_EV_EFi); // must be called after SplitNakedTriangleWith2NakedEdge and CorrectFacesEdgeIndexes since they change V and F

    Update_F_isborder();
}

void Mesh::FixIsNanNormals()
{
    int failedToFix;
    int fixed;
    int loop_num = 0;
    do
    {
        failedToFix = 0;
        fixed = 0;
        for (int fid = 0; fid < F.rows(); fid++)
        {
            V3 normal = F_Z.row(fid);
            if (!isnan(normal(0))) continue;

            V3 friendsAvgNormal(0, 0, 0);
            int foundFriendsCount = 0;
            for (int k = 0; k < 3; k++)
            {
                int fidFriend = FF(fid, k);
                if (fidFriend == -1) continue; // for border edge there will be no friend face
                V3 normalFriend = F_Z.row(fidFriend);
                if (isnan(normalFriend(0))) continue; // friend face normal can be also broken
                foundFriendsCount++;
                friendsAvgNormal += normalFriend;
            }
            if (foundFriendsCount == 0)
            {
                failedToFix++;
                continue;
            }
            friendsAvgNormal /= foundFriendsCount;
            friendsAvgNormal.normalize();
            if (isnan(friendsAvgNormal(0)) || isnan(friendsAvgNormal(1)) || isnan(friendsAvgNormal(2)))
            {
                failedToFix++;
                continue;
            }
            V3 x = F_X.row(fid);
            V3 z = friendsAvgNormal;
            V3 y = utils::vector::Cross(z, x);
            F_Y.row(fid) = y;
            F_Z.row(fid) = z;
            fixed++;
        }
        loop_num++;
    } while (failedToFix > 0 && fixed > 0 && loop_num < 30); // do global fix maximum 30 times - protection against forever loop 
}


void Mesh::RefreshNonCachableData(bool logProgressToConsole)
{
    if (logProgressToConsole) cout << "Calculating edge avg size ...";
    Timer timer_Calculating_edge_avg_size;
    utils::mesh::GetEdgesLenghts(V, EV, E_Length, options.SSEEnabled);

    //avg_edge_length = utils::mesh::GetAvgEdgeLength(V, EV, EF, min_edge_length, max_edge_length);
    avg_edge_length = utils::mesh::GetAvgEdgeLength(E_Length, min_edge_length, max_edge_length);
    timer_Calculating_edge_avg_size.stop(elapsed.RefreshNonCachableData_Calculating_edge_avg_size);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Calculating_edge_avg_size << endl; else cout << endl;


    if (logProgressToConsole) cout << "Detecting topological relations for faces ...";
    Timer timer_Detecting_topological_relations_for_faces;
    if (FF.rows() == 0)
    {
        utils::mesh::triangle_triangle_adjacency(F, TTT, FF, FFi);
    }
    timer_Detecting_topological_relations_for_faces.stop(elapsed.RefreshNonCachableData_Detecting_topological_relations_for_faces);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Detecting_topological_relations_for_faces << endl; else cout << endl;


    if (logProgressToConsole) cout << "Detecting normals ...";
    Timer timer_Detecting_normals;
    //igl::local_basis(V, F, F_X, F_Y, F_Z); //v1 original
    //utils::mesh::local_basis(V, F, F_X, F_Y, F_Z);//v2 faster
    int invalidNormalsCount = utils::mesh::local_basis(V, F, FE, E_Length, F_X, F_Y, F_Z);//v3 even more faster
    if (invalidNormalsCount > 0)
    {
        cout << "!!! warning:   Found " << invalidNormalsCount << " invalid normals in " << GetMeshIdStr() << endl;
        FixIsNanNormals();
    }
    utils::mesh::GetEdgesAngles(EF, F_normals, E_Angles, options.SSEEnabled);

    //utils::mesh::area_of_triangle(V, F, F_Areas);
    utils::mesh::area_of_triangle_fast(FE, E_Length, F_Areas, options.SSEEnabled);


    #ifdef USE_EIGEN
    Area = F_Areas.sum();
    #else
    Area = F_Areas.sum<DD>();
    #endif
    utils::mesh::GetVertexNormals(V, F, F_normals, F_Areas, V_normals);
    timer_Detecting_normals.stop(elapsed.RefreshNonCachableData_Detecting_normals);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Detecting_normals << endl; else cout << endl;


    if (logProgressToConsole) cout << "Gathering mesh adjacency  ...";
    Timer timer_Gathering_mesh_adjacency;
    utils::mesh::barycenter_fast(V, F, F_Barycenters);
    is_border_vertex(V_isborder);
    //igl::vertex_triangle_adjacency(V, F, VF, VFi);
    utils::mesh::vertex_triangle_adjacency(V, F, VF, VFi);
    utils::mesh::vertex_edge_adjacency(V, F, VF, VFi, FE, VE);
    timer_Gathering_mesh_adjacency.stop(elapsed.RefreshNonCachableData_Gathering_mesh_adjacency);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_Gathering_mesh_adjacency << endl; else cout << endl;


    if (logProgressToConsole) cout << "computek_fast  ...";
    Timer timer_computek;
    //for(int i = 0; i < 100;i++)
    switch (options.computeK)
    {
        case MeshLogicOptions_Mesh::computeKType::Fast:
            computek_fast(K);
            break;
        case MeshLogicOptions_Mesh::computeKType::Direct:
            computek_direct(K);
            break;
        case MeshLogicOptions_Mesh::computeKType::CosSin:
            computek_cossin(K, Kcos, Ksin);
            break;
        case MeshLogicOptions_Mesh::computeKType::Default:
        default:
            computek_NPolyVectorFieldSolver(K);
            break;
    }
    timer_computek.stop(elapsed.RefreshNonCachableData_computek_fast);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_computek << endl; else cout << endl;


    if (logProgressToConsole) cout << "GetLoops  ...";
    Timer timer_GetLoops;
    Loops.clear();
    MeshLoop::GetLoops(*this, Loops, true);
    timer_GetLoops.stop(elapsed.RefreshNonCachableData_GetLoops);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_GetLoops << endl; else cout << endl;


    if (logProgressToConsole) cout << "GetLoopTopology  ...";
    Timer timer_GetTopology;
    LoopTopology.Update(*this);
    timer_GetTopology.stop(elapsed.RefreshNonCachableData_GetTopology);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_GetTopology << endl; else cout << endl;


    if (logProgressToConsole) cout << "V_Info.Update  ...";
    Timer timer_V_Info_Update;
    LoopsCount = Loops.size();
    PartsCount = Parts.size();
    V_Info.Update(V);
    timer_V_Info_Update.stop(elapsed.RefreshNonCachableData_V_Info_Update);
    if (logProgressToConsole) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_V_Info_Update << endl; else cout << endl;


    //
    // DEBUG test computek speed
    //
    //auto time = utils::time::Now();
    //int debugTestCount = 10;
    //cout << "computek ... (fast)";
    //Ds Kfast;
    //for (int tests = 0; tests < debugTestCount; tests++)
    //{
    //    computek_fast(Kfast);
    //}
    //cout << " done in " << utils::time::ElapsedSecondsStr(time) << " seconds" << endl;
    //cout << "computek ... (NPolyVectorFieldSolver)";
    //Ds KNPoly;
    //for (int tests = 0; tests < debugTestCount; tests++)
    //{
    //    computek_NPolyVectorFieldSolver(KNPoly);
    //}
    //cout << " done in " << utils::time::ElapsedSecondsStr(time) << " seconds" << endl;
    ////TEST if fast method provides same values from normal method
    //for (int i = 0; i < Kfast.size(); i++)
    //{
    //    if (abs(Kfast(i) - KNPoly(i)) > 0.0000001)
    //    {
    //        cout << endl << endl << "!!! ERROR:    computek_fast  has mistake!!!   " << Kfast(i) << "     " << KNPoly(i) << endl << endl << endl;
    //        break;
    //    }
    //}
    //// END DEBUG test

}




void Mesh::ComputeParts_BaseOnLoops()
{
    if (F.rows() == 0)
    {
        Parts.clear();
        return;
    }

    if (Loops.size() == 0)
    {
        Timer timer;
        MeshLoop::GetLoops(*this, Loops, false);
        timer.stop(elapsed.RefreshNonCachableData_GetLoops);
        //cout << "timer  MeshLoop::GetLoops = " << timer << endl;
    }

    if (FF.rows() == 0)
    {
        Timer timer;
        utils::mesh::triangle_triangle_adjacency(F, TTT, FF, FFi);
        timer.stop(elapsed.RefreshNonCachableData_Gathering_mesh_adjacency);
        //cout << "timer triangle_triangle_adjacency = " << timer << endl;
    }

    Timer timer;
    MeshPart::GetParts_BaseOnLoops(*this, Parts);
    timer.stop(elapsed.RefreshNonCachableData_ComputeParts);
}

void Mesh::ComputeParts_BaseOnEdgeToFaceConnections()
{
    Timer timer;
    MeshPart::GetParts_BaseOnEdgeToFaceConnections(*this, Parts);
    timer.stop(elapsed.RefreshNonCachableData_ComputeParts);
}


void Mesh::is_border_vertex(Bs& ret)  // replacement of igl::is_border_vertex - avoided call to 'igl::triangle_triangle_adjacency'
{
    ret = Bs::Constant(V.rows(), false);

    for (int i = 0; i < F.rows(); ++i)
        for (int j = 0; j < 3; ++j)
            if (FF(i, j) == -1)
            {
                ret[F(i, j)] = true;
                ret[F(i, (j + 1) % F.cols())] = true;
            }
}


void Mesh::computek_NPolyVectorFieldSolver(Ds& K)
{
    K.setZero(EdgesCount);
    // For every non-border edge
    for (int eid = 0; eid < EdgesCount; ++eid)
    {
        if (!E_isborder[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            V3 N0 = F_normals.row(fid0);
            V3 N1 = F_normals.row(fid1);

            // find common edge on triangle 0 and 1
            int fid0_vc = -1;
            int fid1_vc = -1;
            for (int i = 0; i < 3; ++i)
            {
                if (FE(fid0, i) == eid)
                    fid0_vc = i;
                if (FE(fid1, i) == eid)
                    fid1_vc = i;
            }
            assert(fid0_vc != -1);
            assert(fid1_vc != -1);

            V3 common_edge = V.row(F(fid0, (fid0_vc + 1) % 3)) - V.row(F(fid0, fid0_vc));
            common_edge.normalize();

            // Map the two triangles in a new space where the common edge is the x axis and the N0 the z axis
            auto o = V.row(F(fid0, fid0_vc)).transpose();
            V3 tmp = -N0.cross(common_edge);
            D33 P;
            //P << common_edge, tmp, N0;
            P.row(0) = common_edge;
            P.row(1) = tmp;
            P.row(2) = N0;
            //      P.transposeInPlace();


            D33 V0;
            V0.row(0) = V.row(F(fid0, 0)) - o;
            V0.row(1) = V.row(F(fid0, 1)) - o;
            V0.row(2) = V.row(F(fid0, 2)) - o;

            V0 = (P*V0.transpose()).transpose();

            assert(V0(0, 2) < 1e-10);
            assert(V0(1, 2) < 1e-10);
            assert(V0(2, 2) < 1e-10);

            D33 V1;
            V1.row(0) = V.row(F(fid1, 0)) - o;
            V1.row(1) = V.row(F(fid1, 1)) - o;
            V1.row(2) = V.row(F(fid1, 2)) - o;
            V1 = (P*V1.transpose()).transpose();

            assert(V1(fid1_vc, 2) < 10e-10);
            assert(V1((fid1_vc + 1) % 3, 2) < 10e-10);

            // compute rotation R such that R * N1 = N0
            // i.e. map both triangles to the same plane
            D alpha = -atan2(V1((fid1_vc + 2) % 3, 2), V1((fid1_vc + 2) % 3, 1));

            D33 R;
            R.row(0) = V3(1, 0, 0);
            R.row(1) = V3(0.f, cos(alpha), -sin(alpha));
            R.row(2) = V3(0.f, sin(alpha), cos(alpha));
            V1 = (R*V1.transpose()).transpose();

            assert(V1(0, 2) < 1e-10);
            assert(V1(1, 2) < 1e-10);
            assert(V1(2, 2) < 1e-10);

            // measure the angle between the reference frames
            // k_ij is the angle between the triangle on the left and the one on the right
            V3 ref0 = V0.row(1) - V0.row(0);
            V3 ref1 = V1.row(1) - V1.row(0);

            ref0.normalize();
            ref1.normalize();

            D ktemp = atan2(ref1(1), ref1(0)) - atan2(ref0(1), ref0(0));

            // just to be sure, rotate ref0 using angle ktemp...
            Matrix2d R2;
            R2 << cos(ktemp), -sin(ktemp), sin(ktemp), cos(ktemp);

            Vector2d ref02(ref0(0), ref0(1));
            RowVector2d tmp1 = R2 * ref02;

            assert(tmp1(0) - ref1(0) < 1e-10);
            assert(tmp1(1) - ref1(1) < 1e-10);

            K[eid] = (D)ktemp;
            //cout << ktemp << endl;
        }
    }
}

void Mesh::computek_fast(Ds& K)
{

    K.setZero(EdgesCount);
    // For every non-border edge
    // skip multithreading since we will do it for surfaces - and doubling call to multithreading will slow down app
    //    extern bool IsOmpEnabled;
//#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int eid = 0; eid < EdgesCount; ++eid)
    {
        if (E_isborder[eid]) continue;

        int fid0 = EF(eid, 0);
        int fid1 = EF(eid, 1);

        V3 N0 = F_normals.row(fid0);
        //V3 N0 = (F_normals.row(fid0) + F_normals.row(fid1)) /2 ;
        //N0.normalize();

        // find common edge on triangle 0 and 1
        int fid0_vc = -1;
        int fid1_vc = -1;
        for (int i = 0; i < 3; ++i)
        {
            if (FE(fid0, i) == eid)
                fid0_vc = i;
            if (FE(fid1, i) == eid)
                fid1_vc = i;
        }

        auto o = V.row(F(fid0, fid0_vc)).transpose();
        V3 common_edge = V.row(F(fid0, (fid0_vc + 1) % 3)) - o;
        //common_edge.normalize();
        common_edge /= E_Length(eid);

        // Map the two triangles in a new space where the common edge is the x axis and the N0 the z axis
        D33 P;
        V3 tmp = -N0.cross(common_edge);
        //P << common_edge, tmp, N0;
        //P.transposeInPlace();
        P.col(0) = common_edge;
        P.col(1) = tmp;
        P.col(2) = N0;

        D33 V0;
        V0.row(0) = V.row(F(fid0, 0));
        V0.row(1) = V.row(F(fid0, 1));
        V0.row(2) = V.row(F(fid0, 2));
        //V0.row(0) -= o;
        //V0.row(1) -= o;
        //V0.row(2) -= o;
        V0(0, 0) -= o(0); V0(0, 1) -= o(1); V0(0, 2) -= o(2);
        V0(1, 0) -= o(0); V0(1, 1) -= o(1); V0(1, 2) -= o(2);
        V0(2, 0) -= o(0); V0(2, 1) -= o(1); V0(2, 2) -= o(2);
        V0 = V0 * P;

        D33 V1;
        V1.row(0) = V.row(F(fid1, 0));
        V1.row(1) = V.row(F(fid1, 1));
        V1.row(2) = V.row(F(fid1, 2));
        //V1.row(0) -= o;
        //V1.row(1) -= o;
        //V1.row(2) -= o;
        V1(0, 0) -= o(0); V1(0, 1) -= o(1); V1(0, 2) -= o(2);
        V1(1, 0) -= o(0); V1(1, 1) -= o(1); V1(1, 2) -= o(2);
        V1(2, 0) -= o(0); V1(2, 1) -= o(1); V1(2, 2) -= o(2);
        V1 = V1 * P;

        // compute rotation R such that R * N1 = N0
        // i.e. map both triangles to the same plane
        //D alpha = -atan2(V1((fid1_vc + 2) % 3, 2), V1((fid1_vc + 2) % 3, 1));
        int index__fid1_vc_plus_2_div_3 = (fid1_vc + 2) % 3;
        D atan2_1 = V1(index__fid1_vc_plus_2_div_3, 2);
        D atan2_2 = V1(index__fid1_vc_plus_2_div_3, 1);
        D alpha = -atan2(atan2_1, atan2_2);

        D alpha_sin = sin(alpha);
        D alpha_cos = cos(alpha);
        D33 R;
        // v1 slow - operator , takes time
        //R << 1, 0, 0,
        //    0, alpha_cos, alpha_sin,
        //    0, -alpha_sin, alpha_cos;
        // v2 fast
        R(0, 0) = 1;  R(0, 1) = 0; R(0, 2) = 0;
        R(1, 0) = 0;  R(1, 1) = alpha_cos; R(1, 2) = alpha_sin;
        R(2, 0) = 0; R(2, 1) = -alpha_sin; R(2, 2) = alpha_cos;
        V1 = V1 * R;

        // measure the angle between the reference frames
        // k_ij is the angle between the triangle on the left and the one on the right
        //V3 ref0 = V0.row(1) - V0.row(0);
        //V3 ref1 = V1.row(1) - V1.row(0);
        //K[eid] = atan2(ref1(1), ref1(0)) - atan2(ref0(1), ref0(0));
        D ref00 = V0(1, 0) - V0(0, 0);
        D ref01 = V0(1, 1) - V0(0, 1);
        D ref10 = V1(1, 0) - V1(0, 0);
        D ref11 = V1(1, 1) - V1(0, 1);
        K[eid] = atan2(ref11, ref10) - atan2(ref01, ref00);

        //cout << K[eid] << endl;
    }
}

void Mesh::computek_direct(Ds& K)
{
    int EdgesCount = EF.rows();
    K.resize(EdgesCount);
    // For every non-border edge
    D* pK = K.data();
    int* pEF = EF.data();
    for (int eid = 0; eid < EdgesCount; ++eid)
    {
        if (E_isborder[eid])
        {
            *pK = 0;
        }
        else
        {
            int fid0 = *pEF; //  always on the left side of the edge - this is guaranteed by 'edge_topology' method
            int fid1 = *(pEF + 1);//  always on the right side of the edge - this is guaranteed by 'edge_topology' method

            // find common edge on triangle 0 and 1
            int fid0_vc = EFi(eid, 0);
            int fid1_vc = EFi(eid, 1);

            // get common edge
            V3 common_edge;
            if (fid0_vc == 0) // use F_X if possible - speed optimization
            {
                common_edge = F_X.row(fid0);
            }
            else if (fid1_vc == 0) // use F_X if possible - speed optimization
            {
                common_edge = -F_X.row(fid1);
            }
            else
            {
                P3 v0 = V.row(F(fid0, fid0_vc));
                P3 v1 = V.row(F(fid0, (fid0_vc + 1) % 3));
                common_edge = v1 - v0;
                //common_edge.normalize();
                common_edge /= E_Length(eid);
            }

            D angle0 = (fid0_vc == 0) ? 0 : utils::vector::AngleInRadians(F_X.row(fid0), common_edge, true);
            D angle1 = (fid1_vc == 0) ? utils::angle::PI : utils::vector::AngleInRadians(F_X.row(fid1), common_edge, true);
            if (fid0_vc == 2) angle0 = -angle0;
            if (fid1_vc == 1) angle1 = -angle1;
            D angleInRadians = (-angle0) + angle1;

            *pK = angleInRadians;
        }
        pK++;
        pEF += 2;
    }
}

void Mesh::computek_cossin(Ds& K, Ds& Kcos, Ds& Ksin)
{
    int EdgesCount = EF.rows();
    K.resize(EdgesCount);
    Kcos.resize(EdgesCount);
    Ksin.resize(EdgesCount);
    // For every non-border edge
    D* pK = K.data();
    D* pKcos = Kcos.data();
    D* pKsin = Ksin.data();
    int* peid = EF.data();
    for (int eid = 0; eid < EdgesCount; ++eid)
    {
        if (E_isborder[eid])
        {
            *pK = 0;
            *pKcos = 0;
            *pKsin = 0;
        }
        else
        {
            int fid0 = *peid;
            int fid1 = *(peid + 1);

            // find common edge on triangle 0 and 1
            int fid0_vc = EFi(eid, 0);
            int fid1_vc = EFi(eid, 1);

            // get common edge
            V3 common_edge;
            if (fid0_vc == 0) // use F_X if possible - speed optimization
            {
                common_edge = F_X.row(fid0);
            }
            else if (fid1_vc == 0) // use F_X if possible - speed optimization
            {
                common_edge = -F_X.row(fid1);
            }
            else
            {
                P3 v0 = V.row(F(fid0, fid0_vc));
                P3 v1 = V.row(F(fid0, (fid0_vc + 1) % 3));
                common_edge = v1 - v0;
                //common_edge.normalize();
                common_edge /= E_Length(eid);
            }

            D cos0 = (fid0_vc == 0) ? 1 : utils::vector::Cos(F_X.row(fid0), common_edge, true);
            D cos1 = (fid1_vc == 0) ? -1 : utils::vector::Cos(F_X.row(fid1), common_edge, true);
            if (fid0_vc == 2) cos0 = -cos0;
            if (fid1_vc == 1) cos1 = -cos1;
            D angle = acos(cos1) - acos(cos0);

            *pK = angle;
            //*pKcos = cos(angle);
            //*pKsin = sin(angle);
        }
        pK++;
        pKcos++;
        pKsin++;
        peid += 2;
    }
}

void Mesh::ExtractMesh(const vector<int>& faceIds, Mesh& newmesh,
    bool splitNakedTriangleWith2NakedEdge, bool correctFacesEdgeIndexes, bool optimizeVertexIndexesForGPU, bool refreshNonCachableData) const
{
    Timer timer_ExtractMesh;

    P3s newV;
    I3s newF;
    //
    // Copy V
    //

    // get diapason of vertex used by surfaces(faces) - this will improve speed a lot for big meshes
    int vMin = V.rows();
    int vMax = 0;
    for (int fid : faceIds)
    {
        int v1 = F(fid, 0);
        int v2 = F(fid, 1);
        int v3 = F(fid, 2);
        if (v1 < vMin) vMin = v1;
        if (v2 < vMin) vMin = v2;
        if (v3 < vMin) vMin = v3;
        if (v1 > vMax) vMax = v1;
        if (v2 > vMax) vMax = v2;
        if (v3 > vMax) vMax = v3;
    }
    int map_size = vMax - vMin + 1;
    if (faceIds.size() == 0) map_size = 0;

    //create mapping from V origin to V new
    Is VoriginToVnewMap = Is::Constant(map_size, 0);
    for (int fid : faceIds)
    {
        VoriginToVnewMap[F(fid, 0) - vMin] = -1; //mark vertex is used
        VoriginToVnewMap[F(fid, 1) - vMin] = -1; //mark vertex is used
        VoriginToVnewMap[F(fid, 2) - vMin] = -1; //mark vertex is used
    }
    int newV_count = 0;
    for (int i = 0; i < map_size; i++)
    {
        if (VoriginToVnewMap[i] != 0) //if vertex is used by this surface
        {
            VoriginToVnewMap[i] = newV_count + 1; // set mapping from origin index to new index, index started from 1
            newV_count++;//increase size of new V by 1
        }
    }
    //allocate size for F
    newV.resize(newV_count, 3);
    //copy V subset
    for (int i = 0; i < map_size; i++)
    {
        if (VoriginToVnewMap[i] != 0) //if vertex is used by this surface
        {
            //newV.row(VoriginToVnewMap[i]) = V.row(i);
            newV(VoriginToVnewMap[i] - 1, 0) = V(vMin + i, 0);
            newV(VoriginToVnewMap[i] - 1, 1) = V(vMin + i, 1);
            newV(VoriginToVnewMap[i] - 1, 2) = V(vMin + i, 2);
        }
    }


    //
    // Copy F
    //
    //allocate size for F
    newF.resize(faceIds.size(), 3);
    //populate size F and V
    for (int i = 0; i < faceIds.size(); i++)
    {
        I3 v = F.row(faceIds[i]);//get original vertexes
        newF(i, 0) = VoriginToVnewMap[v(0) - vMin] - 1; //convert original vertex index to new
        newF(i, 1) = VoriginToVnewMap[v(1) - vMin] - 1; //convert original vertex index to new
        newF(i, 2) = VoriginToVnewMap[v(2) - vMin] - 1; //convert original vertex index to new
    }

    timer_ExtractMesh.stop(elapsed.AddMeshToModel_ExtractMesh);

    //if (newV.rows() < 3)
    //{
    //    assert(newV.rows() >= 3 && "Not enought vertexes to form mesh");
    //    cout << "!!! error:  Mesh.ExtractMesh():  Not enought vertexes to form mesh. vertexes count = " << newV.rows() << endl;
    //}
    //else
    {
        newmesh.Load(newV, newF, false, splitNakedTriangleWith2NakedEdge, correctFacesEdgeIndexes, optimizeVertexIndexesForGPU);
        if (refreshNonCachableData) newmesh.RefreshNonCachableData(false);
    }
}


//int GetRecusiveFaceIds_CallToTTCounts = 0;
int Mesh::GetRecusiveFaceIds_BaseOnTT(vector<int> nextfaceIds, vector<int>& faceIds, Bs& used_global) const
{
    if (used_global.size() == 0)
    {
        used_global.setZero(F.rows());
    }
    Vector<bool> used;
    used.setZero(F.rows());

    faceIds.clear();
    int faceIds_count = 0;
    if (FacesCount == 0) return 0;
    if (nextfaceIds.size() == 0) return 0;

    Vector<int> nextfaceIds_read;
    Vector<int> nextfaceIds_write;
    nextfaceIds_read.resize(FacesCount);
    nextfaceIds_write.resize(FacesCount);
    int* read_start = &nextfaceIds_read[0];
    int* write_start = &nextfaceIds_write[0];
    int* read = read_start;
    int* read_end = read_start;
    int* write = write_start;
    auto swap_read_write = [&]()
    {
        swap(read, write);
        swap(read_start, write_start);
        write = write_start;
        read_end = read;
        read = read_start;
    };

    // write initial faceids
    for (int i = 0; i < nextfaceIds.size(); i++)
    {
        int fid = nextfaceIds[i];
        if (!used_global[fid])//avoid adding duplicates from 'nextfaceIds'
        {
            faceIds_count++;
            used_global[fid] = true;//mark face as used, to skip it next time
            used[fid] = true;//mark face as used, to skip it next time
            *write = fid;//store results
            write++;
        }
    }
    swap_read_write();

    // check if at least some 'nextfaceIds'  was not used, so this call will not be redundant
    if (faceIds_count == 0)  // all faces already used, and so belongs to some parts
    {
        return 0;
    }

    // for current faces - search all faces that contact with
    register const int* pTT = FF.data();
    #if EROW
    int pTTincRow = FF.cols();
    int pTTincCol = 1;
    #else
    int pTTincRow = 1;
    int pTTincCol = TT.rows();
    #endif
    while (read < read_end)
    {
        while (read < read_end)
        {
            int f = *read;
            read++;
            const int* pfi = pTT + f * pTTincRow;
            for (int k = 0; k < 3; k++) // for each edge of face
            {
                // v1 - simple
                //int fFriend = TT(f, k);
                // v2 - direct memory access - fast
                //GetRecusiveFaceIds_CallToTTCounts++;
                int fFriend = *pfi;
                pfi += pTTincCol;
                //assert(fFriendfast== fFriend);
                if (fFriend != -1 && !used_global[fFriend]) //if face has friend on edge && that face is not used yet
                {
                    faceIds_count++;
                    used_global[fFriend] = true;//mark face as used, to skip it next time
                    used[fFriend] = true;//mark face as used, to skip it next time
                    *write = fFriend;//store results
                    write++;
                }
            }
        }
        swap_read_write();
    }

    if (faceIds_count == 0) return 0;
    if (faceIds_count == used.size())
    {
        return faceIds_count; // if we use all face ids from mesh - then return empty faceids to indicate that all faces used
    }

    // return sorted faceids - for this just iterate bool vector
    used.ToIndexes(faceIds, faceIds_count); // provid precaluclated count to speed up conversion
    return faceIds_count;
}


void Mesh::GetRecusiveFaceIds_BaseOnEFE(int fid_start, Bs& used, vector<int>& faceIds) const
{
    faceIds.clear();
    if (used(fid_start)) return;
    faceIds.reserve(FacesCount);

    faceIds.push_back(fid_start);
    used(fid_start) = true;
    int faceIds_from = 0;
    int faceIds_to = faceIds.size();
    while (faceIds_from < faceIds_to)
    {
        for (int i = faceIds_from; i < faceIds_to; i++)
        {
            int fid = faceIds[i];
            for (int k = 0; k < 3; k++) // for each edge of face
            {
                int eid = FE(fid, k);
                if (E_isborder[eid]) continue;
                I2 ef = EF.row(eid);
                for (int n = 0; n < 2; n++) //  for each face of edge
                {
                    int fidFriend = ef(n);
                    if (fidFriend != -1 && fidFriend != fid && !used(fidFriend)) //if face has friend on edge && that face is not used yet
                    {
                        faceIds.push_back(fidFriend);//store results
                        used(fidFriend) = true;//mark face as used, to skip it next time
                    }
                }
            }
        }
        faceIds_from = faceIds_to;
        faceIds_to = faceIds.size();
    }

    // return results
    //cout << "fid_start = " << fid_start << "   faceIds.size() = " << faceIds.size() << endl;
    //utils::stdvector::sort(faceIds);
    faceIds.shrink_to_fit();
}

void Mesh::GetConnectedFaces_BaseOnLoops(vector<vector<int>>& faceIds) const
{
    // to avoid problems with wrong indentification of outher loops - lets avoid outher loops as strart point to find connected faces
    // problem can be even that surface can have mupltiple outher loops
    // so the only way to know - search connected faces for all loops and then remove duplicates
    // sometimes we will do useless duplicate calculations but this solution will be always correct what is much much important from few milliseconds
    // get all outher loops
    //vector<MeshLoop> outherLoops;
    //for (auto loop : Loops)
    //{
    //    if (loop.Type == MeshLoopType::Outher)
    //    {
    //        outherLoops.push_back(loop);
    //    }
    //}

    faceIds.clear();

    // THIS CONDITION IS NOT VALID - because we can have 2 soild meshes, or 1 with loop and others without loops
    // if we have only 1 group that contains all faces in mesh
    //if (Loops.size() == 0 // if we dont have loops, then model is solid, and we have to add here all faces
    //    || Loops.size() == 1  // we have just 1 loop then we have only 1 group that contains all faces in mesh
    //    )
    //{
    //    return; // return nothing - this will be treated as single unique part
    //}

    //check if all faces are connected each-other and so we have single part
    int used_face_count = 0;
    Bs used_faceIds;
    if (Loops.size() > 0)
    {
        vector<int> firstLoop_nextfaceIds;
        vector<int> firstLoop_faceIds_old_inplace;
        vector<int> firstLoop_faceIds;
        Loops[0].GetLoopFaceIds(firstLoop_nextfaceIds); // we have at leat 1 loop - we have checked this just above
        int firstLoop_addedFacesCount = GetRecusiveFaceIds_BaseOnTT(firstLoop_nextfaceIds, firstLoop_faceIds, used_faceIds);
        if (firstLoop_addedFacesCount == FacesCount) return; // all faces are taken - so we have only single and unique part - so in this case we return empty list that indicates that there is no different parts
        used_face_count += firstLoop_addedFacesCount;

        // for every loop - create find recursively-connected faces 
        // no matter what kind of type is loop - we will find for inner and other loops and than remove duplicates - this will help us to avoid problems with wrongly indentified loop types
        faceIds.resize(Loops.size());
        swap(faceIds[0], firstLoop_faceIds);
        // single threading, since our method inside this loop 'GetRecusiveFaceIds_BaseOnTT' is using iteratevely vector 'used_faceIds' to skip redundant seraches and thus improve performance
        for (int i = 1; i < Loops.size(); i++) // skip first loop because we have already done this just above
        {
            // find all faces that contact with outher loop
            vector<int> nextfaceIds;
            nextfaceIds.reserve(Loops[i].edges.size());
            for (auto e : Loops[i].edges)
            {
                nextfaceIds.push_back(e.FaceId);
            }
            used_face_count += GetRecusiveFaceIds_BaseOnTT(nextfaceIds, faceIds[i], used_faceIds); // thread safe list
        }
    }

    // add solid meshes that dont have loops
    while (used_face_count < F.rows())
    {
        int fid = used_faceIds.FindFirstIndex(false);
        if (fid == -1) break;
        vector<int> nextfaceIds;
        nextfaceIds.push_back(fid);
        faceIds.push_back(vector<int>());
        used_face_count += GetRecusiveFaceIds_BaseOnTT(nextfaceIds, faceIds.back(), used_faceIds);
    }



    // remove redundant searches
    for (int i = faceIds.size() - 1; i >= 0; i--) // from last to second
    {
        if (faceIds[i].size() == 0)
        {
            utils::stdvector::remove_at(faceIds, i);
        }
    }
    //cout << endl << "GetRecusiveFaceIds_CallToTTCounts = " << GetRecusiveFaceIds_CallToTTCounts << endl << endl;

    // sort faceIds
    vector<pair<int, int>> map__surface_faceIds__minvalue__to__index;
    for (int i = 0; i < faceIds.size(); i++)
    {
        auto minvalue = faceIds[i][0]; // same as 'utils::stdvector::min_element(faceIds[i]);' because 'faceIds' is sorted
        map__surface_faceIds__minvalue__to__index.push_back({ minvalue , i });
    }
    sort(map__surface_faceIds__minvalue__to__index.begin(), map__surface_faceIds__minvalue__to__index.end());

    //remove duplicates that comes from inner and outher loops
    for (int i = map__surface_faceIds__minvalue__to__index.size() - 1; i >= 1; i--) // from last to second (exluding first)
    {
        auto minvalue1 = map__surface_faceIds__minvalue__to__index[i].first;
        auto minvalue2 = map__surface_faceIds__minvalue__to__index[i - 1].first;
        if (minvalue1 == minvalue2)
        {
            int index = map__surface_faceIds__minvalue__to__index[i].second;
            utils::stdvector::remove_at(faceIds, index);
        }
    }

    // check if there is only one part
    if (faceIds.size() == 1) // we have only single and unique part - so in this case we return empty list that indicates that there is no different parts
    {
        faceIds.clear();
    }
}

void Mesh::GetConnectedFaces_BaseOnEdgeToFaceConnections(vector<vector<int>>& surface_faceIds) const
{
    surface_faceIds.clear();
    Bs used = Bs::Zero(FacesCount); // set values to false
    for (int fid = 0; fid < FacesCount; fid++)
    {
        if (used(fid)) continue;
        surface_faceIds.push_back(vector<int>());
        vector<int>& faceIds = surface_faceIds.back();
        GetRecusiveFaceIds_BaseOnEFE(fid, used, faceIds);
    }
}

int Mesh::CommonEdgeId_VertexVertex(int vid1, int vid2) const
{
    assert(vid1 != vid2); // no sence to test same vertex against itself - this is for sure algorithm mistake
    if (vid1 == vid2) return -1;
    int vesize1 = VE.size(vid1);
    for (int k = 0; k < vesize1; k++)
    {
        int eid1 = VE(vid1, k);
        if (VE.exists(vid2, eid1))
        {
            return eid1;
        }
    }
    return -1;
}

int Mesh::CommonEdgeId_VertexEdge(int vid, int eid) const
{
    if (VE.exists(vid, eid))
    {
        return eid;
    }
    return -1;
}

int Mesh::CommonEdgeId_FaceFace(int fid1, int fid2) const
{
    int eid1 = FE(fid1, 0);
    if (EF(eid1, 0) == fid2 || EF(eid1, 1) == fid2)return eid1;

    int eid2 = FE(fid1, 1);
    if (EF(eid2, 0) == fid2 || EF(eid2, 1) == fid2)return eid2;

    int eid3 = FE(fid1, 2);
    if (EF(eid3, 0) == fid2 || EF(eid3, 1) == fid2)return eid3;

    return -1;
}

int Mesh::CommonEdgeId(const MeshPoint& p1, const MeshPoint& p2) const
{
    if (p1.Type == MeshPointType::onVertex &&p2.Type == MeshPointType::onVertex)
    {
        return CommonEdgeId_VertexVertex(p1.vid_eid_fid, p2.vid_eid_fid);
    }
    if (p1.Type == MeshPointType::onVertex &&p2.Type == MeshPointType::onEdge)
    {
        return CommonEdgeId_VertexEdge(p1.vid_eid_fid, p2.vid_eid_fid);
    }
    if (p2.Type == MeshPointType::onVertex &&p1.Type == MeshPointType::onEdge)
    {
        return CommonEdgeId_VertexEdge(p2.vid_eid_fid, p1.vid_eid_fid);
    }
    if (p1.Type == MeshPointType::onEdge &&p2.Type == MeshPointType::onEdge)
    {
        if (p1.vid_eid_fid == p2.vid_eid_fid) return p1.vid_eid_fid;
    }
    return -1;
}


void Mesh::CommonFaceIds_VertexEdge(int vid, int eid, int& commonFaceId1, int& commonFaceId2) const
{
    commonFaceId1 = -1;
    commonFaceId2 = -1;

    for (int k = 0; k <= 1; k++)
    {
        int fid = EF(eid, k);
        if (fid != -1 && VF.exists(vid, fid))
        {
            if (commonFaceId1 == -1)
            {
                commonFaceId1 = fid;
            }
            else
            {
                commonFaceId2 = fid;
            }
        }
    }
}


int Mesh::CommonFaceIds_EdgeEdge(int eid1, int eid2) const
{
    I2 fids1 = EF.row(eid1);
    I2 fids2 = EF.row(eid2);
    int fid1 = fids1(0);
    if (fid1 != -1 && (fid1 == fids2(0) || fid1 == fids2(1)))
    {
        return fid1;
    }
    int fid2 = fids1(1);
    if (fid2 != -1 && (fid2 == fids2(0) || fid2 == fids2(1)))
    {
        return fid2;
    }
    return -1;
}
void Mesh::CommonFaceIds_VertexVertex(int vid1, int vid2, int& commonFaceId1, int& commonFaceId2) const
{
    commonFaceId1 = -1;
    commonFaceId2 = -1;
    for (int i = 0; i < VF.size(vid1); i++)
    {
        int fid = VF(vid1, i);
        if (VF.exists(vid2, fid))
        {
            if (commonFaceId1 == -1)
            {
                commonFaceId1 = fid;
            }
            else
            {
                commonFaceId2 = fid;
            }
        }
    }
}


bool Mesh::CommonFaceIds(const MeshPoint& p1, const MeshPoint& p2, int& commonFaceId1, int& commonFaceId2) const
{
    commonFaceId1 = -1;
    commonFaceId2 = -1;
    if (p1.Type == MeshPointType::onVertex &&p2.Type == MeshPointType::onVertex)
    {
        CommonFaceIds_VertexVertex(p1.vid_eid_fid, p2.vid_eid_fid, commonFaceId1, commonFaceId2);
    }
    if (p1.Type == MeshPointType::onVertex &&p2.Type == MeshPointType::onEdge)
    {
        CommonFaceIds_VertexEdge(p1.vid_eid_fid, p2.vid_eid_fid, commonFaceId1, commonFaceId2);
    }
    if (p2.Type == MeshPointType::onVertex &&p1.Type == MeshPointType::onEdge)
    {
        CommonFaceIds_VertexEdge(p2.vid_eid_fid, p1.vid_eid_fid, commonFaceId1, commonFaceId2);
    }
    if (p1.Type == MeshPointType::onEdge &&p2.Type == MeshPointType::onEdge)
    {
        commonFaceId1 = CommonFaceIds_EdgeEdge(p1.vid_eid_fid, p2.vid_eid_fid);
    }

    return commonFaceId1 != -1;
}


bool Mesh::CommonFaceIds(const MeshPoint& p1, const MeshPoint& p2, const MeshPoint& p3, int& commonFaceId1, int& commonFaceId2) const
{
    commonFaceId1 = -1;
    commonFaceId2 = -1;

    int common12[2];
    CommonFaceIds(p1, p2, common12[0], common12[1]);

    int common23[2];
    CommonFaceIds(p2, p3, common23[0], common23[1]);

    for (int k = 0; k <= 1; k++)
    {
        int fid = common12[k];
        if (fid != -1 && (fid == common23[0] || fid == common23[1]))
        {
            if (commonFaceId1 == -1)
            {
                commonFaceId1 = fid;
            }
            else
            {
                commonFaceId2 = fid;
            }
        }
    }

    return commonFaceId1 != -1;
}


bool Mesh::GetVertexFacesAngleSegments(int vertexIndex, vector<VertexToFacesSortedInfo>& faces, vector<D>& facesAngleSegments, D& angleSumm360, bool& isRotationCircular, int& rotationCount) const
{
    const D rotationDegree = 1;
    //
    // Get faces and sort them conter-clockwise
    //
    faces = VertexToFacesSorted(vertexIndex);
    if (faces.size() == 0)
    {
        return false;
    }
    for (int i = 0; i < faces.size(); ++i)
    {
        if (isnan(faces[i].angle))
        {
            cout << "!!!  Mesh::GetVertexFacesAngleSegments - angle is nan!!!: meshid=" << id << "      vertexIndex=" << vertexIndex << endl;
            return false;
        }
    }

    //DEBUG show faces border edges
    //for (int i = 0; i < faces.size(); ++i)
    //{
    //    VertexToFacesSortedInfo& f = faces[i];
    //    if (f.isLeftSideBorder)
    //    {
    //        draw.AddLabel(V.row(f.vertexIndexLeft), "border");
    //    }
    //    if (f.isRightSideBorder)
    //    {
    //        draw.AddLabel(V.row(f.vertexIndexRight), "border");
    //    }
    //}
    //DEBUG show faces indexes
    //draw.ReserveLabels(faces.size());
    //for (int i = 0; i < faces.size(); ++i)
    //{
    //    int fid = faces[i].Id;
    //    draw.AddLabel(F_Barycenters.row(fid), to_string(i));
    //    //draw.AddLabel(F_Barycenters.row(fid), to_string(faces[i].weightBasedOnAngle));
    //}

    //
    // Define rotation
    //
    angleSumm360 = 0;
    facesAngleSegments.resize(faces.size() + 1);
    facesAngleSegments[0];
    for (int i = 0; i < faces.size(); ++i)
    {
        angleSumm360 += faces[i].angle;
        facesAngleSegments[i + 1] = angleSumm360;
    }
    if (angleSumm360 > 360) angleSumm360 = 360;
    isRotationCircular = !faces[0].isRightSideBorder;
    rotationCount = static_cast<int>(round(angleSumm360 / rotationDegree)) + 1; // "+1" to include last degree
    if (rotationCount > 360) rotationCount = 360;
    //DEBUG - show faces and their angles
    //for (int i = 0; i < faces.size(); i++)
    //{
    //    int fid = faces[i].Id;
    //    P3 p = F_Barycenters.row(fid);
    //    draw.AddLabel(p, to_string(i), Color4d(0,0,0,1)); // weight of field vector
    //    draw.AddLabel((vertex + p) / 2, utils::angle::ToString(faces[i].angle));
    //}
    return true;
}

P3 Mesh::ProjectPoint(const P3& p, bool precise) const
{
    int vid = utils::mesh::GetClosestPointIndex(V, p);
    P3 v = V.row(vid);
    V3 vNormal = V_normals.row(vid);
    P3 projectedPointToVertexPlane = utils::point::ProjectToPlane(p, v, vNormal);
    if (!precise)
    {
        return projectedPointToVertexPlane;
    }

    //v1 get best fid from all faces connected to vertex 
    //int bestfid = -1;
    //D bestdist = 0;
    //for (int i = 0; i < VF.size(vid); i++) // for all faces that connects to this vertex find most closest face
    //{
    //    int fid = VF(vid, i);
    //    P3 v0 = V.row(F(fid, 0));
    //    D dist = utils::point::DistToPlane(p, v0, F_normals.row(fid));
    //    if (dist < bestdist || bestfid == -1)
    //    {
    //        bestfid = fid;
    //        bestdist = dist;
    //    }
    //}

    //v2 get best fid from all faces connected to vertex 
    vector<VertexToFacesSortedInfo> faces;
    vector<D> facesAngleSegments;
    D angleSumm360;
    bool isRotationCircular;
    int rotationCount;
    auto faceIndexFromRI = [&facesAngleSegments, &faces](D ri)
    {
        const D rotationDegree = 1;
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

    if (!GetVertexFacesAngleSegments(vid, faces, facesAngleSegments, angleSumm360, isRotationCircular, rotationCount))
    {
        return projectedPointToVertexPlane;
    }

    V3 dir = projectedPointToVertexPlane - v;
    dir.normalize();
    V3 dir_translated_toface0 = utils::vector::Translate(dir, vNormal, F_normals.row(faces[0].FaceId), true);
    D ri = utils::vector::AngleFull(faces[0].RightSideDirection, dir_translated_toface0, F_normals.row(faces[0].FaceId));
    if (ri < 0) ri += angleSumm360;
    int bestfid = faces[faceIndexFromRI(ri)].FaceId;
    //V3 dir_translated_toface_best = utils::vector::Translate(dir, vNormal, F_normals.row(faces[0].FaceId));


    P3 projectedPoint = utils::point::ProjectToPlane(p, V.row(F(bestfid, 0)), F_normals.row(bestfid));


    //DEBUG - show projected point
    //cout << "ProjectPoint:  vid=" << vid << "  fid=" << bestfid << endl;
    //draw.AddPoint(V.row(vid), Color3d(0, 0, 1), "vid");
    //draw.AddPoint(F_Barycenters.row(bestfid), Color3d(1, 0, 0), "fid");
    //draw.AddPoint(projectedPoint, Color3d(0, 1, 0), "projected");
    return projectedPoint;
}

string Mesh::GetMeshIdStr() const
{
    string res = "meshid=" + to_string(id);
    if (!empty(Name))
    {
        res += " meshname=" + Name;
    }
    return res;
}

/*
---How to achieve properly draw of crease---

one vertex can be splited to few vertexes
v(i) => v(i) + v(N+1) + v(N+2)
so vertexes v(i) + v(N+1) + v(N+2) will have same positions and colors, but different normals:
vn(i) => vn(i) + vn(N+1) + vn(N+2)
since we have splited vertexes to few, the faces that are using start vertex v(i) should be updated to point to the new vertexes

since we can't dynamicaly update data in mesh, we have to store our data in class MeshSurface - in such a way we will be able to change dynamicaly data and show them in viewport
additional data to store:
1) additional vertexes
2) additional normals
3) updated faces

so we have to define a new class
class MeshNormalCorrections
{
    P3s V; // additional vertexes
    V3s V_normals; // additional normals (same amount as V)
    vector<pair<int, I3>>; // updated faces: pair<fid, vids>
}

and update class ViewportData_Mesh to keep data
class ViewportData_Mesh
{
    const P3s& V;
    const I3s& F;
    const V3s& F_normals;
    const V3s& V_normals;
    const MeshNormalCorrections& Corrections;
}

while sending data to GPU we will use this addtional data to properly add new vertexes and theirs normals to achieve properly draw of crease
*/
void Mesh::GetMeshNormalCorrections(MeshNormalCorrections& corrections, D creaseAngle) const
{
    corrections.Clear();
    corrections.isInited = true;

    //
    // check what vertexes can be splited
    //
    Vector<int> vertexInvolvedTimes;
    Vector<bool> creaseEdges;
    int creaseAnglesCount = 0;
    auto markEdgeCrease = [&](int eid)
    {
        if (vertexInvolvedTimes.size() == 0)
        {
            vertexInvolvedTimes.setZero(V.rows());
            creaseEdges.setZero(EV.rows());
        }
        auto ev = EV.row(eid);
        int vid0 = ev(0);
        int vid1 = ev(1);
        vertexInvolvedTimes[vid0]++;
        vertexInvolvedTimes[vid1]++;
        creaseEdges[eid] = true;
        creaseAnglesCount++;
    };
    for (int eid = 0; eid < E_Angles.size(); eid++)
    {
        D angle = E_Angles[eid];
        if (isnan(angle)) continue;
        if (angle > creaseAngle)
        {
            markEdgeCrease(eid);
        }
    }
    if (creaseAnglesCount == 0) return;


    //
    // count for how many pieces vertex should be splited
    //
    struct VertexChanges
    {
        int vid_original;
        int vid_new;
        VertexChanges(int _vid_original, int _vid_new)
            : vid_original(_vid_original), vid_new(_vid_new)
        {
        }
    };
    struct VertexChangesEx : VertexChanges
    {
        vector<int> fids;
        VertexChangesEx(int _vid_original, int _vid_new)
            : VertexChanges(_vid_original, _vid_new)
        {
        }
    };
    vector<VertexChangesEx> vchanges;
    vchanges.reserve(creaseAnglesCount);
    vector<vector<VertexChanges>> fchanges;
    fchanges.resize(F.rows());
    Vector<bool> fchangesMade;
    fchangesMade.setConstant(F.rows(), false);
    int next_vid_new = V.rows() - 1;
    for (int vid = 0; vid < vertexInvolvedTimes.size(); vid++)
    {
        // check if we have enought crease to split vertex
        int times = vertexInvolvedTimes[vid];
        if (times == 0) continue;

        // get faces for vertex
        vector<VertexToFacesSortedInfo> faces = VertexToFacesSorted(vid);
        int facesCount = faces.size();
        if (facesCount == 0) continue;

        // check if we have enought crease to split vertex (this time also counting borders)
        int eidStart = faces[0].RightSideEdgeId;
        if (E_isborder[eidStart])
        {
            //creaseEdges[eidStart] = true;
            //vertexInvolvedTimes[vid]++;
            times++;
        }
        if (times < 2) continue; // needed at least: (2 creases)  or  (1 crease and 1 border)

        // set eidStart to border or crease
        int faceIndexStart = 0;
        for (int i = 0; i < facesCount; i++)
        {
            int eidL = faces[i].LeftSideEdgeId;
            int eidR = faces[i].RightSideEdgeId;
            if (creaseEdges[eidR] || E_isborder[eidR])
            {
                faceIndexStart = i;
                break;
            }
        }

        // get changes
        int groupsCount = 0;
        for (int i = 0; i < facesCount; i++)
        {
            int index = faceIndexStart + i;
            if (index >= facesCount) index = index - facesCount;
            int eidR = faces[index].RightSideEdgeId;
            if (creaseEdges[eidR] || E_isborder[eidR])
            {
                groupsCount++;
            }
        }
        if (groupsCount < 2) continue;

        int groupid = 0;
        for (int i = 0; i < facesCount; i++)
        {
            int index = faceIndexStart + i;
            if (index >= facesCount) index = index - facesCount;
            int eidR = faces[index].RightSideEdgeId;
            if (creaseEdges[eidR] || E_isborder[eidR])
            {
                groupid++;
                next_vid_new++;
                vchanges.push_back({ vid,next_vid_new });
            }

            int fid = faces[index].FaceId;
            vchanges.back().fids.push_back(fid);
            fchanges[fid].push_back({ vid,next_vid_new });
            fchangesMade[fid] = true;
        }
    }
    int addedVertexes = vchanges.size();
    if (addedVertexes == 0) return;

    //
    // create result V, V_normals
    //
    corrections.V.resize(addedVertexes, 3);
    corrections.V_normals.resize(addedVertexes, 3);
    corrections.V_correcteIndexes.resize(addedVertexes);
    for (int i = 0; i < vchanges.size(); i++)
    {
        auto& change = vchanges[i];
        V3 normal(0, 0, 0);
        for (int fid : change.fids)
        {
            normal += F_normals.row(fid);
        }
        normal.normalize();
        corrections.V.row(i) = V.row(change.vid_original);
        corrections.V_normals.row(i) = normal;
        corrections.V_correcteIndexes(i) = change.vid_original;
    }


    //
    // create result F
    //
    Vector<int> fids_changed;
    fchangesMade.ToIndexes(fids_changed);
    corrections.Fcorrections.resize(fids_changed.size(), 4);
    for (int i = 0; i < fids_changed.size(); i++)
    {
        int fid = fids_changed[i];
        I3 f = F.row(fid);
        for (int k = 0; k < 3; k++)
        {
            for (auto& c : fchanges[fid])
            {
                if (f(k) == c.vid_original)
                {
                    c.vid_original = -1;
                    f(k) = c.vid_new;
                }
            }
        }
        corrections.Fcorrections.row(i) = I4(fid, f(0), f(1), f(2));
    }
}
