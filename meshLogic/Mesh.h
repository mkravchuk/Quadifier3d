#pragma once


#include "MeshLoop.h"  // incude full header instead of forward declaration just to avoid additional incudes when using class mesh in other classes
#include "MeshTopologyLoop.h" // incude full header instead of forward declaration just to avoid additional incudes when using class mesh in other classes
#include "MeshPart.h"  // incude full header instead of forward declaration just to avoid additional incudes when using class mesh in other classes
#include "CompactVectorVector.h"
class Mesh;

enum MeshPointType
{
    onVertex, onEdge, onFace
};
struct MeshPoint
{
    MeshPointType Type; // point can be on vertex or on edge or on face (this point can be created by intersecting two streams)
    int vid_eid_fid; // vertex id, or edge id, or face id
    P3 point;
    MeshPoint()
        :Type(MeshPointType::onVertex), vid_eid_fid(0), point(P3(0, 0, 0))
    {
    }
    MeshPoint(MeshPointType _Type, int _vid_eid_fid, const P3& _point)
        : Type(_Type), vid_eid_fid(_vid_eid_fid), point(_point)
    {
    }
    string vid_eid_fid_toString() const;
    bool isOnBorder(const Mesh& mesh) const;
    V3 operator-(const MeshPoint& p2) const
    {
        return this->point - p2.point;
    }
    P3 operator+(const MeshPoint& p2) const
    {
        return this->point + p2.point;
    }

};

struct MeshVInfo
{
    P3 Vmin; // minimum point from all vertexes
    P3 Vmax; // maximum point from all vertexes
    P3 Vcenter; // center of all points
    P3 VcenterProjected; // center of all points projected to closer point
    P3 VcenterProjectedIndside; // center of all points projected to closer point + moved a little inside mesh. expensive, so before you need this variable call Init_VcenterProjectedIndside()
    void Update(const P3s& V);
    void Init_VcenterProjectedIndside(const Mesh& mesh);// center of all points projected to closer point + moved a little inside mesh
    void InitMesh_VcenterProjectedIndside_IfRequiredByDraw(const Mesh& mesh);// center of all points projected to closer point + moved a little inside mesh
    MeshVInfo()
    {
        vcenterProjectedIndside_isset = false;
    }
private:
    bool vcenterProjectedIndside_isset;
};

struct VertexToFacesSortedInfo
{
    int FaceId;              // face id
    D angle;    //  angle at vertex between left and rights sides (in degrees)
    D weightBasedOnAngle;  //  the biggest angle - the heigher weight
    int vertexIndex;
    int vertexIndexLeft; // left vertex - conter-clockwise in face normal space
    int vertexIndexRight; // right vertex - conter-clockwise in face normal space
    V3 LeftSideDirection; // vector from vertex to left vertex
    V3 RightSideDirection; // vector from vertex to right vertex
    int LeftSideEdgeId; 
    int RightSideEdgeId;
    bool isLeftSideBorder;
    bool isRightSideBorder;
};

class MeshNormalCorrections
{
public:
    bool isInited;
    P3s V; // additional vertexes
    V3s V_normals; // additional normals (same amount as V)
    Is V_correcteIndexes; //indexes of original vertexes
    I4s Fcorrections; // updated faces: vector<pair<fid, vids>>
    MeshNormalCorrections()
        : isInited(false)
    {
    }
    void Clear();
};


class Mesh
{
    static atomic_int nextMeshID;
public:
    Mesh();
    int id;// unique id
    int originalMeshId;// original mesh id from which this mesh was created by cutting mesh or other operations
    II SizeOF() const;
     
    //
    // Vertex 
    //  
    P3s V;     // Mesh verticies (each row has 3 vertexes coordinates)
    Bs V_isborder;     // Determine vertices on open boundary of a (manifold) mesh with triangle
    CompactVectorVector<int> VF; // vertex to face (from method igl::vertex_triangle_adjacency) - not sorted! (can be sorted if needed in future, for sort use method Mesh::VertexToFacesSorted)
    CompactVectorVector<int> VFi; //  vertex to face edge index - use in junction with VF (from method igl::vertex_triangle_adjacency, for sort use method Mesh::VertexToFacesSorted)  - not sorted! (can be sorted if needed in future)
    CompactVectorVector<int> VE; // vertex to edge  - not sorted! (can be sorted if needed in future, for sort use method Mesh::VertexToFacesSorted)
    V3s V_normals;  // Vertex normals (one normal per vertex)
    MeshVInfo V_Info;

    //
    // Face
    //      
    I3s F;      // Mesh faces (each row has 3 vertexes indexes from Matrix V)
    Bs F_isborder; // if face has border edge, so is border face
    #define F_normals F_Z  // Face normals (one normal per face) (same as F_Z)
    Ds F_Areas;// Face D area (area of triangle face *2 or are of quad mesh*1)
    P3s F_Barycenters; // Face baricenters (one centroid point per face)
    V3s F_X;//Face normalized X axis (from method igl::local_basis - B1) - normalized edge direction: (V.row(F(i,1)) - V.row(F(i,0))).normalized();
    V3s F_Y;//Face normalized Y axis (from method igl::local_basis - B2)
    V3s F_Z;//Face normalized Z axis - should be same as normal - same as F_normals (from method igl::local_basis - B3)  (same as F_normals)
    I3s FE; // Relation Face to Edge (each face has 3 edge indexes)
    // Face Topology (igl::triangle_triangle_adjacency)
    I4s TTT; //intermidiate data for calculating FF, FFi and other(each row has 4 integer values : vertexIndexStart, vertexIndexEnd, face, edgeIndexInFace)
    I3s FF; // Relation Face to Face via edge index(0,1,2) same as igl's TT (each row has 3 face-indexes that share same edge index) so for face1 and edgeindex 0 we can know what is friend, and no one if value is '-1'
    I3s FFi; // addition to 'FF' - where FF shows opposite face, and FFi his opposite edge index. same as igl's TTi  (each row has 3 edge-indexes of opposite face that we know from FF)


    //
    // Edge 
    //
    I2s EV; // Relation Edge to Vertex (each edge has 2 vertex indexes from Matrix V)
    I2s EF; // Relation Edge to Face (each edge has 2 faces indexes, second will be -1 if edge has only 1 face)
    I2s EFi; // addition to 'EF' - Relation Edge to Face edge index (each edge has 2 faces edge indexes, second will be -1 if edge has only 1 face)
    Bs E_isborder; // Relation Edge to "flag if edge is a border (belongs to only 1 face)"
    Ds E_Length; // length of edge
    Ds E_Angles; // angles between faces in edge (in radians)

    // Frame topology (computek)
    Ds K; // Angle between two reference frames in radians (Per Edge information)
    Ds Kcos; // Angle between two reference frames in cos (Per Edge information)
    Ds Ksin; // Angle between two reference frames in sin (Per Edge information)

    vector<MeshLoop> Loops; // loops that belongs to this mesh.  Outher loop comes first - list is sorted by type and then by dist to P3(0,0,0)
    vector<MeshPart> Parts;//separated part of the mesh - some mesh composite from surfaces, so each surface will be separated part
    MeshTopologyLoop LoopTopology;

    DD Area; // area of mesh - summ of all triangles
    D avg_edge_length;
    D min_edge_length;
    D max_edge_length;
    int EdgesCount;
    int FacesCount;
    int VertexCount;
    int LoopsCount;
    int PartsCount;

    // Load file
    string Name;
    string FileName;
    static void CreateFromVF(const P3s& V, const I3s& F, Mesh& m, bool logProgressToConsole, bool do_splitNakedTriangleWith2NakedEdge, bool do_correctFacesEdgeIndexes, bool do_optimizeVertexIndexesForGPU, bool refreshNonCachableData);
    void SaveToFile_Obj(string filename) const;
    void SaveToFile_Q3dMesh(string filename) const;
    void ComputeParts_BaseOnLoops();
    void ComputeParts_BaseOnEdgeToFaceConnections();
    void RefreshNonCachableData(bool logProgressToConsole);

    vector<VertexToFacesSortedInfo> VertexToFacesSorted(int vertexIndex, const P3s& m_V) const;
    vector<VertexToFacesSortedInfo> VertexToFacesSorted(int vertexIndex) const
    {
        return VertexToFacesSorted(vertexIndex, V);
    }
    V3 VertexToDirectionOutOfFaces(int vertexIndex) const;
    P3 VertexToPointOutOfFaces(int vertexIndex, D multiplier) const;
    V3 EdgeNormalToPoint(int eid, const P3& p) const;
    V3 EdgeNormalToFace(int eid, int faceId) const;
    V3 NormalAtPoint(const MeshPoint& point) const;
    P3 EdgeMiddlePoint(int eid) const;
    P3 EdgePointAtPercent(int eid, D percent) const;
    V3 EdgeDirectionForFace(int eid, int fid) const;
    D EdgeDistToPoint(int eid, const P3& p) const;
    int GetClosestVertexIdFromDirectionInsideFace(int fid, const P3& startPoint, const V3& direction) const;
    int GetClosestVertexIdFromPoint(int fid, const P3& p, D& distPow2) const;
    int VertexIndexFromVertexId(int fid, int vertexId) const;
    bool IsDirectionInsideFace(int fid, int direction_start_from_vertexId, const V3& directionStartedFromVertexOnFacePlane, D& angleDiff_PositiveInside_NegativeOutside) const;
    bool IsDirectionInsideFace(int fid, int direction_start_from_vertexId, const V3& directionStartedFromVertexOnFacePlane) const;
    bool IsPointInsideFace(int fid, const P3& p) const;
    void ExtractMesh(const vector<int>& faceIds, Mesh& newmesh, bool splitNakedTriangleWith2NakedEdge, bool correctFacesEdgeIndexes, bool optimizeVertexIndexesForGPU, bool refreshNonCachableData) const;
    int GetRecusiveFaceIds_BaseOnTT(vector<int> nextfaceIds, vector<int>& faceIds, Bs& used_faceids) const; // single thread only! returned 'faceIds' is sorted
    void GetRecusiveFaceIds_BaseOnEFE(int fid, Bs& used, vector<int>& faceIds) const;
    void GetConnectedFaces_BaseOnLoops(vector<vector<int>>& faceIds) const;
    void GetConnectedFaces_BaseOnEdgeToFaceConnections(vector<vector<int>>& surface_faceIds) const;
    int CommonEdgeId_VertexVertex(int vid1, int vid2) const;
    int CommonEdgeId_VertexEdge(int vid, int eid) const;
    int CommonEdgeId_FaceFace(int fid1, int fid2) const;
    int CommonEdgeId(const MeshPoint& p1, const MeshPoint& p2) const;
    void CommonFaceIds_VertexVertex(int vid1, int vid2, int& commonFaceId1, int& commonFaceId2) const;
    void CommonFaceIds_VertexEdge(int vid, int eid, int& commonFaceId1, int& commonFaceId2) const;
    int CommonFaceIds_EdgeEdge(int eid1, int eid2) const;
    bool CommonFaceIds(const MeshPoint& p1, const MeshPoint& p2, int& commonFaceId1, int& commonFaceId2) const;
    bool CommonFaceIds(const MeshPoint& p1, const MeshPoint& p2, const MeshPoint& p3, int& commonFaceId1, int& commonFaceId2) const;
    bool GetVertexFacesAngleSegments(int vertexIndex, vector<VertexToFacesSortedInfo>& faces, vector<D>& facesAngleSegments, D& angleSumm360, bool& isRotationCircular, int& rotationCount) const;
    P3 ProjectPoint(const P3& p, bool precise = true) const;
    string GetMeshIdStr() const;
    void GetMeshNormalCorrections(MeshNormalCorrections& corrections, D creaseAngle) const;
private:
    void Update_F_isborder();
    void FixIsNanNormals();
    void Load(const P3s& _V, const I3s& _F, bool logProgressToConsole,  bool do_splitNakedTriangleWith2NakedEdge, bool do_correctFacesEdgeIndexes, bool do_optimizeVertexIndexesForGPU);
    void SplitBorderTriangleWith2BorderEdges(); // will change V,F,EF,FE,E_isborder,F_isborder
    void OptimizeVertexIndexesForGPU_old(bool updateMeshTopology_correctFacesEdgeIndexes);
    void OptimizeVertexIndexesForGPU(bool updateMeshTopology_correctFacesEdgeIndexes);
    void CorrectFacesEdgeIndexes();
    void computek_NPolyVectorFieldSolver(Ds& K);
    void computek_fast(Ds& K);
    void computek_direct(Ds& K);
    void computek_cossin(Ds& K, Ds& Kcos, Ds& Ksin);
    void is_border_vertex(Bs& v_isborder); // replacement of igl::is_border_vertex - avoided call to 'igl::triangle_triangle_adjacency'
    static void GetNakedEdgesAndFaces(const I2s& EF, const I3s& FE, const Bs& E_isborder, vector<int>& nakedEdges, vector<int>&  nakedFaces);
    void UpdateMeshTopology(bool remove_manifold_edges, bool correctFacesEdgeIndexes, bool calculate_EV_EFi);
};