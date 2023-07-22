#pragma once
#include "ViewerDrawObjects.h"

class Mesh;
class PolygonMesh;

//class ObjFileReader
//{
//public:
//    struct Group
//    {
//        string name;
//        int count; // count of ngons in 'CompactVectorVector<int> F'
//        int indexStartInF; // start index in 'CompactVectorVector<int> F'
//    };
//    string filename;  // obj filename
//    P3s V;     // Mesh vertexes (each row has 3 vertexes coordinates)
//    CompactVectorVector<int> ngons; // Mesh faces (each row has 3-n vertexes indexes from Matrix V)
//    vector<Group> groups;
//
//    PolygonMeshObjFile();
//    bool ReadFromFile(string filename);
//    II SizeOF() const;
//};


class PolygonMesh_VertexesMergeInfo
{
public:
    const PolygonMesh& polygonMesh;
    const P3s& V;
    Is V_IndexesInFile_GlobalOrLocal; // indexes can be global (negative values)  or local (positive values), all starts from 1, no zero    -  [global] common points for each PolygonMesh (topology Points and Connections) (negative values)      [local] points unique to this mesh (positive values) 
    int VertexesCountGlobal; // how many vertex indexes are global (with negative values)
    int VertexesCountLocal;// how many vertex indexes are local (with positive values)   

    PolygonMesh_VertexesMergeInfo(const PolygonMesh& polygonMesh);
    II SizeOF() const;

    void MarkVertexAsGlobal(int vid, int globalIndex); // make some vertex from V as global, so we can properly create obj file with already merged vertexes. after all global vertexes are marked - call method  'UpdateGlobalLocalIndexesCount'
    void UpdateGlobalLocalIndexesCount(); // updates 'VertexesCountGlobal', 'VertexesCountLocal', 'V_IndexesInFile_GlobalOrLocal' after method 'MarkVertexGlobal' was called
};


class PolygonMesh
{
public:
    int groupid;    
    string groupname; 
    P3s V;     // Mesh vertexes (each row has 3 vertexes coordinates)
    CompactVectorVector<int> ngons; // Mesh ngons (each row has 3..N vertexes indexes from Matrix V)
    PolygonMesh_VertexesMergeInfo VMergeInfo;

    PolygonMesh(int groupid, string groupname = "");
    II SizeOF() const;
    int GetVertexesCountLocal() const;
    void Draw(ViewerDrawObjects& draw, bool reserveSpace = true) const;
    void ConvertNgonsToF(I3s& F);
    void ConvertToMesh(Mesh& mesh);
    int MergeVertexes(D tol);
    void Heal(bool IsOmpEnabled);
private:
};


class PolygonMeshes
{
public:
    vector<const PolygonMesh*> meshes;
    void Add(vector<const PolygonMesh*> meshes);
    int GetVertexesCount() const;
    int GetNgonsCount() const;
    int GetGroupsCount() const;
    int GetCommonVertexesCount() const;
    void GetCommonVertexes(P3s& V_common) const;
    int Draw_GetAddedEdgesCount() const;
    void Draw(ViewerDrawObjects& draw) const;
private:
};
