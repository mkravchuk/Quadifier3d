#pragma once
#include "Mesh.h"
#include "PointersCollection.h"



class MeshSimple
{
public:
    P3s V;
    I3s F;
    V3s F_normals;
    V3s V_normals;
    MeshVInfo V_Info;

    void InitNormalsAndV();
    II SizeOF() const;
};

class MeshSimpleCollection : public PointersCollection<MeshSimple>
{
public:
    II SizeOF() const;
};

//class MeshSimpleCollection
//{
//public:
//    vector<MeshSimple*> Objects;
//
//    MeshSimple* Add();
//    void Add(MeshSimple* sm);
//    bool Remove(MeshSimple* obj);
//    void Clear();
//    II SizeOF() const;
//    ~MeshSimpleCollection();
//};


// target_triangles__count__or__threshold < 0 --- threshold simplification - if threshold very small like 1e-10 will be lossless simplification
// target_triangles__count__or__threshold [0..1] - percent simplification - 'target triangles = (triangles count)*percent'
// target_triangles__count__or__threshold > 1 - count simplification - 'target triangles = target_triangles__count__or__threshold'
bool SimplifyMesh(ViewerDrawObjects& draw, string filename, const Mesh& mesh, P3s& V, I3s& F,
    D target_triangles__threshold__precent__count, MeshLogicOptions_MeshSimplification::AlgorithmType algorithmType,  bool verbose = false);

