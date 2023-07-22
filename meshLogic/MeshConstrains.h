#pragma once
#include "FaceConstrain.h"

class Mesh;
class MeshLoop;
class ViewerDrawObjects;

struct MeshConstrains_Optoins
{
    bool Cons_CorrectInAngles;
};


class MeshConstrains
{
protected:
    void Init_AllEdgesWithCorrectionInCorners();
    void Init_QuadOnly(); 
    static void GetConstrains(const vector<Mesh>& meshes, MeshLogicOptions_Constrains::MeshConstrainsType type, vector<MeshConstrains>& constrains); // get contrains for all meshed in multitreading
    void CorrectConstrainesInAngles(const MeshLoop& loop, vector<V3>& constrains, vector<D>& correctionAngles) const;
    void GetBorderConstrains(const MeshLoop& loop, vector<V3>& constrains, vector<V3>& constrainsCorrected, vector<D>& correctionAngles) const;
    void AddContrainsInSharpEdges(int nexId);
public:
    const Mesh& mesh; // for which mesh we generate contrains
    ViewerDrawObjects& draw;
    MeshLogicOptions_Constrains::MeshConstrainsType Type;
    vector<FaceConstrain> Constrains; // constrains for this mesh

    MeshConstrains(const Mesh& mesh, ViewerDrawObjects& draw);
    void Init(MeshLogicOptions_Constrains::MeshConstrainsType type); // initialize Constrains and Type. This functionality is implemented separatelly to allow dynamically change contrains for mesh
    void Clear();// clears Type and Constrains
    II SizeOF() const;
};