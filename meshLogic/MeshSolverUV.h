#pragma once
#include "MeshStreams.h"
#include "MeshConstrainsUV.h"

class Mesh;
class ViewerDrawObjects;


struct MeshSolverUVResult
{
    const Mesh& mesh;
    MeshLogicOptions_SolverUV::AlgorithmType SolverType;
    vector<UV> Field; 
    MeshSolverUVResult(const Mesh& _mesh)
        : mesh(_mesh), SolverType(MeshLogicOptions_SolverUV::AlgorithmType::lscm), Field(vector<UV>())
    {
    }

    void Clear()
    {
        Field.clear();
    }

    bool isEmpty()
    {
        return Field.size() == 0;
    }
};

class MeshSolverUV
{
public:
    const Mesh& mesh;
    ViewerDrawObjects& draw;
    MeshConstrainsUV Constrains; 
    MeshSolverUVResult Result;

    MeshSolverUV(const Mesh& _mesh, ViewerDrawObjects& _draw);
    void Solve(bool logMessages);
    void Clear();
    II SizeOF() const;
private:
};