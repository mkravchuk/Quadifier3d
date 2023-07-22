#pragma once
#include "MeshConstrains.h"
#include "MeshStreams.h"


class Mesh;
class ViewerDrawObjects;

const uint8_t NOMATCH = 8;

struct MeshSolverResult
{
    const Mesh& mesh;
    MeshLogicOptions_SolverNrosy::MeshSolverType SolverType;
    bool IsFieldSorted; // populated in NormalizeNrosyField method
    vector<V3s> Field; // populated in NormalizeNrosyField method, size is same as N
    bool IsFieldSymetric;  // symetric field can be stored with half of fields, others half will be negative: vectors 2,3 is same as -0,-1 - this will help to reduce size of 'Field' by 2-times
    MatchABBAs Field_match_ab_ba; // matrix for stream lines - calculated from field
    Ds Singularities;  //Result - SingularityIndexPerVertex    

    MeshSolverResult(const Mesh& _mesh) //MeshConstrains& _constrains
        : mesh(_mesh), SolverType(MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Complex2x), IsFieldSorted(false), Field(vector<V3s>()), IsFieldSymetric(false)
    {

    }

    void Clear()
    {
        IsFieldSorted = false;
        Field.clear();
        IsFieldSymetric = false;
        Singularities.resize(0);
    }

    int GetSingularityCounts(bool includeSing3, bool includeSing5)
    {
        int count = 0;
        for (int i = 0; i < Singularities.size(); ++i)
        {
            if (includeSing3 && Singularities(i) < -0.001) count++;
            if (includeSing5 && Singularities(i) > 0.001) count++;
        }
        return count;
    }

    bool isEmpty()
    {
        return Field.size() == 0;
    }

    V3 getField(int ni, int fid) const // only vaid for N=4
    {
        assert(ni < 4);
        if (ni >= 2 && IsFieldSymetric)
        {
            return -Field[ni % 2].row(fid);
        }
        else
        {
            return Field[ni].row(fid);
        }
    }

    void Init_match_ab_ba(ViewerDrawObjects& draw,  int N);
private:
    void Init_match_ab_ba__fix_near_singularities(ViewerDrawObjects& draw);
};

class MeshSolverNrosy
{
public:
    const Mesh& mesh;
    ViewerDrawObjects& draw;
    MeshConstrains Constrains; // constrains for each surface
    MeshSolverResult Result;

    MeshSolverNrosy(const Mesh& _mesh, ViewerDrawObjects& _draw);
    void Solve(bool logMessages);
    void Clear();

    II SizeOF() const;
private:
    static void SolveOnce(const Mesh& mesh, ViewerDrawObjects& draw, const MeshConstrains& Constrains,
        const int N,                     //N-rosy dimension
        MeshLogicOptions_SolverNrosy::MeshSolverType solverType,
        const D soft,             //Set the ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
        bool WeightIsRelativeToEdgeLength, bool logMessages, bool ignore_x, bool takeDirectionCorrected, MeshSolverResult& Result
    );

    // Converts a representative vector per face in the full set of vectors that describe an N-RoSy field (based on Result.SolverType properties Result.Field and Result.IsFieldSorted are populated)
    // can clear FieldRaw - becasue data will be moved to Result
    static void NormalizeNrosyField(const Mesh& mesh, ViewerDrawObjects& draw, const MeshConstrains& Constrains, const int N, vector<V3s>& FieldRaw, bool FieldRaw_isSorted, MeshSolverResult& Result);
};