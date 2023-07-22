#include "stdafx.h"
#include "Mesh.h"
#include "MeshSimplifier.h"
//#include <igl/readOBJ.h>
//#include <boundingmesh.h>
#include "FastQuadricMeshSimplification.h"
#define options meshLogicOptions.Mesh


void MeshSimple::InitNormalsAndV()
{
    //if (logProgressToConsole) cout << "Detecting normals ...";
    V3s F_X;//Face normalized X axis (from method igl::local_basis - B1) - normalized edge direction: (V.row(F(i,1)) - V.row(F(i,0))).normalized();
    V3s F_Y;//Face normalized Y axis (from method igl::local_basis - B2)
    Ds F_Areas;// Face D area (area of triangle face *2 or are of quad mesh*1)
    utils::mesh::local_basis(V, F, F_X, F_Y, F_normals);
    utils::mesh::area_of_triangle(V, F, F_Areas);
    utils::mesh::GetVertexNormals(V, F, F_normals, F_Areas, V_normals);
    V_Info.Update(V);
}

II MeshSimple::SizeOF() const
{
    II r = sizeof(MeshSimple);
    r += V.size() * sizeof(D);
    r += F.size() * sizeof(int);
    r += V_normals.size() * sizeof(D);
    r += F_normals.size() * sizeof(D);
    return r;
}


II MeshSimpleCollection::SizeOF() const
{
    II r = sizeof(MeshSimpleCollection);
    for (auto obj : Objects)
    {
        if (obj)
        {
            r += obj->SizeOF();
        }
    }
    r += Objects.size() * sizeof(void*);
    return r;
}




// target_triangles__count__or__threshold < 0 --- threshold simplification - if threshold very small like 1e-10 will be lossless simplification
// target_triangles__count__or__threshold [0..1] - percent simplification - 'target triangles = (triangles count)*percent'
// target_triangles__count__or__threshold > 1 - count simplification - 'target triangles = target_triangles__count__or__threshold'
bool SimplifyMesh(ViewerDrawObjects& draw, string filename, const Mesh& mesh, P3s& V, I3s& F,
    D target_triangles__threshold__precent__count, MeshLogicOptions_MeshSimplification::AlgorithmType algorithmType, bool verbose)
{
    // alg1
    FastQuadricSimplifier fastquadric(draw);
    // alg2 50 times slower and has many issues
    //boundingmesh::BMesh boundmesh;
    //std::shared_ptr<boundingmesh::BMesh> boundmesh_result;

    // currently only one algorithm available - second is completely bad - 50 times slower and has many issues
    //algorithmType = MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric;

    //
    // Load mesh from file
    //
    //TODO convert from mesh insted of using temporally file
    clock_t start = clock();
    int startVertCount = 0;
    int startFaceCount = 0;
    if (algorithmType == MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric)
    {
        fastquadric.load(mesh.V, mesh.F, mesh.V_isborder, mesh.F_normals, mesh.EV, mesh.FE);
        startVertCount = fastquadric.vertices.size();
        startFaceCount = fastquadric.triangles.size();
    }
    else
    {
        //boundmesh.loadObj(filename);
        //startVertCount = boundmesh.nVertices();
        //startFaceCount = boundmesh.nTriangles();
    }
    if (verbose) printf("\n\n\n");
    if (verbose) printf("Input: %d vertices, %d triangles (target %f)\n", startVertCount, startFaceCount, target_triangles__threshold__precent__count);
    float timetaken_read = static_cast<float>(clock() - start) / CLOCKS_PER_SEC;

    //
    // Check for empty mesh
    //
    if ((startVertCount < 3) || (startFaceCount < 3))
    {
        if (verbose) printf("MeshSimplifier::Simplify   cannot simplify - mesh is empty.\n");
        return false;
    }


    //
    // Simplify
    //
    start = clock();
    // lossless simplification for values less from zero
    if (target_triangles__threshold__precent__count <= 0)
    {
        D target__threshold = abs(target_triangles__threshold__precent__count);
        if (target__threshold < DBL_EPSILON) target__threshold = DBL_EPSILON;
        if (algorithmType == MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric)
        {
            fastquadric.simplify_mesh_lossless(target__threshold, verbose);

            /*for (int i = 0; i < 100; i++)
            {
                fastquadric.load(mesh.V, mesh.F, mesh.V_isborder, mesh.F_normals, mesh.EV, mesh.FE);
                fastquadric.simplify_mesh_lossless(target__threshold, verbose);
            }*/
        }
        else
        {
            //boundingmesh::Decimator decimator;
            //decimator.setDirection(boundingmesh::Inward);
            //decimator.setMaximumError(target__threshold);
            //decimator.setMesh(boundmesh);
            //boundmesh_result = decimator.compute();
        }
    }
    // count simplification
    else 
    {
        int target__count = (target_triangles__threshold__precent__count > 1)
            ? static_cast<int>(target_triangles__threshold__precent__count) // count simplification for values more from 1
            : static_cast<int>(startFaceCount * target_triangles__threshold__precent__count); // percent simplification for values in [0..1]
        if (target__count >= startFaceCount)
        {
            if (verbose) printf("MeshSimplifier::Simplify   cannot simplify - 'target_triangles_count==%d' is higher from mesh size.\n", target__count);
            return false;
        }
        if (algorithmType == MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric)
        {
            D agressiveness = 7.0;
            fastquadric.simplify_mesh_count(target__count, agressiveness, verbose);
        }
        else
        {
            //boundingmesh::Decimator decimator;
            //decimator.setDirection(boundingmesh::Inward);
            //decimator.setTargetVertices(target__count);
            //decimator.setMesh(boundmesh);
            //boundmesh_result = decimator.compute();
        }
    }
    float timetaken_simplify = ((float)(clock() - start)) / CLOCKS_PER_SEC;


    //
    // Check result
    //
    int endVertCount = 0;
    int endFaceCount = 0;
    if (algorithmType == MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric)
    {
        endVertCount = fastquadric.vertices.size();
        endFaceCount = fastquadric.triangles.size();
    }
    else
    {
        //endVertCount = boundmesh_result->nVertices();
        //endFaceCount = boundmesh_result->nTriangles();
    }
    if (endFaceCount >= startFaceCount)
    {
        if (verbose) printf("MeshSimplifier::Simplify   Unable to reduce mesh.\n");
        return false;
    }


    //
    // Return results
    //
    start = clock();
    string output_filename = filename + ".simplified.obj";
    if (algorithmType == MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric)
    {
        fastquadric.write(V, F); 
    }
    else
    {
        //boundmesh_result->writeObj(output_filename);
        //boundmesh_result.reset();
        //igl::readOBJ(output_filename, V, F);
    }
    
    float timetaken_output = ((float)(clock() - start)) / CLOCKS_PER_SEC;

    //
    // Log
    //
    if (verbose) printf("Output: %d vertices, %d triangles - reduction %f\n",
        endVertCount, endFaceCount, static_cast<float>(endFaceCount) / static_cast<float>(startFaceCount));
    if (verbose) printf("Time taken:  read %.4f,   simplify %.4f,   output %.4f        TOTAL  %.4f\n",
        timetaken_read, timetaken_simplify, timetaken_output, timetaken_read + timetaken_simplify + timetaken_output);
    if (verbose) printf("\n\n\n");

    return true;
}




