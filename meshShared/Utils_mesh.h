#pragma once
#include "CompactVectorVector.h"


namespace utils
{
    namespace mesh
    {
        __forceinline P3 GetFaceCentroid(int faceId, const I3s& F, const P3s& V);

        void GetEdgesLenghts(const P3s& V, const I2s& EV, Ds& E_Length, bool useSSE);
        void GetEdgesAngles(const I2s& EF, const V3s& F_normals, Ds& E_Angles, bool useSSE);

        // same as igl::avg_edge_length - but 30x times faster since uses already calculated edges lengths
        D GetAvgEdgeLength(const Ds& E_Length, D& min_edge_length, D& max_edge_length);
        // same as igl::avg_edge_length but 10x times faster
        D GetAvgEdgeLength(const P3s& V, const I2s& EV, const I2s& EF, D& min_edge_length, D& max_edge_length);
        // same as igl::avg_edge_length but 3x times faster
        D GetAvgEdgeLength(const P3s& V, const I3s& F);

        void GetDistancesPow2(const P3s& points, const P3& p, std::vector<D>& distances);
        void GetDistances(const P3s& points, const P3& p, std::vector<D>& distances);
        int GetClosestPointIndex(const P3s& points, const P3& p);
        void GetClosestPointIndexes(const P3s& points, const P3& p,  std::vector<int>& closestPointsIndexes, int count);

        //same as igl::per_vertex_normals but 3x time faster
        void GetVertexNormals(const P3s& V, const I3s& F, const V3s& F_normals, const Ds& F_Areas,
            V3s& V_normals);

        //same as igl::triangle_triangle_adjacency_preprocess but faster and using Matrix instead of vector that make even dealocation faster
        // each row of TTT will have { v1 v2 fi eilocal}
        void triangle_triangle_adjacency_preprocess_simple_slow(const I3s& F,
            I4s& TTT);
        void triangle_triangle_adjacency_preprocess_fast(I3s& F, const P3s& V,
            I4s& TTT, bool correctFacesEdgeIndexes);
        void triangle_triangle_adjacency_preprocess_sse(I3s& F, const P3s& V,
            I4s& TTT, bool correctFacesEdgeIndexes);
        void triangle_triangle_adjacency_preprocess_sse4(I3s& F, const P3s& V,
            I4s& TTT, bool correctFacesEdgeIndexes);

        //same as igl::is_edge_manifold but using TTT as MatrixX4i which is faster
        int get_manifold_edges_count(const I3s& F,
            const I4s& TTT);

        // remove faces with manifold edges to avoid any issues
        void remove_manifold_edges(I3s& F, I4s& TTT, int& removedFacesCount);

        //same as igl::edge_topology but using TTT as MatrixX4i which is faster
        void edge_topology(const P3s& V, const I3s& F, const I4s& TTT, I3s& FE, I2s& EF, Bs& E_isborder,
            I2s& EV, I2s& EFi, bool calculate_EV_EFi);

        //same as igl::triangle_triangle_adjacency but using TTT as MatrixX4i which is faster
        void triangle_triangle_adjacency(const I3s& F, const I4s& TTT,
            I3s& FF, I3s& FFi);

        // same as igl::barycenter
        void barycenter_fast(const P3s& V, const I3s& F, P3s& BC);

        // same as igl::vertex_triangle_adjacency but uses CompactVectorVector which is faster and consume less memory
        void vertex_triangle_adjacency(const P3s& V, const I3s& F, CompactVectorVector<int>& VF, CompactVectorVector<int>& VFi);

        // new method - calculates relation: vertex to edges
        void vertex_edge_adjacency(const P3s& V, const I3s& F, const CompactVectorVector<int>& VF, const CompactVectorVector<int>& VFi, const I3s& FE, CompactVectorVector<int>& VE);

        // same as igl::doublearea but faster 
        void area_of_quad(const P3s& V, const I3s& F, Ds& F_Areas);
        // same as igl::doublearea but faster 
        void area_of_triangle(const P3s& V, const I3s& F, Ds& F_Areas);

        // same as igl::doublearea but much faster 
        void area_of_triangle_fast(const I3s& FE, const Ds& E_Length, Ds& F_Areas, bool useSSE);


        // same as igl::local_basis but faster (returns invalidNormalsCount) 
        int local_basis(const P3s& V, const I3s& F, V3s& F_X, V3s& F_Y, V3s& F_Z);
        // same as igl::local_basis but faster (returns invalidNormalsCount) 
        int local_basis(const P3s& V, const I3s& F, const I3s& FE, const Ds& E_Length, V3s& F_X, V3s& F_Y, V3s& F_Z);

        // same as igl::compute_frame_field_bisectors but faster (returns invalidNormalsCount) 
        void compute_frame_field_bisectors(
            const V3s& F_X, const V3s& F_Y,
            const V3s& FF1, const V3s& FF2,
            V3s& BIS1, V3s& BIS2);
        // same as igl::compute_frame_field_bisectors but faster (returns invalidNormalsCount) 
        void compute_frame_field_bisectors(const std::vector<bool>& checkForFaces,
            const V3s& F_X, const V3s& F_Y,
            const V3s& FF1, const V3s& FF2,
            V3s& BIS1, V3s& BIS2);

        void MakeMeshFlat(const I3s& F, const V3s& F_normals, P3s& V);
    }

}