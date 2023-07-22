#include "stdafx.h"
#include "MeshSolverNrosy.h"
#include "ViewerDrawObjects.h"
#include "Mesh.h"
#include "NPolyVectorFieldSolver_Complex2x.h"
#include "NPolyVectorFieldSolver_Polar2x.h"
#if NPolyVectorFieldSolverGeneral_SUPPORTED
#include "NPolyVectorFieldSolver.h"
#include "NPolyVectorFieldSolverGeneral.h"
#endif
#if Comiso_SUPPORTED
#include "../iglcomiso/frame_field.h"
#include "../iglcomiso/nrosy.h"
#endif
#include <igl/frame_to_cross_field.h>
#include "FindCrossFieldSingularities.h"
#include <igl/polyvector_field_matchings.h>
#include <igl/polyvector_field_singularities_from_matchings.h>
#include "MeshStreams.h"
#if MeshLinearSolver_SUPPORTED
#include "MeshLinearSolver.h"
#endif
#include <igl/polyvector_field_matchings.h>
#include <igl/segment_segment_intersect.h>
#include <igl/rotation_matrix_from_directions.h>
#include "../../../libs/vector_class/vectori128.h"
#include "../../../libs/vector_class/vectorf128.h"

MeshLogicOptions_SolverNrosy& options = meshLogicOptions.Solver;






void polyvector_field_matching_fast(
    const V3& ua1, const V3& ua2,
    const V3& ub1, const V3& ub2,
    const V3& e,
    int* mab,
    int* mba)
{
    // make sure the matching preserve ccw order of the vectors across the edge
    // 1) order vectors in a, ccw  e.g. (0,1,2,3)_a not ccw --> (0,3,2,1)_a ccw
    // 2) order vectors in b, ccw  e.g. (0,1,2,3)_b not ccw --> (0,2,1,3)_b ccw
    // 3) the vectors in b that match the ordered vectors in a (in this case  (0,3,2,1)_a ) must be a circular shift of the ccw ordered vectors in b  - so we need to explicitely check only these matchings to find the best ccw one, there are N of them
    const int N = 4;
    int order_b[4] = { 0,1,2,3 };

    Vector4d all_scores;
    // since we have  only 2 unique direction and 2 others are symetric - we can calculate dot product only twice, and other 2 will negative of first two values
    D ua_dot_e[N] = { ua1.dot(e), ua2.dot(e), 0, 0 }; ua_dot_e[2] = -ua_dot_e[0]; ua_dot_e[3] = -ua_dot_e[1];
    D ub_dot_e[N] = { ub1.dot(e), ub2.dot(e), 0, 0 }; ub_dot_e[2] = -ub_dot_e[0]; ub_dot_e[3] = -ub_dot_e[1];
    //checking all possible circshifts of order_b as matches for order_a
    for (int s = 0; s < N; ++s)
    {
        D current_score = 0;
        for (int i = 0; i < N; ++i)
        {
            current_score += fabs(ua_dot_e[i] - ub_dot_e[order_b[i]]);
        }
        all_scores[s] = current_score;
        // do a circshift on order_b to check the next preserving matching
        int temp = order_b[0];
        order_b[0] = order_b[1];
        order_b[1] = order_b[2];
        order_b[2] = order_b[3];
        order_b[3] = temp;
    }

    int best_i;
    all_scores.minCoeff(&best_i);

    // best_matching_for_sorted_a is the matching for the sorted vectors in a
    // get the matching for the initial (unsorted) vectors    
    for (int i = 0; i < N; ++i)
    {
        mab[i] = best_i;
        mba[best_i] = i;
        best_i++;
        if (best_i == 4) best_i = 0;
    }
}

//__declspec(noinline)
__forceinline void polyvector_field_matching_fast2(
    const V3& a1, const V3& a2,
    const V3& b1, const V3& b2,
    const V3& e,
    MatchABBA* mabba)
{
    // make sure the matching preserve ccw order of the vectors across the edge
    // 1) order vectors in a, ccw  e.g. (0,1,2,3)_a not ccw --> (0,3,2,1)_a ccw
    // 2) order vectors in b, ccw  e.g. (0,1,2,3)_b not ccw --> (0,2,1,3)_b ccw
    // 3) the vectors in b that match the ordered vectors in a (in this case  (0,3,2,1)_a ) must be a circular shift of the ccw ordered vectors in b  - so we need to explicitely check only these matchings to find the best ccw one, there are N of them
    //const int N = 4;
    //int order_b[4] = { 0,1,2,3 };

    //Vector4d all_scores;
    //// since we have  only 2 unique direction and 2 others are symetric - we can calculate dot product only twice, and other 2 will negative of first two values
    //D ae[N] = { a1.dot(e), a2.dot(e), 0, 0 }; ae[2] = -ae[0]; ae[3] = -ae[1];
    //D be[N] = { b1.dot(e), b2.dot(e), 0, 0 }; be[2] = -be[0]; be[3] = -be[1];
    ////checking all possible circshifts of order_b as matches for order_a
    //for (int s = 0; s < N; ++s)
    //{
    //    D current_score = 0;
    //    for (int i = 0; i < N; ++i)
    //    {
    //        current_score += fabs(ae[i] - be[order_b[i]]);
    //    }

    //    all_scores[s] = current_score;
    //    // do a circshift on order_b to check the next preserving matching
    //    int temp = order_b[0];
    //    order_b[0] = order_b[1];
    //    order_b[1] = order_b[2];
    //    order_b[2] = order_b[3];
    //    order_b[3] = temp;
    //}

    //int best_i;
    //all_scores.minCoeff(&best_i);


    //D ae1 = a1.dot(e);
    //D ae2 = a2.dot(e);
    //D be1 = b1.dot(e);
    //D be2 = b2.dot(e);
    D ae1 = utils::vector::Dot(a1, e);
    D ae2 = utils::vector::Dot(a2, e);
    D be1 = utils::vector::Dot(b1, e);
    D be2 = utils::vector::Dot(b2, e);
    Vector4d all_scores2;
    //all_scores2(0) = fabs(ae[0] - be[0]) + fabs(ae[1] - be[1]) + fabs(-ae[0] + be[0]) + fabs(-ae[1] + be[1]);
    //all_scores2(1) = fabs(ae[0] - be[1]) + fabs(ae[1] + be[0]) + fabs(-ae[0] + be[1]) + fabs(-ae[1] - be[0]);
    //all_scores2(2) = fabs(ae[0] + be[0]) + fabs(ae[1] + be[1]) + fabs(-ae[0] - be[0]) + fabs(-ae[1] - be[1]);
    //all_scores2(3) = fabs(ae[0] + be[1]) + fabs(ae[1] - be[0]) + fabs(-ae[0] - be[1]) + fabs(-ae[1] + be[0]);

    //all_scores2(0) = 2 * (fabs(ae[0] - be[0]) + fabs(ae[1] - be[1]));
    //all_scores2(1) = 2 * (fabs(ae[0] - be[1]) + fabs(ae[1] + be[0]));
    //all_scores2(2) = 2 * (fabs(ae[0] + be[0]) + fabs(ae[1] + be[1]));
    //all_scores2(3) = 2 * (fabs(ae[0] + be[1]) + fabs(ae[1] - be[0]));

    all_scores2(0) = fabs(ae1 - be1) + fabs(ae2 - be2);
    all_scores2(1) = fabs(ae1 - be2) + fabs(ae2 + be1);
    all_scores2(2) = fabs(ae1 + be1) + fabs(ae2 + be2);
    all_scores2(3) = fabs(ae1 + be2) + fabs(ae2 - be1);

    int best_i2 = 0;
    all_scores2.minCoeff(&best_i2);


    //if (best_i != best_i2)
    //{
    //    cout << "error in polyvector_field_matching_fast2:   best_i != best_i2" << endl;
    //    assert(best_i == best_i2);
    //}

    // best_matching_for_sorted_a is the matching for the sorted vectors in a
    // get the matching for the initial (unsorted) vectors    
    //v0
    //for (int i = 0; i < 4; ++i)
    //{
    //    mab[i] = best_i2;
    //    mba[best_i2] = i;
    //    best_i2++;
    //    if (best_i2 == 4) best_i2 = 0;
    //}

    //v1
    //mab[0] = (0 + best_i2) % 4;
    //mab[1] = (1+ best_i2) % 4;
    //mab[2] = (2 + best_i2) % 4;
    //mab[3] = (3 + best_i2) % 4;
    //mba[(0 + best_i2) % 4] = 0;
    //mba[(1 + best_i2) % 4] = 1;
    //mba[(2 + best_i2) % 4] = 2;
    //mba[(3 + best_i2) % 4] = 3;

    //v2
    *mabba = MatchABBA::Create(best_i2);

    //v3
    //int mabbest[16] = { 0,1,2,3,  1,2,3,0,  2,3,0,1,  3,0,1,2 };
    //int mbabest[16] = { 0,1,2,3,  3,0,1,2,  2,3,0,1,  1,2,3,0 };
    //mab[0] = mabbest[best_i2 * 4 + 0];
    //mab[1] = mabbest[best_i2 * 4 + 1];
    //mab[2] = mabbest[best_i2 * 4 + 2];
    //mab[3] = mabbest[best_i2 * 4 + 3];
    //mba[0] = mbabest[best_i2 * 4 + 0];
    //mba[1] = mbabest[best_i2 * 4 + 1];
    //mba[2] = mbabest[best_i2 * 4 + 2];
    //mba[3] = mbabest[best_i2 * 4 + 3];

    //v4
    //int mabbest[16] = { 0,1,2,3,  1,2,3,0,  2,3,0,1,  3,0,1,2 };
    //int mbabest[16] = { 0,1,2,3,  3,0,1,2,  2,3,0,1,  1,2,3,0 };
    //int* pmabbest = &mabbest[best_i2 * 4];
    //int* pmbabest = &mbabest[best_i2 * 4];
    //mab[0] = pmabbest[0];
    //mab[1] = pmabbest[1];
    //mab[2] = pmabbest[2];
    //mab[3] = pmabbest[3];
    //mba[0] = pmbabest[0];
    //mba[1] = pmbabest[1];
    //mba[2] = pmbabest[2];
    //mba[3] = pmbabest[3];


}



//__declspec(noinline)
//__forceinline 
__forceinline void polyvector_field_matching_fast3_sse4(
    const V3& a1, const V3& a2,
    const V3& b1, const V3& b2,
    const V3& e,
    MatchABBA* mabba)
{
    D ae1 = a1.dot_sse41(e);
    D ae2 = a2.dot_sse41(e);
    D be1 = b1.dot_sse41(e);
    D be2 = b2.dot_sse41(e);

    // test #0 - rotation 0
    D min = abs(ae1 - be1) + abs(ae2 - be2);
    MatchABBA res(0, 1, 2, 3, 0, 1, 2, 3);

    // test #1 - rotation 90
    D min1 = abs(ae1 - be2) + abs(ae2 + be1);
    if (min1 < min)
    {
        min = min1;
        res = MatchABBA(1, 2, 3, 0, 3, 0, 1, 2);
    }

    // test #2 - rotation 180
    D min2 = abs(ae1 + be1) + abs(ae2 + be2);
    if (min2 < min)
    {
        min = min2;
        res = MatchABBA(2, 3, 0, 1, 2, 3, 0, 1);
    }

    // test #3 - rotation 270
    D min3 = abs(ae1 + be2) + abs(ae2 - be1);
    if (min3 < min)
    {
        res = MatchABBA(3, 0, 1, 2, 1, 2, 3, 0);
    }

    // store result
    *mabba = res;
}

//__forceinline 
//__declspec(noinline) 
__forceinline void polyvector_field_matching_fast4_sse4(
    const V3& field1_x, const V3& field1_y,
    const V3& field2_x, const V3& field2_y,
    const V3& common_edge,
    MatchABBA* mabba)
{
    D x1 = field1_x.dot_sse41(common_edge);
    D y1 = field1_y.dot_sse41(common_edge);
    D x2 = field2_x.dot_sse41(common_edge);
    D y2 = field2_y.dot_sse41(common_edge);

    Vec4f s1(x1, y1, 0, 0);


    auto lenPow2 = [](Vec4f a) // 'a*a + b*b' is same as 'abs(a) + abs(b)' when we compare result with '<'
    {
        return _mm_dp_ps(a, a, 49).m128_f32[0]; // 49 - dot x and y (z and w treat as zero) and write result  to first float
    };

    //v0
    Vec4f s2(x2, y2, -x2, -y2);
    #define SHUFFLE(i0,i1,i2,i3) (((i3) << 6) | ((i2) << 4) | ((i1) << 2) | ((i0)))
    D min0 = lenPow2(s1 - s2);
    D min90 = lenPow2(s1 - _mm_shuffle_ps(s2, s2, SHUFFLE(1, 2, 3, 0)));
    D min180 = lenPow2(s1 - _mm_shuffle_ps(s2, s2, SHUFFLE(2, 3, 0, 1)));
    D min270 = lenPow2(s1 - _mm_shuffle_ps(s2, s2, SHUFFLE(3, 0, 1, 2)));

    //v1
    //Vec4f s2_ratated_0(x2, y2, 0, 0);
    //Vec4f s2_ratated_90(y2, -x2, 0, 0);
    //Vec4f s2_ratated_180(-x2, -y2, 0, 0);
    //Vec4f s2_ratated_270(-y2, x2, 0,0);
    //D min0 = lenPow2(s1 - s2_ratated_0);
    //D min90 = lenPow2(s1 - s2_ratated_90);
    //D min180 = lenPow2(s1 - s2_ratated_180);
    //D min270 = lenPow2(s1 - s2_ratated_270);


    // test rotation by 0 degree
    D min = min0;
    MatchABBA res(0, 1, 2, 3, 0, 1, 2, 3);

    // test rotation by 90 degree
    if (min90 < min)
    {
        min = min90;
        res = MatchABBA(1, 2, 3, 0, 3, 0, 1, 2);
    }

    // test rotation by 180 degree
    if (min180 < min)
    {
        min = min180;
        res = MatchABBA(2, 3, 0, 1, 2, 3, 0, 1);
    }

    // test rotation by 270 degree
    if (min270 < min)
    {
        min = min270;
        res = MatchABBA(3, 0, 1, 2, 1, 2, 3, 0);
    }

    // store result
    *mabba = res;
}

void polyvector_field_matchings_fast(
    const vector<V3s>& field,
    const P3s& V,
    const I3s& F,
    const V3s& F_X,
    const I2s&EV,
    const V3s& FN,
    const I2s& EF,
    const Ds& E_Length,
    const Bs& isBorderEdge,
    MatchABBAs& match_ab_ba, bool isOmpEnabled)
{
    int numEdges = EV.rows();
    match_ab_ba.resize(numEdges);
    MatchABBA* mabba = match_ab_ba.data();

    //bool IsOmpEnabled = isOmpEnabled;
    //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    const int* pEF = EF.data();
    const int* pEV = EV.data();

    if (false)
        //    if (utils::cpu::isSupportedSSE4)
    {
        for (int ei = 0; ei < numEdges; ++ei)
        {
            if (!isBorderEdge[ei])
            {
                // the two faces of the flap
                //int a = EF(ei, 0);
                //int b = EF(ei, 1);
                int a = pEF[0];
                int b = pEF[1];
                //V3 ce = (V.row(EV(ei, 1)) - V.row(EV(ei, 0)));
                V3 ce = (V.row(pEV[1]) - V.row(pEV[0]));
                //ce /= E_Length(ei); // no need in normalization, since we take all crosses along this edge and compare them by '<', so comparision result will be same
                polyvector_field_matching_fast4_sse4(
                    field[0].row(a), field[1].row(a),
                    field[0].row(b), field[1].row(b),
                    ce,
                    mabba);
                //if (ei == 3045)
                //{
                //    cout << "mab={" << match_ab_ba[ei].ab(0) << "," << match_ab_ba[ei].ab(1) << "," << match_ab_ba[ei].ab(2) << "," << match_ab_ba[3045].ab(3) << "}  mba={" << match_ab_ba[ei].ba(0) << "," << match_ab_ba[ei].ba(1) << "," << match_ab_ba[ei].ba(2) << "," << match_ab_ba[ei].ba(3) << "}" << endl;
                //}
                mabba++;
                pEF += 2;
                pEV += 2;
            }
            else
            {
                *mabba = MatchABBA(NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH);
                mabba++;
                pEF += 2;
                pEV += 2;
            }
        }
    }
    else
    {
        for (int ei = 0; ei < numEdges; ++ei)
        {
            if (!isBorderEdge[ei])
            {
                // the two faces of the flap
                //int a = EF(ei, 0);
                //int b = EF(ei, 1);
                int a = pEF[0];
                int b = pEF[1];
                //V3 ce = (V.row(EV(ei, 1)) - V.row(EV(ei, 0)));
                V3 ce = (V.row(pEV[1]) - V.row(pEV[0]));
                //ce /= E_Length(ei); // no need in normalization, since we take all crosses along this edge and compare them by '<', so comparision result will be same
                polyvector_field_matching_fast2(
                    field[0].row(a), field[1].row(a),
                    field[0].row(b), field[1].row(b),
                    ce,
                    mabba);
                //if (ei == 3045)
                //{
                //    cout << "mab={" << match_ab_ba[ei].ab(0) << "," << match_ab_ba[ei].ab(1) << "," << match_ab_ba[ei].ab(2) << "," << match_ab_ba[3045].ab(3) << "}  mba={" << match_ab_ba[ei].ba(0) << "," << match_ab_ba[ei].ba(1) << "," << match_ab_ba[ei].ba(2) << "," << match_ab_ba[ei].ba(3) << "}" << endl;
                //}

                mabba++;
                pEF += 2;
                pEV += 2;
            }
            else
            {
                *mabba = MatchABBA(NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH, NOMATCH);
                mabba++;
                pEF += 2;
                pEV += 2;
            }
        }
    }
}

void MeshSolverResult::Init_match_ab_ba(ViewerDrawObjects& draw, int N)
{
    bool match_with_curl = true;
    extern bool IsOmpEnabled;
    const P3s& V = mesh.V;
    const I3s& F = mesh.F;
    if (Field.size() == 0)
    {
        //cout << "!!!warning:     nrosyField.size() == 0" << endl;
    }
    // try to use my optimized method insted of general igl method
    if (IsFieldSorted && match_with_curl && N == 4)
    {
        //for (int i = 0; i < 100; i++)
        polyvector_field_matchings_fast(Field, V, F, mesh.F_X, mesh.EV, mesh.F_normals, mesh.EF, mesh.E_Length, mesh.E_isborder, Field_match_ab_ba, IsOmpEnabled);
        //polyvector_field_matching_fast(
    }
    else
    {
        #ifdef USE_EIGEN
        if (nrosyField.size() != 0)
        {
            MatrixXd nrosyFieldMatrix;
            int rows = nrosyField[0].rows();
            int cols = nrosyField.size() * 3;
            nrosyFieldMatrix.setZero(rows, cols);
            for (int i = 0; i < nrosyField[0].rows(); i++)
            {
                for (int ni = 0; ni < nrosyField.size(); ni++)
                {
                    nrosyFieldMatrix.block(i, ni * 3, 1, 3) = nrosyField[ni].row(i);
                }
            }
            igl::polyvector_field_matchings(nrosyFieldMatrix, V, F, mesh.EV, mesh.F_normals, mesh.EF, match_with_curl, false, match_ab, match_ba, isFieldVectorsSorted, IsOmpEnabled);
        }
        #else
        cout << "error!   method 'MeshStreams::Init_match_ab_ba()' not implemented for N != 4" << endl;
        assert(false && "method 'MeshStreams::Init_match_ab_ba()' not implemented for N != 4");
        #endif
    }

    // near singularities, it can be difficault to detect correct match - so we have to take into account what kind of singularity we have and what angle difference we want to achieve
    Init_match_ab_ba__fix_near_singularities(draw);
}

void MeshSolverResult::Init_match_ab_ba__fix_near_singularities(ViewerDrawObjects& draw)
{
    const bool debug = false;

    auto drawComment = [&](int fid, int ni, D angleChange, string prefix)
    {
        V3 fieldDirection = getField(ni, fid);
        D len = (mesh.avg_edge_length / 4);
        const P3s& C = mesh.F_Barycenters;
        P3 c = C.row(fid);
        draw.AddEdge(c, c + fieldDirection * len, Color3d(1, 0, 0), prefix + "ni=" + to_string(ni) + "   anglechange=" + to_string(angleChange));
    };


    for (int vid = 0; vid < mesh.V.rows(); vid++)
    {
        bool isSing3 = (Singularities(vid) < -0.001);
        bool isSing5 = (Singularities(vid) > 0.001);
        if (isSing3 || isSing5) 
        //if (isSing3)// T O D O remove this line and uncomment upper line (temporaly used for debuging)
        {
            // DEBUG show singularity info
            //draw.AddLabel(mesh.V.row(vid), isSing3 ? "   3" : "   5", Color3d(1, 0, 0), 5);
            //cout << "singularity  "<<(isSing3 ? "3" : "5") <<" at   vid = " << vid << endl;

            //
            // detect issue
            //
            auto v = mesh.VertexToFacesSorted(vid);
            auto ni_start = 0;
            auto ni = ni_start;
            int imax = v.size() - 1;
            for (int i = 0; i <= imax; i++)
            {
                auto fid = v[i].FaceId;
                auto eid = v[i].LeftSideEdgeId;
                auto niNext = (mesh.EF(eid, 0) == fid)
                    ? Field_match_ab_ba(eid).ab(ni)
                    : Field_match_ab_ba(eid).ba(ni);
                assert(niNext <= 3);
                ni = niNext;
            }

            int ni_change_correct = isSing3 ? 1 : 4-1; // Sing3 - rotation counter-clockwaise, and for Sing5 - rotation clockwise
            int ni_change_current = ni - ni_start;

            //
            // try to fix issue
            //
            if (ni_change_current == 0) // for Sing3 change should be 1, and for isSing5 change should be -1
            {
                // find the worst angle change
                ni_start = 0;
                ni = ni_start;
                int worst__index = -1;
                D worst__angle_change = 0;
                int worst__ni = -1;
                int worst__niNext = -1;
                for (int i = 0; i <= imax; i++)
                {
                    int iNext = (i == imax) ? 0 : i + 1;
                    auto fid = v[i].FaceId;
                    auto fidNext = v[iNext].FaceId;
                    auto eid = v[i].LeftSideEdgeId;
                    auto niNext = (mesh.EF(eid, 0) == fid)
                        ? Field_match_ab_ba(eid).ab(ni)
                        : Field_match_ab_ba(eid).ba(ni);
                    assert(niNext <= 3);

                    V3 common_edge = v[i].LeftSideDirection;
                    D angle0 = utils::vector::AngleFull(getField(ni, fid), common_edge, mesh.F_Z.row(fid));
                    D angle1 = utils::vector::AngleFull(common_edge, getField(niNext, fidNext), mesh.F_Z.row(fidNext));
                    D angleChange = angle0 + angle1;
                    if (angleChange > 360) angleChange -= 360; // 360 degree because we work in same space, so here full angle will be 360
                    if (angleChange > 180) angleChange -= 360; // we want to have value close to zero: plus or minus
                    if ((isSing3 && (angleChange < worst__angle_change)) // singularity3 should have only positive angle changes
                        || (isSing5 && (angleChange > worst__angle_change))) // singularity3 should have only negative angle changes
                    {
                        worst__angle_change = angleChange;
                        worst__index = i;
                        worst__ni = ni;
                        worst__niNext = niNext;
                    }

                    ni = niNext;
                }

                // try to fix if found
                if (worst__index != -1)
                {
                    if (debug) cout << "found bad angle change near to singularity(" << (isSing3 ? "3" : "5") << ") at vertex " << vid << " is incorrect:  expected " << ni_change_correct << " but is " << ni_change_current << endl;
                    if (debug) cout << "-  bad angle change " << worst__angle_change << " at index#" << worst__index << endl;
                    int IncDegree = 0;
                    if (isSing3 && worst__angle_change < -30) // usually bad angle will be -45 what is exactly when we cant guarantee a correct decision
                    {
                        IncDegree = 1;
                    }
                    if (isSing5 && worst__angle_change > 20) // for Sing5 angle will be lower from Sing3
                    {
                        IncDegree = -1;
                    }

                    if (IncDegree != 0)
                    {
                        auto worst__eid = v[worst__index].LeftSideEdgeId;
                        if (mesh.EF(worst__eid, 0) != v[worst__index].FaceId)
                        {
                            //eid = v[worst__index].RightSideEdgeId;
                            IncDegree = -IncDegree;
                        }
                        MatchABBA fix = Field_match_ab_ba(worst__eid);
                        fix.IncDegree(IncDegree);  // this is the fix - we decrease rotation degree by 1

                        // test if our fix fixed the issue
                        ni_start = 0;
                        ni = ni_start;
                        for (int i = 0; i <= imax; i++)
                        {
                            auto fid = v[i].FaceId;
                            auto eid = v[i].LeftSideEdgeId;
                            auto match_ab_ba = Field_match_ab_ba(eid);
                            if (i == worst__index) match_ab_ba = fix;
                            auto niNext = (mesh.EF(eid, 0) == fid)
                                ? match_ab_ba.ab(ni)
                                : match_ab_ba.ba(ni);
                            assert(niNext <= 3);
                            ni = niNext;
                        }
                        ni_change_current = ni - ni_start;

                        if (ni_change_current == ni_change_correct) // if fix really works
                        {
                            if (debug) cout << "-  bad angle fixed! " << endl;
                            Field_match_ab_ba(worst__eid) = fix; // apply fix
                        }
                    }
                }
            }


            //
            // DEBUG show angles
            // 
            if (debug)
            {
                D sum_angleChange = 0;
                D rotation_angle = 0;
                ni = ni_start;
                for (int i = 0; i <= imax; i++)
                {
                    int iNext = (i == imax) ? 0 : i + 1;
                    auto fid = v[i].FaceId;
                    auto fidNext = v[iNext].FaceId;
                    auto eid = v[i].LeftSideEdgeId;
                    auto niNext = (mesh.EF(eid, 0) == fid)
                        ? Field_match_ab_ba(eid).ab(ni)
                        : Field_match_ab_ba(eid).ba(ni);
                    assert(niNext <= 3);

                    V3 common_edge = v[i].LeftSideDirection;
                    D angle0 = utils::vector::AngleFull(getField(ni, fid), common_edge, mesh.F_Z.row(fid));
                    D angle1 = utils::vector::AngleFull(common_edge, getField(niNext, fidNext), mesh.F_Z.row(fidNext));
                    D angleChange = angle0 + angle1;
                    if (angleChange > 360) angleChange -= 360; // 360 degree because we work in same space, so here full angle will be 360
                    if (angleChange > 180) angleChange -= 360; // we want to have value close to zero: plus for Sing3 or minus for Sing5
                    drawComment(fid, ni, angleChange, "#" + to_string(i) + " ");
                    sum_angleChange += angleChange;
                    rotation_angle += v[i].angle;
                    ni = niNext;
                }
                drawComment(v[0].FaceId, ni, sum_angleChange, "                                                                                         ROTATION angle=" + to_string(rotation_angle) + "   ");
                //cout << "      ni_start = " << ni_start << "     ni_end = " << ni << endl;
            }

            // show fail message if we failed to fix this issue
            if (ni_change_correct != ni_change_current)
            {
                cout << "wrong!   match_ab_ba near to singularity(" << (isSing3 ? "3" : "5") << ") at vertex " << vid << " mesh " << mesh.Name << " meshid "<<mesh.id << " is incorrect:  expected " << ni_change_correct << " but is " << ni_change_current << endl;
                assert(ni_change_correct == ni_change_current && "match_ab_ba near to singularity is incorrect");
            }
        }
    }
}























MeshSolverNrosy::MeshSolverNrosy(const Mesh& _mesh, ViewerDrawObjects& _draw)
    : mesh(_mesh), draw(_draw), Constrains(_mesh, _draw), Result(_mesh)
{
}


void MeshSolverNrosy::Solve(bool logMessages)
{
    //Timer timeTotal;
    auto N = static_cast<int>(options.N) + 1;
    MeshLogicOptions_SolverNrosy::MeshSolverType solverType = options.Solver;
    D soft = 1;
    #if alternative_solvers_SUPPORTED
    soft = options.SoftPercent;
    #endif
    MeshLogicOptions_Constrains::MeshConstrainsType constrainType = meshLogicOptions.Constrains.Type;
    bool WeightIsRelativeToEdgeLength = meshLogicOptions.Constrains.WeightIsRelativeToEdgeLength;


    auto time = utils::time::Now();

    //
    // init constains
    //
    if (Constrains.Constrains.size() == 0)
    {
        //auto time = utils::time::Now();
        //cout << "detectBorderConstraines ...";
        Timer t;
        Constrains.Init(constrainType);
        t.stop(elapsedTimers.Solver.ConstrainsInit);
        //if (meshLogicOptions.showTimeElapsed) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl; else cout << endl;
    }


    //
    // fail to solve if there is not enought inner faces - non border faces
    //
    Result.SolverType = solverType;
    //int fBorderCounts = 0;    
    //for (int i = 0; i < mesh.F_isborder.size();i++) if (mesh.F_isborder(i)) fBorderCounts++;
    int fBorderCounts = mesh.F_isborder.countOfValues(true);
    if (mesh.F.rows() < 7 || 1.0*fBorderCounts / mesh.F.rows() > 0.9) // at least 10% must be non-border faces
    {
        Result.Clear();
        //timeTotal.stop(elapsedTimers.Solver.Total);
        return;
    }

    //
    // detect ignore_x (we ignore x if dirrection is not corrected)
    //
    bool ignore_x = false;
    #if NPolyVectorFieldSolverGeneral_SUPPORTED
    ignore_x = options.ignore_x;
    if (ignore_x)
    {
        D ignore_x__max_correction_angle = options.ignore_x__max_correction_angle;
        for (int i = 0; i < Constrains.Constrains.size(); ++i)
        {
            if (Constrains.Constrains[i].DirectionY_Corrected_Angle > ignore_x__max_correction_angle) // if direction is changed - we cannot ignore X
            {
                ignore_x = false;
                break;
            }
        }
    }
    if (options.ignore_x_always) ignore_x = true;
    #endif

    //
    // Solve (2 times to get best result -  for corrected and uncorrected border contstrains)
    //
    SolveOnce(mesh, draw, Constrains, N, solverType, soft, WeightIsRelativeToEdgeLength, logMessages,
        ignore_x, true, Result);

    if (!ignore_x && options.check_angle_correction_validity && Result.Field.size() > 0) // if solution is for corrected directions and we have to check validity of corrected solution
    {
        MeshSolverResult Result_uncorrected(mesh);

        // get solution for uncorrected contrains        
        SolveOnce(mesh, draw, Constrains, N, solverType, soft, WeightIsRelativeToEdgeLength, logMessages,
            true, false, Result_uncorrected);// ignore x because it is always 0 for uncorrectedContrains

        if (Result.Field.size() > 0 && Result_uncorrected.Field.size() > 0)
        {
            int  singCount = Result.GetSingularityCounts(true, true);
            int  singCount_uncorrected = Result_uncorrected.GetSingularityCounts(true, true);
            bool isUncorrectedSolutionBetter = false;
            if (singCount_uncorrected < singCount)
            {
                if (logMessages) cout << "Ignore corrected contrains since uncorrected is better:   singCount = " << singCount << ",  singCount_uncorrected = " << singCount_uncorrected << endl;
                isUncorrectedSolutionBetter = true;
            }
            else if (singCount_uncorrected == singCount)
            {
                auto minFieldLengthPow2 = [](const V3s& field)
                {
                    Ds W(field.rows());
                    for (int i = 0; i < field.rows(); ++i)
                    {
                        D x = field(i, 0);
                        D y = field(i, 1);
                        D z = field(i, 2);
                        W(i) = x * x + y * y + z * z;
                    }
                    return W.minCoeff();
                };
                D wmin = minFieldLengthPow2(Result.Field[0]);// take first field since length of both are indentical
                D wmin_uncorrected = minFieldLengthPow2(Result_uncorrected.Field[0]);// take first field since length of both are indentical
                //cout << "wmin = " << wmin << "     wumin = " << wumin << endl;
                // if uncorrected constrains produce better result - then return that result (return result with lowest error)
                if (wmin < wmin_uncorrected)
                {
                    if (logMessages) cout << "Ignore corrected contrains since uncorrected is better:   wmin = " << wmin << ",  wmin_uncorrected = " << wmin_uncorrected << endl;
                    isUncorrectedSolutionBetter = true;
                }
            }

            if (isUncorrectedSolutionBetter)
            {
                Result.IsFieldSorted = Result_uncorrected.IsFieldSorted;
                Result.Field = Result_uncorrected.Field;
                Result.Field_match_ab_ba = Result_uncorrected.Field_match_ab_ba;
                Result.Singularities = Result_uncorrected.Singularities;
            }
        }
    }
    //timeTotal.stop(elapsedTimers.Solver.Total);
}


void MeshSolverNrosy::SolveOnce(const Mesh& mesh, ViewerDrawObjects& draw, const MeshConstrains& Constrains,
    const int N,                     //N-rosy dimension
    MeshLogicOptions_SolverNrosy::MeshSolverType solverType,
    const D soft,             //Set the ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
    bool WeightIsRelativeToEdgeLength, bool logMessages, bool ignore_x, bool takeDirectionCorrected, MeshSolverResult& Result
)
{

    FindCrossFieldSingularities findCrossFieldSing(draw, mesh.V, mesh.F,
        mesh.F_X, mesh.F_Y, mesh.F_normals, mesh.F_Barycenters,
        mesh.FE, mesh.EV, mesh.EF, mesh.FF,
        mesh.V_isborder, mesh.VF);

    if (logMessages) cout << "Solving  ...  (mesh size = " << mesh.F.rows() << ")" << endl;


    #if NPolyVectorFieldSolverGeneral_SUPPORTED
    if (ignore_x && solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorFieldGeneral)
    {
        solverType = MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField;
    }
    #endif


    //
    // NPolyVectorField2x
    //
    if (solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Complex2x
        || solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Polar2x)
    {
        //
        // n_polyvector
        //
        vector<V3s> Fraw;
        bool Fraw_isSorted = false;
        if (solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Complex2x)
        {
            n_polyvector_complex2x(draw,
                mesh.V, mesh.F, mesh.F_Areas, mesh.E_isborder, mesh.EV, mesh.E_Length, mesh.FE, mesh.EF, mesh.K, mesh.F_X, mesh.F_Y, mesh.F_Z, mesh.avg_edge_length, mesh.max_edge_length,
                Constrains.Constrains, soft, Fraw, Fraw_isSorted, WeightIsRelativeToEdgeLength, logMessages, ignore_x, takeDirectionCorrected); // supports only hard constrains
        }
        else
        {
            n_polyvector_polar2x(draw, mesh.id,
                mesh.V, mesh.F, mesh.F_Areas, mesh.E_isborder, mesh.EV, mesh.E_Length, mesh.FE, mesh.EF, mesh.K, mesh.F_X, mesh.F_Y, mesh.F_Z, mesh.avg_edge_length, mesh.max_edge_length,
                Constrains.Constrains, soft, Fraw, Fraw_isSorted, WeightIsRelativeToEdgeLength, logMessages, ignore_x, takeDirectionCorrected); // supports only hard constrains
        }


        //
        // NormalizeNrosyField
        //
        if (logMessages) cout << "Extracting results   NormalizeNrosyField ...";
        Timer timeNormalizeNrosyField;
        //for(int i = 0; i < 100 ; i++)
        NormalizeNrosyField(mesh, draw, Constrains, N, Fraw, Fraw_isSorted, Result);
        timeNormalizeNrosyField.stop(elapsedTimers.Solver.NormalizeNrosyField);
        if (logMessages) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timeNormalizeNrosyField << endl; else cout << endl;


        //
        // findCrossFieldSingularities
        //
        bool is2x = true; // always true for this solver
        if (logMessages) cout << "Extracting results   findCrossFieldSingularities ...";
        Timer timefindCrossFieldSingularities;
        if (Result.Field.size() > 0)
        {
            findCrossFieldSing.findCrossFieldSingularities(Result.Field[0], Result.Field[1], Result.Singularities, is2x);
            Result.Init_match_ab_ba(draw, N); // we need to call it after finding singularities
        }
        timefindCrossFieldSingularities.stop(elapsedTimers.Solver.findCrossFieldSingularities);
        if (logMessages) if (meshLogicOptions.showTimeElapsed) cout << " done in " << timefindCrossFieldSingularities << endl; else cout << endl;


        //if (logMessages) cout << "Extracting results   findCrossFieldSingularities  2 ...";
        //time = utils::time::Now();
        //if (Result.Field.size() > 0)
        //{
        //    printf("--Matchings and curl--\n");
        //    MatrixXi match_ab, match_ba;  // matchings across interior edges
        //    Ds curl; // curl per edge
        //    MatrixXd two_pv ; // field
        //    --here we need to populate 'two_pv'--
        //    D avgCurl = igl::polyvector_field_matchings(two_pv, mesh.V, mesh.F, true, true, match_ab, match_ba, curl);
        //    D maxCurl = curl.maxCoeff();
        //    printf("curl -- max: %.5g, avg: %.5g\n", maxCurl, avgCurl);

        //    printf("--Singularities--\n");
        //    Is singularities; // singularities
        //    igl::polyvector_field_singularities_from_matchings(mesh.V, mesh.F, match_ab, match_ba, singularities);
        //    printf("#singularities: %ld\n", singularities.rows());
        //    findCrossFieldSing.findCrossFieldSingularities(Result.Field[0], Result.Field[1], Result.Singularities, is2x);
        //    Result.Init_match_ab_ba(draw, N); // we need to call it after finding singularities
        //}
        //options.time_elapsed_findCrossFieldSingularities += utils::time::ElapsedMilliseconds(time);
        //if (logMessages) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    }



    //
    // MeshLinearSolver
    //
    #if MeshLinearSolver_SUPPORTED
    else if (solverType == MeshLogicOptions_Solver::MeshSolverType::MeshLinearSolver)
    {
        MeshLinearSolver solver(draw, mesh.V, mesh.F, mesh.F_Areas, mesh.F_Barycenters, mesh.EV, mesh.FE, mesh.EF, mesh.K, mesh.F_X, mesh.F_Y, mesh.F_Z, mesh.FF, mesh.FFi, mesh.avg_edge_length, mesh.max_edge_length);
        solver.SetConstrains(Constrains.Constrains, soft, WeightIsRelativeToEdgeLength, logMessages, ignore_x, takeDirectionCorrected);

        vector<MatrixX3d> Fraw;
        bool Fraw_isSorted = false;
        solver.Solve(Fraw, Fraw_isSorted);
        return;

        //
        // NormalizeNrosyField
        //
        if (logMessages) cout << "Extracting results   NormalizeNrosyField ...";
        Timer timeNormalizeNrosyField;
        //for(int i = 0; i < 100 ; i++)
        NormalizeNrosyField(mesh, draw, Constrains, N, Fraw, Fraw_isSorted, Result);
        timeNormalizeNrosyField.stop(elapsedTimers.Solver.NormalizeNrosyField);
        if (logMessages) cout << " done in " << timeNormalizeNrosyField << endl;


        //
        // findCrossFieldSingularities
        //
        if (logMessages) cout << "Extracting results   findCrossFieldSingularities ...";
        Timer timefindCrossFieldSingularities;
        if (Result.Field.size() > 0)
        {
            findCrossFieldSing.findCrossFieldSingularities(Result.Field[0], Result.Field[1], Result.Singularities, true);
            Result.Init_match_ab_ba(draw, N); // we need to call it after finding singularities
        }
        timefindCrossFieldSingularities.stop(elapsedTimers.Solver.findCrossFieldSingularities);
        if (logMessages) cout << " done in " << timefindCrossFieldSingularities << endl;
    }
    #endif

    //
    // NPolyVectorField
    //
    #if NPolyVectorFieldSolverGeneral_SUPPORTED
    else if (solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField || solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorFieldGeneral)
    {
        //
        // n_polyvector
        //
        vector<V3s> Fraw;
        bool Fraw_isSorted = false;
        bool is2x = (solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField);
        if (is2x)
        {
            n_polyvector_soft(draw,
                mesh.V, mesh.F, mesh.F_Areas, mesh.E_isborder, mesh.EV, mesh.FE, mesh.EF, mesh.K, mesh.F_X, mesh.F_Y, mesh.F_Z, mesh.avg_edge_length, mesh.max_edge_length,
                Constrains.Constrains, soft, Fraw, Fraw_isSorted, WeightIsRelativeToEdgeLength, logMessages, ignore_x, takeDirectionCorrected); // supports only hard constrains
        }
        else
        {
            n_polyvector_general_soft(draw,
                mesh.V, mesh.F, mesh.F_Areas, mesh.EV, mesh.FE, mesh.EF, mesh.K, mesh.F_X, mesh.F_Y, mesh.F_Z,
                Constrains.Constrains, soft, Fraw, takeDirectionCorrected); // supports only hard constrains
        }


        //
        // NormalizeNrosyField
        //
        if (logMessages) cout << "Extracting results   NormalizeNrosyField ...";
        Timer timeNormalizeNrosyField;
        //for(int i = 0; i < 100 ; i++)
        NormalizeNrosyField(mesh, draw, Constrains, N, Fraw, Fraw_isSorted, Result);
        timeNormalizeNrosyField.stop(elapsedTimers.Solver.NormalizeNrosyField);
        if (logMessages) cout << " done in " << timeNormalizeNrosyField << endl;


        //
        // findCrossFieldSingularities
        //
        if (logMessages) cout << "Extracting results   findCrossFieldSingularities ...";
        Timer timefindCrossFieldSingularities;
        if (Result.Field.size() > 0)
        {
            findCrossFieldSing.findCrossFieldSingularities(Result.Field[0], Result.Field[1], Result.Singularities, is2x);
            Result.Init_match_ab_ba(draw, N); // we need to call it after finding singularities
        }
        timefindCrossFieldSingularities.stop(elapsedTimers.Solver.findCrossFieldSingularities);
        if (logMessages) cout << " done in " << timefindCrossFieldSingularities << endl;


        //if (logMessages) cout << "Extracting results   findCrossFieldSingularities  2 ...";
        //time = utils::time::Now();
        //if (Result.Field.size() > 0)
        //{
        //    printf("--Matchings and curl--\n");
        //    MatrixXi match_ab, match_ba;  // matchings across interior edges
        //    Ds curl; // curl per edge
        //    MatrixXd two_pv ; // field
        //    --here we need to populate 'two_pv'--
        //    D avgCurl = igl::polyvector_field_matchings(two_pv, mesh.V, mesh.F, true, true, match_ab, match_ba, curl);
        //    D maxCurl = curl.maxCoeff();
        //    printf("curl -- max: %.5g, avg: %.5g\n", maxCurl, avgCurl);

        //    printf("--Singularities--\n");
        //    Is singularities; // singularities
        //    igl::polyvector_field_singularities_from_matchings(mesh.V, mesh.F, match_ab, match_ba, singularities);
        //    printf("#singularities: %ld\n", singularities.rows());
        //    findCrossFieldSing.findCrossFieldSingularities(Result.Field[0], Result.Field[1], Result.Singularities, is2x);
        //    Result.Init_match_ab_ba(draw, N); // we need to call it after finding singularities
        //}
        //options.time_elapsed_findCrossFieldSingularities += utils::time::ElapsedMilliseconds(time);
        //if (logMessages) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    }
    #endif


    #if Comiso_SUPPORTED
    else if (solverType == MeshLogicOptions_SolverNrosy::MeshSolverType::FrameField)
    {
        // Interpolated frame field
        MatrixXd FF1, FF2;
        // Interpolate the frame field
        Is b;
        MatrixXd bc1;
        MatrixXd bc2;
        b.resize(Constrains.Constrains.size());
        bc1.resize(b.size(), 3);
        bc2.resize(b.size(), 3);

        for (int i = 0; i < Constrains.Constrains.size(); ++i)
        {
            int fid = Constrains.Constrains[i].FaceId;
            V3 constrDirection = takeDirectionCorrected
                ? Constrains.Constrains[i].DirectionY_Corrected
                : Constrains.Constrains[i].DirectionY;
            int eid = mesh.FE(fid, 0);
            V3 edgeDirection = Constrains.Constrains[i].DirectionX;// mesh.F_X.row(fid);
            //V3 direction = mesh.F_Y.row(fid);
            b(i) = fid;
            bc1.row(i) = convertV3ToEigenDouble(edgeDirection);
            bc2.row(i) = convertV3ToEigenDouble(constrDirection);
        }
        MatrixXd V = convertP3sToEigenDouble(mesh.V);
        const I3s& F = mesh.F;
        igl::copyleft::comiso::frame_field(V, F, b, bc1, bc2, FF1, FF2);
        MatrixXd YrawCross;
        igl::frame_to_cross_field(V, F, FF1, FF2, YrawCross);

        cout << "Extracting results ...";
        auto time = utils::time::Now();
        MatrixX3d Yraw1 = YrawCross.block(0, 0, YrawCross.rows(), 3);
        vector<V3s> Yraw;
        Yraw.push_back(convertEigenToV3s(Yraw1));
        NormalizeNrosyField(mesh, draw, Constrains, N, Yraw, false, Result);
        findCrossFieldSing.findCrossFieldSingularities(convertEigenToV3s(FF1), convertEigenToV3s(FF2), Result.Singularities, false);
        Result.Init_match_ab_ba(draw, N); // we need to call it after finding singularities
        cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    }
    else
    {
        MatrixXd V = convertP3sToEigenDouble(mesh.V);
        const I3s& F = mesh.F;
        MatrixXd F_normals = convertV3sToEigenDouble(mesh.F_normals);
        VectorXd K;
        K.resize(mesh.K.size());
        for (int i = 0; i < K.size(); i++) K(i) = mesh.K(i);
        igl::copyleft::comiso::NRosyField nrosy(igl::copyleft::comiso::NRosyField(V, F, F_normals, mesh.EV, mesh.FE, mesh.EF, mesh.FF, mesh.FFi, K));
        nrosy.IsInited = false;
        nrosy.Init();

        // Add constraints
        nrosy.resetConstraints();

        //for (int i = 0; i < bHard.size(); ++i)
        //{
        //    nrosy.setConstraintSoft(bHard(i), 1, bcHard.row(i)); //  add hard contrains as soft - works better than adding hard constrains
        //}
        for (int i = 0; i < Constrains.Constrains.size(); ++i)
        {
            int fid = Constrains.Constrains[i].FaceId;
            V3 constrDirection = takeDirectionCorrected
                ? Constrains.Constrains[i].DirectionY_Corrected
                : Constrains.Constrains[i].DirectionY;
            V3 y = mesh.F_Y.row(fid);
            //cout << "angle="<<utils::vector::Angle(Constrains.Constrains[i].DirectionY, y)<<endl;
            //nrosy.setConstraintSoft(fid, 1, convertV3ToEigenDouble(constrDirection)); //  DOESNT WORK add hard contrains as soft - works better than adding hard constrains
            nrosy.setConstraintHard(fid, convertV3ToEigenDouble(constrDirection));
        }

        // Set the soft constraints global weight
        nrosy.setSoftAlpha(soft);

        // Interpolate
        nrosy.solve(N, static_cast<NRosySolver>(solverType));

        cout << "Extracting results ...";
        // Field
        MatrixX3d Yraw1 = nrosy.getFieldPerFace();
        vector<V3s> Yraw;
        Yraw.push_back(convertEigenToV3s(Yraw1));
        NormalizeNrosyField(mesh, draw, Constrains, N, Yraw, false, Result);

        // Extract singularity indices

        Result.Singularities = convertEigenToDs(nrosy.getSingularityIndexPerVertex());
        cout << " done." << endl;
    }
    #endif
}


// Converts a representative vector per face in the full set of vectors that describe an N-RoSy field (based on Result.SolverType properties Result.Field and Result.IsFieldSorted are populated)
void MeshSolverNrosy::NormalizeNrosyField(const Mesh& mesh, ViewerDrawObjects& draw, const MeshConstrains& Constrains, const int N, vector<V3s>& FieldRaw, bool FieldRaw_isSorted, MeshSolverResult& Result)
{
    const P3s& V = mesh.V;
    const I3s& F = mesh.F;
    const V3s& F_normals = mesh.F_normals;
    bool doRollbackDirectionsOnConstrainedFaces = true;  // rollback directions back to constrains on constrained faces since solver may change indexes - so this will fix indexing (works faster from doHealDirectionsOnConstrainedFaces)
    bool doHealDirectionsOnConstrainedFaces = false;      // rotate directions on contrains to fix indexes (works slower from doRollbackDirectionsOnConstrainedFaces but allows to use solver field without change expect indexes)

    auto resizeField = [&](int n)
    {
        Result.Field.resize(n);
        for (int i = 0; i < Result.Field.size(); i++)
        {
            Result.Field[i].resize(F.rows(), 3);
        }
    };

    if (Result.SolverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Complex2x
        || Result.SolverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Polar2x)
    {
        Result.IsFieldSorted = false;
        bool needNormalization = options.NormalizeNrosyField_sort_field_directions;
        if (needNormalization && FieldRaw_isSorted)
        {
            needNormalization = false;
            Result.IsFieldSorted = true;
        }

        // bool IsOmpEnabled;
        //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
        if (needNormalization)
        {
            resizeField(2);
            Result.IsFieldSymetric = true; // replace adding two opposite vectors:  Result.Field[2].row(faceID) = -d1;  Result.Field[3].row(faceID) = -d2 * m;
            for (int faceID = 0; faceID < F.rows(); faceID++)
            {
                const V3& d1 = FieldRaw[0].row(faceID);
                const V3& d2 = FieldRaw[1].row(faceID);
                V3 fnormal = F_normals.row(faceID);
                V3 dnormal = d1.cross(d2);

                int m = 1;
                // we must to fix second field-direction to make field direction clock-wise p1->p2
                // this is important for singularity detection and for speed improvement for Divider
                if (utils::vector::OppositeDirectionIfTheyParallel(dnormal, fnormal))
                {
                    m = -1;
                    /*if (options.DebugEnabled)
                    {
                    draw.AddLabel(mesh.F_Barycenters.row(faceID), "utils::vector::OppositeDirectionIfTheyParallel(dnormal, fnormal)");
                    cout << "utils::vector::OppositeDirectionIfTheyParallel(dnormal, fnormal)" << endl;
                    }*/
                }


                Result.Field[0].row(faceID) = d1;
                Result.Field[1].row(faceID) = d2 * m;

                //DEBUG
                //cout << utils::vector::Angle(dnormal, fnormal)  << "   fnormal len = " << fnormal.norm() << "  dnormal len = " << dnormal.norm() << "    fnormal.dot(dnormal) = " << fnormal.dot(dnormal) << "    fnormal.cross(dnormal) = " << fnormal.cross(dnormal).sum() << endl;
                //cout << utils::vector::Angle(dnormal, fnormal) << "      SameDirection = " << utils::vector::SameDirection(dnormal, fnormal) << endl;
                /*if (faceID == 4 || faceID == 11 || faceID == 23)
                {
                P3 faceCenter = mesh.F_Barycenters.row(faceID);
                const V3& d1L = d1.transpose()*(mesh.avg_edge_length / 2);
                const V3& d2L = d2.transpose()*(mesh.avg_edge_length / 2);
                const V3& dn = F_normals.row(faceID).transpose()*(mesh.avg_edge_length );
                draw.AddLabel(faceCenter + d1L, "0");
                draw.AddLabel(faceCenter + d2L, "1");
                draw.AddLabel(faceCenter - d1L, "2");
                draw.AddLabel(faceCenter - d2L, "3");
                draw.AddLabel(faceCenter + dn, "normal");
                }*/
            }
            Result.IsFieldSorted = true;
        }
        else
        {
            resizeField(2);
            Result.IsFieldSymetric = true; // replace adding two opposite vectors:  Result.Field[2].row(faceID) = -d1;  Result.Field[3].row(faceID) = -d2 * m;
            // v0 - copy
            //Result.Field[0] = FieldRaw[0];
            //Result.Field[1] = FieldRaw[1];
            // v1 - swap - faster
            swap(Result.Field[0], FieldRaw[0]);
            swap(Result.Field[1], FieldRaw[1]);

            // v0 - copy field
            //Result.Field[2] = -FieldRaw[0];  
            //Result.Field[3] = -FieldRaw[1];
            // v1 - set flag
        }
    }
    #if NPolyVectorFieldSolverGeneral_SUPPORTED
    if (Result.SolverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField)
    {
        Result.IsFieldSorted = false;
        bool needNormalization = options.NormalizeNrosyField_sort_field_directions;
        if (needNormalization && FieldRaw_isSorted)
        {
            needNormalization = false;
            Result.IsFieldSorted = true;
        }

        // bool IsOmpEnabled;
        //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
        if (needNormalization)
        {
            resizeField(2);
            Result.IsFieldSymetric = true; // replace adding two opposite vectors:  Result.Field[2].row(faceID) = -d1;  Result.Field[3].row(faceID) = -d2 * m;
            for (int faceID = 0; faceID < F.rows(); faceID++)
            {
                const V3& d1 = FieldRaw[0].row(faceID);
                const V3& d2 = FieldRaw[1].row(faceID);
                V3 fnormal = F_normals.row(faceID);
                V3 dnormal = d1.cross(d2);

                int m = 1;
                // we must to fix second field-direction to make field direction clock-wise p1->p2
                // this is important for singularity detection and for speed improvement for Divider
                if (utils::vector::OppositeDirectionIfTheyParallel(dnormal, fnormal))
                {
                    m = -1;
                    /*if (options.DebugEnabled)
                    {
                        draw.AddLabel(mesh.F_Barycenters.row(faceID), "utils::vector::OppositeDirectionIfTheyParallel(dnormal, fnormal)");
                        cout << "utils::vector::OppositeDirectionIfTheyParallel(dnormal, fnormal)" << endl;
                    }*/
                }


                Result.Field[0].row(faceID) = d1;
                Result.Field[1].row(faceID) = d2 * m;

                //DEBUG
                //cout << utils::vector::Angle(dnormal, fnormal)  << "   fnormal len = " << fnormal.norm() << "  dnormal len = " << dnormal.norm() << "    fnormal.dot(dnormal) = " << fnormal.dot(dnormal) << "    fnormal.cross(dnormal) = " << fnormal.cross(dnormal).sum() << endl;
                //cout << utils::vector::Angle(dnormal, fnormal) << "      SameDirection = " << utils::vector::SameDirection(dnormal, fnormal) << endl;
                /*if (faceID == 4 || faceID == 11 || faceID == 23)
                {
                    P3 faceCenter = mesh.F_Barycenters.row(faceID);
                    const V3& d1L = d1.transpose()*(mesh.avg_edge_length / 2);
                    const V3& d2L = d2.transpose()*(mesh.avg_edge_length / 2);
                    const V3& dn = F_normals.row(faceID).transpose()*(mesh.avg_edge_length );
                    draw.AddLabel(faceCenter + d1L, "0");
                    draw.AddLabel(faceCenter + d2L, "1");
                    draw.AddLabel(faceCenter - d1L, "2");
                    draw.AddLabel(faceCenter - d2L, "3");
                    draw.AddLabel(faceCenter + dn, "normal");
                }*/
            }
            Result.IsFieldSorted = true;
        }
        else
        {
            resizeField(2);
            Result.IsFieldSymetric = true; // replace adding two opposite vectors:  Result.Field[2] = -FieldRaw[0];  Result.Field[3] = -FieldRaw[1];
            Result.Field[0] = FieldRaw[0];
            Result.Field[1] = FieldRaw[1];
        }
    }
    else if (Result.SolverType == MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorFieldGeneral)
    {
        resizeField(4);
        Result.IsFieldSorted = false;
        if (FieldRaw.size() == 2) // can happend when ignore_x options is enabled
        {
            Result.Field[0] = FieldRaw[1];
            Result.Field[1] = -FieldRaw[0];
            Result.Field[2] = -FieldRaw[1];
            Result.Field[3] = FieldRaw[0];
        }
        else
        {
            Result.Field[0] = FieldRaw[1];
            Result.Field[1] = FieldRaw[2];
            Result.Field[2] = FieldRaw[3];
            Result.Field[3] = FieldRaw[0];
        }
    }
    #endif
    #if Comiso_SUPPORTED
    else if (Result.SolverType == MeshLogicOptions_SolverNrosy::MeshSolverType::FrameField)
    {
        resizeField(2);
        Result.IsFieldSymetric = true;
        Result.IsFieldSorted = true;
        for (int fid = 0; fid < F.rows(); fid++)
        {
            V3 direction = FieldRaw[0].row(fid);
            Result.Field[0].row(fid) = direction;
            V3 directionRotated = utils::vector::Rotate(direction, mesh.F_Z.row(fid), 90, true);
            Result.Field[1].row(fid) = directionRotated;
        }
    }
    else
    {
        resizeField(2);
        Result.IsFieldSymetric = true;
        Result.IsFieldSorted = true;
        // Result.SolverType == one of these: 
        //     SimplicialLDLT,
        //      gurobi_rounding,  //solve_gurobi(_A, _x, _rhs, _to_round);
        //      cplex_rounding,    //solve_cplex(_A, _x, _rhs, _to_round);
        //      no_rounding,       //solve_no_rounding(_A, _x, _rhs);
        //      direct_rounding,    //solve_direct_rounding( _A, _x, _rhs, _to_round);
        //      multiple_rounding, //solve_multiple_rounding(_A, _x, _rhs, _to_round);
        //      iterative,                 //solve_iterative(_A, _x, _rhs, _to_round, _fixed_order);
         // 
         //

        //v0
        for (int fid = 0; fid < F.rows(); fid++)
        {
            V3 direction = FieldRaw[0].row(fid);
            Result.Field[0].row(fid) = direction;
            V3 directionRotated = utils::vector::Rotate(direction, mesh.F_Z.row(fid), 90, true);
            Result.Field[1].row(fid) = directionRotated;
        }

        // v1
       //for (int fid = 0; fid < F.rows(); fid++)
       // {
       //    V3 direction = FieldRaw[0].row(fid);
       //    direction.normalize();
       //     const V3& b1 = mesh.F_X.row(fid);
       //     const V3& b2 = mesh.F_Y.row(fid);
       //     D x = direction.dot(b1);
       //     D y = direction.dot(b2);
       //     D angle = atan2(y, x);
       //     //D angle2 = y;
       //     //cout << "angle=" << angle << "    angle2=" << angle2 << endl;
       //     for (int ni = 0; ni < 2; ++ni)
       //     {
       //         D anglej = angle + M_PI*D(ni) / D(2);
       //         D xj = cos(anglej);
       //         D yj = sin(anglej);
       //         direction = xj * b1 + yj * b2;
       //         //direction.normalize();
       //         Result.Field[ni].row(fid) = direction;
       //     }
       // }

        //v2
        //V3 basedirection = Result.Field[0].row(0);
        //vector<V3s> fields = Result.Field;
        //for (int fid = 0; fid < fields[0].rows(); fid++)
        //{
        //    V3 direction = fields[0].row(fid);
        //    direction.normalize();
        //    V3 direction2 = fields[1].row(fid);
        //    V3 normal =direction.normalized().cross(direction2.normalized());
        //    normal.normalize();
        //    D angle = utils::vector::Angle(direction.normalized(), basedirection.normalized());// utils::vector::AngleFull(direction, basedirection, normal);
        //    if (angle < -45) angle = 360 + angle;
        //    for (int k = 0; k <= 4; k++)
        //        if (angle > 45 || angle < -45)
        //        {
        //            direction = utils::vector::Rotate(direction, normal, 90, true);
        //            //direction = normal.cross(direction);
        //            angle = utils::vector::Angle(direction.normalized(), basedirection.normalized());
        //        }
        //    //cout << "angle=" << angle << endl;
        //    fields[0].row(fid) = direction;
        //    V3 directionRotated = utils::vector::Rotate(direction, normal, 90, true);
        //    fields[1].row(fid) = directionRotated;            
        //}
        //Result.Field = fields;
        //doFixDirectionsOnConstrainedFaces = false;
    }
    #endif

    if (meshLogicOptions.SolverAngleBound.Enabled)
    {
        doRollbackDirectionsOnConstrainedFaces = false;
        doHealDirectionsOnConstrainedFaces = true;
    }

    //
    // Rollback directions on constrained faces - this is very important for dividing, since we take solver field to divide meshes
    //
    if (doRollbackDirectionsOnConstrainedFaces)
    {
        for (int i = 0; i < Constrains.Constrains.size(); i++) // for all constrained faces
        {
            const FaceConstrain& fc = Constrains.Constrains[i];
            int fid = fc.FaceId;
            const V3& constrainX = fc.DirectionX;
            const V3& constrainY = fc.DirectionY_Corrected;

            // v0 - force field same as constrains
            Result.Field[0].row(fid) = constrainX;
            Result.Field[1].row(fid) = constrainY;
            if (Result.Field.size() > 2)
            {
                Result.Field[2].row(fid) = -constrainX;
                Result.Field[3].row(fid) = -constrainY;
            }

            //v1 - use exactly as solver field
            //const V3& dx = Result.Field[0].row(fid);
            //const V3& dy = Result.Field[1].row(fid);
            //// ensure 'dy' goes same direction as 'constrainX'
            //D cosXX = abs(utils::vector::Cos(constrainX, dx));
            //D cosXY = abs(utils::vector::Cos(constrainX, dy));
            //if (cosXX < cosXY)
            //{
            //    draw.AddLabel(mesh.EdgeMiddlePoint(fc.EdgeId), "swapped x and y", Color3d(1, 0, 0), 4);
            //    //exchange x and y
            //    V3 x = Result.Field[0].row(fid);
            //    V3 y = Result.Field[1].row(fid);
            //    Result.Field[0].row(fid) = y;
            //    Result.Field[1].row(fid) = x;
            //    if (Result.Field.size() > 2)
            //    {
            //        x = Result.Field[2].row(fid);
            //        y = Result.Field[3].row(fid);
            //        Result.Field[2].row(fid) = y;
            //        Result.Field[3].row(fid) = x;
            //    }
            //}

        }
    }

    //
    // Rollback directions on constrained faces - this is very important for dividing, since we take solver field to divide meshes
    //
    if (doHealDirectionsOnConstrainedFaces)
    {
        bool isFieldSymetric = Result.IsFieldSymetric;
        int kmax = (isFieldSymetric ? 2 : 4);
        for (int i = 0; i < Constrains.Constrains.size(); i++) // for all constrained faces
        {
            const FaceConstrain& fc = Constrains.Constrains[i];
            int fid = fc.FaceId;
            //const V3& constrainX = fc.DirectionX;
            V3 constrainY = fc.DirectionY_Corrected;
            //constrainY.normalize(); - no need since we project all dir on same vector, so proportion will be same
            V3 field[4];
            D coss[4];
            for (int k = 0; k < kmax; k++)
            {
                V3 dir = Result.Field[k].row(fid);
                field[k] = dir;
                dir.normalize();
                coss[k] = utils::vector::Dot(constrainY, dir);
            }
            if (isFieldSymetric)
            {
                field[2] = -field[0];
                field[3] = -field[1];
                coss[2] = -coss[0];
                coss[3] = -coss[1];
            }
            int bestIndex = 0;
            if (coss[1] > coss[bestIndex]) bestIndex = 1;
            if (coss[2] > coss[bestIndex]) bestIndex = 2;
            if (coss[3] > coss[bestIndex]) bestIndex = 3;

            // if field requires change indexing
            if (bestIndex != 1)
            {
                int shift = bestIndex - 1;
                if (shift < 0) shift += 4;
                for (int k = 0; k < kmax; k++)
                {
                    int k_new = (k + shift) % 4;
                    Result.Field[k].row(fid) = field[k_new];
                }
            }
        }
    }

}

void MeshSolverNrosy::Clear()
{
    Constrains.Clear();
    Result.Clear();
}

II MeshSolverNrosy::SizeOF() const
{
    II r = sizeof(MeshSolverNrosy);

    r += Constrains.SizeOF();

    for (int ni = 0; ni < Result.Field.size(); ni++)
    {
        r += Result.Field[ni].size() * sizeof(D);
    }
    r += Result.Field_match_ab_ba.size() * sizeof(MatchABBA);
    r += Result.Singularities.size() * sizeof(D);

    return r;
}