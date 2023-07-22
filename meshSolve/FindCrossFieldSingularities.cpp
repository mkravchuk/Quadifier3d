#include "stdafx.h"
#include <igl/compute_frame_field_bisectors.h>
#include <igl/cross_field_missmatch.h>
#include <igl/find_cross_field_singularities.h>
//#include <igl/rotation_matrix_from_directions.h>
#include <igl/comb_cross_field.h>
#include <deque>
#include "FindCrossFieldSingularities.h"
#include "CompactVectorVector.h"
#include <igl/rotation_matrix_from_directions.h>


FindCrossFieldSingularities::FindCrossFieldSingularities(ViewerDrawObjects& _draw, const P3s& _V, const I3s& _F,
    const V3s& _F_X, const V3s& _F_Y, const V3s& _F_normals, const P3s& _F_Barycenters,
    const I3s& _FE, const I2s& _EV, const I2s& _EF, const I3s& _FF,
    const Bs&  _V_border, const CompactVectorVector<int>& _VF)
    : draw(_draw), V(_V), F(_F),
    F_X(_F_X), F_Y(_F_Y), F_normals(_F_normals), F_Barycenters(_F_Barycenters),
    FE(_FE), EV(_EV), EF(_EF), FF(_FF),
    V_border(_V_border), VF(_VF)
{
}


#ifdef FULL_FindCrossFieldSingularities_IS_AVAILABLE

int FindCrossFieldSingularities::find_cross_field_singularities_full(const MatrixXi& Handle_MMatch, Ds& Result_Singularities)
{
    int singcount = 0;

    Result_Singularities.setZero(V.rows(), 1);
    // DEBUG show numbers
    //cout << endl;
    extern bool IsOmpEnabled;
    #pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int vid = 0; vid < V.rows(); vid++)
    {
        if (V_border[vid])
            continue;

        int missmatchNoTranc = 0;
        bool found = false;
        for (int i = 0; i < VF.size(vid); i++)
        {
            int fid = VF(vid, i);
            // look for the vertex
            int j = -1;
            for (int z = 0; z < 3; ++z)
                if (F(fid, z) == vid)
                    j = z;
            assert(j != -1);

            missmatchNoTranc += Handle_MMatch(fid, j);
            found = true;
        }
        int missmatch = missmatchNoTranc % 4;

        if (found
            && missmatch != 0
            && missmatch != 2 // filter 5-singularity when numbers are: 2,6,10
            )
        {
            Result_Singularities(vid) = (missmatch == 1) ? -1 : 1;
            singcount++;
            // DEBUG show numbers
            /*if (Result_Singularities(vid) != 0)
            {
                cout << endl <<  "   singularity "<< ((Result_Singularities(vid)<0) ? 3:5) << "     missmatchNoTranc = " << missmatchNoTranc << ":" << missmatch << "  at vertex " << vid << endl;
            }*/

            // DEBUG show Handle_MMatch
            //for (int i = 0; i < VF[vid].size(); i++)
            //{
            //    int fid = VF[vid][i];
            //    // look for the vertex
            //    int j = -1;
            //    for (int z = 0; z < 3; ++z)
            //        if (F(fid, z) == vid)
            //            j = z;
            //    assert(j != -1);

            //    missmatchNoTranc += Handle_MMatch(fid, j);
            //    cout << "fid = " << fid << "       " << Handle_MMatch(fid, j) << endl;
            //}
        }
    }
    return singcount;
}

inline int FindCrossFieldSingularities::calculateMissmatch_MissMatchByCross(const int f0, const int f1, const V3s& PD1, const V3s& PD2)
{
    //V3 dir0 = PD1.row(f0);
    V3 dir1 = PD1.row(f1);
    V3 n0 = F_normals.row(f0);
    V3 n1 = F_normals.row(f1);

    D33 rotM;
    utils::vector::rotation_matrix_from_directions(n1, n0, rotM);
    V3 dir1onF0 = rotM * dir1;
    dir1onF0.normalize();


    D angle_diff = atan2(dir1onF0.dot(PD2.row(f0)), dir1onF0.dot(PD1.row(f0)));
    //D angle_diff = utils::vector::AngleInRadians(PD1.row(f0), dir1onF0);

    //    std::cerr << "Dani: " << dir0(0) << " " << dir0(1) << " " << dir0(2) << " " << dir1Rot(0) << " " << dir1Rot(1) << " " << dir1Rot(2) << " " << angle_diff << std::endl;

    D step = M_PI / 2.0;
    int i = static_cast<int>(std::floor((angle_diff / step) + 0.5));
    int k = 0;
    if (i >= 0)
        k = i % 4;
    else
        k = (-(3 * i)) % 4;
    return k;
}

void FindCrossFieldSingularities::calculateMissmatch_full(const V3s& BIS1, const V3s& BIS2, MatrixXi &Handle_MMatch)
{
    Handle_MMatch.setZero(F.rows(), 3);
    extern bool IsOmpEnabled;
    #pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int fi = 0; fi < F.rows(); fi++)
    {
        for (int ei = 0; ei < 3; ei++)
        {
            int fiOposite = TT(fi, ei);
            if (fi != fiOposite && fiOposite != -1)
            {
                Handle_MMatch(fi, ei) = calculateMissmatch_MissMatchByCross(fi, fiOposite, BIS1, BIS2);
            }
        }
    }
}

static inline D Sign(D a)
{
    return static_cast<D>((a > 0) ? +1 : -1);
}
static inline V3 K_PI_new(const V3& a, const V3& b, const V3& n)
{
    V3 c = (a.cross(n)).normalized();
    D scorea = a.dot(b);
    D scorec = c.dot(b);
    if (fabs(scorea) >= fabs(scorec))
        return a * Sign(scorea);
    else
        return c * Sign(scorec);
}

bool FindCrossFieldSingularities::comb_cross_field_full(const V3s& PD1, const V3s& PD2, MatrixXd& PD1out, MatrixXd& PD2out)
{
    //      PD1out = PD1;
    //      PD2out = PD2;
    PD1out.setZero(F.rows(), 3); PD1out << PD1;
    PD2out.setZero(F.rows(), 3); PD2out << PD2;

    Bs mark = Bs::Constant(F.rows(), false);

    deque<int> d;

    // everything should be marked
    for (int i = 0; i < F.rows(); i++)
    {
        if (!mark(i))
        {
            d.push_back(i);
            mark(i) = true;

            while (!d.empty())
            {
                int f0 = d.at(0);
                //DEBUG
                //draw.AddLabel(F_Barycenters.row(f0).transpose(), to_string(f0), Color4d(1, 0, 0, 1));
                d.pop_front();
                for (int k = 0; k < 3; k++)
                {
                    int f1 = TT(f0, k);
                    if (f1 == -1) continue;
                    if (mark(f1)) continue;

                    V3 dir0 = PD1out.row(f0);
                    V3 dir1 = PD1out.row(f1);
                    V3 n0 = F_normals.row(f0);
                    V3 n1 = F_normals.row(f1);

                    D33 rotM;
                    utils::vector::rotation_matrix_from_directions(n0, n1, rotM);
                    V3 dir0Rot = rotM * dir0;
                    dir0Rot.normalize();
                    V3 targD = K_PI_new(dir1, dir0Rot, n1);

                    PD1out.row(f1) = targD;
                    PD2out.row(f1) = n1.cross(targD).normalized();

                    mark(f1) = true;
                    d.push_back(f1);
                }
            }
        }
    }
    // everything should be marked
    for (int i = 0; i < F.rows(); i++)
    {
        if (!mark(i))
        {
            cout << endl << endl << "!!! comb_cross_field_fast  failed" << endl << endl << endl;
            return false;
        }
    }
    return true;
}

int FindCrossFieldSingularities::findCrossFieldSingularities_full(const V3s& FF1, const V3s& FF2, Ds& Result_Singularities, bool log)
{
    auto time = utils::time::Now();

    //
    // BIS1, BIS2
    //
    if (log) cout << "   findCrossFieldSingularities   compute_frame_field_bisectors ...";
    MatrixXd BIS1, BIS2;
    igl::compute_frame_field_bisectors(V, F, F_X, F_Y, FF1, FF2, BIS1, BIS2);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;

    // this fails sometimes - but we dont need it - i dont know why we can work without calling "comb"
    bool useBcombed = false;
    //cout << "   findCrossFieldSingularities   comb_cross_field ..."; sw.restart();
    MatrixXd BIS1_combed, BIS2_combed;
    //igl::comb_cross_field(V, F, BIS1, BIS2, BIS1_combed, BIS2_combed);
    //useBcombed = comb_cross_field_full(mesh, BIS1, BIS2, BIS1_combed, BIS2_combed);
    //cout << " done in " << sw.restart() / 1000.0 << endl;

    //
    // Handle_MMatch
    // 
    if (log) cout << "   findCrossFieldSingularities   calculateMissmatch ...";
    MatrixXi Handle_MMatch;
    //igl::cross_field_missmatch(V, F, BIS1, BIS2, true, Handle_MMatch);
    if (useBcombed)
    {
        calculateMissmatch_full(BIS1_combed, BIS2_combed, Handle_MMatch);
    }
    else
    {
        calculateMissmatch_full(BIS1, BIS2, Handle_MMatch);
    }
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;

    //
    // Result_Singularities
    //
    //Matrix<int, Dynamic, 1> isSingularity, singularityIndex;
    //igl::find_cross_field_singularities(V, F, Handle_MMatch, isSingularity, singularityIndex);
    //Result_Singularities.setZero(isSingularity.rows());
    //for (int i = 0; i < isSingularity.rows(); i++)
    //{
    //    if (isSingularity(i))
    //    {
    //         //singularityIndex(i) == 1  - singularity 3 directions
    //         //singularityIndex(i) == 3  - singularity 5 directions
    //        Result_Singularities(i) = (singularityIndex(i) == 1) ? -1 : 1;
    //        cout << endl << "singularity " << isSingularity(i) << ":" << singularityIndex(i) << "  at vertex " << i << endl;
    //    }
    //}
    if (log) cout << "   findCrossFieldSingularities   find ...";
    int singcount = find_cross_field_singularities_full(Handle_MMatch, Result_Singularities);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;

    return singcount;
}
#endif

//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------


int FindCrossFieldSingularities::find_cross_field_singularities_fast(const vector<bool>& vertexesToCheck, const MatrixXi& Handle_MMatch, Ds& Result_Singularities)
{
    int singcount = 0;

    Result_Singularities.setZero(V.rows());
    //extern bool IsOmpEnabled;
//#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int vid = 0; vid < V.rows(); vid++)
    {
        if (!vertexesToCheck[vid])
            continue;

        int missmatchNoTranc = 0;
        bool found = false;
        for (int i = 0; i < VF.size(vid); i++)
        {
            int fid = VF(vid, i);
            // look for the vertex
            int j = -1;
            for (int z = 0; z < 3; ++z)
                if (F(fid, z) == vid)
                    j = z;
            assert(j != -1);

            missmatchNoTranc += Handle_MMatch(fid, j);
            found = true;
        }
        int missmatch = missmatchNoTranc % 4;

        if (found
            && missmatch != 0
            && missmatch != 2 // filter 5-singularity when numbers are: 2,6,10
            )
        {
            Result_Singularities(vid) = (D)((missmatch == 1) ? -1 : 1);
            singcount++;
            // DEBUG show numbers
            /*if (Result_Singularities(vid) != 0)
            {
            cout << endl <<  "   singularity "<< ((Result_Singularities(vid)<0) ? 3:5) << "     missmatchNoTranc = " << missmatchNoTranc << ":" << missmatch << "  at vertex " << vid << endl;
            }*/

            // DEBUG show Handle_MMatch
            //for (int i = 0; i < VF[vid].size(); i++)
            //{
            //    int fid = VF[vid][i];
            //    // look for the vertex
            //    int j = -1;
            //    for (int z = 0; z < 3; ++z)
            //        if (F(fid, z) == vid)
            //            j = z;
            //    assert(j != -1);

            //    missmatchNoTranc += Handle_MMatch(fid, j);
            //    cout << "fid = " << fid << "       " << Handle_MMatch(fid, j) << endl;
            //}
        }
    }
    return singcount;
}

inline int calculateMissmatch_MissMatchByCross_fast(const int f0, const int f1, const D33& transformationMatrix, const V3s& PD1, const V3s& PD2)
{
    V3 dir1 = PD1.row(f1);

    // v1 - origin
    //V3 n0 = mesh.F_normals.row(f0);
    //V3 n1 = mesh.F_normals.row(f1);
    //V3 dir1onF0 = igl::rotation_matrix_from_directions(n1, n0)*dir1;
    // v2 - fast - using cached transformation matrix
    V3 dir1onF0 = transformationMatrix * dir1;

    dir1onF0.normalize();

    D angle_diff = atan2(dir1onF0.dot(PD2.row(f0)), dir1onF0.dot(PD1.row(f0)));
    //D angle_diff2 = utils::vector::AngleInRadians(dir1onF0, PD1.row(f0));
    //cout << "angle_diff = " << angle_diff << "     angle_diff 2 = " << angle_diff2 << endl;

    D step = (D)(M_PI / 2.0);
    int i = static_cast<int>(std::floor((angle_diff / step) + 0.5f));
    int k = 0;
    if (i >= 0)
        k = i % 4;
    else
        k = (-(3 * i)) % 4;
    return k;
}

void FindCrossFieldSingularities::calculateMissmatch_fast(
    const vector<bool>& checkForFaces, const vector<int>& edges_Ids, const vector<D33>& edges_transformationMatrixes,
    const V3s& BIS1, const V3s& BIS2, MatrixXi &Handle_MMatch) //, MatrixXi &Handle_MMatchFull
{


    Handle_MMatch.setZero(F.rows(), 3);
    //extern bool IsOmpEnabled;
//#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int i = 0; i < edges_Ids.size(); i++)
    {
        int eid = edges_Ids[i];
        int f0 = EF(eid, 0);
        int f1 = EF(eid, 1);
        const D33& t = edges_transformationMatrixes[i];
        if (f1 < f0) // transformation matrix will always from smaller faceId to bigger faceId
        {
            swap(f0, f1);
        }
        int f0edgeIndex = -1;
        for (int ei = 0; ei < 3; ei++) if (FE(f0, ei) == eid) f0edgeIndex = ei;
        assert(f0edgeIndex != -1);

        int f1edgeIndex = -1;
        for (int ei = 0; ei < 3; ei++) if (FE(f1, ei) == eid) f1edgeIndex = ei;
        assert(f1edgeIndex != -1);

        Handle_MMatch(f0, f0edgeIndex) = calculateMissmatch_MissMatchByCross_fast(f0, f1, t.transpose(), BIS1, BIS2);
        Handle_MMatch(f1, f1edgeIndex) = calculateMissmatch_MissMatchByCross_fast(f1, f0, t, BIS1, BIS2);

        //DEBUG
        //if (Handle_MMatch(f0, f0edgeIndex) != Handle_MMatchFull(f0, f0edgeIndex))
        //{
        //    cout << "Face " << f0 << " edge " << f0edgeIndex << "  Handle_MMatch  " << Handle_MMatch(f0, f0edgeIndex) << "  but has to be  " << Handle_MMatchFull(f0, f0edgeIndex) << endl;
        //}
        //if (Handle_MMatch(f1, f1edgeIndex) != Handle_MMatchFull(f1, f1edgeIndex))
        //{
        //    cout << "Face " << f1 << " edge " << f1edgeIndex << "  Handle_MMatch  " << Handle_MMatch(f1, f1edgeIndex) << "  but has to be  " << Handle_MMatchFull(f1, f1edgeIndex) << endl;
        //}
    }
}


int FindCrossFieldSingularities::getFacesWithWeakWeight(const V3s& FF1, const V3s& FF2, Vector<bool>& facesWithWeakWeight, bool f1andf2_isAlmostSameLength)
{
    int count = 0;
    if (f1andf2_isAlmostSameLength)
    {
        // v0
        //facesWithWeakWeight.setConstant(F.rows(), false);
        //for (int fi = 0; fi < F.rows(); fi++)
        //{
        //    D FF1squaredNorm = FF1.row(fi).squaredNorm();
        //    D FF2squaredNorm = FF2.row(fi).squaredNorm();
        //    if (FF1squaredNorm < 0.9*0.9 || FF2squaredNorm < 0.9*0.9)
        //    {
        //        facesWithWeakWeight[fi] = true;
        //        count++;
        //    }
        //}

        // v1 unrolled - just a little faster, so is no usefull
        facesWithWeakWeight.resize(F.rows());
        const V3* pFF1 = FF1.data();
        const V3* pFF2 = FF2.data();
        auto isFF1orFF2Weak = [&](int index) -> int
        {
            int isWeak = ((int)((*(pFF1+ index)).squaredNorm() < 0.9*0.9  || (*(pFF2 + index)).squaredNorm() < 0.9*0.9));
            return isWeak;
        };
        int* pIsWeak = facesWithWeakWeight.data();
        for (int i = 0; i < F.rows() / boolBitsCount; i++)
        {
            int boolBits = 0;
            constexpr int UNROLL_COUNT = 4;
            for (int k = 0; k < boolBitsCount / UNROLL_COUNT; k++)
            {
                int shift = k * UNROLL_COUNT;
                CommonClasses::IntBits(boolBits, shift+0) = isFF1orFF2Weak(shift + 0);
                CommonClasses::IntBits(boolBits, shift + 1) = isFF1orFF2Weak(shift + 1);
                CommonClasses::IntBits(boolBits, shift + 2) = isFF1orFF2Weak(shift + 2);
                CommonClasses::IntBits(boolBits, shift + 3) = isFF1orFF2Weak(shift + 3);
            }
            *pIsWeak = boolBits;
            pIsWeak++;
            count += utils::sse::bitsIsSetCount(boolBits);
            pFF1+= boolBitsCount;
            pFF2+= boolBitsCount;
        }

        for (int fi = F.rows()- F.rows() % boolBitsCount; fi < F.rows(); fi++)
        {
            D FF1squaredNorm = FF1.row(fi).squaredNorm();
            D FF2squaredNorm = FF2.row(fi).squaredNorm();
            bool isWeak = (FF1squaredNorm < 0.9 * 0.9 || FF2squaredNorm < 0.9*0.9);
            facesWithWeakWeight[fi] = isWeak;
            if (isWeak)
            {
                count++;
            }
        }
    }
    else
    {
        facesWithWeakWeight.setConstant(F.rows(), false);
        for (int fi = 0; fi < F.rows(); fi++)
        {
            D FF1squaredNorm = FF1.row(fi).squaredNorm();
            D FF2squaredNorm = FF2.row(fi).squaredNorm();
            //*squaredNorm = FF1squaredNorm; squaredNorm++;
            //*squaredNorm = FF2squaredNorm; squaredNorm++;

            D weight1 = sqrt(FF1squaredNorm);
            D weight2 = sqrt(FF2squaredNorm);
            D weight = (weight1 + weight2) / 2;
            if (weight < 0.9)
            {
                facesWithWeakWeight[fi] = true;
                count++;
            }
        }
    }

    //DEBUG show faces
    //for (int fi = 0; fi < F.rows(); fi++)
    //{
    //    if (facesWithWeakWeight[fi])
    //    {
    //        D weight1 = FF1.row(fi).norm();
    //        D weight2 = FF2.row(fi).norm();
    //        D weight = (weight1 + weight2) / 2;
    //        draw.AddLabel(F_Barycenters.row(fi).transpose(), to_string(weight), Color4d(1, 0, 0, 1));
    //        //draw.AddLabel(F_Barycenters.row(fi).transpose(), to_string(weight1) + ", " + to_string(weight2), Color4d(1, 0, 0, 1));
    //    }
    //    //facesCheckForSing(fi) = true;
    //}

    return count;
}

void FindCrossFieldSingularities::getVertexesToCheckForSing(const V3s& FF1, const V3s& FF2, const Vector<bool>& facesWithWeakWeight, vector<bool>& vertexesToCheck)
{
    constexpr bool SHOW_ANGLES = false;


    vertexesToCheck.resize(V.rows(), false);

    //v1 - slow
    //extern bool IsOmpEnabled;
    //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    /*for (int vid = 0; vid < V.rows(); vid++)
    {
        if (V_border[vid])
            continue;

        for (int fid : VF[vid])
        {
            if (facesWithWeakWeight[fid])
            {
                vertexesToCheck[vid] = true;
                break;
            }
        }
    }*/

    // v2 - faster
    for (int fi = 0; fi < F.rows(); fi++)
    {
        if (!facesWithWeakWeight[fi]) continue;
        if (!V_border[F(fi, 0)]) vertexesToCheck[F(fi, 0)] = true;
        if (!V_border[F(fi, 1)]) vertexesToCheck[F(fi, 1)] = true;
        if (!V_border[F(fi, 2)]) vertexesToCheck[F(fi, 2)] = true;
    }
    //DEBUG show vertexes to check
    //
    //for (int vid = 0; vid < V.rows(); vid++)
    //{
    //    if (vertexesToCheck[vid])
    //    {
    //        draw.AddPoint(V.row(vid).transpose(), Color3d(1, 0, 0));
    //    }
    //}

    // make fast additional check - changes in PP1 has to be significant to create singularity - so skipp 'stable' vertexes
    //vector<D> vertexesToCheck_AngleDiffSumm;
    vector<D> vertexesToCheck_AngleDiffMax;
    const float Cos10 = utils::angle::DegreesToCos(10);
    const float Cos20 = utils::angle::DegreesToCos(20);
    const float Cos70 = utils::angle::DegreesToCos(70);
    if (SHOW_ANGLES) vertexesToCheck_AngleDiffMax.resize(V.rows(), false);
    //extern bool IsOmpEnabled;
    //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int vid = 0; vid < V.rows(); vid++)
    {
        if (!vertexesToCheck[vid]) continue;
        int fidFirst = VF(vid, VF.size(vid) - 1);
        //if (vid == 10591)
        //{
        //    cout << "fidFirst = " << fidFirst << endl;
        //}
        V3 d1First = FF1.row(fidFirst);
        V3 d2First = FF2.row(fidFirst);
        // v0
        //d1First.normalize();
        //d2First.normalize();
        // v1 - fast
        __m128 fidFirst_squaredNorms;
        fidFirst_squaredNorms.m128_f32[0] = d1First.squaredNorm();
        fidFirst_squaredNorms.m128_f32[1] = d2First.squaredNorm();
        __m128 fidFirst_norms = _mm_sqrt_ps(fidFirst_squaredNorms);
        d1First /= fidFirst_norms.m128_f32[0];
        d2First /= fidFirst_norms.m128_f32[1];


        //dFirst /= sqrt(dFirst.squaredNorm());
        //float sqrt = _mm_sqrt_ss(_mm_set1_ps(dFirst.squaredNorm())).m128_f32[0];
        //dFirst /= sqrt;

        D angleDiffMax = 0;
        D angleDiffMinCos = 1;
        //if (vid == 82 || vid == 162) cout << "vid="<<vid << endl;
        //if (vid == 82 || vid == 162) cout << "  VF.size(vid)=" << VF.size(vid)<<"   fidPrev="<< fidFirst << endl;
        for (int i = 0; i < VF.size(vid); i++)
        {
            int fid = VF(vid, i);
            V3 d1 = FF1.row(fid);
            V3 d2 = FF2.row(fid);

            //
            // v0 - angle
            //
            //D angle1 = utils::vector::Angle(d1, d1First);
            //if (angle1 > 90) angle1 = 180 - angle1; // vectors opposite but parallel
            //D angle2 = utils::vector::Angle(d2, d1First);
            //if (angle2 > 90) angle2 = 180 - angle2; // vectors opposite but parallel
            //D angle_to_d1First = min(angle1, angle2);
            //D angle_to_d2First = 90 - max(angle1, angle2); // approximatelly estimate angle to ortogonal vector
            //D angle = max(angle_to_d1First, angle_to_d2First); // get sharpes angle - very important, because we take only 1 vector 'dFirst' but normaly we have to test 2 vectors 'd1First' and 'd2First', so to simplify we use this condition '90 - angle_max' to approximatelly estimate angle to 'd2First'
            ////if (vid == 82 || vid == 162) cout << "  fid=" << fid << "  angle=" << angle << "  angle1=" << angle1 << "  angle2=" << angle2 << endl;
            //if (angle > angleDiffMax) angleDiffMax = angle;

            //
            // v1 - cos + sse - even fasterer
            //
            __m128 squaredNorms;
            squaredNorms.m128_f32[0] = d1.squaredNorm();
            squaredNorms.m128_f32[1] = d2.squaredNorm();
            //__m128 norms = _mm_sqrt_ps(squaredNorms); // very precise and a bit slower from '_mm_rsqrt_ps'
            //d1 /= norms.m128_f32[0];
            //d2 /= norms.m128_f32[1];
            __m128 norms_reversed = _mm_rsqrt_ps(squaredNorms); // not so prcise but alittle faster - we dont need here high precision
            d1 *= norms_reversed.m128_f32[0];
            d2 *= norms_reversed.m128_f32[1];
            D angle1cos2 = abs(utils::vector::Dot(d1, d1First));
            D angle2cos2 = abs(utils::vector::Dot(d2, d1First));
            D anglecos = max(angle1cos2, angle2cos2);
            if (anglecos < angleDiffMinCos) angleDiffMinCos = anglecos;
            D d1d2cos = abs(utils::vector::Dot(d1, d2));
            if (d1d2cos > Cos70) // d1 and d2 around singular points are sligthly no ortogonal, so if angle bettwen d1 and d2  < 70 degree, then we have to check vertex for singularity
            {
                angleDiffMinCos = min(angleDiffMinCos, d1d2cos);
            }
            ////cout << "dFirst.norm = " << dFirst.norm() << "   d1.norm = " << d1.norm() << "   d2.norm = " << d2.norm() << endl;
            //cout << "angle_to_d1First = " << angle_to_d1First << "   anglecos = " << utils::angle::RadiansToDegrees(acos(anglecos_to_d1First)) << "   angle_to_d2First = " << angle_to_d2First << "   anglecos = " << utils::angle::RadiansToDegrees(acos(anglecos_to_d2First)) << endl;

        }

        // remove vertexes from check if max angle diff is small between faces
        //
        // v0
        //
        //if (angleDiffMax < 20)
        //{
        //    vertexesToCheck[vid] = false;
        //}
        //if (SHOW_ANGLES) vertexesToCheck_AngleDiffMax[vid] = angleDiffMax;

        //
        //v1
        //
        if (angleDiffMinCos > Cos20)
        {
            vertexesToCheck[vid] = false;
        }
        if (SHOW_ANGLES) vertexesToCheck_AngleDiffMax[vid] = utils::angle::RadiansToDegrees(acos(angleDiffMinCos));

        //bool cos_works_correct = (angleDiffMax < 20) == (angleDiffMinCos > Cos20);
        //if (!cos_works_correct)
        {
            //cout << "! warning   cos_works_correct==failed!   angleDiffMax=" << angleDiffMax << "     angleDiffMinCos=" << angleDiffMinCos << "(" << utils::angle::RadiansToDegrees(acos(angleDiffMinCos)) << ")   Cos20=" << Cos20 << "(20)" << endl;
            //cout << "! warning   cos_works_correct==failed!   angleDiffMax=" << angleDiffMax 
            //<< "     angleDiffMinCos="  << utils::angle::RadiansToDegrees(acos(angleDiffMinCos)) << endl;
        }


    }

    //DEBUG show vertexes to check after small-angle-filtering
    if (SHOW_ANGLES)
    {
        for (int vid = 0; vid < V.rows(); vid++)
        {
            if (vertexesToCheck[vid])
            {
                //draw.AddLabel(V.row(vid).transpose(), to_string(round(vertexesToCheck_AngleDiffSumm[vid])), Color4d(1, 0, 0, 1));
                draw.AddLabel(V.row(vid).transpose(), "    " + to_string(round(vertexesToCheck_AngleDiffMax[vid])), Color4d(1, 0, 0, 1));
            }
        }
    }
}

void FindCrossFieldSingularities::cacheTransformationMatrixes(const vector<bool>& vertexesToCheck, vector<bool>& checkForFaces, vector<int>& edges_Ids, vector<D33>& edges_transformationMatrixes)
{


    // get faces for which we have to cache transformation matrix   (all faces contacted to vertexesToCheck)
    // cannot run parallel, coz vertexes share same faces and we store result base on face index
    checkForFaces.resize(F.rows(), false);
    for (int vid = 0; vid < V.rows(); vid++)
    {
        if (!vertexesToCheck[vid]) continue;
        for (int i = 0; i < VF.size(vid); i++)
        {
            int fid = VF(vid, i);
            checkForFaces[fid] = true;
        }
    }

    //DEBUG
    //
    //for (int fid = 0; fid < F.rows(); fid++)
    //{
    //    if (checkForFaces[fid])
    //    {
    //        draw.AddPoint(F_Barycenters.row(fid).transpose(), Color3d(1, 0, 0));
    //    }
    //}

    // check for what edges we need to cache transformation matrix   (every edge that has 2 faces with weak weight)
    // cannot run parallel, coz faces share same edges and we store result base on edge index
    Vector<bool> checkForEdges;
    checkForEdges.setConstant(EV.rows(), false);
    for (int fid = 0; fid < F.rows(); fid++)
    {
        if (!checkForFaces[fid]) continue;
        for (int ei = 0; ei < 3; ei++)
        {
            int fidOposite = FF(fid, ei);
            if (fidOposite != -1 && checkForFaces[fidOposite])
            {
                int eid = FE(fid, ei);
                checkForEdges[eid] = true;
            }
        }
    }
    int checkForEdgesCount = checkForEdges.countOfValues(true);// static_cast<int>(count(checkForEdges.begin(), checkForEdges.end(), true));

    // create new list to make parallel run faster (since if we will allocate some faces for which we dont have to check - then some CPU core will be idle)
    int checkForEdgesIDsIndex = 0;
    edges_Ids.resize(checkForEdgesCount);
    for (int eid = 0; eid < EV.rows(); eid++)
    {
        if (!checkForEdges[eid]) continue;
        edges_Ids[checkForEdgesIDsIndex] = eid;
        checkForEdgesIDsIndex++;
    }

    //DEBUG - show edges for cache transformation matrix
    //for (int eid : edges_Ids)
    //{
    //    draw.AddEdge(V.row(EV(eid, 0)), V.row(EV(eid, 1)), Color3d(1, 0, 0));
    //}

    // cache transformation matrix
    edges_transformationMatrixes.resize(checkForEdgesCount);
    //extern bool IsOmpEnabled;
//#pragma omp parallel for schedule(static) if(IsOmpEnabled)
    for (int i = 0; i < checkForEdgesCount; i++)
    {
        int eid = edges_Ids[i];
        int f0 = EF(eid, 0);
        int f1 = EF(eid, 1);
        if (f1 < f0) // transformation matrix will always from smaller faceId to bigger faceId
        {
            swap(f1, f0);
        }

        V3 n0 = F_normals.row(f0);
        V3 n1 = F_normals.row(f1);

        // v0 - eigen
        //Matrix3f rotMEigen = igl::rotation_matrix_from_directions(convertV3ToEigen(n0), convertV3ToEigen(n1));
        //Vector3f row0 = rotMEigen.row(0);
        //Vector3f row1 = rotMEigen.row(1);
        //Vector3f row2 = rotMEigen.row(2);
        //edges_transformationMatrixes[i].row(0) = convertEigenToV3(row0);
        //edges_transformationMatrixes[i].row(1) = convertEigenToV3(row1);
        //edges_transformationMatrixes[i].row(2) = convertEigenToV3(row2);

        // v1 - V3
        if (utils::cpu::isSupportedSSE4)
        {
            utils::vector::rotation_matrix_from_directions_sse41(n0, n1, edges_transformationMatrixes[i], true);
        }
        else
        {
            utils::vector::rotation_matrix_from_directions(n0, n1, edges_transformationMatrixes[i], true);
        }
    }
}

int FindCrossFieldSingularities::findCrossFieldSingularities_fast(const V3s& FF1, const V3s& FF2, Ds& Result_Singularities, bool log, bool f1andf2_isAlmostSameLength)
{
    auto time = utils::time::Now();
    //do calculation only on dots which have at leat one trinagle with weight <0.9
    if (log) cout << "   findCrossFieldSingularities   getFacesWithWeakWeight ...";
    Vector<bool> facesWithWeakWeight;
    int weakCount = getFacesWithWeakWeight(FF1, FF2, facesWithWeakWeight, f1andf2_isAlmostSameLength);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    if (weakCount == 0)
    {
        Result_Singularities.setZero(V.rows());
        return 0;
    }

    if (log) cout << "   findCrossFieldSingularities   getVertexesToCheckForSing ...";
    vector<bool> vertexesToCheck;
    getVertexesToCheckForSing(FF1, FF2, facesWithWeakWeight, vertexesToCheck);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    if (log) cout << "      checking  " << count(vertexesToCheck.begin(), vertexesToCheck.end(), true) << "  vertexes from  " << V.rows() << "  total" << endl;

    if (log) cout << "   findCrossFieldSingularities   cacheTransformationMatrixes ...";
    vector<bool> checkForFaces;
    vector<int> edges_Ids;
    vector<D33> edges_transformationMatrixes;
    cacheTransformationMatrixes(vertexesToCheck, checkForFaces, edges_Ids, edges_transformationMatrixes);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;


    //
    // BIS1, BIS2
    //
    if (log) cout << "   findCrossFieldSingularities   compute_frame_field_bisectors ...";
    V3s BIS1, BIS2;
    utils::mesh::compute_frame_field_bisectors(checkForFaces, F_X, F_Y, FF1, FF2, BIS1, BIS2);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;

    //
    // Handle_MMatch
    // 
    if (log) cout << "   findCrossFieldSingularities   calculateMissmatch ...";
    //MatrixXi Handle_MMatchFull;
    //calculateMissmatch_full(mesh, BIS1, BIS2, Handle_MMatchFull);
    MatrixXi Handle_MMatch;
    //calculateMissmatch_full(mesh, BIS1, BIS2, Handle_MMatch);
    calculateMissmatch_fast(checkForFaces, edges_Ids, edges_transformationMatrixes, BIS1, BIS2, Handle_MMatch); //, Handle_MMatchFull
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;

    //
    // Result_Singularities
    //
    if (log) cout << "   findCrossFieldSingularities   find ...";
    int singCount = find_cross_field_singularities_fast(vertexesToCheck, Handle_MMatch, Result_Singularities);
    if (log) cout << " done in " << utils::time::ElapsedSecondsStr(time) << endl;
    return singCount;
}



int FindCrossFieldSingularities::findCrossFieldSingularities(const V3s& FF1, const V3s& FF2, Ds& Result_Singularities, bool f1andf2_isAlmostSameLength)
{
    //DEBUG - check if fast method works correct
    //auto time = utils::time::Now();
    //bool log = false;
    //cout << endl << "FULL" << endl;
    //int singCountFull = findCrossFieldSingularities_full(mesh, FF1, FF2, Result_Singularities, log);
    //cout << "found " << singCountFull << " singularities  in  " << utils::time::ElapsedSecondsStr(time) << endl;

    //cout << endl << "FAST" << endl;
    ////for(int i = 0; i < 1000; i ++) findCrossFieldSingularities_fast(mesh, FF1, FF2, Result_Singularities, log);
    //int singCountFast = findCrossFieldSingularities_fast(mesh, FF1, FF2, Result_Singularities, log);
    //cout << "found " << singCountFast << " singularities  in  " << utils::time::ElapsedSecondsStr(time) << endl;

    //if (singCountFull != singCountFast)
    //{
    //    cout << endl << endl << endl << "!!!! fast method has different result  " << singCountFast << "  from full method  " << singCountFull << endl << endl << endl << endl;
    //}
    //return singCountFull;

    return findCrossFieldSingularities_fast(FF1, FF2, Result_Singularities, false, f1andf2_isAlmostSameLength);;
}