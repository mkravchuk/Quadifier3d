#include "stdafx.h"
#include "NPolyVectorFieldSolver_Polar2x.h"
#include "FaceConstrain.h"
#include "LinearEquationSolver_Polar2x.h"
#include "SparceMatrixDirect.h"
#include "AngleBoundFFSolver_V3.h"
#include "AngleBoundFFSolver_Eigen.h"
#include "AngleBoundFFSolver_Polar.h"

#define test_linear_equation

//const MeshLogicOptions_Solver& options = meshLogicOptions.Solver;
//Elapsed_Solver& elapsed = elapsedTimers.Solver;
#define options meshLogicOptions.Solver
#define elapsed elapsedTimers.Solver


class NPolyVectorFieldSolver_Polar2x
{
public:
    ViewerDrawObjects& draw;
    int meshid;
    LinearEquationSolver_Polar2x solver;
    Is known_index_to_fid;
    Is unknown_index_to_fid;
    SparceMatrixType Quu;
    SparceMatrixTypeComplex Quk;


    NPolyVectorFieldSolver_Polar2x(
        ViewerDrawObjects& draw,
        int meshid, // use for debugging only
        const P3s& _V,
        const I3s& _F,
        const int& _n,
        const Bs& _isBorderEdge,
        const I2s& _EV,
        const Ds& _E_Length,
        const I3s& _FE,
        const I2s& _EF,
        const Ds& _K,
        const V3s& _F_X,
        const V3s& _F_Y,
        const V3s& _FN,
        D avg_edge_length,
        D max_edge_length,
        bool weightIsRelativeToEdgeLength
    );

    bool solve(const Bs& isFaceConstrained,
        const MatrixXd &faceConstrains,
        vector<V3s>& Result_F, bool& Result_F_isSorted, bool logMessages, bool ignore_x);

private:
    const P3s &V;
    const I3s& F;
    int n;
    int facesCount;

    const Bs& isBorderEdge;
    const I2s& EV;
    const Ds& E_Length;
    const I3s& FE;
    const I2s& EF;
    int edgesCount;
    const Ds& K;

    //int numInteriorEdges;
    //Matrix<int, Dynamic, 2> E2F_int;
    //Is indInteriorToFull;
    //Is indFullToInterior;

    const V3s& F_X;
    const V3s& F_Y;
    const V3s& FN;
    D avg_edge_length;
    D max_edge_length;
    bool weightIsRelativeToEdgeLength;

    RealScalar getComplex(int index, const MatrixXd &faceConstrains, const V3& x, const V3& y, int fid);

    void getGeneralCoeffConstraints(const Bs& isFaceConstrained, int numConstrained,
        const MatrixXd &faceConstrains,
        VectorTypeComplex &xknown);


};



NPolyVectorFieldSolver_Polar2x::NPolyVectorFieldSolver_Polar2x(
    ViewerDrawObjects& _draw,
    int _meshid, // use for debugging only
    const P3s& _V,
    const I3s& _F,
    const int& _n,
    const Bs& _isBorderEdge,
    const I2s& _EV,
    const Ds& _E_Length,
    const I3s& _FE,
    const I2s& _EF,
    const Ds& _K,
    const V3s& _F_X,
    const V3s& _F_Y,
    const V3s& _FN,
    D _avg_edge_length,
    D _max_edge_length,
    bool _weightIsRelativeToEdgeLength) :
    draw(_draw),
    meshid(_meshid),
    solver(_draw),
    V(_V),
    F(_F),
    n(_n),
    facesCount(_F.rows()),
    isBorderEdge(_isBorderEdge),
    EV(_EV),
    E_Length(_E_Length),
    FE(_FE),
    EF(_EF),
    K(_K),
    F_X(_F_X),
    F_Y(_F_Y),
    FN(_FN),
    avg_edge_length(_avg_edge_length),
    max_edge_length(_max_edge_length),
    weightIsRelativeToEdgeLength(_weightIsRelativeToEdgeLength)
{
    edgesCount = EV.rows();
};





//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//--- SOLVER HELPER METHODS ------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------




//D getFacePartialArea(int fid, int eid)
//{
//    P3 p1 = V.row(F(fid, 0));
//    P3 p2 = V.row(F(fid, 1));
//    P3 p3 = V.row(F(fid, 2));
//    P3 pMiddle = (p1 + p2 + p3) / 3;
//    P3 edgep1 = V.row(EV(eid, 0));
//    P3 edgep2 = V.row(EV(eid, 1));
//    P3 closest = utils::vector::ClosestPoint_ToLine(edgep1, edgep2, pMiddle);
//    D area = ((closest - pMiddle).norm() * (edgep1 - edgep2).norm()) / 2;
//    return area;
//}


RealScalar NPolyVectorFieldSolver_Polar2x::getComplex(int index, const MatrixXd &faceConstrains, const V3& x, const V3& y, int fid)
{
    const Vector3d& w = faceConstrains.block<1, 3>(fid, 3 * index).transpose();
    V3 wv3 = convertEigenToV3(w);
    //cout << "w norm()=" << w.norm() << endl;
    D wx = wv3.dot(x);
    D wy = wv3.dot(y);

    if (abs(wx) < 0.0000000001) wx = 0;
    if (abs(wy) < 0.0000000001) wy = 0;
    if (abs(1 - wx) < 0.0000000001) wx = 1;
    if (abs(1 - wy) < 0.0000000001) wy = 1;
    return RealScalar((float)wx, (float)wy);
};

//this gives the coefficients without the (-1)^k that multiplies them
void NPolyVectorFieldSolver_Polar2x::getGeneralCoeffConstraints(const Bs& isFaceConstrained, int numConstrained,
    const MatrixXd &faceConstrains,
    VectorTypeComplex &xknown)
{
    xknown.resize(numConstrained, 1);

    int ind = 0;
    for (int fid = 0; fid < facesCount; ++fid)
    {
        if (!isFaceConstrained[fid]) continue;
        const V3& x = F_X.row(fid);
        const V3& y = F_Y.row(fid);
        RealScalar c0 = getComplex(0, faceConstrains, x, y, fid);
        RealScalar c1 = getComplex(1, faceConstrains, x, y, fid);
        RealScalar c0pow2 = c0 * c0;
        RealScalar c1pow2 = c1 * c1;
        // v0 - with weight
         //RealScalar weight(1.);
         //xknown(ind) = Scalar(weight * (c0pow2 + c1pow2), weight * c0pow2 * weight * c1pow2);
        // v1 - without weight
        xknown(ind) = Scalar(c0pow2 + c1pow2, c0pow2  * c1pow2);
        ind++;
    }
}

void computeFacesLaplacianCoefficient_Polar_usingTriplet(int facesCount, const I2s& EF, const Ds& K, const Bs& isBorderEdge,
    const Bs& isFaceConstrained, int numConstrained,
    SparceMatrixType &Quu, SparceMatrixTypeComplex &Quk,
    Is& known_index_to_fid, Is& unknown_index_to_fid
)
{
    Is fid_to_known_index;
    Is fid_to_unknown_index;
    int edgesCount = isBorderEdge.size();
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    known_index_to_fid.resize(knownCount);
    unknown_index_to_fid.resize(unknownCount);
    fid_to_unknown_index.resize(facesCount);
    fid_to_known_index.resize(facesCount);

    int indk = 0;
    int indu = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        if (isFaceConstrained[fi])
        {
            fid_to_known_index[fi] = indk;
            known_index_to_fid[indk] = fi;
            indk++;
        }
        else
        {
            fid_to_unknown_index[fi] = indu;
            unknown_index_to_fid[indu] = fi;
            indu++;
        }
    }

    //#define test_computeFacesLaplacianCoefficient_Polar_fast

    #ifdef test_computeFacesLaplacianCoefficient_Polar_fast
    vector<Triplet<Scalar >> tripletList;
    tripletList.reserve(edgesCount * 4 / 2);
    #endif



    vector<Triplet<Scalar >> tripletList_Quu;
    tripletList_Quu.reserve(edgesCount * 4 / 2);
    vector<Triplet<Scalar >> tripletList_Quk;
    tripletList_Quk.reserve(edgesCount * 4 / 2);


    //Bs fxfxIsAdded = Bs::Constant(facesCount, false);
    //Bs fxfxIsAdded_Quu = Bs::Constant(facesCount, false);
    // For every non-border edge
    for (int eid = 0; eid < edgesCount; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            complex<D> s2 = -std::polar(1., 1. * 2 * K[eid]);
            complex<D> s4 = -std::polar(1., 1. * 4 * K[eid]);
            complex<Real> s2r = { static_cast<Real>(s2.real()), static_cast<Real>(s2.imag()) }; // convert from D to float
            complex<Real> s4r = { static_cast<Real>(s4.real()), static_cast<Real>(s4.imag()) }; // convert from D to float
            Scalar k10 = Scalar(s2r.real(), s2r.imag(), s4r.real(), s4r.imag());// origin,  degree is responsible of K^2 if degree=2, and K^4 if degree=4, this is becuase in polar coordinates (r,a)^2 == (r*r,a+a) == (r^2, 2a)
            Scalar k01 = k10.getconjugated();

            //DEBUG making direct solver
            //Scalar k01 = Scalar(fid_to_unknown_index[fid0], fid_to_unknown_index[fid1]);
            //Scalar k01 = Scalar(fid0, fid1);
            //Scalar k10 = -k01;

            #ifdef test_computeFacesLaplacianCoefficient_Polar_fast
            tripletList.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.)));
            tripletList.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.)));
            tripletList.push_back(Triplet<Scalar >(fid0, fid1, k01));
            tripletList.push_back(Triplet<Scalar >(fid1, fid0, k10));
            #endif

            bool knownFound = false;
            int fid_k;
            int fid_u;
            Scalar knowVal;
            if (!isFaceConstrained[fid0] && !isFaceConstrained[fid1])
            {
                fid0 = fid_to_unknown_index[fid0];
                fid1 = fid_to_unknown_index[fid1];
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.f))); // f0xf0 add always
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.f))); // f1xf1 add always
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid1, k01));
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid0, k10));
            }
            else if (!isFaceConstrained[fid0])
            {
                fid0 = fid_to_unknown_index[fid0];
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.f)));
                knownFound = true;
                fid_u = fid0;
                fid_k = fid_to_known_index[fid1];
                knowVal = k01;

            }
            else if (!isFaceConstrained[fid1])
            {
                fid1 = fid_to_unknown_index[fid1];
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.f)));
                knownFound = true;
                fid_u = fid1;
                fid_k = fid_to_known_index[fid0];
                knowVal = k10;
            }
            if (knownFound)
            {
                tripletList_Quk.push_back(Triplet<Scalar >(fid_u, fid_k, knowVal)); // f0xf0 add always
                                                                                    //tripletList_Quk.push_back(Triplet<Scalar >(fid_k, fid_k, Scalar(1.))); // f0xf0 add always
            }
        }
    }



    #ifdef test_computeFacesLaplacianCoefficient_Polar_fast
    SparceMatrixType facesLaplacianCoefficient;
    facesLaplacianCoefficient.resize(facesCount, facesCount);
    facesLaplacianCoefficient.setFromTriplets(tripletList.begin(), tripletList.end());

    //DEBUG - test Quu
    //igl::slice(facesLaplacianCoefficient, unknown, unknown, Quu);
    //Matrix<Scalar, Dynamic, Dynamic> QuuM(Quu);
    //SparceMatrixType Quufast;
    //Quufast.resize(unknownCount, unknownCount);
    //Quufast.setFromTriplets(tripletList_Quu.begin(), tripletList_Quu.end());
    //Matrix<Scalar, Dynamic, Dynamic> QuuMfast(Quufast);
    //cout << "  " << endl << endl << "degree = " << degree << endl;
    //for (int ri = 0; ri < Quu.rows(); ri++)
    //{
    //    Scalar zero = 0;
    //    cout << endl << "ri = " << ri << "  ";
    //    for (int ci = 0; ci < 50; ci++)
    //    {
    //        char c = (QuuM(ri, ci) == zero) ? '.' : '#';
    //        cout << c;
    //    }
    //    cout << endl << "ri = " << ri << "  ";
    //    for (int ci = 0; ci < 50; ci++)
    //    {
    //        char c = (QuuMfast(ri, ci) == zero) ? '.' : '#';
    //        cout << c;
    //    }
    //    //for (int ci = 0; ci < 50; ci++)
    //    //{
    //    //    if (QuuM(ri, ci) != zero)
    //    //    {
    //    //        cout << endl << QuuM(ri, ci).real()<<", " << QuuM(ri, ci).imag() << "        " << QuuMfast(ri, ci).real() << ", " << QuuMfast(ri, ci).imag();
    //    //    }
    //    //}
    //    for (int ci = 0; ci < Quu.cols(); ci++)
    //    {
    //        if (ri == ci)
    //        {
    //            cout << endl << QuuM(ri, ci);// << "     " << facesLaplacianCoefficient.coeff(ri, ci);
    //        }
    //        if (Quu.coeff(ri, ci) != Quufast.coeff(ri, ci))
    //        {
    //            cout << endl << QuuM(ri, ci).real() << ", " << QuuM(ri, ci).imag() << "        " << QuuMfast(ri, ci).real() << ", " << QuuMfast(ri, ci).imag();
    //        }
    //    }
    //    cout << endl;
    //}


    //DEBUG - test Quk
    //igl::slice(facesLaplacianCoefficient, unknown, known, Quk);
    //Matrix<Scalar, Dynamic, Dynamic> QukM(Quk);
    //SparceMatrixType Qukfast;
    //Qukfast.resize(unknownCount, knownCount);
    //Qukfast.setFromTriplets(tripletList_Quk.begin(), tripletList_Quk.end());
    //Matrix<Scalar, Dynamic, Dynamic> QukMfast(Qukfast);
    //cout << "  " << endl << endl << "degree = " << degree << endl;
    //for (int ri = 0; ri < Quk.rows(); ri++)
    //{
    //    int cols = min(50, Quk.cols());
    //    Scalar zero = 0;
    //    cout << endl << "ri = " << ri << "  ";
    //    for (int ci = 0; ci < cols; ci++)
    //    {
    //        char c = (QukM(ri, ci) == zero) ? '.' : '#';
    //        cout << c;
    //    }
    //    cout << endl << "ri = " << ri << "  ";
    //    for (int ci = 0; ci < cols; ci++)
    //    {
    //        char c = (QukMfast(ri, ci) == zero) ? '.' : '#';
    //        cout << c;
    //    }
    //    //for (int ci = 0; ci < 50; ci++)
    //    //{
    //    //    if (QukM(ri, ci) != zero)
    //    //    {
    //    //        cout << endl << QukM(ri, ci).real()<<", " << QukM(ri, ci).imag() << "        " << QukMfast(ri, ci).real() << ", " << QukMfast(ri, ci).imag();
    //    //    }
    //    //}
    //    for (int ci = 0; ci < cols; ci++)
    //    {
    //        if (ri == ci)
    //        {
    //            cout << endl << QukM(ri, ci);// << "     " << facesLaplacianCoefficient.coeff(ri, ci);
    //        }
    //        if (Quk.coeff(ri, ci) != Qukfast.coeff(ri, ci))
    //        {
    //            cout << endl << QukM(ri, ci).real() << ", " << QukM(ri, ci).imag() << "        " << QukMfast(ri, ci).real() << ", " << QukMfast(ri, ci).imag();
    //        }
    //    }
    //    cout << endl;
    //}
    #endif


    Quu.resize(unknownCount, unknownCount);
    Quu.setFromTriplets(tripletList_Quu.begin(), tripletList_Quu.end());
    Quk.resize(unknownCount, knownCount);
    Quk.setFromTriplets(tripletList_Quk.begin(), tripletList_Quk.end());


    //DEBUG making direct solver
    //if (degree == 2)
    //{
    //    Matrix<Scalar, Dynamic, Dynamic> QuuM(Quu);
    //    cout << "  " << endl << endl << "degree = " << degree << endl;
    //    cout << " ;";
    //    for (int j = 0; j < Quu.cols(); j++)
    //        cout << ";" << j;
    //    cout << endl;
    //    for (int i = 0; i < Quu.rows(); i++)
    //    {
    //        Scalar zero = 0;
    //        cout << "uu#" << i << ";  ";
    //        for (int j = 0; j < Quu.cols(); j++)
    //        {
    //            cout << ";";
    //            if (QuuM(i, j) != zero)
    //            {
    //                cout << (int)QuuM(i, j).real()<<","<< (int)QuuM(i, j).imag();
    //            }
    //        }
    //        cout << endl;
    //    }
    //}
    //if (degree == 2)
    //{
    //    Matrix<Scalar, Dynamic, Dynamic> QukM(Quk);
    //    cout << "  " << endl << endl << "degree = " << degree << endl;
    //    cout << " ;";
    //    for (int j = 0; j < Quk.cols(); j++)
    //        cout << ";"<<j;
    //    cout << endl;
    //    for (int i = 0; i < Quk.rows(); i++)
    //    {
    //        Scalar zero = 0;
    //        cout << "uk#" << i << ";  ";
    //        for (int j = 0; j < Quk.cols(); j++)
    //        {
    //            cout << ";";
    //            if (QukM(i, j) != zero)
    //            {
    //                cout << (int)QukM(i, j).real()<<","<< (int)QukM(i, j).imag();
    //            }
    //        }
    //        cout << endl;
    //    }
    //}
}



void computeFacesLaplacianCoefficient_Polar_direct(int facesCount, const I3s& FE, const I2s& EF, const V3s& FN, D max_edge_length, const Ds& E_Length, const Ds& K, const Bs& isBorderEdge,
    const Bs& isFaceConstrained, int numConstrained, SparceMatrixType &Quu, SparceMatrixTypeComplex &Quk,
    Is& known_index_to_fid, Is& unknown_index_to_fid
)
{
    Is fid_to_known_index;
    Is fid_to_unknown_index;
    int edgesCount = isBorderEdge.size();
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    known_index_to_fid.resize(knownCount);
    fid_to_known_index.resize(facesCount);
    fid_to_unknown_index.resize(facesCount);
    unknown_index_to_fid.resize(unknownCount);
    int indk = 0;
    int indu = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        if (isFaceConstrained[fi])
        {
            fid_to_known_index[fi] = indk;
            known_index_to_fid[indk] = fi;
            indk++;
        }
        else
        {
            fid_to_unknown_index[fi] = indu;
            unknown_index_to_fid[indu] = fi;
            indu++;
        }
    }



    if (meshLogicOptions.LDLTFloatSolver.DebugEnabled && meshLogicOptions.LDLTFloatSolver.debug_skip_permuting)
    {
        vector<int> indu_to_permutation_indu;
        indu_to_permutation_indu.resize(unknownCount);
        Is faceBorderLevel = Is::Zero(facesCount);
        for (int fi = 0; fi < facesCount; ++fi)
        {
            if (isFaceConstrained(fi)) faceBorderLevel(fi) = 1;
        }
        auto getNextFid = [&faceBorderLevel, &facesCount, &FE, &EF](int prevfid)
        {
            int bestIndex_fid = -1;
            int bestIndex_minFriendBorderLevel = -1;
            int prev_bestIndex_fid = -1;
            int prev_bestIndex_minFriendBorderLevel = -1;
            if (prevfid != -1)
            {
                for (int ke = 0; ke < 3; ke++)
                {
                    int eid = FE(prevfid, ke);
                    int fid1 = EF(eid, 0);
                    int fid2 = EF(eid, 1);
                    int fidFriend = (fid1 != prevfid) ? fid1 : fid2;
                    if (faceBorderLevel[fidFriend] == 0)
                    {
                        prev_bestIndex_fid = fidFriend;
                        prev_bestIndex_minFriendBorderLevel = faceBorderLevel[prevfid];
                    }
                }
            }
            for (int fi = 0; fi < facesCount; ++fi)
            {
                if (faceBorderLevel[fi] > 0) continue;
                int maxFriendBorderLevel = -1;
                for (int ke = 0; ke < 3; ke++)
                {
                    int eid = FE(fi, ke);
                    int fid1 = EF(eid, 0);
                    int fid2 = EF(eid, 1);
                    int fidFriend = (fid1 != fi) ? fid1 : fid2;
                    maxFriendBorderLevel = max(maxFriendBorderLevel, faceBorderLevel[fidFriend]);
                }
                if (maxFriendBorderLevel == 0) continue;
                if (bestIndex_fid == -1 || maxFriendBorderLevel < bestIndex_minFriendBorderLevel)
                {
                    bestIndex_fid = fi;
                    bestIndex_minFriendBorderLevel = maxFriendBorderLevel;
                }
            }
            if (bestIndex_fid != -1)
            {
                //if (prev_bestIndex_fid != -1 && bestIndex_minFriendBorderLevel + 3 > prev_bestIndex_minFriendBorderLevel)
                //{
                //    bestIndex_fid = prev_bestIndex_fid;
                //    bestIndex_minFriendBorderLevel = prev_bestIndex_minFriendBorderLevel;
                //}
                faceBorderLevel[bestIndex_fid] = bestIndex_minFriendBorderLevel + 1;
            }
            return bestIndex_fid;
        };
        int permutation_indu = 0;
        int prevfid = -1;
        while (permutation_indu < unknownCount)
        {
            int fid = getNextFid(prevfid);
            assert(fid != -1 && "all faces must be mapped");
            indu = fid_to_unknown_index[fid];
            indu_to_permutation_indu[indu] = permutation_indu;
            permutation_indu++;
            prevfid = fid;
        }
        //reverse(indu_to_permutation_indu.begin(), indu_to_permutation_indu.end());
        //vector<int>  tmp = { 11, 76, 31, 74, 32, 61, 63, 33, 35, 129, 24, 51, 126, 125, 49, 6, 115, 14, 105, 41, 107, 109, 113, 130, 131, 116, 114, 169, 38, 40, 102, 119, 28, 54, 57, 59, 58, 133, 29, 134, 118, 10, 71, 12, 62, 70, 68, 66, 19, 55, 18,
        //    17, 7, 4, 121, 9, 117, 73, 8, 67, 64, 72, 3, 56, 50, 5, 69, 16, 13, 143, 108, 65, 110, 60, 15, 136, 127, 106, 132, 103, 135, 148, 101, 147, 142, 112, 139, 111, 146, 138, 25, 42, 43, 100, 153, 27, 145, 144, 45, 26, 93, 92
        //    , 91, 152, 120, 44, 123, 84, 75, 78, 30, 82, 81, 77, 166, 36, 37, 90, 52, 88, 53, 22, 159, 2, 168, 96, 98, 164, 167, 83, 162, 85, 94, 161, 1, 79, 21, 23, 80, 0, 149, 137, 48, 154, 156, 46, 160, 157, 47, 158, 89, 86, 87, 99
        //    , 20, 165, 163, 97, 155, 128, 141, 140, 34, 104, 95, 122, 124, 151, 150, 39 };
        ////reverse(tmp.begin(), tmp.end());
        //for (int i = 0; i < unknownCount; i++)
        //{
        //    indu_to_permutation_indu[tmp[i]] = i;
        //}

        indu = 0;
        for (int fi = 0; fi < facesCount; ++fi)
        {
            if (!isFaceConstrained[fi])
            {
                permutation_indu = indu_to_permutation_indu[indu];
                fid_to_unknown_index[fi] = permutation_indu;
                unknown_index_to_fid[permutation_indu] = fi;
                indu++;
            }
        }
    }


    const bool test = false;

    vector<Triplet<Scalar >> tripletList_Quu;
    vector<Triplet<ScalarComplex >> tripletList_Quk;
    if (test)
    {
        tripletList_Quu.reserve(edgesCount * 4 / 2);
        tripletList_Quk.reserve(edgesCount * 4 / 2);
    }

    bool adjustK = options.adjustK;
    D adjustK_percent = options.adjustK_Percent;
    D adjustK_useCos = options.adjustK_useCos;
    D adjustK_quadric = options.adjustK_quadric;
    SparceMatrixDirect<Scalar, true> uu(unknownCount, unknownCount);
    SparceMatrixDirect<ScalarComplex, false> kk(unknownCount, knownCount);

    bool use_sse_cin_cos = utils::cpu::isSupportedSSE && options.use_sse_cin_cos && !adjustK;
    __m128 sse_sin, sse_cos;
    for (int eid = 0; eid < edgesCount; eid++)
    {
        // precase sin-cos for 4 angles - speed optimization
        if (use_sse_cin_cos && eid % 4 == 0)// for every 4 items - cache 4 sin-cos
        {
            __m128 sse_angle = _mm_set1_ps(0);
            for (int i = 0; i < min(4, edgesCount - eid); i++)
            {
                sse_angle.m128_f32[i] = K[eid + i] * 2;
            }
            utils::sse::sincos(sse_angle, sse_sin, sse_cos);
        }

        if (isBorderEdge[eid]) continue;
        int fid0 = EF(eid, 0);
        int fid1 = EF(eid, 1);
        float radius = 1;
        float angle = static_cast<float>(K[eid]);
        if (adjustK)
        {
            // v0 - Normal
            //V3 N0 = FN.row(fid0);
            //V3 N1 = FN.row(fid1);
            //D ratio = adjustK_useCos ? abs(utils::vector::Cos(N0, N1, true)) : abs(utils::vector::Sin(N0, N1, true));
            //if (adjustK_quadric) ratio = ratio * ratio;
            //D takePercent = 1 - (1 - ratio)*adjustK_percent;
            //D r = 1.*takePercent;

            // v1 - edge length
            D ratio = E_Length(eid) / max_edge_length;
            if (adjustK_quadric) ratio = ratio * ratio;
            D takePercent = 1 - (1 - ratio)*adjustK_percent;
            radius = static_cast<float>(1.*takePercent);
        }
        RealScalar polar(radius, angle);
        //RealScalar p2 = polar * polar;                     pseudo code - doesnt work   
        //RealScalar p4 = polar * polar*polar*polar;    pseudo code - doesnt work
        RealScalar p2(pow(polar.real(), 2), polar.imag() * 2); // in polar coordinates (r,a)^2 == (r*r,a+a) == (r^2, 2a)
        RealScalar p4(pow(polar.real(), 4), polar.imag() * 4); // in polar coordinates (r,a)^4 == (r*r*r*r,a+a+a+a) == (r^4, 4a)



        //Scalar k10polar = -Scalar(p2.real(), p2.imag(), p4.real(), p4.imag()); pseudo code - doesnt work   
        //Scalar k01polar = k10polar.getconjugated();

        complex<Real> c2; // complex representation of polar vector <radius, angle>
        if (use_sse_cin_cos)
        {
            // v0 - calculate every time
            //__m128 sse_angle = _mm_set1_ps(p2.imag());
            //utils::sse::sincos(sse_angle, sse_sin, sse_cos);
            //float sin = sse_sin.m128_f32[0];
            //float cos = sse_cos.m128_f32[0];
            // v1 - use precalculated values that made at the begin of the loop for every 4 items
            float sin = sse_sin.m128_f32[eid % 4];
            float cos = sse_cos.m128_f32[eid % 4];
            c2 = complex<float>(p2.real()*cos, p2.real()*sin);
        }
        else
        {
            c2 = std::polar(p2.real(), p2.imag());
        }



        //v0
        //complex<D> s4 = std::polar(1., 1. * 4 * K[eid]);  // in polar coordinates (r,a)^4 == (r*r*r*r,a+a+a+a) == (r^4, 4a)
        //v1 - fast - using trigonometric formula, since we use in s4 D of angle in s2
        Real cos = c2.real();//!!! should be taken without weight, that currently is equal to 1
        Real sin = c2.imag();//!!! should be taken without weight, that currently is equal to 1
        complex<Real> c4 = complex<Real>(cos*cos - sin * sin, 2 * sin*cos); // since: cos(2a) = cos^2(a) - sin^2(a),  and sin(2a) = 2sin(a)cos(a)

        ScalarComplex k10complex = -ScalarComplex(c2.real(), c2.imag(), c4.real(), c4.imag());// origin,  degree is responsible of K^2 if degree=2, and K^4 if degree=4, this is becuase in polar coordinates (r,a)^2 == (r*r,a+a) == (r^2, 2a)
        ScalarComplex k01complex = k10complex.getconjugated();

        // Test polar coordinates
        //complex<Real>  k10polar_c1 = std::polar(k10polar.extract(0), k10polar.extract(1));
        //complex<Real>  k10polar_c2 = std::polar(k10polar.extract(2), k10polar.extract(3));
        //ScalarComplex  k10polar_c = ScalarComplex(k10polar_c1.real(), k10polar_c1.imag(), k10polar_c2.real(), k10polar_c2.imag());
        //complex<Real>  k01polar_c1 = std::polar(k01polar.extract(0), k01polar.extract(1));
        //complex<Real>  k01polar_c2 = std::polar(k01polar.extract(2), k01polar.extract(3));
        //ScalarComplex  k01polar_c = ScalarComplex(k01polar_c1.real(), k01polar_c1.imag(), k01polar_c2.real(), k01polar_c2.imag());

        Scalar k10polar = k10complex;
        Scalar k01polar = k01complex;


        bool knownFound = false;
        int fid_k = 0;
        int fid_u = 0;
        ScalarComplex knowVal;
        if (!isFaceConstrained[fid0] && !isFaceConstrained[fid1])
        {
            fid0 = fid_to_unknown_index[fid0];
            fid1 = fid_to_unknown_index[fid1];
            if (test)
            {
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.f))); // f0xf0 add always
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.f))); // f1xf1 add always
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid1, k01polar));
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid0, k10polar));
            }
            uu.increaseDiagonalCounts(fid0);
            uu.increaseDiagonalCounts(fid1);
            uu.setValue(fid0, fid1, k01polar);
            uu.setValue(fid1, fid0, k10polar);
        }
        else if (!isFaceConstrained[fid0])
        {
            fid0 = fid_to_unknown_index[fid0];
            if (test) tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.f)));
            uu.increaseDiagonalCounts(fid0);
            knownFound = true;
            fid_u = fid0;
            fid_k = fid_to_known_index[fid1];
            knowVal = k01complex;
        }
        else if (!isFaceConstrained[fid1])
        {
            fid1 = fid_to_unknown_index[fid1];
            if (test) tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.f)));
            uu.increaseDiagonalCounts(fid1);
            knownFound = true;
            fid_u = fid1;
            fid_k = fid_to_known_index[fid0];
            knowVal = k10complex;
        }
        if (knownFound)
        {
            if (test) tripletList_Quk.push_back(Triplet<ScalarComplex >(fid_u, fid_k, knowVal)); // f0xf0 add always
            kk.setValue(fid_u, fid_k, knowVal);
        }
    }



    if (test)
    {
        CholSparceMatrixType quu;
        uu.setSparseMatrix(quu);
        CholSparceMatrixTypeComplex quk;
        kk.setSparseMatrix(quk);
        Quu.resize(unknownCount, unknownCount);
        Quu.setFromTriplets(tripletList_Quu.begin(), tripletList_Quu.end());
        Quk.resize(unknownCount, knownCount);
        Quk.setFromTriplets(tripletList_Quk.begin(), tripletList_Quk.end());
        //for (Index i = 0; i < Quu.outerIndexPtr()[unknownCount]; i++)
        //{
        //    //int i1 = Quu.innerIndexPtr()[i];
        //    //int i2=  quu.innerIndexPtr()[i];
        //    //cout << "innerIndex   Quu[" << i << "]=" << i1 << "     quu[" << i << "]=" << i2 << "      " << ((i1!=i2) ? "fail":"") << endl;
        //    complex<float> v1 = Quu.valuePtr()[i].get_low().toComplex();
        //    complex<float> v2 = quu.valuePtr()[i].get_low().toComplex();
        //    cout << "value          Quu[" << i << "]=" << v1 << "     quu[" << i << "]=" << v2 << "      " << ((v1 != v2) ? "fail" : "") << endl;
        //}
        for (Index i = 0; i < Quk.outerIndexPtr()[knownCount]; i++)
        {
            int i1 = Quk.innerIndexPtr()[i];
            int i2 = quk.innerIndexPtr()[i];
            cout << "innerIndex   Quk[" << i << "]=" << i1 << "     quk[" << i << "]=" << i2 << "      " << ((i1 != i2) ? "fail" : "") << endl;
            //int i1 = Quk.outerIndexPtr()[i];
            //int i2=  quk.outerIndexPtr()[i];
            //cout << "outerIndexPtr   Quk[" << i << "]=" << i1 << "     quk[" << i << "]=" << i2 << "      " << ((i1!=i2) ? "fail":"") << endl;
            //complex<float> v1 = Quk.valuePtr()[i].get_low().toComplex();
            //complex<float> v2 = quk.valuePtr()[i].get_low().toComplex();
            //cout << "value          Quk[" << i << "]=" << v1 << "     quk[" << i << "]=" << v2 << "      " << ((v1 != v2) ? "fail" : "") << endl;
        }
    }
    else
    {
        uu.setSparseMatrix(Quu);
        kk.setSparseMatrix(Quk);
    }
}


void setFieldFromGeneralCoefficientsPolar(const  VectorTypeComplex &coeffs, VectorTypeComplex &pv)
{
    const int facesCount = coeffs.rows();
    pv.resize(facesCount);
    bool sort = options.sort_complex_vectors;

    for (int fid = 0; fid < facesCount; ++fid)
    {
        RealScalarComplex d1 = coeffs(fid).get_low();
        RealScalarComplex d2 = -coeffs(fid).get_high();

        //
        // v3 - direct calculation of 2x2 matrix
        //                                                                      |x  y|
        // Eigen value of 2x2 matrix defined by equation      |1 0|*a=0
        // from this equation we will get quadric equation (a-x)*a - y = 0
        // solving this equation we will find solution for a = (x +- sqrt(x^2 + 4y))/2
        //
        //                                             |a-x  y|   |x1|
        // Solving equation of 2x2 matrix  |-1   a | * |x2| = 0   we can find a solutions: 
        // solution 1:   x1 = x2*a,  where   a = (x+sqrt(x^2 + 4y))/2
        // solution 2:   x1 = x2*a,  where   a = (x-sqrt(x^2 + 4y))/2
        // so, solutions differs only by '+' and '-' in 'a' calculus
        // we can assume that x2 == 1, then solution will be x1 = a
        // and since 'a' of 2x2 will be equal to sqrt('a' of 4x4), so final solution will be:
        // eigen values1: sqrt((x+sqrt(x^2 + 4y))/2)
        // eigen values2: sqrt((x-sqrt(x^2 + 4y))/2)
        //
        //Scalar sqrt_det = sqrt(d1*d1 + 4.0*d2);
        //Scalar a1_2x2 = (d1 + sqrt_det)*0.5;
        //Scalar a2_2x2 = (d1 - sqrt_det)*0.5;
        //Scalar r0fast = sqrt(a1_2x2); // convert 'a' of 2x2 to 'a' of 4x4
        //Scalar r1fast = sqrt(a2_2x2); // convert 'a' of 2x2 to 'a' of 4x4

        //
        // v4 - direct calculation of 2x2 matrix  with fast complex sqrt
        //  
        // we can simplify v3 by dividing x by 2
        // eigen values1: sqrt((x+sqrt(x^2 + 4y))/2)
        // eigen values1: sqrt(x/2+sqrt((x/2)^2 + y))
        d1 *= 0.5;
        //Scalar sqrt_det = csqrt(d1*d1 + d2);

        RealScalarComplex sqrt_det = sqrt(d1*d1 + d2);
        RealScalarComplex a1_2x2 = (d1 + sqrt_det);
        RealScalarComplex a2_2x2 = (d1 - sqrt_det);

        ScalarComplex res = sqrt(ScalarComplex(a1_2x2, a2_2x2));

        //DEBUG - normalize r0fast and r1fast to test agains oldest method
        //RealScalar r0fast = res.get_low();
        //RealScalar r1fast = res.get_high();
        //Matrix<Scalar, 2, 1> roots2fast_norm;
        //roots2fast_norm(0) = r0fast;
        //roots2fast_norm(1) = r1fast;
        //roots2fast_norm.normalize();
        //Scalar r0fast_norm = roots2fast_norm(0);
        //Scalar r1fast_norm = roots2fast_norm(1);
        if (isnan(res[0]))
        {
            cout << "!  warning   setFieldFromGeneralCoefficientsPolar  isnan(res[0])" << endl;
        }

        // sort complex vectors - this is very important for detecting singularities
        if (sort)// && !isFaceConstrained[fid]
        {
            //D x0 = r0fast.real();
            //D y0 = r0fast.imag();
            //D x1 = r1fast.real();
            //D y1 = r1fast.imag();

            //// ensure first vector is on right side (between [-90..90] degree), and second on left
            //if (x0 < 0)
            //{
            //    x0 = -x0;//rotate first vector by 180 degree
            //    y0 = -y0;//rotate first vector by 180 degree
            //    x1 = -x1;//rotate second vector by 180 degree
            //    y1 = -y1;//rotate second vector by 180 degree
            //}

            ////// ensure first vector in first sector (between [0..90] degree)
            ////if (y0 < 0)
            ////{
            ////    if (y1 < 0) // ensure second vector in first sector (between [0..90] degree)
            ////    {
            ////        x1 = -x1;//flip vector
            ////        y1 = -y1;//flip vector
            ////    }
            ////    swap(x0, x1);
            ////    swap(y0, y1);
            ////}

            ////// ensure vectors are sorted
            //if (y1 < 0)
            //{
            //    x1 = -x1;//flip vector
            //    y1 = -y1;//flip vector
            //}

            //if (x1 > x0)
            //{
            //    x1 = -x1;//flip vector
            //    y1 = -y1;//flip vector
            //}
            //

            //    r0fast = Scalar(x0, y0);
            //    r1fast = Scalar(x1, y1);
            //    //if (x0 < 0 || y0 < 0)
            //    //{
            //    //    cout << "r0fast = " << r0fast << "   r1fast = " << r1fast << endl;
            //    //}

            ScalarComplex absPow2 = absPow2fast(res);
            Real r0fastabsPow2 = absPow2.extract(0);
            Real r1fastabsPow2 = absPow2.extract(2);
            if (r1fastabsPow2 < r0fastabsPow2)
            {
                res = ScalarComplex(res.get_high(), res.get_low());
            }
        }

        // return results
        pv(fid) = res;
    }

}

//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//--- SOLVE -----------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------








bool NPolyVectorFieldSolver_Polar2x::
solve(const Bs& isFaceConstrained,
    const MatrixXd &faceConstrains,
    vector<V3s>& Result_F, bool& Result_F_isSorted, bool logMessages, bool ignore_x)
{
    Result_F_isSorted = false;
    auto time = utils::time::Now();
    // polynomial is of the form:
    // (-1)^0 z^(2n) +
    // (-1)^1 c[0]z^(2n-2) +
    // (-1)^2 c[1]z^(2n-4) +
    // (-1)^3 c[2]z^(2n-6) +
    // ... +
    // (-1)^n c[n-1]


    int numConstrained = 0;
    for (int i = 0; i < isFaceConstrained.size(); i++)
    {
        if (isFaceConstrained[i]) numConstrained++;
    }

    //
    //  minQuadWithKnownMini
    //
    time = utils::time::Now();
    VectorTypeComplex coeffs;
    coeffs.resize(n);
    if (logMessages) cout << "NPolyVectorFieldSolver::solve()  minQuadWithKnownMini  ...";
    //cout << "NPolyVectorFieldSolver::solve()  minQuadWithKnownMini (n=" << to_string(n) << ") ...";
    Timer timer_getGeneralCoeffConstraints;
    VectorTypeComplex xknown;
    getGeneralCoeffConstraints(isFaceConstrained, numConstrained,
        faceConstrains,
        xknown);
    timer_getGeneralCoeffConstraints.stop(elapsed.getGeneralCoeffConstraints);

    Timer timer_computeFacesLaplacianCoefficient;
    //for(int i = 0; i < 100 ; i++)
    if (options.fast_sparce_matrix_creation)
        computeFacesLaplacianCoefficient_Polar_direct(facesCount, FE, EF, FN, max_edge_length, E_Length, K, isBorderEdge,
            isFaceConstrained, numConstrained, Quu, Quk,
            known_index_to_fid, unknown_index_to_fid);
    else
        computeFacesLaplacianCoefficient_Polar_usingTriplet(facesCount, EF, K, isBorderEdge,
            isFaceConstrained, numConstrained, Quu, Quk,
            known_index_to_fid, unknown_index_to_fid);
    timer_computeFacesLaplacianCoefficient.stop(elapsed.computeFacesLaplacianCoefficient);
    //for (Index i = 0; i < Quu.outerIndexPtr()[Quu.cols()]; i++)
    //{
    //    cout << "Quu[" << i << "]=" << Quu.valuePtr()[i].get_low().toComplex() << endl;
    //}

    //Timer timer_minQuadWithKnownMini;
    //cout << "i = " << i << endl;

    solver.compute(Quu, logMessages, elapsed.minQuadWithKnownMini_compute);
    VectorType f;
    solver.solve(isFaceConstrained, Quk, xknown, f, coeffs, known_index_to_fid, unknown_index_to_fid, logMessages, elapsed.minQuadWithKnownMini_solve);
    //timer_minQuadWithKnownMini.stop(elapsed.minQuadWithKnownMini);
    if (logMessages) cout << "  done in " << utils::time::ElapsedSecondsStr(time) << endl;


    if (meshLogicOptions.LDLTFloatSolver.DebugEnabled && meshLogicOptions.LDLTFloatSolver.debug_trace_compute_iterationsCount >= 0)
    {
        if (logMessages) cout << "showing compute_face_ids_iterations  ...";
        auto compute_unknown_indexes = solver.Get_compute_unknown_indexes();
        draw.meshColor.Scheme = MeshColorSceme::PerFace;
        draw.meshColor.Colors = MatrixXf::Constant(F.rows(), 3, 0.6f);//by default make all faces of silver color
        for (int unknown_index : compute_unknown_indexes)
        {
            int fid = unknown_index_to_fid[unknown_index];
            draw.meshColor.Colors.row(fid) = Color3f(1, 0, 0); // for boundary faces - use red color
        }

    }


    //
    //  setFieldFromGeneralCoefficientsPolar
    //
    if (logMessages) cout << "NPolyVectorFieldSolver::solve()  setFieldFromGeneralCoefficientsPolar  ...";
    Timer timer_setFieldFromGeneralCoefficients;
    VectorTypeComplex pv;
    //for(int i = 0; i < 100; i++) 
    setFieldFromGeneralCoefficientsPolar(coeffs, pv);
    timer_setFieldFromGeneralCoefficients.stop(elapsed.setFieldFromGeneralCoefficients);
    if (logMessages) cout << " done in " << timer_setFieldFromGeneralCoefficients << endl;



    //
    // Generate Result_F
    //
    if (logMessages) cout << "NPolyVectorFieldSolver::solve()  generate output nrosy field  ...";
    Timer timer_GenerateResult;
    Result_F.resize(n);
    for (int i = 0; i < n; i++)
    {
        Result_F[i].resize(facesCount, 3);
    }
    // no need in  multithreading - anyway code is very simple and fast
    //extern bool IsOmpEnabled;
    //#pragma omp parallel for schedule(static) if(IsOmpEnabled)

    if (meshLogicOptions.Solver.fast_Generate_Result_F && n == 2)
    {
        ScalarComplex* p = pv.data();
        for (int fid = 0; fid < facesCount; ++fid)
        {
            ScalarComplex xxxx = *p;
            float x[4];
            xxxx.store(x);
            D x0 = x[0];
            D y0 = x[1];
            D x1 = x[2];
            D y1 = x[3];

            // sort vectors
            D cross01 = x0 * y1 - y0 * x1;
            if (cross01 < 0)
            {
                x1 = -x1;
                y1 = -y1;
            }
            p++;

            const V3 &b1 = F_X.row(fid);
            const V3 &b2 = F_Y.row(fid);
            Result_F[0].row(fid) = b1 * x0 + b2 * y0;
            Result_F[1].row(fid) = b1 * x1 + b2 * y1;
        }
        Result_F_isSorted = true;
    }
    else
    {
        for (int fid = 0; fid < facesCount; ++fid)
        {
            const V3& b1 = F_X.row(fid);
            const V3& b2 = F_Y.row(fid);
            ScalarComplex xxxx = pv(fid);
            float x[4];
            xxxx.store(x);
            D x0 = x[0];
            D y0 = x[1];
            D x1 = x[2];
            D y1 = x[3];
            for (int i = 0; i < n; i++)
            {
                D f1 = xxxx.extract(i * 2 + 0);
                D f2 = xxxx.extract(i * 2 + 1);
                Result_F[i].row(fid) = f1 * b1 + f2 * b2;
            }
        }
    }
    timer_GenerateResult.stop(elapsed.GenerateResult);
    if (logMessages) cout << " done in " << timer_GenerateResult << endl;
    return true;
}



//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------



void n_polyvector_polar2x(
    ViewerDrawObjects& draw,
    int meshid, // use for debugging only
    const P3s &V,
    const I3s& F,
    const Ds &F_Areas,
    const Bs& E_isborder,
    const I2s& EV,
    const Ds& E_Length,
    const I3s& FE,
    const I2s& EF,
    const Ds& K,
    const V3s& F_X,
    const V3s& F_Y,
    const V3s& F_Z,
    const D avg_edge_length,
    const D max_edge_length,
    const vector<FaceConstrain>& Constrains,  // Constrained faces representative vector
    D softPercent, //ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
    vector<V3s> &Result_F, bool& Result_F_isSorted,
    bool WeightIsRelativeToEdgeLength, bool logMessages, bool ignore_x, bool takeDirectionCorrected)
{
    Timer timer;

    Result_F_isSorted = false;

    // Set weights base on edge length and triangle weight
    Ds edgeDirectionWeights = Ds::Constant(Constrains.size(), 1);
    Ds directionWeights = Ds::Constant(Constrains.size(), 1);
    if (WeightIsRelativeToEdgeLength)
    {
        D maxedgeLength = 0;
        D maxfaceArea = 0;
        for (int i = 0; i < Constrains.size(); ++i)
        {
            int fid = Constrains[i].FaceId;
            int eid = FE(fid, 0);
            V3 edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
            //D edgeLength = utils::vector::Length(edgeDirection);
            D edgeLength = E_Length[eid];
            D faceArea = F_Areas(fid);
            maxedgeLength = max(maxedgeLength, edgeLength);
            maxfaceArea = max(maxfaceArea, faceArea);
        }
        for (int i = 0; i < Constrains.size(); ++i)
        {
            int fid = Constrains[i].FaceId;
            int eid = FE(fid, 0);
            V3 edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
            //D edgeLength = utils::vector::Length(edgeDirection);
            D edgeLength = E_Length[eid];
            D faceArea = F_Areas(fid);
            //D edgeDirectionWeight = edgeLength / maxedgeLength;
            //D directionWeight = faceArea / maxfaceArea;
            //D weight = faceArea / maxfaceArea;// works bad
            D weight = edgeLength / maxedgeLength;// works ok
            D edgeDirectionWeight = weight;
            D directionWeight = weight;
            edgeDirectionWeights(i) = edgeDirectionWeight;
            directionWeights(i) = directionWeight;
        }
    }

    // Set solver contrains base on face constrains and edge directions
    Bs isFaceConstrained = Bs::Zero(F.rows());
    MatrixXd faceConstrains = MatrixXd::Constant(F.rows(), 3 * 2, 0);

    for (int i = 0; i < Constrains.size(); ++i)
    {
        int fid = Constrains[i].FaceId;

        //v1
        //int eid = Constrains[i].EdgeId;
        //P3 v0 = V.row(EV(eid, 0));
        //P3 v1 = V.row(EV(eid, 1));
        //V3 edgeDirection = v1 - v0;
        V3 edgeDirection = Constrains[i].DirectionX;
        edgeDirection.normalize();

        //v2 - faster but requieres fix in mesh and dont work with solid meshes without borders
        //V3 edgeDirection = F_X.row(fid);

        V3 direction = takeDirectionCorrected
            ? Constrains[i].DirectionY_Corrected
            : Constrains[i].DirectionY;
        direction.normalize();

        //DEBUG - check consistance of X directions for borders-constrains - they should all follow each others
        //V3 x = F_X.row(fid);
        //D dot = edgeDirection.dot(x);
        //if (dot < 0)
        //{
        //    cout << "wrong!!!   edgeDirection.dot(x) < 0  for fid=" << fid << endl;
        //}

        //if (eid == 135)
        //{
        //    P3 edgemiddle = (v0 + v1) / 2;
        //    draw.AddPoint(v0, Color3d(0, 0, 0), "v0");
        //    draw.AddPoint(v1, Color3d(0,0,0), "v1");
        //    draw.AddPoint(edgemiddle);
        //    draw.AddEdge(edgemiddle, edgemiddle + edgeDirection*avg_edge_length, Color3d(0,1,0), "edgeDirection");
        //    //draw.AddEdge(edgemiddle, edgemiddle + direction*avg_edge_length, Color3d(0, 0, 1), "direction");
        //    e.directionToFace = mesh.EdgeNormalToFace(eid, fid);
        //}

        if (WeightIsRelativeToEdgeLength)
        {
            edgeDirection *= edgeDirectionWeights(i);
            direction *= directionWeights(i);
        }
        D m = softPercent;
        // i dont know why results are wrong when softPercent ==1 - in this case we have to avoid multiply opperaion - why i dont know
        if (abs(1 - m) > 0.001)
        {
            direction *= m;
        }
        #if alternative_solvers_SUPPORTED
        if (meshLogicOptions.Solver.SolveForVectorsXY == MeshLogicOptions_Solver::SolveForVectorsXYEnum::xx) direction = edgeDirection;
        if (meshLogicOptions.Solver.SolveForVectorsXY == MeshLogicOptions_Solver::SolveForVectorsXYEnum::yy) edgeDirection = direction;
        #endif
        //faceConstrains.row(fid) << edgeDirection, direction;
        faceConstrains(fid, 0) = edgeDirection(0);
        faceConstrains(fid, 1) = edgeDirection(1);
        faceConstrains(fid, 2) = edgeDirection(2);
        faceConstrains(fid, 3) = direction(0);
        faceConstrains(fid, 4) = direction(1);
        faceConstrains(fid, 5) = direction(2);
        isFaceConstrained(fid) = true;
    }
    //DEBUG show bc2
    //draw.ReservePoints(bSoft.size()); // reseve space - maximum of what we can add
    //draw.ReserveEdges(4 * bSoft.size()); // reseve space - maximum of what we can add
    //for (int i = 0; i < bSoft.size(); i++)
    //{
    //    //if (i != 0 && i != 10 && i != 20) continue;
    //    for (int ni = 0; ni < 2; ni++)
    //    {
    //        P3 v = mesh.F_Barycenters.row(bSoft(i));
    //        V3 directoin = bc2.block<1,3>(i, 3 * ni).transpose();
    //        draw.AddEdge(v, v + directoin.transpose()*mesh.avg_edge_length/2, Color3d(1, 0, 0));
    //    }
    //    //show first face edge
    //    //int eid = mesh.FE(bSoft(i), 0);
    //    //P3 p = mesh.V.row(mesh.EV(eid, 0));
    //    //RowVector3d edgeDirection = mesh.V.row(mesh.EV(eid, 1)) - mesh.V.row(mesh.EV(eid, 0));
    //    //edgeDirection.normalize();
    //    //draw.AddEdge(p, p + edgeDirection.transpose()*mesh.avg_edge_length, Color3d(1, 0, 0));
    //   // draw.AddPoint(p, Color3d(0, 0, 0));
    //}


    NPolyVectorFieldSolver_Polar2x nrosySolver(draw, meshid, V, F, 2, E_isborder, EV, E_Length, FE, EF, K, F_X, F_Y, F_Z, avg_edge_length, max_edge_length, WeightIsRelativeToEdgeLength);
    timer.stop(elapsed.initSolver);
    nrosySolver.solve(isFaceConstrained, faceConstrains, Result_F, Result_F_isSorted, logMessages, ignore_x);// ignore x because it is always 0 for uncorrectedContrains


    //
    // angle_bound_frame_fields
    //
    if (meshLogicOptions.SolverAngleBound.Enabled
        && meshLogicOptions.SolverAngleBound.iterations_count > 0)
    {
        int numF = F.rows();
        vector<V3s> output;

        Timer timeAngleBound;
        int callTimes = meshLogicOptions.SolverAngleBound.TestPerformanceCall ? meshLogicOptions.SolverAngleBound.TestPerformanceCall_repeatTimes : 1;
        if (meshLogicOptions.Solver.TestPerformanceCall) cout << "Testing performance of SolverAngleBound - calling solver " << meshLogicOptions.SolverAngleBound.TestPerformanceCall_repeatTimes << " times..." << endl;
        if (callTimes > 0)
        {
            for (int perfi = 0; perfi < callTimes; perfi++) // exluding 1 call to leave it to non debug call
            {
                if (meshLogicOptions.SolverAngleBound.Algorithm == MeshLogicOptions_SolverAngleBound::AlgorithmType::eigen)
                {
                    angle_bound_frame_fields_eigen(V, F, isFaceConstrained, Result_F,
                        meshLogicOptions.SolverAngleBound.thetatMin,
                        meshLogicOptions.SolverAngleBound.iterations_count,
                        meshLogicOptions.SolverAngleBound.lambdaInit,
                        meshLogicOptions.SolverAngleBound.lambdaMultFactor,
                        meshLogicOptions.SolverAngleBound.doHardConstraints,
                        output
                    );
                }
                else if (meshLogicOptions.SolverAngleBound.Algorithm == MeshLogicOptions_SolverAngleBound::AlgorithmType::v3)
                {
                    AngleBoundFFSolver_V3 angleBoundSolver(V, F, EV, FE, EF, F_X, F_Y, F_Z, K, E_isborder,
                        meshLogicOptions.SolverAngleBound.iterations_count,
                        meshLogicOptions.SolverAngleBound.thetatMin,
                        meshLogicOptions.SolverAngleBound.lambdaInit,
                        meshLogicOptions.SolverAngleBound.lambdaMultFactor,
                        meshLogicOptions.SolverAngleBound.doHardConstraints
                    );
                    angleBoundSolver.solve(isFaceConstrained, Result_F, output, logMessages);
                }
                else if (meshLogicOptions.SolverAngleBound.Algorithm == MeshLogicOptions_SolverAngleBound::AlgorithmType::polar)
                {
                    AngleBoundFFSolver_Polar angleBoundSolver(draw, V, F, EV, FE, EF, F_X, F_Y, F_Z, K, E_isborder, max_edge_length, E_Length,
                        nrosySolver.known_index_to_fid, nrosySolver.unknown_index_to_fid, nrosySolver.Quu, nrosySolver.Quk,
                        meshLogicOptions.SolverAngleBound.iterations_count,
                        meshLogicOptions.SolverAngleBound.thetatMin,
                        meshLogicOptions.SolverAngleBound.lambdaInit,
                        meshLogicOptions.SolverAngleBound.lambdaMultFactor,
                        meshLogicOptions.SolverAngleBound.doHardConstraints
                    );
                    bool reuse_ordering = meshLogicOptions.SolverAngleBound.reuse_ordering;
                    if (reuse_ordering)
                    {
                        angleBoundSolver.solver.copy_ordering_from_another_solver__from_limited_to_full(nrosySolver.solver, nrosySolver.known_index_to_fid, nrosySolver.unknown_index_to_fid);
                    }
                    angleBoundSolver.solve(isFaceConstrained, Result_F, output, logMessages, reuse_ordering);
                }
                else
                {
                    cout << "!error  uknown meshLogicOptions.SolverAngleBound.Algorithm" << endl;
                }
                cout << ".";
            }

            Result_F_isSorted = false;
            for (int fi = 0; fi < numF; ++fi)
            {
                V3 x_fixedAngle = output[0].row(fi);
                V3 y_fixedAngle = output[1].row(fi);

                if (!isFaceConstrained(fi))
                {
                    Result_F[0].row(fi) = x_fixedAngle;
                    Result_F[1].row(fi) = y_fixedAngle;
                    continue;
                }

                //  constrained faces must have same directions as initial
                V3 x = Result_F[0].row(fi);
                V3 y = Result_F[1].row(fi);
                if (abs(utils::vector::Dot(x.normalized(), x_fixedAngle.normalized())) < 0.5)
                {
                    swap(x_fixedAngle, y_fixedAngle);// exchange x with y
                }
                if (!meshLogicOptions.SolverAngleBound.leave_x_original)
                {
                    Result_F[0].row(fi) = utils::vector::Dot(x, x_fixedAngle) > 0 ? x_fixedAngle : -x_fixedAngle;
                }
                Result_F[1].row(fi) = utils::vector::Dot(y, y_fixedAngle) > 0 ? y_fixedAngle : -y_fixedAngle;
            }
        }

        timeAngleBound.stop(elapsedTimers.SolverAngleBound.Total);
        //elapsedTimers.Solver.Total -= 
        //elapsed_ms
        //timeAngleBound.elapsedDuration
    }
}