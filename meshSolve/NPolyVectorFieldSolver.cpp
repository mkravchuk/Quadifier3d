#include "stdafx.h"
#include "NPolyVectorFieldSolver.h"
//#include <igl/polyroots.h>
#include "FaceConstrain.h"
//#include <igl/n_polyvector.h>
//#include <igl/n_polyvector_general.h>
#include "LinearEquationSolver.h"
#include "SparceMatrixDirect.h"

#define test_linear_equation

//const MeshLogicOptions_Solver& options = meshLogicOptions.Solver;
//Elapsed_Solver& elapsed = elapsedTimers.Solver;
#define options meshLogicOptions.Solver
#define elapsed elapsedTimers.Solver


class NPolyVectorFieldSolver
{
public:
    NPolyVectorFieldSolver(
        const P3s& _V,
        const I3s& _F,
        const int& _n,
        const Bs& _isBorderEdge,
        const I2s& _EV,
        const I3s& _FE,
        const I2s& _EF,
        const Ds& _K,
        const V3s& _F_X,
        const V3s& _F_Y,
        const V3s& _FN,
        const D avg_edge_length,
        const D max_edge_length
    );
    bool solve(const Is& isFaceConstrained,
        const MatrixXd &faceConstrains,
        vector<V3s>& Result_F, bool& Result_F_isSorted, bool logMessages, bool ignore_x);

private:
    const P3s &V;
    const I3s& F;
    int n;
    int facesCount;

    const Bs& isBorderEdge;
    const I2s& EV;
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
    const D avg_edge_length;
    const D max_edge_length;

    D getFacePartialArea(int fid, int eid);

    void getGeneralCoeffConstraints(const Is& isFaceConstrained, int numConstrained,
        const MatrixXd &faceConstrains,
        int ni,
        VectorType &xknown);

    __forceinline Scalar getComplex(int index, const MatrixXd &faceConstrains, const V3& x, const V3& y, int fid);
    void getGeneralCoeffConstraints2_fast(const Is& isFaceConstrained, int numConstrained,
        const MatrixXd &faceConstrains,
        int ni,
        VectorType &xknown);


    void precomputeInteriorEdges();


};



NPolyVectorFieldSolver::NPolyVectorFieldSolver(const P3s& _V,
    const I3s& _F,
    const int& _n,
    const Bs& _isBorderEdge,
    const I2s& _EV,
    const I3s& _FE,
    const I2s& _EF,
    const Ds& _K,
    const V3s& _F_X,
    const V3s& _F_Y,
    const V3s& _FN,
    const D _avg_edge_length,
    const D _max_edge_length) :
    V(_V),
    F(_F),
    n(_n),
    facesCount(_F.rows()),
    isBorderEdge(_isBorderEdge),
    EV(_EV),
    FE(_FE),
    EF(_EF),
    K(_K),
    F_X(_F_X),
    F_Y(_F_Y),
    FN(_FN),
    avg_edge_length(_avg_edge_length),
    max_edge_length(_max_edge_length)
{
    edgesCount = EV.rows();

    precomputeInteriorEdges();
};


//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//--- SOLVER HELPER METHODS ------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------





void computeFacesLaplacianCoefficient_origin(int facesCount, int edgesCount, const I2s& EF, const Ds& K, const Bs& isBorderEdge,
    const Is& isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    VectorXi known;
    known.resize(knownCount);
    VectorXi unknown;
    unknown.resize(unknownCount);
    VectorXi fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount);

    int indk = 0;
    int indu = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        if (isFaceConstrained[fi])
        {
            known[indk] = fi;
            indk++;
        }
        else
        {
            fid_to_unknown_index[fi] = indu;
            unknown[indu] = fi;
            indu++;
        }
    }

    vector<Triplet<Scalar >> tripletList;
    tripletList.reserve(edgesCount * 4 / 2);
    // For every non-border edge
    for (int eid = 0; eid < edgesCount; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            Scalar polarPlus = std::polar(1., 1.*degree*K[eid]);
            Scalar polarMinus = Scalar(polarPlus.real(), -polarPlus.imag());
            tripletList.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.)));
            tripletList.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.)));
            tripletList.push_back(Triplet<Scalar >(fid0, fid1, -polarMinus));
            tripletList.push_back(Triplet<Scalar >(fid1, fid0, -polarPlus));
        }
    }
    SparceMatrixType facesLaplacianCoefficient;
    facesLaplacianCoefficient.resize(facesCount, facesCount);
    facesLaplacianCoefficient.setFromTriplets(tripletList.begin(), tripletList.end());
    igl::slice(facesLaplacianCoefficient, unknown, unknown, Quu);
    igl::slice(facesLaplacianCoefficient, unknown, known, Quk);
}

void computeFacesLaplacianCoefficient_fast(int facesCount, int edgesCount, const I2s& EF, const V3s& FN, const Ds& K, const Bs& isBorderEdge,
    const Is& isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    Is known;
    known.resize(knownCount);
    Is unknown;
    unknown.resize(unknownCount);
    Is fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount);
    Is fid_to_known_index;
    fid_to_known_index.resize(facesCount);

    int indk = 0;
    int indu = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        if (isFaceConstrained[fi])
        {
            fid_to_known_index[fi] = indk;
            known[indk] = fi;
            indk++;
        }
        else
        {
            fid_to_unknown_index[fi] = indu;
            unknown[indu] = fi;
            indu++;
        }
    }

    //#define test_computeFacesLaplacianCoefficient_fast

    #ifdef test_computeFacesLaplacianCoefficient_fast
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

            Scalar k10 = -std::polar(1., 1. * degree * K[eid]);// origin
            //Scalar k10 = -std::polar(1., 1.*4*K[eid]); // works same good as origin with dynamic value 'degree' that could be 2 or 4
            Scalar k01 = Scalar(k10.real(), -k10.imag());

            //DEBUG making direct solver
            //Scalar k01 = Scalar(fid_to_unknown_index[fid0], fid_to_unknown_index[fid1]);
            //Scalar k01 = Scalar(fid0, fid1);
            //Scalar k10 = -k01;

            #ifdef test_computeFacesLaplacianCoefficient_fast
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
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.))); // f0xf0 add always
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.))); // f1xf1 add always
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid1, k01));
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid0, k10));
            }
            else if (!isFaceConstrained[fid0])
            {
                fid0 = fid_to_unknown_index[fid0];
                tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.)));
                knownFound = true;
                fid_u = fid0;
                fid_k = fid_to_known_index[fid1];
                knowVal = k01;

            }
            else if (!isFaceConstrained[fid1])
            {
                fid1 = fid_to_unknown_index[fid1];
                tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.)));
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
    #ifdef test_computeFacesLaplacianCoefficient_fast
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



void computeFacesLaplacianCoefficient_fast_fast(int facesCount, int edgesCount, const I2s& EF, const Ds& K, const Bs& isBorderEdge,
    const Is& isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    Is known;
    known.resize(knownCount);
    Is unknown;
    unknown.resize(unknownCount);
    Is fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount);
    Is fid_to_known_index;
    fid_to_known_index.resize(facesCount);

    int indk = 0;
    int indu = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        if (isFaceConstrained[fi])
        {
            fid_to_known_index[fi] = indk;
            known[indk] = fi;
            indk++;
        }
        else
        {
            fid_to_unknown_index[fi] = indu;
            unknown[indu] = fi;
            indu++;
        }
    }




    #define P pair<int,Scalar>
    Matrix <P, -1, -1, ColMajor> quu;
    quu.resize(4, unknownCount);
    Is quuCountValuesInCol = Is::Constant(unknownCount, 0);
    Matrix <P, -1, -1, ColMajor> quk;
    quk.resize(4, knownCount);
    Is qukCountValuesInCol = Is::Constant(knownCount, 0);

    auto quu_setMiddleValue = [&quu, &quuCountValuesInCol](int fid)
    {
        if (quuCountValuesInCol(fid) == 0)
        {
            quuCountValuesInCol(fid) = 1;
            quu(0, fid) = { fid, Scalar(1.) };
        }
        else
        {
            quu(0, fid).second += Scalar(1.);
        }
    };

    // For every non-border edge
    bool WeightIsRelativeToEdgeLength = meshLogicOptions.Constrains.WeightIsRelativeToEdgeLength;
    for (int eid = 0; eid < edgesCount; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            D real = 1.;
            // v1 - edges avg - works very bad
            //if (WeightIsRelativeToEdgeLength)
            //{
            //    D edgeLength = (V.row(EV(eid, 0)) - V.row(EV(eid, 1))).norm();
            //    real = edgeLength / avg_edge_length;
            //}
            // v2 - edges max - works better but still wrong
            //if (WeightIsRelativeToEdgeLength)
            //{
            //    D edgeLength = (V.row(EV(eid, 0)) - V.row(EV(eid, 1))).norm();
            //    real = edgeLength / max_edge_length;
            //}
            // v3 - triangles areas
            //if (WeightIsRelativeToEdgeLength)
            //{
            //    D area0 = getFacePartialArea(fid0, eid)*100;
            //    D area1 = getFacePartialArea(fid1, eid)*100;
            //    real = area0 + area1;
            //}

            Scalar polarPlus = std::polar(1., 1.*degree*K[eid]);
            Scalar polarMinus = Scalar(polarPlus.real(), -polarPlus.imag());

            bool knownFound = false;
            int fid_k = 0;
            int fid_u = 0;
            Scalar knowVal;
            if (!isFaceConstrained[fid0] && !isFaceConstrained[fid1])
            {
                fid0 = fid_to_unknown_index[fid0];
                fid1 = fid_to_unknown_index[fid1];
                //tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.))); // f0xf0 add always
                //tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.))); // f1xf1 add always
                quu_setMiddleValue(fid0);
                quu_setMiddleValue(fid1);
                //tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid1, -1.*polarMinus));
                quu(quuCountValuesInCol(fid1), fid1) = { fid0 , -polarMinus };
                quuCountValuesInCol(fid1)++;
                //tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid0, -1.*polarPlus));
                quu(quuCountValuesInCol(fid0), fid0) = { fid1 , -polarPlus };
                quuCountValuesInCol(fid0)++;
            }
            else if (!isFaceConstrained[fid0])
            {
                fid0 = fid_to_unknown_index[fid0];
                //tripletList_Quu.push_back(Triplet<Scalar >(fid0, fid0, Scalar(1.)));
                quu_setMiddleValue(fid0);
                knownFound = true;
                fid_u = fid0;
                fid_k = fid_to_known_index[fid1];
                knowVal = -polarMinus;

            }
            else if (!isFaceConstrained[fid1])
            {
                fid1 = fid_to_unknown_index[fid1];
                //tripletList_Quu.push_back(Triplet<Scalar >(fid1, fid1, Scalar(1.)));
                quu_setMiddleValue(fid1);
                knownFound = true;
                fid_u = fid1;
                fid_k = fid_to_known_index[fid0];
                knowVal = -polarPlus;
            }
            if (knownFound)
            {
                //tripletList_Quk.push_back(Triplet<Scalar >(fid_u, fid_k, knowVal));
                quk(qukCountValuesInCol(fid_k), fid_k) = { fid_u , knowVal };
                qukCountValuesInCol(fid_k)++;
            }
        }
    }

    //
    //EXAMPLE of fast populating sparce matrix 
    //
    //SparseMatrix<float> S(N, N);
    //for (int j = 0; j<N; j++)
    //{
    //    S.startVec(j);
    //    for (int i = 0; i<N; i++)
    //    {
    //        S.insertBack(i, j) = 1;
    //    }
    //}
    //S.finalize();
    //
    // see also 
    // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
    // https://eigen.tuxfamily.org/dox/group__SparseQuickRefPage.html
    //

    auto populate = [](int facesCount, SparceMatrixType& S, int rows, int cols, const Is& countValuesInCol, Matrix <P, -1, -1, ColMajor>& q)
    {
        int maxIndex = facesCount + 1;
        S.resize(rows, cols);
        S.data().reserve(countValuesInCol.sum());
        for (int ci = 0; ci < cols; ++ci)
        {
            int count = countValuesInCol[ci];
            S.startVec(ci);
            for (int k = 0; k < count; k++)
            {
                int minIndex = 0;
                int minFid = q(minIndex, ci).first;
                for (int ri = 1; ri < count; ri++)//skip first index, since we took it as start min-value
                {
                    int fid = q(ri, ci).first;
                    if (fid < minFid)
                    {
                        minFid = fid;
                        minIndex = ri;
                    }
                }
                q(minIndex, ci).first = maxIndex; // remove from next search of minimum fid
                Scalar val = q(minIndex, ci).second;
                S.insertBack(minFid, ci) = val;
            }
        }
        S.finalize();
    };

    populate(facesCount, Quu, unknownCount, unknownCount, quuCountValuesInCol, quu);
    populate(facesCount, Quk, unknownCount, knownCount, qukCountValuesInCol, quk);
}



void computeFacesLaplacianCoefficient_direct(int facesCount, int edgesCount, const I2s& EF, const V3s& FN, const Ds& K, const Bs& isBorderEdge,
    const Is& isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    Is known;
    known.resize(knownCount);
    Is unknown;
    unknown.resize(unknownCount);
    Is fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount);
    Is fid_to_known_index;
    fid_to_known_index.resize(facesCount);

    int indk = 0;
    int indu = 0;
    for (int fi = 0; fi < facesCount; ++fi)
    {
        if (isFaceConstrained[fi])
        {
            fid_to_known_index[fi] = indk;
            known[indk] = fi;
            indk++;
        }
        else
        {
            fid_to_unknown_index[fi] = indu;
            unknown[indu] = fi;
            indu++;
        }
    }

    bool adjustK = options.adjustK;
    D adjustK_percent = options.adjustK_Percent;
     

    SparceMatrixDirect<Scalar, true> uu(unknownCount, unknownCount);
    SparceMatrixDirect<Scalar, false> kk(unknownCount, knownCount);
    for (int eid = 0; eid < edgesCount; ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            Scalar k10 = -std::polar(1., 1. * degree * K[eid]);// origin

            if (eid == 0)
            {
                cout << "!!!  adjustK = " << adjustK << endl;
            }
            if (adjustK)
            { 
                V3 N0 = FN.row(fid0);
                V3 N1 = FN.row(fid1);
                D ration = abs(utils::vector::Cos(N0, N1, true));
                RealScalar takePercent = static_cast<RealScalar>(1 - (1 - ration) * adjustK_percent);
                k10 = { k10.real() * takePercent, k10.imag() };
                
            }

            Scalar k01 = Scalar(k10.real(), -k10.imag());

            bool knownFound = false;
            int fid_k = 0;
            int fid_u = 0;
            Scalar knowVal;
            if (!isFaceConstrained[fid0] && !isFaceConstrained[fid1])
            {
                fid0 = fid_to_unknown_index[fid0];
                fid1 = fid_to_unknown_index[fid1];
                uu.increaseDiagonalCounts(fid0);
                uu.increaseDiagonalCounts(fid1);
                uu.setValue(fid0, fid1, k01);
                uu.setValue(fid1, fid0, k10);
            }
            else if (!isFaceConstrained[fid0])
            {
                fid0 = fid_to_unknown_index[fid0];
                uu.increaseDiagonalCounts(fid0);
                knownFound = true;
                fid_u = fid0;
                fid_k = fid_to_known_index[fid1];
                knowVal = k01;

            }
            else if (!isFaceConstrained[fid1])
            {
                fid1 = fid_to_unknown_index[fid1];
                uu.increaseDiagonalCounts(fid1);
                knownFound = true;
                fid_u = fid1;
                fid_k = fid_to_known_index[fid0];
                knowVal = k10;
            }
            if (knownFound)
            {
                kk.setValue(fid_u, fid_k, knowVal);
            }
        }
    }

    uu.setSparseMatrix(Quu);
    kk.setSparseMatrix(Quk);
}


void computeFacesLaplacianCoefficient(int facesCount, const I2s& EF, const V3s& FN, const Ds& K, const Bs& isBorderEdge,
    const Is& isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk)
{
    int edgesCount = isBorderEdge.size();
    if (options.fast_sparce_matrix_creation)
        computeFacesLaplacianCoefficient_direct(facesCount, edgesCount, EF, FN, K, isBorderEdge,
            isFaceConstrained, numConstrained, degree, Quu, Quk);
    else
        computeFacesLaplacianCoefficient_fast(facesCount, edgesCount, EF, FN, K, isBorderEdge,
            isFaceConstrained, numConstrained, degree, Quu, Quk);
}





void setFieldFromGeneralCoefficients_origin(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    Matrix<Scalar, 3, 3> Identity3x3;
    Identity3x3.setIdentity(3, 3);
    Matrix<Scalar, 3, 1> z;
    z.setZero(3, 1);

    //Matrix<Scalar, 1, 1> I1;
    //I1.setIdentity(1, 1);
    //Matrix<Scalar, 1, 1> z1;
    //z1.setZero(1, 1);

    //Matrix<Scalar, 2, 2> I2;
    //I2.setIdentity(2, 2);
    //Matrix<Scalar, 2, 1> z2;
    //z2.setZero(2, 1);

    Scalar polyCoeff1_prev = Scalar(0, 0);
    Scalar polyCoeff2_prev = Scalar(0, 0);

    for (int i = 0; i < facesCount; ++i)
    {
        //poly coefficients: 1, 0, -Acoeff, 0, Bcoeff
        //matlab code from roots (given there are no trailing zeros in the polynomial coefficients)
        ////////////// v1 origin
        /*VectorType polyCoeff;
        polyCoeff.setZero(2 * n + 1, 1);
        polyCoeff[0] = 1.;
        int sign = 1;
        for (int k = 0; k < n; ++k)
        {
        sign = -sign;
        int degree = 2 * (k + 1);
        polyCoeff[degree] = (1.*sign)*coeffs[k](i);
        }

        VectorType roots;
        igl::polyRoots<Scalar, D >(polyCoeff, roots);*/

        ////////////// v2 fast
        Matrix<Scalar, 1, 4> polyCoeff;
        polyCoeff.setZero();
        int sign = 1;
        for (int k = 0; k < n; ++k)
        {
            int degree = 2 * (k + 1) - 1;
            polyCoeff[degree] = (static_cast<float>(1.)*sign)*coeffs[k](i);
            sign = -sign;
        }


        //cache - for speed up 1%
        if (polyCoeff1_prev == polyCoeff(1)
            && polyCoeff2_prev == polyCoeff(3))
        {
            for (int k = 0; k < n; ++k)
            {
                pv[k](i, 0) = pv[k](i - 1, 0);
                pv[k](i, 1) = pv[k](i - 1, 1);
            }
            continue;
        }
        polyCoeff1_prev = polyCoeff(1);
        polyCoeff2_prev = polyCoeff(3);

        Matrix<Scalar, 4, 4> a;
        a << polyCoeff, Identity3x3, z;
        const Matrix<Scalar, 4, 1> roots = a.eigenvalues();

        ////////////// v3 test
        /*Matrix<Scalar, 1, 3> polyCoeff2;
        polyCoeff2.setZero();
        int sign2 = 1;
        for (int k = 0; k < n; ++k)
        {
        polyCoeff2[k+1] = (1.*sign2)*coeffs[k](i);
        sign2 = -sign2;
        }

        Matrix<Scalar, 3, 3> a2;
        a2 << polyCoeff2, I2, z2;

        Matrix<Scalar, 3, 1> roots2 = a2.eigenvalues();*/

        ////////////// v4 test
        /*Matrix<Scalar, 1, 2> polyCoeff1;
        polyCoeff1.setZero();
        int sign2 = 1;
        for (int k = 0; k < n; ++k)
        {
        polyCoeff1[1] = (1.*sign2)*coeffs[k](i);
        Matrix<Scalar, 2, 2> a1;
        a1 << polyCoeff1, I1, z1;

        Matrix<Scalar, 2, 1> roots1 = a1.eigenvalues();

        sign2 = -sign2;
        }*/
        //////////////

        Eigen::Vector4i done;
        done.setZero();

        Eigen::Matrix<Scalar, 2, 1> u(n, 1);
        int ind = 0;
        for (int k = 0; k < 2 * n; ++k)
        {
            if (done[k])
                continue;
            u[ind] = roots[k];
            done[k] = 1;

            int mini = -1;
            D mind = 1e10;
            for (int l = k + 1; l < 2 * n; ++l)
            {
                D dist = abs(roots[l] + u[ind]);
                if (dist < mind)
                {
                    mind = dist;
                    mini = l;
                }
            }
            done[mini] = 1;
            ind++;
        }
        for (int k = 0; k < n; ++k)
        {
            pv[k](i, 0) = real(u[k]);
            pv[k](i, 1) = imag(u[k]);
        }
    }


}



void setFieldFromGeneralCoefficients2_fast(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    Scalar d1;
    Scalar d2;
    Matrix<Scalar, 2, 2> a2;
    a2(1, 0) = 1;
    a2(1, 1) = 0;

    for (int i = 0; i < facesCount; ++i)
    {
        d1 = coeffs[0](i);
        d2 = -coeffs[1](i);

        // Matrix 4x4 can be simplified to matrix 2x2
        //   0  x  0  y          x  y
        //   1  0  0  0   ==>  1  0
        //   0  1  0  0
        //   0  0  1  0

        a2(0, 0) = d1;
        a2(0, 1) = d2;
        Matrix<Scalar, 2, 1> roots2 = a2.eigenvalues();
        Scalar r0 = sqrt(roots2(0));
        Scalar r1 = sqrt(roots2(1));

        // skip sorting - it is not important

        //return result
        pv[0](i, 0) = r0.real();
        pv[0](i, 1) = r0.imag();
        pv[1](i, 0) = r1.real();
        pv[1](i, 1) = r1.imag();
    }


}


void setFieldFromGeneralCoefficients3_fast_fast(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv, bool ignore_x, const Is& isFaceConstrained)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    for (int i = 0; i < facesCount; ++i)
    {
        Scalar d1;
        if (ignore_x)
            d1 = 0.;
        else
            d1 = coeffs[0](i);
        Scalar d2 = -coeffs[1](i);

        D temp = 0;
        ////DEBUG - calc solution using matrix to test agains new solution
        ////        
        //// v2 - 2x2 matrix
        ////
        //// Matrix 4x4 can be simplified to matrix 2x2, with small correction: 'a' of 2x2 will be equal to sqrt('a' of 4x4), so final eigen values should be 'sqrt'
        ////   0  x  0  y          x  y
        ////   1  0  0  0   ==>  1  0
        ////   0  1  0  0
        ////   0  0  1  0
        //Matrix<Scalar, 2, 2> m2;
        //m2(1, 0) = 1;
        //m2(1, 1) = 0;
        //m2(0, 0) = d1;
        //m2(0, 1) = d2;
        //Matrix<Scalar, 2, 1> roots2 = m2.eigenvalues();
        //roots2(0) = sqrt(roots2(0)); // convert 'a' of 2x2 to 'a' of 4x4
        //roots2(1) = sqrt(roots2(1)); // convert 'a' of 2x2 to 'a' of 4x4
        //Scalar r0 = roots2(0);
        //Scalar r1 = roots2(1);

        ////DEBUG - normalize r0 and r1 to test agains newest method
        //Matrix<Scalar, 2, 1> roots2_norm;
        //roots2_norm(0) = r0;
        //roots2_norm(1) = r1;
        //roots2_norm.normalize();
        //Scalar r0_norm = roots2_norm(0);
        //Scalar r1_norm = roots2_norm(1);

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
        Scalar sqrt_det = sqrt(d1*d1 + (float)4.0*d2);
        Scalar a1_2x2 = (d1 + sqrt_det)*(float)0.5;
        Scalar a2_2x2 = (d1 - sqrt_det)*(float)0.5;
        Scalar r0fast = sqrt(a1_2x2); // convert 'a' of 2x2 to 'a' of 4x4
        Scalar r1fast = sqrt(a2_2x2); // convert 'a' of 2x2 to 'a' of 4x4

                                      //DEBUG - normalize r0fast and r1fast to test agains oldest method
                                      //Matrix<Scalar, 2, 1> roots2fast_norm;
                                      //roots2fast_norm(0) = r0fast;
                                      //roots2fast_norm(1) = r1fast;
                                      //roots2fast_norm.normalize();
                                      //Scalar r0fast_norm = roots2fast_norm(0);
                                      //Scalar r1fast_norm = roots2fast_norm(1);


                                      // sort complex vectors - this is very important for detecting singularities
        if (options.sort_complex_vectors && !isFaceConstrained[i])
        {
            //if (r1fast.real() > r0fast.real())
            //{
            //    swap(r0fast, r1fast);
            //}

            D r0fastabsPow2 = utils::complex::absPow2(r0fast);
            D r1fastabsPow2 = utils::complex::absPow2(r1fast);
            if (r1fastabsPow2 < r0fastabsPow2)
            {
                swap(r0fast, r1fast);
            }
        }

        // return results
        pv[0](i, 0) = r0fast.real();
        pv[0](i, 1) = r0fast.imag();
        pv[1](i, 0) = r1fast.real();
        pv[1](i, 1) = r1fast.imag();
    }


}




void setFieldFromGeneralCoefficients4_fast_fast_fast(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv, bool ignore_x, const Is& isFaceConstrained)
{
    const int n = coeffs.size();
    const int facesCount = coeffs[0].rows();
    pv.assign(n, MatrixX2d::Zero(facesCount, 2));

    //if (ignore_x) // special case where always d1==0, it happend when second directionis not changed
    //{
    //    for (int fid = 0; fid < facesCount; ++fid)
    //    {
    //        // Scalar d1 = 0  - always for ignore_x, so we can simplify our solution
    //        Scalar d2 = -1.0*coeffs[1](fid);
    //        Scalar sqrt_det = utils::complex::sqrtFast(d2); // d1 eqaul to 0, so sqrt is simple of d2
    //        Scalar r0fast = utils::complex::sqrtFast(sqrt_det); // convert 'a' of 2x2 to 'a' of 4x4 (here we calculate 'sqrt(d1+sqrt_det)')
    //        Scalar r1fast = Scalar(r0fast.imag(), -r0fast.real()); // r1fast is miror of r0fast, since 'utils::complex::sqrtFast(-sqrt_det)' is miror of 'utils::complex::sqrtFast(sqrt_det)'
    //                                                                                // here we dont have to sort, since both r0fast and r1fast are of same length
    //        pv[0](fid, 0) = r0fast.real();
    //        pv[0](fid, 1) = r0fast.imag();
    //        pv[1](fid, 0) = r1fast.real();
    //        pv[1](fid, 1) = r1fast.imag();
    //    }
    //}
    //else
    {

        for (int fid = 0; fid < facesCount; ++fid)
        {
            Scalar d1 = coeffs[0](fid);
            Scalar d2 = -coeffs[1](fid);

            D temp = 0;
            ////DEBUG - calc solution using matrix to test agains new solution
            ////        
            //// v2 - 2x2 matrix
            ////
            //// Matrix 4x4 can be simplified to matrix 2x2, with small correction: 'a' of 2x2 will be equal to sqrt('a' of 4x4), so final eigen values should be 'sqrt'
            ////   0  x  0  y          x  y
            ////   1  0  0  0   ==>  1  0
            ////   0  1  0  0
            ////   0  0  1  0
            //Matrix<Scalar, 2, 2> m2;
            //m2(1, 0) = 1;
            //m2(1, 1) = 0;
            //m2(0, 0) = d1;
            //m2(0, 1) = d2;
            //Matrix<Scalar, 2, 1> roots2 = m2.eigenvalues();
            //roots2(0) = sqrt(roots2(0)); // convert 'a' of 2x2 to 'a' of 4x4
            //roots2(1) = sqrt(roots2(1)); // convert 'a' of 2x2 to 'a' of 4x4
            //Scalar r0 = roots2(0);
            //Scalar r1 = roots2(1);

            ////DEBUG - normalize r0 and r1 to test agains newest method
            //Matrix<Scalar, 2, 1> roots2_norm;
            //roots2_norm(0) = r0;
            //roots2_norm(1) = r1;
            //roots2_norm.normalize();
            //Scalar r0_norm = roots2_norm(0);
            //Scalar r1_norm = roots2_norm(1);

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
            Scalar sqrt_det = utils::complex::sqrtFast(d1*d1 + d2);
            Scalar a1_2x2 = (d1 + sqrt_det);
            Scalar a2_2x2 = (d1 - sqrt_det);
            //v0
            //Scalar r0fast = utils::complex::sqrtFast(a1_2x2); // convert 'a' of 2x2 to 'a' of 4x4
            //Scalar r1fast = utils::complex::sqrtFast(a2_2x2); // convert 'a' of 2x2 to 'a' of 4x4
            //v1
            Scalar r0fast;
            Scalar r1fast;
            utils::complex::sqrtFast2(a1_2x2, a2_2x2, r0fast, r1fast);

            //DEBUG - normalize r0fast and r1fast to test agains oldest method
            //Matrix<Scalar, 2, 1> roots2fast_norm;
            //roots2fast_norm(0) = r0fast;
            //roots2fast_norm(1) = r1fast;
            //roots2fast_norm.normalize();
            //Scalar r0fast_norm = roots2fast_norm(0);
            //Scalar r1fast_norm = roots2fast_norm(1);


            // sort complex vectors - this is very important for detecting singularities
            if (options.sort_complex_vectors && !isFaceConstrained[fid])
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

                RealScalar r0fastabsPow2 = utils::complex::absPow2(r0fast);
                RealScalar r1fastabsPow2 = utils::complex::absPow2(r1fast);
                if (r1fastabsPow2 < r0fastabsPow2)
                {
                    swap(r0fast, r1fast);
                }
            }

            // return results
            pv[0](fid, 0) = r0fast.real();
            pv[0](fid, 1) = r0fast.imag();
            pv[1](fid, 0) = r1fast.real();
            pv[1](fid, 1) = r1fast.imag();
        }
    }

}

void setFieldFromGeneralCoefficients(const  vector<VectorType> &coeffs,
    vector<MatrixX2d> &pv, bool ignore_x, const Is& isFaceConstrained)
{
    //setFieldFromGeneralCoefficients_origin(coeffs, pv);
    //setFieldFromGeneralCoefficients2_fast(coeffs, pv);
    //setFieldFromGeneralCoefficients3_fast_fast(coeffs, pv, ignore_x, isFaceConstrained);
    setFieldFromGeneralCoefficients4_fast_fast_fast(coeffs, pv, ignore_x, isFaceConstrained);
}

//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//--- SOLVE -----------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------








bool NPolyVectorFieldSolver::
solve(const Is &isFaceConstrained,
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


    int numConstrained = isFaceConstrained.sum();

    //
    //  minQuadWithKnownMini
    //
    time = utils::time::Now();
    vector<VectorType> coeffs(n, VectorType::Zero(facesCount, 1));
    if (logMessages) cout << "NPolyVectorFieldSolver::solve()  minQuadWithKnownMini  ...";
    //cout << "NPolyVectorFieldSolver::solve()  minQuadWithKnownMini (n=" << to_string(n) << ") ...";
    extern bool IsOmpEnabled;
    bool parallel = IsOmpEnabled;
    LinearEquationSolver solver_shared;
    #pragma omp parallel for schedule(static) if(parallel)
    for (int ni = 0; ni < n; ++ni)
    {
        if (ignore_x && ni == 0) continue;
        Timer timer_getGeneralCoeffConstraints;
        int degree = 2 * (ni + 1);
        VectorType xknown;
        getGeneralCoeffConstraints2_fast(isFaceConstrained, numConstrained,
            faceConstrains,
            ni,
            xknown);
        timer_getGeneralCoeffConstraints.stop(elapsed.getGeneralCoeffConstraints);

        Timer timer_computeFacesLaplacianCoefficient;
        SparceMatrixType Quu, Quk; // best to be col major
        //for(int i = 0; i < 100 ; i++)
        computeFacesLaplacianCoefficient(facesCount, EF, FN, K, isBorderEdge,
            isFaceConstrained, numConstrained, degree, Quu, Quk);
        timer_computeFacesLaplacianCoefficient.stop(elapsed.computeFacesLaplacianCoefficient);
        //for (Index i = 0; i < Quu.outerIndexPtr()[Quu.cols()]; i++)
        //{
        //    cout << "Quu[" << i << "]=" << Quu.valuePtr()[i] << endl;
        //}
        //Timer timer_minQuadWithKnownMini;
        //cout << "i = " << i << endl;
        LinearEquationSolver solver;
        solver.compute(Quu, logMessages);
        solver.solve(isFaceConstrained, Quk, xknown, coeffs[ni], logMessages);
        //timer_minQuadWithKnownMini.stop(elapsed.minQuadWithKnownMini);
    }
    if (logMessages) cout << "  done in " << utils::time::ElapsedSecondsStr(time) << endl;


    //
    //  setFieldFromGeneralCoefficients
    //
    if (logMessages) cout << "NPolyVectorFieldSolver::solve()  setFieldFromGeneralCoefficients  ...";
    Timer timer_setFieldFromGeneralCoefficients;
    vector<MatrixX2d > pv;
    setFieldFromGeneralCoefficients(coeffs, pv, ignore_x, isFaceConstrained);
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
        RealScalar* pv0 = pv[0].data();
        RealScalar* pv1 = pv[1].data();
        for (int fid = 0; fid < facesCount; ++fid)
        {
            D x0 = *pv0;
            D y0 = *(pv0 + 1);
            D x1 = *pv1;
            D y1 = *(pv1 + 1);

            // sort vectors
            D cross01 = x0 * y1 - y0 * x1;
            if (cross01 < 0)
            {
                x1 = -x1;
                y1 = -y1;
            }
            pv0 += 2;
            pv1 += 2;

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
            const V3 &b1 = F_X.row(fid);
            const V3 &b2 = F_Y.row(fid);
            for (int i = 0; i < n; i++)
            {
                D f1 = pv[i](fid, 0);
                D f2 = pv[i](fid, 1);
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


D NPolyVectorFieldSolver::getFacePartialArea(int fid, int eid)
{
    P3 p1 = V.row(F(fid, 0));
    P3 p2 = V.row(F(fid, 1));
    P3 p3 = V.row(F(fid, 2));
    P3 pMiddle = (p1 + p2 + p3) / 3;
    P3 edgep1 = V.row(EV(eid, 0));
    P3 edgep2 = V.row(EV(eid, 1));
    P3 closest = utils::vector::ClosestPoint_ToLine(edgep1, edgep2, pMiddle);
    D area = ((closest - pMiddle).norm() * (edgep1 - edgep2).norm()) / 2;
    return area;
}



//this gives the coefficients without the (-1)^k that multiplies them
void NPolyVectorFieldSolver::getGeneralCoeffConstraints(const Is& isFaceConstrained, int numConstrained,
    const MatrixXd &faceConstrains,
    int ni,
    VectorType &xknown)
{
    xknown.resize(numConstrained, 1);

    MatrixXi allCombs;
    {
        VectorXi V = VectorXi::LinSpaced(n, 0, n - 1);
        igl::nchoosek(V, ni + 1, allCombs);
    }

    int ind = 0;
    for (int fid = 0; fid < facesCount; ++fid)
    {
        if (isFaceConstrained[fid])
        {
            const V3& x = F_X.row(fid);
            const V3& y = F_Y.row(fid);
            //cout << "b1.dot(b2) = " << b1.dot(b2) << endl;
            Scalar ck(0);

            for (int j = 0; j < allCombs.rows(); ++j)
            {
                Scalar tk(1.);
                //collect products
                for (int i = 0; i < allCombs.cols(); ++i)
                {
                    int index = allCombs(j, i);
                    const Vector3d& w = faceConstrains.block<1,3>(fid, 3 * index).transpose();
                    V3 wv3 = convertEigenToV3(w);
                    D wx = wv3.dot(x);
                    D wy = wv3.dot(y);

                    if (abs(wx) < 0.0000000001) wx = 0;
                    if (abs(wy) < 0.0000000001) wy = 0;
                    if (abs(1 - wx) < 0.0000000001) wx = 1;
                    if (abs(1 - wy) < 0.0000000001) wy = 1;
                    Scalar u(static_cast<RealScalar>(wx), static_cast<RealScalar>(wy));
                    tk *= u * u;
                }
                //collect sum
                ck += tk;
            }
            xknown(ind) = ck;
            ind++;
        }
    }


}

__forceinline Scalar NPolyVectorFieldSolver::getComplex(int index, const MatrixXd &faceConstrains, const V3& x, const V3& y, int fid)
{
    const Vector3d& w = faceConstrains.block<1,3>(fid, 3 * index).transpose();
    V3 wv3 = convertEigenToV3(w);
    D wx = wv3.dot(x);
    D wy = wv3.dot(y);

    if (abs(wx) < 0.0000000001) wx = 0;
    if (abs(wy) < 0.0000000001) wy = 0;
    if (abs(1 - wx) < 0.0000000001) wx = 1;
    if (abs(1 - wy) < 0.0000000001) wy = 1;
    return Scalar(static_cast<RealScalar>(wx), static_cast<RealScalar>(wy));
};

//this gives the coefficients without the (-1)^k that multiplies them
void NPolyVectorFieldSolver::getGeneralCoeffConstraints2_fast(const Is& isFaceConstrained, int numConstrained,
    const MatrixXd &faceConstrains,
    int ni,
    VectorType &xknown)
{
    assert(ni <= 1);
    xknown.resize(numConstrained, 1);

    int ind = 0;
    for (int fid = 0; fid < facesCount; ++fid)
    {
        if (!isFaceConstrained[fid]) continue;
        const V3& x = F_X.row(fid);
        const V3& y = F_Y.row(fid);
        Scalar weight(1.);
        Scalar c0 = getComplex(0, faceConstrains, x, y, fid);
        Scalar c1 = getComplex(1, faceConstrains, x, y, fid);
        if (ni == 0)
            xknown(ind) = weight * (c0*c0 + c1 * c1);
        else
            xknown(ind) = weight * c0*c0 * weight * c1 * c1;
        ind++;
    }
}


void NPolyVectorFieldSolver::precomputeInteriorEdges()
{
    //numInteriorEdges = 0;
    //indFullToInterior = -1 * VectorXi::Ones(edgesCount, 1);

    //for (int i = 0; i < edgesCount; ++i)
    //{
    //    if ((EF(i, 0) == -1) || ((EF(i, 1) == -1)))
    //    {
    //    }
    //    else
    //    {
    //        indFullToInterior[i] = numInteriorEdges;
    //        numInteriorEdges++;
    //    }
    //}

    //E2F_int.resize(numInteriorEdges, 2);
    //indInteriorToFull.setZero(numInteriorEdges, 1);
    //int ii = 0;
    //for (int k = 0; k < edgesCount; ++k)
    //{
    //    if (isBorderEdge[k])
    //        continue;
    //    E2F_int.row(ii) = EF.row(k);
    //    indInteriorToFull[ii] = k;
    //    ii++;
    //}

}



void n_polyvector_soft(
    ViewerDrawObjects& draw,
    const P3s &V,
    const I3s& F,
    const Ds &F_Areas,
    const Bs& E_isborder,
    const I2s& EV,
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
    Matrix<D, Dynamic, Dynamic> edgeWeights = Matrix<D, Dynamic, Dynamic>::Constant(Constrains.size(), 2, 1);
    if (WeightIsRelativeToEdgeLength)
    {
        D maxedgeLength = 0;
        D maxfaceArea = 0;
        for (int i = 0; i < Constrains.size(); ++i)
        {
            int fid = Constrains[i].FaceId;
            int eid = FE(fid, 0);
            V3 edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
            D edgeLength = utils::vector::Length(edgeDirection);
            D faceArea = F_Areas(fid);
            maxedgeLength = max(maxedgeLength, edgeLength);
            maxfaceArea = max(maxfaceArea, faceArea);
        }
        for (int i = 0; i < Constrains.size(); ++i)
        {
            int fid = Constrains[i].FaceId;
            int eid = FE(fid, 0);
            V3 edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
            D edgeLength = utils::vector::Length(edgeDirection);
            D faceArea = F_Areas(fid);
            //D edgeDirectionWeight = edgeLength / maxedgeLength;
            //D directionWeight = faceArea / maxfaceArea;
            //D weight = faceArea / maxfaceArea;// works bad
            D weight = edgeLength / maxedgeLength;// works ok
            D edgeDirectionWeight = weight;
            D directionWeight = weight;
            edgeWeights.row(i) << edgeDirectionWeight, directionWeight;
        }
    } 

    // Set solver contrains base on face constrains and edge directions
    Is isFaceConstrained = Is::Constant(F.rows(), 0);
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
            edgeDirection *= edgeWeights(i, 0);
            direction *= edgeWeights(i, 1);
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
        isFaceConstrained(fid) = 1;
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
    //        RowVector3d directoin = bc2.block<1,3>(i, 3 * ni);
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


    NPolyVectorFieldSolver pvff(V, F, 2, E_isborder, EV, FE, EF, K, F_X, F_Y, F_Z, avg_edge_length, max_edge_length);
    timer.stop(elapsed.initSolver);
    pvff.solve(isFaceConstrained, faceConstrains, Result_F, Result_F_isSorted, logMessages, ignore_x);// ignore x because it is always 0 for uncorrectedContrains

}