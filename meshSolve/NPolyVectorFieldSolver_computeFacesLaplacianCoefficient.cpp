#include "stdafx.h"
#include "NPolyVectorFieldSolver_computeFacesLaplacianCoefficient.h"
#include "FaceConstrain.h"

const MeshLogicOptions_Solver& options3 = meshLogicOptions.Solver;




void computeFacesLaplacianCoefficient_origin(int facesCount, int edgesCount, const MatrixXi& EF, const VectorXd& K, const VectorXi& isBorderEdge,
    const VectorXi &isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    VectorXi known;
    known.resize(knownCount, 1);
    VectorXi unknown;
    unknown.resize(unknownCount, 1);
    VectorXi fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount, 1);

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

void computeFacesLaplacianCoefficient_fast(int facesCount, int edgesCount, const MatrixXi& EF, const VectorXd& K, const VectorXi& isBorderEdge,
    const VectorXi &isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    VectorXi known;
    known.resize(knownCount, 1);
    VectorXi unknown;
    unknown.resize(unknownCount, 1);
    VectorXi fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount, 1);
    VectorXi fid_to_known_index;
    fid_to_known_index.resize(facesCount, 1);

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


    //VectorXb fxfxIsAdded = VectorXb::Constant(facesCount, false);
    //VectorXb fxfxIsAdded_Quu = VectorXb::Constant(facesCount, false);
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



void computeFacesLaplacianCoefficient_fast_fast(int facesCount, int edgesCount, const MatrixXi& EF, const VectorXd& K, const VectorXi& isBorderEdge,
    const VectorXi &isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk
)
{
    int knownCount = numConstrained;
    int unknownCount = facesCount - knownCount;
    VectorXi known;
    known.resize(knownCount, 1);
    VectorXi unknown;
    unknown.resize(unknownCount, 1);
    VectorXi fid_to_unknown_index;
    fid_to_unknown_index.resize(facesCount, 1);
    VectorXi fid_to_known_index;
    fid_to_known_index.resize(facesCount, 1);

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
    VectorXi quuCountValuesInCol = VectorXi::Constant(unknownCount, 0);
    Matrix <P, -1, -1, ColMajor> quk;
    quk.resize(4, knownCount);
    VectorXi qukCountValuesInCol = VectorXi::Constant(knownCount, 0);

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

            double real = 1.;
            // v1 - edges avg - works very bad
            //if (WeightIsRelativeToEdgeLength)
            //{
            //    double edgeLength = (V.row(EV(eid, 0)) - V.row(EV(eid, 1))).norm();
            //    real = edgeLength / avg_edge_length;
            //}
            // v2 - edges max - works better but still wrong
            //if (WeightIsRelativeToEdgeLength)
            //{
            //    double edgeLength = (V.row(EV(eid, 0)) - V.row(EV(eid, 1))).norm();
            //    real = edgeLength / max_edge_length;
            //}
            // v3 - triangles areas
            //if (WeightIsRelativeToEdgeLength)
            //{
            //    double area0 = getFacePartialArea(fid0, eid)*100;
            //    double area1 = getFacePartialArea(fid1, eid)*100;
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

    auto populate = [](int facesCount, SparceMatrixType& S, int rows, int cols, const VectorXi& countValuesInCol, Matrix <P, -1, -1, ColMajor>& q)
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
  

  
void computeFacesLaplacianCoefficient(int facesCount, const MatrixXi& EF, const VectorXd& K, const VectorXi& isBorderEdge,
    const VectorXi &isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk)
{
    int edgesCount = isBorderEdge.size();
    computeFacesLaplacianCoefficient_fast(facesCount, edgesCount, EF, K, isBorderEdge,
        isFaceConstrained, numConstrained, degree, Quu, Quk);
}
