#include "stdafx.h"
#include "testEigen.h"
#include "Utils.h"
#include "CompactVectorVector.h"
//#include "arrayfire.h"


void testEigenMatrix_Default()
{  
    MatrixXi m;
    m.resize(5, 3);
    m <<
        11, 12, 13,
        21, 22, 23,
        31, 32, 33,
        41, 42, 43,
        51, 52, 53;

    Vector3i row = m.row(0);
    Vector3i rowadd = row * 2;
    RowVector3i rowrow = m.row(0);
    RowVector3i rowrowadd = m.row(0) * 2;
    Vector5i col = m.col(0);
    Vector5i coladd = m.col(0) * 2;
    int colCount = m.cols();
    int rowCount = m.rows();

    MatrixXi mT;
    mT = m.transpose();
    Vector5i rowT = mT.row(0);

    // this method is SLOW for every access (reading vector from matrix by m.row() is slow)
    RowVectorXi colSumm = rowadd.array() + rowrowadd.transpose().array();
}
void testEigenMatrix_RowMajor()
{
    Matrix<int, Dynamic, 3, RowMajor> m;
    m.resize(5, 3);
    m <<
        11, 12, 13,
        21, 22, 23,
        31, 32, 33,
        41, 42, 43,
        51, 52, 53;
    RowVector3i row2 = m.row(0);
    RowVector3i row2add = row2 * 2;
    Vector3i row = m.row(0);
    Vector3i rowadd = row * 2;
    Vector5i col = m.col(0);    // longer 2 times from accessing row since our Marix is row alligmnet
    VectorXi coldynamic = m.col(0); // much longer since allocates dynamic memory
    int colCount = m.cols();
    int rowCount = m.rows();
    //RowVectorXi colSumm = rowadd.array() +  row2add.transpose().array() + col.array() + coldynamic.array();
    // this method is fast for every access
    // requires matrix TRANSPOSE when we multiply matrix by vector
}
void testEigenMatrix_ColMajor()
{
    Matrix<int, 3, Dynamic, ColMajor> m;
    m.resize(3, 5);
    m <<
        11, 21, 31, 32, 33,
        12, 22, 41, 42, 43,
        13, 23, 51, 52, 53;
    Vector5i row = m.row(0);
    Vector3i col = m.col(0);
    int colCount = m.cols();
    int rowCount = m.rows();
    // this method is fast for every access
    // this method is fast for matrix by vector multiplication - because doesnt requires matrix transpose
    // requires to use col() instead of row()
    //RowVectorXi colSumm = row.array() + col.array() ;
}
void testEigenMatrix_PassMatrix()
{
    auto printM = [](string title, const MatrixXi& mxi)
    {
        cout << title << "   ";
        cout << mxi.data()[0] << ",";
        cout << mxi.data()[1] << ",";
        cout << mxi.data()[2] << ",";
        cout << mxi.data()[3];
        cout << endl;
    };
    auto printData = [](string title, const int* data)
    {
        cout << title << "   ";
        cout << data[0] << ",";
        cout << data[1] << ",";
        cout << data[2] << ",";
        cout << data[3];
        cout << endl;
    };
    auto transpose_data_doesnt_work = [](string title, MatrixXi& mxi)
    {
        cout << title << "   ";
        cout << mxi.data()[0] << ",";
        cout << mxi.data()[1] << ",";
        cout << mxi.data()[2] << ",";
        cout << mxi.data()[3];
        cout << endl;
        cout << title << "   ";
        cout << mxi.transpose().data()[0];
        cout << mxi.transpose().data()[1];
        cout << mxi.transpose().data()[2];
        cout << mxi.transpose().data()[3];
        cout << endl;
    };

    MatrixXi m;
    m.resize(2, 2);
    m << 1, 2, 3, 4;

    //transpose().data() - doesnt works!
    printData("m.data()  ", m.data());
    printData("m.transpose().data()  ", m.transpose().data());


    printM("m ", m);
    printM("m.transpose", m.transpose());
    transpose_data_doesnt_work("m and m.transpose().data", m);


    MatrixXi mT;
    mT = m.transpose();
    printM("mT", mT);

    Matrix2Xi mCol;
    mCol.resize(2, 2);
    MatrixX2i mRow;
    mRow.resize(2, 2);
    mCol.row(0) = m.row(0);
    mCol.row(1) = m.row(1);
    mRow.row(0) = m.row(0);
    mRow.row(1) = m.row(1);
    printM("mCol", mCol);
    printM("mRow", mRow);

    //printM("mCol*m", mCol*m);
    //printM("mRow*m", mRow*m);

    //test1(m.transpose());
}
void testEigenMatrix_Natvis()
{
    auto printM = [](string title, const MatrixXi& mxi)
    {
        cout << title << "   " << endl;;
        for (int r = 0; r < mxi.rows(); r++)
        {
            cout << "     ";
            for (int c = 0; c < mxi.cols(); c++)
            {
                cout << mxi(r, c);
                if (c != mxi.cols() - 1)
                {
                    cout << ", ";
                }
            }
            cout << endl;
        }
        cout << endl;
    };
    MatrixXi m;
    m.resize(5, 3);
    m <<
        11, 12, 13,
        21, 22, 23,
        31, 32, 33,
        41, 42, 43,
        51, 52, 53;
    printM("m", m);



    vector<vector<int>> mv;
    for (int r = 0; r < m.rows(); r++)
    {
        vector<int> mvr(m.data(), m.data() + m.cols());
        mv.push_back(mvr);
    }


    MatrixX4i TTT;
    TTT.resize(2, 4);
    TTT.row(0) = RowVector4i(0, 1, 2, 3);
    TTT.row(1) = RowVector4i(10, 11, 12, 13);

    MatrixX4d TTTd;
    TTTd.resize(2, 4);
    TTTd.row(0) = RowVector4d(0.1, 1.1, 2.1, 3.1);
    TTTd.row(1) = RowVector4d(10.2, 11.2, 12.2, 13.2);
}

void testEigenMatrix_NewOperators()
{
    MatrixXi m;
    m.resize(3, 3);
    m <<
        11, 12, 13,
        21, 22, 23,
        31, 32, 33;

    for (int i = 0; i < 3; i++)
    {
        // v1  - row
        Vector3i row = m.row(i);
        Vector3i rowadd = row * 1;
        cout << "row  " << rowadd(0) << ", " << rowadd(1) << ", " << rowadd(2) << endl;

        // v2 - row - const 
        const Vector3i& rowConst = m.row(i);
        Vector3i rowaddConst = rowConst * 1;
        cout << "row  " << rowaddConst(0) << ", " << rowaddConst(1) << ", " << rowaddConst(2) << endl;

        // v3 - row - const assigned to row
        const RowVector3i& rowConst2 = m.row(i);
        Vector3i rowaddConst2 = rowConst2 * 1;
        cout << "row  " << rowaddConst2(0) << ", " << rowaddConst2(1) << ", " << rowaddConst2(2) << endl;

        // v4 - row - block
        const Vector3i& row3 = m.block<1,3>(i, 0).transpose();
        Vector3i rowadd3 = row3 * 1;
        cout << "row  " << rowadd3(0) << ", " << rowadd3(1) << ", " << rowadd3(2) << endl;

        // v5 - row - direct get
        Vector3i rowDR = Vector3i(m(i, 0), m(i, 1), m(i, 2));
        Vector3i rowDRadd = rowDR * 1;
        cout << "row  " << rowDRadd(0) << ", " << rowDRadd(1) << ", " << rowDRadd(2) << endl;

        // v6 - col
        Vector3i col = m.col(i);
        Vector3i coladd = col * 1;
        cout << "col  " << coladd(0) << ", " << coladd(1) << ", " << coladd(2) << endl;

        // v7 - col - const
        const Vector3i& colConst = m.col(i);
        Vector3i coladdConst = colConst * 1;
        cout << "col  " << coladdConst(0) << ", " << coladdConst(1) << ", " << coladdConst(2) << endl;

        // v8 - col - block
        const Vector3i& col3 = m.block<3,1>(0, i);
        Vector3i coladd3 = col3 * 1;
        cout << "col  " << coladd3(0) << ", " << coladd3(1) << ", " << coladd3(2) << endl;

        // v6 - col - direct get
        Vector3i rowDC = Vector3i(m(0, i), m(1, i), m(2, i));
        Vector3i rowDCadd = rowDC * 1;
        cout << "row  " << rowDCadd(0) << ", " << rowDCadd(1) << ", " << rowDCadd(2) << endl;


        cout << "end" << endl;
    }
    /*

    Vector3i row = m.row(i);
    mov         r8,qword ptr [rbp-79h]
    add         r8,r15
    mov         ebx,dword ptr [r8]
    movsxd      rcx,dword ptr [rbp-71h]
    mov         edi,dword ptr [r8+rcx*4]
    lea         eax,[rcx+rcx]
    movsxd      rcx,eax
    mov         esi,dword ptr [r8+rcx*4]

    const Vector3i& rowConst = m.row(i);
    mov         r8,qword ptr [rbp-79h]
    add         r8,r15
    mov         ebx,dword ptr [r8]
    movsxd      rcx,dword ptr [rbp-71h]
    mov         edi,dword ptr [r8+rcx*4]
    lea         eax,[rcx+rcx]
    movsxd      rcx,eax
    mov         esi,dword ptr [r8+rcx*4]

    const RowVector3i& rowConst2 = m.row(i);
    mov         r8,qword ptr [rbp-79h]
    add         r8,r15
    mov         ebx,dword ptr [r8]
    movsxd      rcx,dword ptr [rbp-71h]
    mov         edi,dword ptr [r8+rcx*4]
    lea         eax,[rcx+rcx]
    movsxd      rcx,eax
    mov         esi,dword ptr [r8+rcx*4]

    const RowVector3i& row3 = m.block(i, 0, 1, 3);
    mov         r8,qword ptr [rbp-79h]
    add         r8,r15
    mov         ebx,dword ptr [r8]
    movsxd      rcx,dword ptr [rbp-71h]
    mov         edi,dword ptr [r8+rcx*4]
    lea         eax,[rcx+rcx]
    movsxd      rcx,eax
    mov         esi,dword ptr [r8+rcx*4]

    Vector3i rowC = Vector3i(m(i, 0), m(i, 1), m(i, 2));
    mov         r10d,dword ptr [rsp+28h]
    lea         eax,[r15+r10*2]
    movsxd      rcx,eax
    mov         r8,qword ptr [m]
    lea         r9,[r8+rcx*4]
    mov         ebx,dword ptr [r8+r13]
    lea         eax,[r10+r15]
    movsxd      rcx,eax
    mov         edi,dword ptr [r8+rcx*4]
    mov         esi,dword ptr [r9]

    Vector3i col = m.col(i);
    mov         eax,dword ptr [rbp-71h]
    imul        eax,r12d
    movsxd      rcx,eax
    mov         rax,qword ptr [rbp-79h]
    mov         ebx,dword ptr [rax+rcx*4]
    mov         edi,dword ptr [rax+rcx*4+4]
    mov         esi,dword ptr [rax+rcx*4+8]

    const Vector3i& colConst = m.col(i);
    mov         eax,dword ptr [rbp-71h]
    imul        eax,r12d
    movsxd      rcx,eax
    mov         rax,qword ptr [rbp-79h]
    mov         ebx,dword ptr [rax+rcx*4]
    mov         edi,dword ptr [rax+rcx*4+4]
    mov         esi,dword ptr [rax+rcx*4+8]

    const Vector3i& col3 = m.block(i, 0, 3, 1);
    mov         rax,qword ptr [rbp-79h]
    mov         ebx,dword ptr [r15+rax]
    mov         edi,dword ptr [r15+rax+4]
    mov         esi,dword ptr [r15+rax+8]

    Vector3i rowDC = Vector3i(m(0, i), m(1, i), m(2, i));
    imul        edx,r15d
    lea         eax,[rdx+2]
    movsxd      rcx,eax
    mov         r8,qword ptr [m]
    lea         r9,[r8+rcx*4]
    lea         eax,[rdx+1]
    movsxd      rcx,eax
    movsxd      rax,edx
    mov         ebx,dword ptr [r8+rax*4]
    mov         edi,dword ptr [r8+rcx*4]
    mov         esi,dword ptr [r9]


    */

}


void testEigenMatrix()
{
    cout << "------------------------------------------------" << endl;
    cout << "     TESTING EIGEN" << endl;
    cout << "------------------------------------------------" << endl;
    testEigenMatrix_Default();
    testEigenMatrix_RowMajor();
    testEigenMatrix_ColMajor();
    testEigenMatrix_PassMatrix();
    testEigenMatrix_Natvis();
    testEigenMatrix_NewOperators();
    CompactVectorVector<int>::run_example_of_usage();

    V3 x(0., 1.1, 2.3);
    x.normalize();
    cout << "utils::vector::Dot(x, x) = " << utils::vector::Dot(x, x) << endl;
    cout << "utils::vector::Cross(x, x) = " << utils::vector::Cross(x, x) << endl;
    cout << "------------------------------------------------" << endl;
}


