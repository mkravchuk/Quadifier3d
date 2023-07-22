#include "stdafx.h"
#include "LDLTFloatSolver.h"
#include "complexvec.h"

//const MeshLogicOptions_LDLTFloatSolver& options = meshLogicOptions.LDLTFloatSolver;
#define options meshLogicOptions.LDLTFloatSolver



void LDLTFloatSolver::compute(const SparceMatrixType& a)
{
    assert(a.rows() == a.cols());
    CholSparceMatrixType ap;
    analyzePattern(a, ap);
    factorize_preordered(ap);
}

void LDLTFloatSolver::recompute(const SparceMatrixType& a)
{
    factorize(a);
}

void LDLTFloatSolver::create_permuted_a(const SparceMatrixType& a, CholSparceMatrixType& ap, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo)
{
    const Index size = a.rows();
    ap.resize(size, size);
    //ap.selfadjointView<Upper>() = a.selfadjointView<UpLo>().twistedBy(m_P);
    ap.selfadjointView<Upper>() = a_uplo.twistedBy(m_P);

    //for (Index i = 0; i < a.outerIndexPtr()[a.cols()]; i++)
    //for (Index i = 0; i < 40; i++)
    //{
    //    cout << "a[" << i << "]=" << a.valuePtr()[i] << endl;
    //    cout << "ap[" << i << "]=" << ap.valuePtr()[i] << endl;
    //}
}

void LDLTFloatSolver::analyzePattern(const SparceMatrixType& a, CholSparceMatrixType& ap)
{
    assert(a.rows() == a.cols());

    // creates permute matrices 'm_P', 'm_Pinv'
    SparseSelfAdjointView<SparceMatrixType, UpLo> a_uplo = a.selfadjointView<UpLo>();
    ordering(a, a_uplo);

    // creates 'aPermuted'
    create_permuted_a(a, ap, a_uplo);

    // DEBUG skip permuting 
    if (options.DebugEnabled && options.debug_skip_permuting)
    {
        ordering(ap, ap.selfadjointView<UpLo>());
        create_permuted_a(a, ap, ap.selfadjointView<UpLo>());
    }

    // allocate matrix 'L', and create tree 'm_parent
    analyzePattern_preordered(ap);
}

void LDLTFloatSolver::analyzePattern_preordered(const CholSparceMatrixType& ap)
{
    const Index size = ap.rows();
    L.resize(size, size);
    m_parent.resize(size);
    L_nonZerosPerCol.resize(size);

    ei_declare_aligned_stack_constructed_variable(Index, tags, size, 0);

    for (Index k = 0; k < size; ++k)
    {
        /* L(k,:) pattern: all nodes reachable in etree from nz in A(0:k-1,k) */
        m_parent[k] = -1;             /* parent of k is not yet known */
        tags[k] = k;                  /* mark node k as visited */
        L_nonZerosPerCol[k] = 0;      /* count of nonzeros in column k of L */
        for (CholSparceMatrixType::InnerIterator it(ap, k); it; ++it)
        {
            Index i = it.index();
            if (i < k)
            {
                /* follow path from i to root of etree, stop at flagged node */
                for (; tags[i] != k; i = m_parent[i])
                {
                    /* find parent of i if not yet determined */
                    if (m_parent[i] == -1)
                        m_parent[i] = k;
                    L_nonZerosPerCol[i]++;        /* L (k,i) is nonzero */
                    tags[i] = k;                  /* mark i as visited */
                }
            }
        }
    }

    /* construct Lp index array from L_nonZerosPerCol column counts */
    Index* Lp = L.outerIndexPtr();
    Lp[0] = 0;
    for (Index k = 0; k < size; ++k)
        Lp[k + 1] = Lp[k] + L_nonZerosPerCol[k];

    L.resizeNonZeros(Lp[size]);

    m_isInitialized = true;
    m_info = Success;
    m_analysisIsOk = true;
    m_factorizationIsOk = false;
}

void LDLTFloatSolver::ordering(const SparceMatrixType& a, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo)
{
    bool TRACE = options.DebugEnabled && options.debug_trace_factorize_preordered;

    assert(a.rows() == a.cols());
    const Index size = a.rows();
    // Note that amd compute the inverse permutation
    {
        //auto a_uplo = a.selfadjointView<UpLo>();
        //for (Index i = 0; i < 40; i++)
        //{
        //    cout << "a_uplo[" << i << "]=" << a_uplo.valuePtr()[i] << endl;
        //}
        Ordering ordering;
        ordering(a_uplo, m_Pinv);
    }

    // DEBUG show 'm_Pinv.indices'
    if (TRACE)
    {
        cout << "m_Pinv.indices={";
        for (int i = 0; i < m_Pinv.indices().size(); i++)
        {
            if (i != 0) cout << ",";
            cout << m_Pinv.indices()[i];
        }
        cout << "}" << endl;
    }

    // Inverse indices
    if (m_Pinv.size() > 0)
        m_P = m_Pinv.inverse();
    else
        m_P.resize(0);

    // DEBUG show 'm_Pinv.indices'
    if (TRACE)
    {
        cout << "m_P.indices={";
        for (int i = 0; i < m_P.indices().size(); i++)
        {
            if (i != 0) cout << ",";
            cout << m_P.indices()[i];
        }
        cout << "}" << endl;
    }
}

void LDLTFloatSolver::factorize(const SparceMatrixType& a)
{
    assert(a.rows() == a.cols());
    CholSparceMatrixType ap;
    SparseSelfAdjointView<SparceMatrixType, UpLo> a_uplo = a.selfadjointView<UpLo>();
    create_permuted_a(a, ap, a_uplo);
    factorize_preordered(ap);
}

void LDLTFloatSolver::factorize_preordered(const CholSparceMatrixType& ap)
{
    chrono::high_resolution_clock::time_point time;
    if (options.DebugEnabled && options.debug_trace_iterationsCount)
    {
        time = utils::time::Now();
    }

    if (options.fast_factorize_preordered)
        factorize_preordered_fast(ap);
    else
        factorize_preordered_origin(ap);

    if (options.DebugEnabled && options.debug_trace_iterationsCount)
    {
        cout << endl << "factorize_preordered  time=" << utils::time::ElapsedSecondsStr(time) << endl << endl;
    }
}

void LDLTFloatSolver::factorize_preordered_origin(const CholSparceMatrixType& ap)
{
    assert(m_analysisIsOk && "You must first call analyzePattern()");
    assert(ap.rows() == ap.cols());
    const Index size = ap.rows();
    assert(m_parent.size() == size);
    assert(L_nonZerosPerCol.size() == size);

    const Index* Lp = L.outerIndexPtr();
    Index* Li = L.innerIndexPtr();
    Scalar* Lx = L.valuePtr();
    //for (Index i = 0; i < ap.outerIndexPtr()[size]; i++)
    //{
    //    cout << "Lx[" << i << "]=" << ap.valuePtr()[i] << endl;
    //}

    ei_declare_aligned_stack_constructed_variable(Scalar, y, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, pattern, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, tags, size, 0);

    bool ok = true;
    D.resize(size);

    for (Index k = 0; k < size; ++k)
    {
        // compute nonzero pattern of kth row of L, in topological order
        y[k] = 0.0;                     // Y(0:k) is now all zero
        Index top = size;               // stack for pattern is empty
        tags[k] = k;                    // mark node k as visited
        L_nonZerosPerCol[k] = 0;        // count of nonzeros in column k of L
        for (SparceMatrixType::InnerIterator it(ap, k); it; ++it)
        {
            Index i = it.index();
            if (i <= k)
            {
                //cout << "it.value()=" << it.value() << "  it.value().conj=" << numext::conj(it.value()) << endl;
                y[i] += numext::conj(it.value());            /* scatter A(i,k) into Y (sum duplicates) */
                Index len;
                for (len = 0; tags[i] != k; i = m_parent[i])
                {
                    pattern[len++] = i;     /* L(k,i) is nonzero */
                    tags[i] = k;            /* mark i as visited */
                }
                while (len > 0)
                    pattern[--top] = pattern[--len];
            }
        }

        /* compute numerical values kth row of L (a sparse triangular solve) */

        RealScalar d = numext::real(y[k]);    // get D(k,k), and clear Y(k)
        //cout << "k#" << k << "   y[k]=" << y[k] << "   d=" << d << endl;
        y[k] = 0.0;
        for (; top < size; ++top)
        {
            Index i = pattern[top];       /* pattern[top:n-1] is pattern of L(:,k) */
            Scalar yi = y[i];             /* get and clear Y(i) */
            y[i] = 0.0;

            Scalar l_ki = yi / D[i];

            Index p2 = Lp[i] + L_nonZerosPerCol[i];
            Index p;
            for (p = Lp[i]; p < p2; ++p)
            {
                y[Li[p]] -= numext::conj(Lx[p]) * yi;
                //cout << "   p#" << p << "   sub=" << (numext::conj(Lx[p]) * yi) << endl;
            }
            d -= numext::real(l_ki * numext::conj(yi));
            //cout << "   top#" << top << "   yi=" << yi << "   dl_ki=" << l_ki << "   d=" << d << endl;
            Li[p] = k;                          /* store L(k,i) in column form of L */
            Lx[p] = l_ki;
            ++L_nonZerosPerCol[i];              /* increment count of nonzeros in col i */
        }

        D[k] = d;
        if (d == RealScalar(0))
        {
            ok = false;                         /* failure, D(k,k) is zero */
            break;
        }
    }
    //for (Index i = 0; i < ap.outerIndexPtr()[size]; i++)
    //{
    //    cout << "Lx[" << i << "]=" << ap.valuePtr()[i] << endl;
    //}
    m_info = ok ? Success : NumericalIssue;
    m_factorizationIsOk = true;
}
void LDLTFloatSolver::factorize_preordered_fast(const CholSparceMatrixType& ap)
{
    bool use_AVX = options.fast_factorize_preordered_use_AVX;
    bool TRACE = options.DebugEnabled && options.debug_trace_factorize_preordered;

    assert(m_analysisIsOk && "You must first call analyzePattern()");
    assert(ap.rows() == ap.cols());
    const Index size = ap.rows();
    assert(m_parent.size() == size);
    assert(L_nonZerosPerCol.size() == size);

    const Index* Lp = L.outerIndexPtr();
    Index* Li = L.innerIndexPtr();
    Scalar* Lx = L.valuePtr();

    ei_declare_aligned_stack_constructed_variable(Scalar, y, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, pattern, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, tags, size, 0);

    D.resize(size);
    long long iterationsCount = 0;
    if (TRACE) cout << "----- factorize_preordered_fast --- size=" << size << endl;
    //extern bool IsOmpEnabled;
    bool IsOmpEnabled = options.fast_factorize_preordered__multithreading;

    for (Index k = 0; k < size; ++k)
    {
        if (TRACE) cout << "k=" << k << endl;



        //
        // compute nonzero pattern of kth row of L, in topological order
        //
        y[k] = 0.0;                               // Y(0:k) is now all zero
        Index top = size;                       // stack for pattern is empty
        tags[k] = k;                             // mark node k as visited
        L_nonZerosPerCol[k] = 0;          // count of nonzeros in column k of L
        for (SparceMatrixType::InnerIterator it(ap, k); it; ++it)
        {
            Index i = it.index();
            if (i <= k)
            { 
                y[i] += numext::conj(it.value());            /* scatter A(i,k) into Y (sum duplicates) */
                Index len;
                for (len = 0; tags[i] != k; i = m_parent[i])
                {
                    pattern[len++] = i;     /* L(k,i) is nonzero */
                    tags[i] = k;            /* mark i as visited */
                }
                while (len > 0)
                    pattern[--top] = pattern[--len];
            }
        }

        //
        // compute numerical values kth row of L (a sparse triangular solve)
        //
        RealScalar d = numext::real(y[k]);    // get D(k,k), and clear Y(k)
        y[k] = 0.0;

        //#pragma omp parallel for  if(IsOmpEnabled) reduction(-:d)
        for (int patternI = top; patternI < size; patternI++)
        {
            //if (TRACE) cout << "  patternI=" << patternI << "   i=" << pattern[patternI] << endl;
            if (TRACE) cout << "  column=" << pattern[patternI] << endl;

            Index columnIndex = pattern[patternI];       /* pattern[top:n-1] is pattern of L(:,k), column of SparceMatrix */
            Index p = Lp[columnIndex];                                                 // index to sparce column data #i 
            Index pEnd = p + L_nonZerosPerCol[columnIndex];                  // end of the column data

            Scalar yi = y[columnIndex];             /* get and clear Y(i) */
            Complex2f yiX(&yi);
            Complex4f yiX2(yiX, yiX);
            #if INSTRSET >= 8
            Complex8f yiX4;
            if (use_AVX)
            {
                yiX4 = Complex8f(yiX2, yiX2); 
            }
            #endif
            y[columnIndex] = 0.0;

            // v0 - origin
            //Scalar l_ki = yi / D[i];
            // v1 - optimized using SSE (5% of total solve speed)
            //Scalar l_ki;
            //Complex2f m_diagX(D.data() + columnIndex);
            //Complex2f l_kiX = yiX / m_diagX;
            //l_kiX.store(&l_ki);
            // v2 - using fact that D is just RealScalar - not a complex but a numerix value, so in complex it will have zero imaginary value:  (d, 0*i)
            RealScalar c = D[columnIndex].real();
            Scalar l_ki = Scalar(yi.real() / c, yi.imag() / c);




            //v0 - origin
            //for (; p < pEnd; ++p)                                            // for each value in column
            //{
            //    if (TRACE)
            //    {
            //        iterationsCount++;
            //        cout << "      p=" << p << "  Li[p]=" << Li[p] << endl;//<<"   len="<< (p2-p)
            //    }
            //    y[Li[p]] -= numext::conj(Lx[p]) * yi;
            //}

            // v1 - fast using SSE, unroling, and 'until' instead of 'for'
            //if (p < pEnd)
            //{
            //    do
            //    {
            //        //y[Li[p]] -= numext::conj(Lx[p]) * yi;
            //        Complex2f Lx_p_X;
            //        Lx_p_X.load(reinterpret_cast<float*>(&Lx[p]));
            //        Lx_p_X.conjugate();
            //        Scalar* y_Li_p = &y[Li[p]];
            //        Complex2f y_Li_p_X;
            //        y_Li_p_X.load(reinterpret_cast<float*>(y_Li_p));
            //        y_Li_p_X -= Lx_p_X * yiX;
            //        y_Li_p_X.store(reinterpret_cast<float*>(y_Li_p));
            //        p++;
            //    } while (p < pEnd);
            //}

            //v2
            //Index p;
            //for (p = Lp[i]; p < pEnd; ++p)
            //{
            //    //y[Li[p]] -= numext::conj(Lx[p]) * yi;
            //    Scalar* y_Li_p = &y[Li[p]];
            //    Complex2f Lx_p_X(&Lx[p]);
            //    Complex2f y_Li_p_X(y_Li_p);
            //    Lx_p_X.conjugate();
            //    y_Li_p_X -= Lx_p_X * yiX;
            //    y_Li_p_X.store(y_Li_p);
            //}

            // v3 - fast using SSE, unroling, and 'until' instead of 'for'
            //Index pEnd2 = pEnd - 1;                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once
            //if (p < pEnd2)
            //{
            //    do
            //    {
            //        //y[Li[p]] -= numext::conj(Lx[p]) * yi;
            //        //y[Li[p+1]] -= numext::conj(Lx[p+1]) * yi;
            //        int i1 = Li[p];
            //        int i2 = Li[p + 1];
            //        Complex4f x2(&Lx[p]);
            //        Scalar* y1 = &y[i1];
            //        Scalar* y2 = &y[i2];
            //        x2.conjugate();
            //        Complex4f yX2(y1, y2);
            //        yX2 -= x2 * yiX2;
            //        yX2.store_low(y1);
            //        yX2.store_high(y2);
            //        p += 2;
            //        iterationsCount += 2;
            //    } while (p < pEnd2);
            //}
            //if (p < pEnd)
            //{
            //    do
            //    {
            //        //y[Li[p]] -= numext::conj(Lx[p]) * yi;
            //        Scalar* y1 = &y[Li[p]];
            //        Complex2f x(&Lx[p]);
            //        x.conjugate();
            //        Complex2f yX(y1);
            //        yX -= x * yiX;
            //        yX.store(y1);
            //        p++;
            //        iterationsCount++;
            //    } while (p < pEnd);
            //}


            // v4 - fast using SSE, unroling, optimal ordering of commands, pointers instead of indexes, and 'until' instead of 'for'
            #if INSTRSET >= 8
            Index pEnd4 = pEnd - 3;                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once
            if (p < pEnd4 && use_AVX) // proceed 2 items at once
            {
                Index* pLi = &Li[p];
                Scalar* pLx = &Lx[p];
                Scalar* pLxEnd4 = &Lx[pEnd4];                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once
                do
                {
                    //y[Li[p]] -= numext::conj(Lx[p]) * yi;
                    //y[Li[p+1]] -= numext::conj(Lx[p+1]) * yi;
                    Scalar* y1 = &y[*pLi];
                    Scalar* y2 = &y[*(pLi + 1)];
                    Scalar* y3 = &y[*(pLi + 2)];
                    Scalar* y4 = &y[*(pLi + 3)];
                    Complex8f x4(pLx);
                    x4.conjugate();

                    Complex8f yX4(y1, y2, y3, y4);
                    yX4 -= x4 * yiX4;
                    pLx += 4;
                    auto low = yX4.get_low();
                    auto high = yX4.get_high();
                    low.store_low(y1);
                    low.store_high(y2);
                    high.store_low(y3);
                    high.store_high(y4);
                } while (pLx < pLxEnd4);
                iterationsCount += pEnd - ((pEnd - p) % 4) - p;
                p = pEnd - ((pEnd - p) % 4);
            }
            #endif
            Index pEnd2 = pEnd - 1;                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once
            if (p < pEnd2) // proceed 2 items at once
            {
                Index* pLi = &Li[p];
                Scalar* pLx = &Lx[p];
                Scalar* pLxEnd2 = &Lx[pEnd2];                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once
                do
                {
                    //y[Li[p]] -= numext::conj(Lx[p]) * yi;
                    //y[Li[p+1]] -= numext::conj(Lx[p+1]) * yi;
                    Scalar* y1 = &y[*pLi];
                    Scalar* y2 = &y[*(pLi + 1)];
                    pLi += 2;
                    Complex4f yX2(y1, y2);
                    Complex4f x2(pLx);
                    x2.conjugate();
                    yX2 -= x2 * yiX2;
                    pLx += 2;
                    yX2.store_low(y1);
                    yX2.store_high(y2);
                } while (pLx < pLxEnd2);
                iterationsCount += pEnd - ((pEnd - p) & 1) - p;
                p = pEnd - ((pEnd - p) & 1);
            }
            if (p < pEnd) // proceed last item
            {
                iterationsCount += pEnd - p;
                //y[Li[p]] -= numext::conj(Lx[p]) * yi;
                Scalar* y1 = &y[Li[p]];
                Complex2f x(&Lx[p]);
                x.conjugate();
                Complex2f yX(y1);
                yX -= x * yiX;
                yX.store(y1);
                p++;
            }
            /**/


            // store result into matrix L, and D
            d = d - numext::real(l_ki *numext::conj(yi));
            Li[p] = k;                          /* store L(k,i) in column form of L */
            Lx[p] = l_ki;
            ++L_nonZerosPerCol[columnIndex];              /* increment count of nonzeros in col i */
        }

        D[k] = d;
    }

    if (options.DebugEnabled && options.debug_trace_iterationsCount)
    {
        cout << "iterationsCount = " << iterationsCount << "   size = " << size << endl;
    }

    m_info = Success;
    m_factorizationIsOk = true;
}


void LDLTFloatSolver::solve(const SolveMatrixType &rhs, SolveMatrixType &res) const
{
    assert(m_isInitialized && "Simplicial LDLT is not initialized.");
    assert(rows() == rhs.rows() && "SimplicialCholesky::solve(): invalid number of rows of the right hand side matrix b");
    assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");
    assert(L.rows() == rhs.rows());

    if (m_info != Success)
        return;

    if (m_P.size() > 0)
        res = m_P * rhs;
    else
        res = rhs;

    if (L.nonZeros() > 0) // otherwise L==I
        matrixL().solveInPlace(res);

    if (D.size() > 0)
        res = D.asDiagonal().inverse() * res;

    if (L.nonZeros() > 0) // otherwise U==I
        matrixU().solveInPlace(res);

    if (m_P.size() > 0)
        res = m_Pinv * res;
}