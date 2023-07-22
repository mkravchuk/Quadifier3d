#include "stdafx.h"
#include "LDLTFloatSolver_Polar2x.h"
#include "complexvec.h"
#include "Amd_Polar2x.h"

//const MeshLogicOptions_LDLTFloatSolver& options = meshLogicOptions.LDLTFloatSolver;
#define options meshLogicOptions.LDLTFloatSolver


void LDLTFloatSolver_Polar2x::compute(const SparceMatrixType& a, bool reuse_ordering)
{
    assert(a.rows() == a.cols());
    CholSparceMatrixType ap;
    analyzePattern(a, ap, reuse_ordering);
    factorize_preordered(ap);
}

void LDLTFloatSolver_Polar2x::recompute(const SparceMatrixType& a)
{
    factorize(a);
}

void LDLTFloatSolver_Polar2x::create__permuted_a__ap(const SparceMatrixType& a, CholSparceMatrixType& ap, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo)
{

    const Index size = a.rows();
    ap.resize(size, size);
    //ap.selfadjointView<Upper>() = a.selfadjointView<UpLo>().twistedBy(m_P); 
    ap.selfadjointView<Upper>() = a_uplo.twistedBy(m_P);
    //for (Index i = 0; i < a.outerIndexPtr()[a.cols()]; i++)
    //for (Index i = 0; i < 40; i++)
    //{
    //    cout << "a[" << i << "]=" << a.valuePtr()[i].get_low().toComplex() << endl;
    //    cout << "ap[" << i << "]=" << ap.valuePtr()[i].get_low().toComplex() << endl;
    //}
}

void LDLTFloatSolver_Polar2x::analyzePattern(const SparceMatrixType& a, CholSparceMatrixType& ap, bool reuse_ordering)
{
    assert(a.rows() == a.cols());

    // creates permute matrices 'm_P', 'm_Pinv'
    SparseSelfAdjointView<SparceMatrixType, UpLo> a_uplo = a.selfadjointView<UpLo>();
    if (reuse_ordering && m_P.size() == a.rows() && m_Pinv.size() == a.rows()) //  m_P and m_Pinv must be same size as matrix
    {
        // do nothing - we dont have to create m_P

        //DEBUG
        //cout << endl << "***** reusing m_P *****" << endl;
        //auto m_P_reuse = m_P;
        //auto m_Pinv_reuse = m_Pinv;
        //create__m_P(a, a_uplo); // create 'm_P' and 'm_Pinv'
        //for (int i = 0; i < m_P.indices().size(); i++)
        //{
        //    //cout << "#" << i << "    m_P=" << m_P.indices()[i] << "   m_P_reuse=" << m_P_reuse.indices()[i] << endl;
        //}
    }
    else
    {
        create__m_P(a, a_uplo);  // create 'm_P' and 'm_Pinv'
    }

    // creates 'aPermuted'
    create__permuted_a__ap(a, ap, a_uplo); // create 'ap' using 'm_P', where positions of columns are changed to be more linear compare to original 'a'

    // DEBUG skip permuting 
    //if (options.DebugEnabled && options.debug_skip_permuting)
    //{
    //    create__m_P(ap, ap.selfadjointView<UpLo>());
    //    create__permuted_a__ap(a, ap, ap.selfadjointView<UpLo>());
    //}

    // allocate matrix 'L', and create tree 'm_parent
    analyzePattern_preordered(ap);
}

void LDLTFloatSolver_Polar2x::analyzePattern_preordered(const CholSparceMatrixType& ap)
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

void LDLTFloatSolver_Polar2x::create__m_P(const SparceMatrixType& a, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo)
{
    bool TRACE = options.DebugEnabled && options.debug_trace_factorize_preordered;

    assert(a.rows() == a.cols());
    const Index size = a.rows();
    // Note that amd compute the inverse permutation
    //if (options.DebugEnabled && options.debug_skip_permuting)
    //{
    //    //cout << endl;
    //    m_Pinv.resize(a_uplo.cols());
    //    for (int i = 0; i < m_Pinv.indices().size(); i++)
    //    {
    //        m_Pinv.indices()[i] = i;
    //        //cout << m_Pinv.indices()[i] << ",";
    //    }
    //    //cout << endl;
    //}
    //else
    {
        //auto a_uplo = a.selfadjointView<UpLo>(); 
        //for (Index i = 0; i < 40; i++)
        //{
        //    cout << "a_uplo[" << i << "]=" << a_uplo.valuePtr()[i].get_low().toComplex() << endl;
        //}

        // produces very slow permutation - iterations increase by 3-10 times
        //COLAMDOrdering<Index> ordering;
        //CholSparceMatrixType aaa = a_uplo;
        //ordering(aaa, m_P);
        //m_Pinv = m_P.inverse();


        // produces the best permutation
        if (options.fast_permutation)
        {
            amd_minimum_degree_ordering(a, m_Pinv);
        }
        else
        {
            AMDOrdering<Index> ordering;
            ordering(a_uplo, m_Pinv);

            //DEBUG test if our optimized alrgorithm producing same results
            //PermutationMatrix<Dynamic, Dynamic, Index> m_Pinv2;
            //amd_minimum_degree_ordering(a, m_Pinv2);
        }
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

void LDLTFloatSolver_Polar2x::factorize(const SparceMatrixType& a)
{
    assert(a.rows() == a.cols());
    CholSparceMatrixType ap;
    SparseSelfAdjointView<SparceMatrixType, UpLo> a_uplo = a.selfadjointView<UpLo>();
    create__permuted_a__ap(a, ap, a_uplo);
    factorize_preordered(ap);
}

void LDLTFloatSolver_Polar2x::factorize_preordered(const CholSparceMatrixType& ap)
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

void LDLTFloatSolver_Polar2x::factorize_preordered_origin(const CholSparceMatrixType& ap)
{
    long long iterationsCount = 0;
    assert(m_analysisIsOk && "You must first call analyzePattern()");
    assert(ap.rows() == ap.cols());
    const Index size = ap.rows();
    assert(m_parent.size() == size);
    assert(L_nonZerosPerCol.size() == size);

    const Index* L_colStartIndexes = L.outerIndexPtr();
    Index* L_rowIndexes = L.innerIndexPtr();
    Scalar* L_values = L.valuePtr();
    //for (Index i = 0; i < ap.outerIndexPtr()[size]; i++)
    //{
    //    cout << "Lx[" << i << "]=" << ap.valuePtr()[i].get_low().toComplex() << endl;
    //}
    Timer time;
    ei_declare_aligned_stack_constructed_variable(Scalar, y, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, pattern, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, tags, size, 0);

    bool ok = true;
    Dreal.resize(size);
    //vector<int> savedValueIndexes;
    //savedValueIndexes.reserve(L.nonZeros()*10);
    //savedValueIndexes.push_back(i);
    int iterationNum = 0;
    if (options.DebugEnabled&& options.debug_trace_compute_iterationsCount >= 0)
    {
        compute_ap_indexes.clear();
    }

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
                //cout << "it.value()=" << it.value().get_low().toComplex() << "  it.value().conj=" << it.value().getconjugated().get_low().toComplex() << endl;
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
        Complex4fReal d = y[k];    //RealScalar d = y[k];    // get D(k,k), and clear Y(k)
        //cout << "k#" << k << "   y[k]=" << y[k].get_low().toComplex() << "   d=" << d.toComplex().real() << endl;
        y[k] = 0.f;
        for (; top < size; ++top)
        {
            Index col = pattern[top];       /* pattern[top:n-1] is pattern of L(:,k) */
            Scalar yi = y[col];             /* get and clear Y(i) */
            y[col] = 0.f;

            Index i = L_colStartIndexes[col];
            Index iEND = i + L_nonZerosPerCol[col];
            iterationsCount += iEND - i;
            for (; i < iEND; ++i)
            {
                y[L_rowIndexes[i]] -= numext::conj(L_values[i]) * yi;
                //cout << "   p#" << p << "   sub=" << ((Lx[p]).getconjugated() * yi).get_low().toComplex() << endl;
            }
            Scalar l_ki = yi / Dreal[col]; // Scalar l_ki = yi / D[col];   since D[col] has only reals and 2 reals same for each equation - then we can divide only by 1 real
            Real dMinus0 = yi[0] * yi[0] + yi[1] * yi[1]; // auto dMinus = yi * numext::conj(yi); - take first real because 2 reals same for each equation
            Real dMinus1 = yi[2] * yi[2] + yi[3] * yi[3]; // auto dMinus = yi * numext::conj(yi); - take first real because 2 reals same for each equation
            //auto yimull = _mm_mul_ps(yi, yi);
            //Real dMinus = yimull.m128_f32[0] + yimull.m128_f32[1];
            Complex4fReal d_div = Complex4fReal(dMinus0, dMinus1) / Dreal[col]; // d -= numext::real(dMinus / D[col]);  since D[col] has only reals and 2 reals same for each equation - then we can divide only by 1 real
            d -= d_div;
            //cout << "d = " <<d.toComplex()<< endl;
            //cout << "   top#" << top << "   yi=" << yi.get_low().toComplex() << "   dl_ki=" << l_ki.get_low().toComplex() << "   d=" << d.toComplex().real() << endl;
            L_rowIndexes[i] = k;                          /* store L(k,i) in column form of L */

            L_values[i] = l_ki;
            ++L_nonZerosPerCol[col];              /* increment count of nonzeros in col i */
            if (options.DebugEnabled&& options.debug_trace_compute_iterationsCount >= 0 && iterationNum <= options.debug_trace_compute_iterationsCount)
            {
                //cout << "col = " << col << "  iterationsCount=" << L_nonZerosPerCol[col] << endl;
                compute_ap_indexes.push_back(col);
            }
            iterationNum++;
        }

        Dreal[k] = d;
        if (d == 0.f)
        {
            ok = false;                         /* failure, D(k,k) is zero */
            break;
        }
    }
    //for (Index i = 0; i < ap.outerIndexPtr()[size]; i++)
    //{
    //    cout << "Lx[" << i << "]=" << ap.valuePtr()[i].get_low().toComplex() << endl;
    //}

    if (options.DebugEnabled && options.debug_trace_iterationsCount)
    {
        time.stop();
        auto sss = [](__int64 num, int output_string_length) -> string
        {
            string res = to_string(num);
            while (res.size() < output_string_length)
            {
                res += " ";
            }
            return res;
        };

        if (iterationsCount != 0)
        {
            cout << "size = " << sss(size, 10) << "non zeros count = " << sss(L.nonZeros(), 10) << "       iterationsCount = " << sss(iterationsCount, 10);
            if (L.nonZeros() != 0) cout << "   x times = " << sss(iterationsCount / L.nonZeros(), 10);
            cout << "  time=" << time << endl;
        }
    }
    m_info = ok ? Success : NumericalIssue;
    m_factorizationIsOk = true;
}


void LDLTFloatSolver_Polar2x::factorize_preordered_fast(const CholSparceMatrixType& ap)
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

    ei_declare_aligned_stack_constructed_variable(Scalar, y, size, 0); // value +1 is to be able take future index without out of buffer length
    ei_declare_aligned_stack_constructed_variable(Index, pattern, size, 0);
    ei_declare_aligned_stack_constructed_variable(Index, tags, size, 0);

    Dreal.resize(size);
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
        Complex4fReal d = y[k];    // get D(k,k), and clear Y(k)
        y[k] = 0.0;

        //#pragma omp parallel for  if(IsOmpEnabled) reduction(-:d)
        for (int patternI = top; patternI < size; patternI++)
        {
            //if (TRACE) cout << "  patternI=" << patternI << "   i=" << pattern[patternI] << endl;
            if (TRACE) cout << "  column=" << pattern[patternI] << endl;

            Index col = pattern[patternI];       /* pattern[top:n-1] is pattern of L(:,k), column of SparceMatrix */
            Index p = Lp[col];                                                 // index to sparce column data #i 
            Index pEnd = p + L_nonZerosPerCol[col];                  // end of the column data

            Scalar yi = y[col];             /* get and clear Y(i) */
            y[col] = 0.0;

            //
            //v0 - origin - simple loop
            //
            //for (; p < pEnd; ++p)
            //{
            //    y[Li[p]] -= numext::conj(Lx[p]) * yi;
            //    //cout << "   p#" << p << "   sub=" << ((Lx[p]).getconjugated() * yi).get_low().toComplex() << endl;
            //}

            //
            // v2 - sse + simplification of  x4.conjugate();  x4 *= yi;
            // instread of doing full mupltiply of 2 complex numers - cache some epansion of 'yi' and inlcude in it conjugate of 'x' - so every time we will do 3 operations less
            //
            {
                /*

                a.conjugate()      =     {+ a0           , -a1}
                a*b                     =     {+ a0*b0 - a1*b1, + a0*b1 + b0*a1}
                a.conjugate()*b   =     {+ a0*b0 + a1*b1, + a0*b1 - b0*a1}
                -a.conjugate()*b  =     {- a0*b0 - a1*b1, - a0*b1 + b0*a1}


                a*b
                b_flip = _mm_shuffle_ps(b,b,0xB1);    // (b1, b0)
                a_re   = _mm_moveldup_ps(a);           // (a0, a0)
                a_im   = _mm_movehdup_ps(a);         // (a1, a1)
                arb    = _mm_mul_ps(a_re, b);           // (a0*b0, a0*b1)
                aib    = _mm_mul_ps(a_im, b_flip);    // (a1*b1, a1*b0)
                return _mm_addsub_ps(arb, aib);      // (a0*b0 - a1*b1,  a0*b1 + a1*b0)

                b*a
                a_flip = _mm_shuffle_ps(a,a,0xB1);    // (a1, a0)
                b_re   = _mm_moveldup_ps(b);          // (b0, b0)
                b_im   = _mm_movehdup_ps(b);        // (b1, b1)
                bra    = _mm_mul_ps(b_re, a);           // (b0*a0, b0*a1)
                bia    = _mm_mul_ps(b_im, a_flip);    // (b1*a1, b1*a0)
                return _mm_addsub_ps(bra, bia);      // (b0*a0 - b1*a1,  b0*a1 + b1*a0)

                b*a.conjugate()
                b_re   = _mm_moveldup_ps(b);          // (b0, b0)
                b_re.conjugate()                             // (b0, -b0)
                b_im   = _mm_movehdup_ps(b);       // (b1, b1)
                a_flip = _mm_shuffle_ps(a,a,0xB1);    // (a1, a0)
                bra    = _mm_mul_ps(b_re, a);          // (b0*a0, -b0*a1)
                bia    = _mm_mul_ps(b_im, a_flip);   // (b1*a1, b1*a0)
                return _mm_add_ps(bra, bia);          // (b0*a0 + b1*a1,  -b0*a1 + b1*a0)


                */
                Scalar yiReals = yi.getOnlyReals(); // b_re   = (b0, b0)
                yiReals.conjugate();                    // b_re   = (b0, -b0)
                Scalar yiImgs = yi.getOnlyImgs(); // b_im  = (b1, b1)
                if (p < pEnd) // proceed 2x complex numbers at once using Complex4f 
                {
                    iterationsCount += pEnd - p;
                    Index* pLi = &Li[p];
                    Scalar* pLx = &Lx[p];
                    Scalar* pLxEnd = &Lx[pEnd];                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once
                    Scalar* pLxEnd4 = pLxEnd - 4;                                      // end of the column data for SSE loop that proceeds 2 complex<float> at once

                    // unrolling
                    if (pLx < pLxEnd4)
                    {
                        // v0 - simple - operations miced with load/save
                        //do
                        //{
                        //    int yindex0 = pLi[0];
                        //    int yindex1 = pLi[1];
                        //    int yindex2 = pLi[2];
                        //    int yindex3 = pLi[3];
                        //    Scalar x40(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 0)));
                        //    Scalar x41(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 1)));
                        //    Scalar x42(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 2)));
                        //    Scalar x43(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 3)));
                        //    y[yindex0] -= Scalar(Vec4f(yiReals)*Vec4f(x40) + Vec4f(yiImgs)*Vec4f(x40.getflipRealAndImg()));
                        //    y[yindex1] -= Scalar(Vec4f(yiReals)*Vec4f(x41) + Vec4f(yiImgs)*Vec4f(x41.getflipRealAndImg()));
                        //    y[yindex2] -= Scalar(Vec4f(yiReals)*Vec4f(x42) + Vec4f(yiImgs)*Vec4f(x42.getflipRealAndImg()));
                        //    y[yindex3] -= Scalar(Vec4f(yiReals)*Vec4f(x43) + Vec4f(yiImgs)*Vec4f(x43.getflipRealAndImg()));
                        //    pLi += 4;
                        //    pLx += 4;
                        //} while (pLx < pLxEnd4);

                        //v1 - separated - operations load/save are separated from arithetic operations
                        do
                        {
                            int yindex0 = pLi[0];
                            int yindex1 = pLi[1];
                            int yindex2 = pLi[2];
                            int yindex3 = pLi[3];
                            Scalar x40(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 0)));
                            Scalar x41(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 1)));
                            Scalar x42(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 2)));
                            Scalar x43(Vec4f().load_a(reinterpret_cast<float const*>(pLx + 3)));
                            Scalar y40 = y[yindex0];
                            Scalar y41 = y[yindex1];
                            Scalar y42 = y[yindex2];
                            Scalar y43 = y[yindex3];
                            y40 -= Scalar(Vec4f(yiReals)*Vec4f(x40) + Vec4f(yiImgs)*Vec4f(x40.getflipRealAndImg()));
                            y41 -= Scalar(Vec4f(yiReals)*Vec4f(x41) + Vec4f(yiImgs)*Vec4f(x41.getflipRealAndImg()));
                            y42 -= Scalar(Vec4f(yiReals)*Vec4f(x42) + Vec4f(yiImgs)*Vec4f(x42.getflipRealAndImg()));
                            y43 -= Scalar(Vec4f(yiReals)*Vec4f(x43) + Vec4f(yiImgs)*Vec4f(x43.getflipRealAndImg()));
                            y[yindex0] = y40;
                            y[yindex1] = y41;
                            y[yindex2] = y42;
                            y[yindex3] = y43;
                            pLi += 4;
                            pLx += 4;
                        } while (pLx < pLxEnd4);


                    }

                    // single  - v0 - simple
                    if (pLx < pLxEnd)
                    {
                        do
                        {
                            //v0 - origin
                            //y[Li[p]] -= numext::conj(Lx[p]) * yi;

                            //v1
                            //int yindex = *pLi;
                            //Scalar x4(Vec4f().load_a(reinterpret_cast<float const*>(pLx)));
                            //pLi++;
                            //Scalar y4 = y[yindex];
                            //pLx++;
                            //x4.conjugate();
                            //x4 *= yi;
                            //y4 -= x4;
                            //y[yindex] = y4;

                            //v2 - optimized    x4.conjugate();  x4 *= yi;
                            int yindex = *pLi;
                            Scalar x4(Vec4f().load_a(reinterpret_cast<float const*>(pLx)));
                            pLi++;
                            Scalar y4 = y[yindex];
                            pLx++;
                            x4 = Vec4f(yiReals)*Vec4f(x4) + Vec4f(yiImgs)*Vec4f(x4.getflipRealAndImg());
                            y4 -= x4;
                            y[yindex] = y4;

                        } while (pLx < pLxEnd);


                        p = pEnd;
                    }
                }
            }

            // store result into matrix L, and D
//            d -= numext::real(l_ki *numext::conj(yi)).real();
            Scalar l_ki = yi / Dreal[col]; // Scalar l_ki = yi / D[col];   since D[col] has only reals and 2 reals same for each equation - then we can divide only by 1 real
            Real dMinus0 = yi[0] * yi[0] + yi[1] * yi[1]; // auto dMinus = yi * numext::conj(yi); - take first 
            Real dMinus1 = yi[2] * yi[2] + yi[3] * yi[3]; // auto dMinus = yi * numext::conj(yi); - take second
            Complex4fReal d_div = Complex4fReal(dMinus0, dMinus1) / Dreal[col]; // d -= numext::real(dMinus / D[col]);  since D[col] has only reals and 2 reals same for each equation - then we can divide only by 1 real
            d -= d_div;
            Li[p] = k;                          /* store L(k,i) in column form of L */
            Lx[p] = l_ki;
            ++L_nonZerosPerCol[col];              /* increment count of nonzeros in col i */
        }

        //D[k] = d;
        //D[k] = Scalar(d.real(), 0, d.imag(), 0);
        Dreal[k] = d;
    }

    if (options.DebugEnabled && options.debug_trace_iterationsCount)
    {
        cout << "iterationsCount = " << iterationsCount << "   size = " << size << endl;
    }

    m_info = Success;
    m_factorizationIsOk = true;
}

template<typename Scalar, int _Options, typename _Index>
class InnerIterator
{
public:
    InnerIterator(const MatrixU& mat, Index outer)
        : m_values(mat.valuePtr()), m_indices(mat.innerIndexPtr()), m_outer(outer), m_id(mat.m_outerIndex[outer])
    {
        if (mat.isCompressed())
            m_end = mat.m_outerIndex[outer + 1];
        else
            m_end = m_id + mat.m_innerNonZeros[outer];
    }

    inline InnerIterator& operator++()
    {
        m_id++; return *this;
    }

    inline const Scalar& value() const
    {
        return m_values[m_id];
    }
    inline Scalar& valueRef()
    {
        return const_cast<Scalar&>(m_values[m_id]);
    }

    inline Index index() const
    {
        return m_indices[m_id];
    }
    inline Index outer() const
    {
        return m_outer;
    }
    inline Index row() const
    {
        return IsRowMajor ? m_outer : index();
    }
    inline Index col() const
    {
        return IsRowMajor ? index() : m_outer;
    }

    inline operator bool() const
    {
        return (m_id < m_end);
    }

protected:
    const Scalar* m_values;
    const Index* m_indices;
    const Index m_outer;
    Index m_id;
    Index m_end;
};

// v0
//template<typename Lhs, typename Rhs>
//static void solveLower(const Lhs& lhs, Rhs& other)
//{
//    //for (int col = 0; col < other.cols(); ++col)
//    {
//        constexpr int col = 0;
//        for (int i = 0; i < lhs.cols(); ++i)
//        {
//            Scalar& tmp = other.coeffRef(i, col);
//            if (tmp != Scalar(0.f)) // optimization when other is actually sparse
//            {
//                typename Lhs::InnerIterator it(lhs, i);
//                while (it && it.index() < i) ++it;
//                if (it && it.index() == i) ++it;
//                for (; it; ++it)
//                    other.coeffRef(it.index(), col) -= tmp * it.value();
//            }
//        }
//    }
//}

// v1
__declspec(noinline)  void solveLower(const MatrixL& lhs, SolveMatrixTypeComplex& res)
{
    Scalar* v = res.data();
    for (int i = 0; i < lhs.cols(); ++i)
    {
        //cout << "i=" << i << endl;
        const Scalar& tmp = v[i];
        if (tmp != Scalar(0.f)) // optimization when other is actually sparse
        {
            typename MatrixL::InnerIterator it(lhs, i);
            while (it && it.index() < i) ++it;
            if (it && it.index() == i) ++it;
            for (; it; ++it)
            {
                int index = it.index();
                //cout << "    index=" << index << endl;
                v[index] -= tmp * it.value();
            }
        }
    }
}


// v0
//__declspec(noinline) void solveUpper(const MatrixL& lhs, SolveMatrixTypeComplex& res)
//{
//    Scalar* v = res.data();
//    for (int i = lhs.rows() - 1; i >= 0; --i)
//    {
//        Scalar tmp = v[i];
//        typename MatrixL::InnerIterator it(lhs, i);
//        while (it && it.index() < i) ++it;
//        if (it && it.index() == i) ++it;
//        for (; it; ++it)
//        {
//            Scalar vi = v[it.index()];
//            if (vi != Scalar(0.f)) // optimization when other is actually sparse
//            {
//                tmp -= conj(it.value()) * vi;
//            }
//        }
//
//        v[i] = tmp;
//    }
//}

// v1 - simplified
__declspec(noinline) void solveUpper(const MatrixL& lhs, SolveMatrixTypeComplex& res)
{
    Scalar* v = res.data();
    for (int i = lhs.rows() - 1; i >= 0; --i)
    {
        //cout << "i=" << i << endl;
        Scalar tmp = Scalar(0.f);
        typename MatrixL::InnerIterator it(lhs, i);
        while (it && it.index() < i) ++it;
        if (it && it.index() == i) ++it;
        for (; it; ++it)
        {
            int index = it.index();
            //cout << "    index=" << index << endl;
            Scalar vi = v[index];
            tmp += conj(it.value()) * vi;
        }

        v[i] -= tmp;
    }
}


__declspec(noinline) void inverse(Scalar* source, int count, Scalar* dest)
{
    Scalar* sourceEnd = source + count;
    while (sourceEnd > source)
    {
        sourceEnd--;
        *dest = *sourceEnd;
        dest++;
    }
}



// v2 - inversing to get better performance
//__declspec(noinline) void solveUpper(const MatrixL& lhs, SolveMatrixTypeComplex& res)
//{
//    int res_size = res.size();
//    Vector<Scalar> res_inversed;
//    res_inversed.resize(res_size);
//    inverse(res.data(), res_size, res_inversed.data());
//    auto inverseindex = [&](int index)
//    {
//        return (res_size - 1) - index;
//    };
//
//    Scalar* v = res_inversed.data();
//    for (int i = 0; i < lhs.cols(); ++i)
//    {
//        //cout << "i=" << i << endl;
//        Scalar tmp = Scalar(0.f);
//        typename MatrixL::InnerIterator it(lhs, inverseindex(i));
//        while (it && it.index() < inverseindex(i)) ++it;
//        if (it && it.index() == inverseindex(i)) ++it;
//        for (; it; ++it)
//        {
//            int index = inverseindex(it.index());
//            //cout << "    index=" << index << endl;
//            Scalar vi = v[index];
//            tmp += conj(it.value()) * vi;
//        }
//
//        v[i] -= tmp;
//    }
//    inverse(res_inversed.data(), res_size, res.data());
//}

__declspec(noinline) void D_mult_res(const VectorXf& D, SolveMatrixTypeComplex& res)
{
    Scalar* pres = res.data();
    Scalar* presEnd = pres + res.size();
    const float* pD = D.data();
    while (pres < presEnd)
    {
        float D_inversed = 1 / (*pD);
        *pres *= D_inversed;
        pres++;
        pD++;
    }
}

__declspec(noinline) SolveMatrixTypeComplex permute(const PermutationMatrix<Dynamic, Dynamic, Index>& P, const SolveMatrixTypeComplex& res)
{
    SolveMatrixTypeComplex resPermuted;
    resPermuted.resize(res.rows(), res.cols());
    //v0 - using indexes
    //for (int i = 0; i < res.size(); i++)
    //{
    //    int index = P.indices()[i];
    //    resPermuted.data()[index] = res.data()[i];
    //}
    //v1 - using pointers
    const Scalar* pres = res.data();
    const int* pindexes = P.indices().data();
    Scalar* presPermuted = resPermuted.data();
    for (int i = 0; i < res.size(); i++)
    {
        int index = *pindexes;
        presPermuted[index] = *pres;
        pres++;
        pindexes++;
    }
    //const int* pindexes = P.indices();

    return resPermuted;
}

// v1
//void LDLTFloatSolver_Polar2x::solve(const SolveMatrixTypeComplex &rhs, SolveMatrixTypeComplex &res) const
//{
//    assert(m_isInitialized && "Simplicial LDLT is not initialized.");
//    assert(rows() == rhs.rows() && "SimplicialCholesky::solve(): invalid number of rows of the right hand side matrix b");
//    assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");
//
//    assert(L.rows() == rhs.rows());
//
//    if (m_info != Success)
//        return;
//
//    //const Scalar* data = L.valuePtr();
//    //for (int i = 0; i < 10; i++)
//    //{
//    //    RealScalar c2 = data[i].get_low();
//    //    RealScalar c4 = data[i].get_high();
//    //    RealScalar c22 = c2 * c2;
//    //    RealScalar c4_div_c2 = RealScalar(c4.real() / c2.real(), c4.imag() / c2.imag());
//    //    RealScalar c4_sqrt = sqrt(c4);
//    //    int temp = 0;
//    //}
//
//    if (m_P.size() > 0)
//        res = m_P * rhs;
//    else
//        res = rhs;
//
//    if (L.nonZeros() > 0) // otherwise L==I
//       //matrixLower().solveInPlace(res);
//        solveLower(matrixLower(), res);
//
//    if (D.size() > 0)
//        res = D.cast<Scalar>().asDiagonal().inverse() * res;
//
//    if (L.nonZeros() > 0) // otherwise U==I
//        //matrixUpper().solveInPlace(res);
//        solveUpper(matrixLower(), res);
//
//    if (m_P.size() > 0)
//        res = m_Pinv * res;
//}

// v2
void LDLTFloatSolver_Polar2x::solve(const SolveMatrixTypeComplex &rhs, SolveMatrixTypeComplex &res) const
{
    assert(m_isInitialized && "Simplicial LDLT is not initialized.");
    assert(rows() == rhs.rows() && "SimplicialCholesky::solve(): invalid number of rows of the right hand side matrix b");
    assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");

    assert(L.rows() == rhs.rows());

    if (m_info != Success)
        return;

    if (m_P.size() > 0)
        //res = m_P * rhs;
        res = permute(m_P, rhs);
    else
        res = rhs;

    if (L.nonZeros() > 0) // otherwise L==I
       //matrixLower().solveInPlace(res);
        solveLower(matrixLower(), res);

    if (Dreal.size() > 0)
    {
        // v0 - origin
        //res = D.asDiagonal().inverse() * res;
        // v1 - Dreal
        //res = Dreal.cast<Scalar>().asDiagonal().inverse() * res;
        // v2 - Dreal optimized
        Matrix<Complex4fReal, Dynamic, 1> Dreal_inversed;
        Dreal_inversed.resize(Dreal.rows());
        for (int i = 0; i < Dreal.rows(); i++)
        {
            Dreal_inversed(i) = 1 / Dreal(i);
        }
        res = Dreal_inversed.cast<Scalar>().asDiagonal() * res;
        // v3 - direct
        //D_mult_res(Dreal, res);
    }

    if (L.nonZeros() > 0) // otherwise U==I
        //matrixUpper().solveInPlace(res);
        solveUpper(matrixLower(), res);

    if (m_P.size() > 0)
        //res = m_Pinv * res;
        res = permute(m_Pinv, res);
}



