#include "stdafx.h"
#include "LDLTFloatSolver_Complex2x.h"
#include "complexvec.h"

//const MeshLogicOptions_LDLTFloatSolver& options = meshLogicOptions.LDLTFloatSolver;
#define options meshLogicOptions.LDLTFloatSolver


void LDLTFloatSolver_Complex2x::compute(const SparceMatrixType& a)
{
    assert(a.rows() == a.cols());
    CholSparceMatrixType ap;
    analyzePattern(a, ap);
    factorize_preordered(ap);
}

void LDLTFloatSolver_Complex2x::recompute(const SparceMatrixType& a)
{
    factorize(a);
}

void LDLTFloatSolver_Complex2x::create_permuted_a(const SparceMatrixType& a, CholSparceMatrixType& ap, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo)
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

void LDLTFloatSolver_Complex2x::analyzePattern(const SparceMatrixType& a, CholSparceMatrixType& ap)
{
    assert(a.rows() == a.cols());

    // creates permute matrices 'm_P', 'm_Pinv'
    SparseSelfAdjointView<SparceMatrixType, UpLo> a_uplo = a.selfadjointView<UpLo>();
    ordering(a, a_uplo);

    // creates 'aPermuted'
    create_permuted_a(a, ap, a_uplo);

    // DEBUG skip permuting 
    //if (options.DebugEnabled && options.debug_skip_permuting)
    //{
    //    ordering(ap, ap.selfadjointView<UpLo>());
    //    create_permuted_a(a, ap, ap.selfadjointView<UpLo>());
    //}

    // allocate matrix 'L', and create tree 'm_parent
    analyzePattern_preordered(ap);
}

void LDLTFloatSolver_Complex2x::analyzePattern_preordered(const CholSparceMatrixType& ap)
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

void LDLTFloatSolver_Complex2x::ordering(const SparceMatrixType& a, const SparseSelfAdjointView<SparceMatrixType, UpLo>& a_uplo)
{
    bool TRACE = options.DebugEnabled && options.debug_trace_factorize_preordered;

    assert(a.rows() == a.cols());
    const Index size = a.rows();
    // Note that amd compute the inverse permutation

    {
        //auto a_uplo = a.selfadjointView<UpLo>(); 
        //for (Index i = 0; i < 40; i++)
        //{
        //    cout << "a_uplo[" << i << "]=" << a_uplo.valuePtr()[i].get_low().toComplex() << endl;
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

void LDLTFloatSolver_Complex2x::factorize(const SparceMatrixType& a)
{
    assert(a.rows() == a.cols());
    CholSparceMatrixType ap;
    SparseSelfAdjointView<SparceMatrixType, UpLo> a_uplo = a.selfadjointView<UpLo>();
    create_permuted_a(a, ap, a_uplo);
    factorize_preordered(ap);
}

void LDLTFloatSolver_Complex2x::factorize_preordered(const CholSparceMatrixType& ap)
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

void LDLTFloatSolver_Complex2x::factorize_preordered_origin(const CholSparceMatrixType& ap)
{
    long long iterationsCount = 0;
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
    //    cout << "Lx[" << i << "]=" << ap.valuePtr()[i].get_low().toComplex() << endl;
    //}

    Timer time;
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

        RealScalar d = numext::real(y[k]);    // get D(k,k), and clear Y(k)
        //cout << "k#" << k << "   y[k]=" << y[k].get_low().toComplex() << "   d=" << d.toComplex().real() << endl;
        y[k] = 0.f;
        for (; top < size; ++top)
        {
            Index i = pattern[top];       /* pattern[top:n-1] is pattern of L(:,k) */
            Scalar yi = y[i];             /* get and clear Y(i) */
            y[i] = 0.f;

            Scalar l_ki = yi / D[i];

            Index p = Lp[i];
            Index p2 = p + L_nonZerosPerCol[i];
            iterationsCount += p2 - p;
            for (; p < p2; ++p)
            {
                y[Li[p]] -= numext::conj(Lx[p]) * yi;
                //cout << "   p#" << p << "   sub=" << ((Lx[p]).getconjugated() * yi).get_low().toComplex() << endl;
            }
            d -= numext::real(l_ki * numext::conj(yi));
            //cout << "   top#" << top << "   yi=" << yi.get_low().toComplex() << "   dl_ki=" << l_ki.get_low().toComplex() << "   d=" << d.toComplex().real() << endl;
            Li[p] = k;                          /* store L(k,i) in column form of L */
            Lx[p] = l_ki;
            ++L_nonZerosPerCol[i];              /* increment count of nonzeros in col i */
        }

        D[k] = Scalar(d.real(), 0, d.imag(), 0);
        if (d == RealScalar(0.f))
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


void LDLTFloatSolver_Complex2x::factorize_preordered_fast(const CholSparceMatrixType& ap)
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
            y[columnIndex] = 0.0;

            // v0 - origin
            Scalar l_ki = yi / D[columnIndex];
            // v1 - optimized using SSE (5% of total solve speed)
            //Scalar l_ki;
            //Complex2f m_diagX(D.data() + columnIndex);
            //Complex2f l_kiX = yiX / m_diagX;
            //l_kiX.store(&l_ki);
            // v2 - using fact that D is just RealScalar - not a complex but a numerix value, so in complex it will have zero imaginary value:  (d, 0*i)
            //RealScalar c = D[columnIndex].real();
            //Scalar l_ki = Scalar(yi.real() / c, yi.imag() / c); // LDLT solver works like it was designed for polar coordiantes: decrease r in polar coordinates, but leave angle untachable

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


            // store result into matrix L, and D
            d = d - numext::real(l_ki *numext::conj(yi));
            Li[p] = k;                          /* store L(k,i) in column form of L */
            Lx[p] = l_ki;
            ++L_nonZerosPerCol[columnIndex];              /* increment count of nonzeros in col i */
        }

        //D[k] = d;
        D[k] = Scalar(d.real(), 0, d.imag(), 0);
    }

    if (options.DebugEnabled && options.debug_trace_iterationsCount)
    {
        cout << "iterationsCount = " << iterationsCount << "   size = " << size << endl;
    }

    m_info = Success;
    m_factorizationIsOk = true;
}


template<typename Lhs, typename Rhs>
static void solveLower(const Lhs& lhs, Rhs& other)
{
    int Mode = Lower;
    for (int col = 0; col < other.cols(); ++col)
    {
        for (int i = 0; i < lhs.cols(); ++i)
        {
            Scalar& tmp = other.coeffRef(i, col);
            Vec2qb equals = (tmp == Scalar(0.f));
            if (equals[0] && equals[1]) // optimization when other is actually sparse
            {
                typename Lhs::InnerIterator it(lhs, i);
                while (it && it.index() < i)
                    ++it;
                if (!(Mode & UnitDiag))
                {
                    assert(it && it.index() == i);
                    tmp /= it.value();
                }
                if (it && it.index() == i)
                    ++it;
                for (; it; ++it)
                    other.coeffRef(it.index(), col) -= tmp * it.value();
            }
        }
    }
}

template<typename Lhs, typename Rhs>
static void solveUpper(const Lhs& lhs, Rhs& other)
{
    int Mode = Upper;
    for (int col = 0; col < other.cols(); ++col)
    {
        for (int i = lhs.rows() - 1; i >= 0; --i)
        {
            Scalar tmp = other.coeff(i, col);
            Scalar l_ii(0.f);
            typename Lhs::InnerIterator it(lhs, i);
            while (it && it.index() < i)
                ++it;
            if (!(Mode & UnitDiag))
            {
                assert(it && it.index() == i);
                l_ii = it.value();
                ++it;
            }
            else if (it && it.index() == i)
                ++it;
            for (; it; ++it)
            {
                tmp -= it.value() * other.coeff(it.index(), col);
            }

            if (Mode & UnitDiag)
                other.coeffRef(i, col) = tmp;
            else
                other.coeffRef(i, col) = tmp / l_ii;
        }
    }
}


void LDLTFloatSolver_Complex2x::solve(const SolveMatrixType &rhs, SolveMatrixType &res) const
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
        matrixLower().solveInPlace(res);
    //solveLower(matrixLower(), res);

    if (D.size() > 0)
        res = D.asDiagonal().inverse() * res;

    if (L.nonZeros() > 0) // otherwise U==I
        matrixUpper().solveInPlace(res);
    //solveUpper(matrixUpper(), res);

    if (m_P.size() > 0)
        res = m_Pinv * res;
}