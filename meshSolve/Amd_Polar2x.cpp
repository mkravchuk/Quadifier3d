#include "stdafx.h"
#include "Amd_Polar2x.h"



template<typename T> inline T amd_flip(const T& i)
{
    return -i - 2;
}
template<typename T> inline T amd_unflip(const T& i)
{
    return i < 0 ? amd_flip(i) : i;
}
template<typename T0, typename T1> inline bool amd_marked(const T0* w, const T1& j)
{
    return w[j] < 0;
}
template<typename T0, typename T1> inline void amd_mark(const T0* w, const T1& j)
{
    return w[j] = amd_flip(w[j]);
}

/* clear w */
static int cs_wclear(Index mark, Index lemax, Index *w, Index n)
{
    if (mark < 2 || (mark + lemax < 0))
    {
        for (Index k = 0; k < n; k++)
            if (w[k] != 0)
                w[k] = 1;
        mark = 2;
    }
    return mark;     /* at this point, w[0..n-1] < mark holds */
}

/* depth-first search and postorder of a tree rooted at node j */
Index cs_tdfs(Index i, Index k, Index *head, const Index *next, Index *last, Index *w)
{
    int top = 0;
    if (!head || !next || !last || !w)  /* check inputs */
    {
        k = -1;
    }
    else
    {
        w[0] = i;                 /* place j on the stack */
        while (top >= 0)                /* while (stack is not empty) */
        {
            int w_top = w[top];           /* p = top of stack */
            int head_wtop = head[w_top];              /* i = youngest child of p */
            if (head_wtop == -1)
            {
                top--;                 /* p has no unordered children left */
                last[k++] = w_top;        /* node p is the kth postordered node */
            }
            else
            {
                head[w_top] = next[head_wtop];   /* remove i from children of p */
                w[++top] = head_wtop;     /* start dfs on child node i */
            }
        }
    }
    return k;
}


void amd_minimum_degree_ordering__IndexObject(const SparceMatrixType& _C, PermutationMatrix<Dynamic, Dynamic, Index>& perm)
{
    int d, lemax = 0, e, eln, i, j, k,
        ln, mindeg = 0, nvi, nvj,
        nel = 0, p, pj, pk, q;
    unsigned int h;

    Index n = _C.cols();
    Index cnz = _C.nonZeros();
    int dense = std::max<Index>(16, Index(10 * std::sqrt(double(n))));   /* find dense threshold */
    dense = std::min<Index>(n - 2, dense);


    bool deleteCo = false;
    bool deleteCi = false;
    // v0 - copy all data
    //SparceMatrixType C = _C; // copy
    //int t = cnz + cnz / 5 + 2 * n;                 /* add elbow room to C */
    //C.resizeNonZeros(t); // modify
    //Index* Co = C.outerIndexPtr();
    //Index* Ci = C.innerIndexPtr();
    //bool deleteC = false;

    // v1 - copy only what is needed
    int t = cnz + cnz / 5 + 2 * n;                 /* add elbow room to C */
    //Index co_size = _C.outerSize() + 1; // we allocate +1 index is because in EigenSparce allocated 1 more index:  m_outerIndex = static_cast<Index*>(std::malloc((outerSize + 1) * sizeof(Index)));
    //Index* Co = new Index[co_size]; deleteCo = true;// copy
    //memcpy(Co, _C.outerIndexPtr(), co_size * sizeof(Index));
    Index* Ci = new Index[t]; deleteCi = true;// C.resizeNonZeros(t);
    memcpy(Ci, _C.innerIndexPtr(), cnz * sizeof(Index)); // copy

    struct IndexObject
    {
        Index Co;
        Index len;
        Index elen;
        Index nv;
        Index next;
        Index last;
        Index degree;
        Index dummy;//just to have 8 ints in structure - 256 bits
    };
    IndexObject* o = new IndexObject[n + 1];/* get workspace */
    
    Index* W = new Index[3 * (n + 1)]; /* get workspace */
    Index* w = W + 0 * (n + 1);
    Index* head = W + 1 * (n + 1);
    Index* hhead = W + 2 * (n + 1);

    /* --- Initialize quotient graph ---------------------------------------- */
    for (i = 0; i <= n; i++)
    {
        o[i].Co = _C.outerIndexPtr()[i];
    }
    for (k = 0; k < n; k++)
    {
        o[k].len = o[k + 1].Co - o[k].Co;
    }
    o[n].len = 0;
    int nzmax = t;

    for (i = 0; i <= n; i++)
    {
        o[i].elen = 0;                      // Ek of node i is empty
        o[i].nv = 1;                      // node i is just one node
        o[i].next = -1;
        o[i].last = -1;
        o[i].degree = o[i].len;                 // degree of node i
        w[i] = 1;                      // node i is alive
        head[i] = -1;                     // degree list i is empty
        hhead[i] = -1;                     // hash list i is empty 
    }
    int mark = cs_wclear(0, 0, w, n);         /* clear w */

    /* --- Initialize degree lists ------------------------------------------ */
    for (i = 0; i < n; i++)
    {
        bool has_diag = false;
        for (p = o[i].Co; p < o[i + 1].Co; ++p)
            if (Ci[p] == i)
            {
                has_diag = true;
                break;
            }

        d = o[i].degree;
        if (d == 1 && has_diag)           /* node i is empty */
        {
            o[i].elen = -2;                 /* element i is dead */
            nel++;
            o[i].Co = -1;                   /* i is a root of assembly tree */
            w[i] = 0;
        }
        else if (d > dense || !has_diag)  /* node i is dense or has no structural diagonal element */
        {
            o[i].nv = 0;                    /* absorb i into element n */
            o[i].elen = -1;                 /* node i is dead */
            nel++;
            o[i].Co = amd_flip(n);
            o[n].nv++;
        }
        else
        {
            if (head[d] != -1) o[head[d]].last = i;
            o[i].next = head[d];           /* put node i in degree list d */
            head[d] = i;
        }
    }

    o[n].elen = -2;                         /* n is a dead element */
    o[n].Co = -1;                           /* n is a root of assembly tree */
    w[n] = 0;                             /* n is a dead element */

    while (nel < n)                         /* while (selecting pivots) do */
    {
        /* --- Select node of minimum approximate degree -------------------- */
        for (k = -1; mindeg < n && (k = head[mindeg]) == -1; mindeg++)
        {
        }
        if (o[k].next != -1) o[o[k].next].last = -1;
        head[mindeg] = o[k].next;          /* remove k from degree list */
        int elenk = o[k].elen;                  /* elenk = |Ek| */
        int nvk = o[k].nv;                      /* # of nodes k represents */
        nel += nvk;                        /* nv[k] nodes of A eliminated */

        /* --- Garbage collection ------------------------------------------- */
        if (elenk > 0 && cnz + mindeg >= nzmax)
        {
            for (j = 0; j < n; j++)
            {
                if ((p = o[j].Co) >= 0)      /* j is a live node or element */
                {
                    o[j].Co = Ci[p];          /* save first entry of object */
                    Ci[p] = amd_flip(j);    /* first entry is now amd_flip(j) */
                }
            }
            for (q = 0, p = 0; p < cnz; ) /* scan all of memory */
            {
                if ((j = amd_flip(Ci[p++])) >= 0)  /* found object j */
                {
                    Ci[q] = o[j].Co;       /* restore first entry of object */
                    o[j].Co = q++;          /* new pointer to object j */
                    for (int k3 = 0; k3 < o[j].len - 1; k3++) Ci[q++] = Ci[p++];
                }
            }
            cnz = q;                       /* Ci[cnz...nzmax-1] now free */
        }

        /* --- Construct new element ---------------------------------------- */
        int dk = 0;
        o[k].nv = -nvk;                     /* flag k as in Lk */
        p = o[k].Co;
        int pk1 = (elenk == 0) ? p : cnz;      /* do in place if elen[k] == 0 */
        int pk2 = pk1;
        for (int k1 = 1; k1 <= elenk + 1; k1++)
        {
            if (k1 > elenk)
            {
                e = k;                     /* search the nodes in k */
                pj = p;                    /* list of nodes starts at Ci[pj]*/
                ln = o[k].len - elenk;      /* length of list of nodes in k */
            }
            else
            {
                e = Ci[p++];              /* search the nodes in e */
                pj = o[e].Co;
                ln = o[e].len;              /* length of list of nodes in e */
            }
            for (int k2 = 1; k2 <= ln; k2++)
            {
                i = Ci[pj++];
                if ((nvi = o[i].nv) <= 0) continue; /* node i dead, or seen */
                dk += nvi;                 /* degree[Lk] += size of node i */
                o[i].nv = -nvi;             /* negate nv[i] to denote i in Lk*/
                Ci[pk2++] = i;            /* place i in Lk */
                if (o[i].next != -1) o[o[i].next].last = o[i].last;
                if (o[i].last != -1)         /* remove i from degree list */
                {
                    o[o[i].last].next = o[i].next;
                }
                else
                {
                    head[o[i].degree] = o[i].next;
                }
            }
            if (e != k)
            {
                o[e].Co = amd_flip(k);      /* absorb e into k */
                w[e] = 0;                 /* e is now a dead element */
            }
        }
        if (elenk != 0) cnz = pk2;         /* Ci[cnz...nzmax] is free */
        o[k].degree = dk;                   /* external degree of k - |Lk\i| */
        o[k].Co = pk1;                      /* element k is in Ci[pk1..pk2-1] */
        o[k].len = pk2 - pk1;
        o[k].elen = -2;                     /* k is now an element */

        /* --- Find set differences ----------------------------------------- */
        mark = cs_wclear(mark, lemax, w, n);  /* clear w if necessary */
        for (pk = pk1; pk < pk2; pk++)    /* scan 1: find |Le\Lk| */
        {
            i = Ci[pk];
            if ((eln = o[i].elen) <= 0) continue;/* skip if elen[i] empty */
            nvi = -o[i].nv;                      /* nv[i] was negated */
            int wnvi = mark - nvi;
            for (p = o[i].Co; p <= o[i].Co + eln - 1; p++)  /* scan Ei */
            {
                e = Ci[p];
                if (w[e] >= mark)
                {
                    w[e] -= nvi;          /* decrement |Le\Lk| */
                }
                else if (w[e] != 0)        /* ensure e is a live element */
                {
                    w[e] = o[e].degree + wnvi; /* 1st time e seen in scan 1 */
                }
            }
        }

        /* --- Degree update ------------------------------------------------ */
        for (pk = pk1; pk < pk2; pk++)    /* scan2: degree update */
        {
            i = Ci[pk];                   /* consider node i in Lk */
            int p1 = o[i].Co;
            int p2 = p1 + o[i].elen - 1;
            int pn = p1;
            for (h = 0, d = 0, p = p1; p <= p2; p++)    /* scan Ei */
            {
                e = Ci[p];
                if (w[e] != 0)             /* e is an unabsorbed element */
                {
                    int dext = w[e] - mark;   /* dext = |Le\Lk| */
                    if (dext > 0)
                    {
                        d += dext;         /* sum up the set differences */
                        Ci[pn++] = e;     /* keep e in Ei */
                        h += e;            /* compute the hash of node i */
                    }
                    else
                    {
                        o[e].Co = amd_flip(k);  /* aggressive absorb. e->k */
                        w[e] = 0;             /* e is a dead element */
                    }
                }
            }
            o[i].elen = pn - p1 + 1;        /* elen[i] = |Ei| */
            int p3 = pn;
            int p4 = p1 + o[i].len;
            assert(p4 <= cnz);
            for (p = p2 + 1; p < p4; p++) /* prune edges in Ai */
            {
                assert(p < cnz);
                j = Ci[p];
                if ((nvj = o[j].nv) <= 0) continue; /* node j dead or in Lk */
                d += nvj;                  /* degree(i) += |j| */
                Ci[pn++] = j;             /* place j in node list of i */
                h += j;                    /* compute hash for node i */
            }
            if (d == 0)                     /* check for mass elimination */
            {
                o[i].Co = amd_flip(k);      /* absorb i into k */
                nvi = -o[i].nv;
                dk -= nvi;                 /* |Lk| -= |i| */
                nvk += nvi;                /* |k| += nv[i] */
                nel += nvi;
                o[i].nv = 0;
                o[i].elen = -1;             /* node i is dead */
            }
            else
            {
                o[i].degree = std::min<Index>(o[i].degree, d);   /* update degree(i) */
                Ci[pn] = Ci[p3];         /* move first node to end */
                Ci[p3] = Ci[p1];         /* move 1st el. to end of Ei */
                Ci[p1] = k;               /* add k as 1st element in of Ei */
                o[i].len = pn - p1 + 1;     /* new len of adj. list of node i */
                h %= n;                    /* finalize hash of i */
                o[i].next = hhead[h];      /* place i in hash bucket */
                hhead[h] = i;
                o[i].last = h;              /* save hash of i in last[i] */
            }
        }                                   /* scan2 is done */
        o[k].degree = dk;                   /* finalize |Lk| */
        lemax = std::max<Index>(lemax, dk);
        mark = cs_wclear(mark + lemax, lemax, w, n);    /* clear w */

        /* --- Supernode detection ------------------------------------------ */
        for (pk = pk1; pk < pk2; pk++)
        {
            i = Ci[pk];
            if (o[i].nv >= 0) continue;         /* skip if i is dead */
            h = o[i].last;                      /* scan hash bucket of node i */
            i = hhead[h];
            hhead[h] = -1;                    /* hash bucket will be empty */
            for (; i != -1 && o[i].next != -1; i = o[i].next, mark++)
            {
                ln = o[i].len;
                eln = o[i].elen;
                for (p = o[i].Co + 1; p <= o[i].Co + ln - 1; p++)
                {
                    w[Ci[p]] = mark;
                }
                int jlast = i;
                for (j = o[i].next; j != -1; ) /* compare i with all j */
                {
                    int ok = (o[j].len == ln) && (o[j].elen == eln);
                    for (p = o[j].Co + 1; ok && p <= o[j].Co + ln - 1; p++)
                    {
                        if (w[Ci[p]] != mark) ok = 0;    /* compare i and j*/
                    }
                    if (ok)                     /* i and j are identical */
                    {
                        o[j].Co = amd_flip(i);  /* absorb j into i */
                        o[i].nv += o[j].nv;
                        o[j].nv = 0;
                        o[j].elen = -1;         /* node j is dead */
                        j = o[j].next;          /* delete j from hash bucket */
                        o[jlast].next = j;
                    }
                    else
                    {
                        jlast = j;             /* j and i are different */
                        j = o[j].next;
                    }
                }
            }
        }

        /* --- Finalize new element------------------------------------------ */
        for (p = pk1, pk = pk1; pk < pk2; pk++)   /* finalize Lk */
        {
            i = Ci[pk];
            if ((nvi = -o[i].nv) <= 0) continue;/* skip if i is dead */
            o[i].nv = nvi;                      /* restore nv[i] */
            d = o[i].degree + dk - nvi;         /* compute external degree(i) */
            d = std::min<Index>(d, n - nel - nvi);
            if (head[d] != -1) o[head[d]].last = i;
            o[i].next = head[d];               /* put i back in degree list */
            o[i].last = -1;
            head[d] = i;
            mindeg = std::min<Index>(mindeg, d);       /* find new minimum degree */
            o[i].degree = d;
            Ci[p++] = i;                      /* place i in Lk */
        }
        o[k].nv = nvk;                      /* # nodes absorbed into k */
        if ((o[k].len = p - pk1) == 0)         /* length of adj list of element k*/
        {
            o[k].Co = -1;                   /* k is a root of the tree */
            w[k] = 0;                     /* k is now a dead element */
        }
        if (elenk != 0) cnz = p;           /* free unused space in Lk */
    }

    /* --- Postordering ----------------------------------------------------- */
    for (i = 0; i < n; i++) o[i].Co = amd_flip(o[i].Co);/* fix assembly tree */
    for (j = 0; j <= n; j++) head[j] = -1;
    for (j = n; j >= 0; j--)              /* place unordered nodes in lists */
    {
        if (o[j].nv > 0) continue;          /* skip if j is an element */
        o[j].next = head[o[j].Co];          /* place j in list of its parent */
        head[o[j].Co] = j;
    }
    for (e = n; e >= 0; e--)              /* place elements in lists */
    {
        if (o[e].nv <= 0) continue;         /* skip unless e is an element */
        if (o[e].Co != -1)
        {
            o[e].next = head[o[e].Co];      /* place e in list of its parent */
            head[o[e].Co] = e;
        }
    }
    for (k = 0, i = 0; i <= n; i++)       /* postorder the assembly tree */
    {
        if (o[i].Co == -1)
        {
            //k = cs_tdfs(i, k, head, next, last, w);
            int top = 0;
            w[0] = i;                 /* place j on the stack */
            while (top >= 0)                /* while (stack is not empty) */
            {
                int w_top = w[top];           /* p = top of stack */
                int head_wtop = head[w_top];              /* i = youngest child of p */
                if (head_wtop == -1)
                {
                    top--;                 /* p has no unordered children left */
                    o[k++].last = w_top;        /* node p is the kth postordered node */
                }
                else
                {
                    head[w_top] = o[head_wtop].next;   /* remove i from children of p */
                    w[++top] = head_wtop;     /* start dfs on child node i */
                }
            }
        }
    }


    perm.resize(n);
    for (i = 0; i < n; i++)
    {
        perm.indices().data()[i] = o[i].last;
    }

    delete[] W;
    delete[] o;
    //if (deleteCo) delete[] Co;
    if (deleteCi) delete[] Ci;
}


void amd_minimum_degree_ordering__separated(const SparceMatrixType& _C, PermutationMatrix<Dynamic, Dynamic, Index>& perm)
{
    int d, dk, dext, lemax = 0, e, elenk, eln, i, j, k, k1,
        k2, k3, jlast, ln, dense, nzmax, mindeg = 0, nvi, nvj, nvk, mark, wnvi,
        ok, nel = 0, p, p1, p2, p3, p4, pj, pk, pk1, pk2, pn, q, t;
    unsigned int h;

    Index n = _C.cols();
    Index cnz = _C.nonZeros();
    dense = std::max<Index>(16, Index(10 * std::sqrt(double(n))));   /* find dense threshold */
    dense = std::min<Index>(n - 2, dense);
    perm.resize(n + 1);


    bool deleteCo = false;
    bool deleteCi = false;
    // v0 - copy all data
    //SparceMatrixType C = _C; // copy
    //t = cnz + cnz / 5 + 2 * n;                 /* add elbow room to C */
    //C.resizeNonZeros(t); // modify
    //Index* Co = C.outerIndexPtr();
    //Index* Ci = C.innerIndexPtr();
    //bool deleteC = false;

    // v1 - copy only what is needed
    t = cnz + cnz / 5 + 2 * n;                 /* add elbow room to C */
    Index co_size = _C.outerSize() + 1; // we allocate +1 index is because in EigenSparce allocated 1 more index:  m_outerIndex = static_cast<Index*>(std::malloc((outerSize + 1) * sizeof(Index)));
    Index* Co = new Index[co_size]; deleteCo = true;// copy
    memcpy(Co, _C.outerIndexPtr(), co_size * sizeof(Index));
    Index* Ci = new Index[t]; deleteCi = true;// C.resizeNonZeros(t);
    memcpy(Ci, _C.innerIndexPtr(), cnz * sizeof(Index)); // copy




    Index* W = new Index[8 * (n + 1)]; /* get workspace */
    Index* len = W;
    Index* nv = W + (n + 1);
    Index* next = W + 2 * (n + 1);
    Index* head = W + 3 * (n + 1);
    Index* elen = W + 4 * (n + 1);
    Index* degree = W + 5 * (n + 1);
    Index* w = W + 6 * (n + 1);
    Index* hhead = W + 7 * (n + 1);
    Index* last = perm.indices().data();                              /* use P as workspace for last */

    /* --- Initialize quotient graph ---------------------------------------- */
    for (k = 0; k < n; k++)
        len[k] = Co[k + 1] - Co[k];
    len[n] = 0;
    nzmax = t;

    for (i = 0; i <= n; i++)
    {
        head[i] = -1;                     // degree list i is empty
        last[i] = -1;
        next[i] = -1;
        hhead[i] = -1;                     // hash list i is empty 
        nv[i] = 1;                      // node i is just one node
        w[i] = 1;                      // node i is alive
        elen[i] = 0;                      // Ek of node i is empty
        degree[i] = len[i];                 // degree of node i
    }
    mark = internal::cs_wclear<Index>(0, 0, w, n);         /* clear w */

    /* --- Initialize degree lists ------------------------------------------ */
    for (i = 0; i < n; i++)
    {
        bool has_diag = false;
        for (p = Co[i]; p < Co[i + 1]; ++p)
            if (Ci[p] == i)
            {
                has_diag = true;
                break;
            }

        d = degree[i];
        if (d == 1 && has_diag)           /* node i is empty */
        {
            elen[i] = -2;                 /* element i is dead */
            nel++;
            Co[i] = -1;                   /* i is a root of assembly tree */
            w[i] = 0;
        }
        else if (d > dense || !has_diag)  /* node i is dense or has no structural diagonal element */
        {
            nv[i] = 0;                    /* absorb i into element n */
            elen[i] = -1;                 /* node i is dead */
            nel++;
            Co[i] = amd_flip(n);
            nv[n]++;
        }
        else
        {
            if (head[d] != -1) last[head[d]] = i;
            next[i] = head[d];           /* put node i in degree list d */
            head[d] = i;
        }
    }

    elen[n] = -2;                         /* n is a dead element */
    Co[n] = -1;                           /* n is a root of assembly tree */
    w[n] = 0;                             /* n is a dead element */

    while (nel < n)                         /* while (selecting pivots) do */
    {
        /* --- Select node of minimum approximate degree -------------------- */
        for (k = -1; mindeg < n && (k = head[mindeg]) == -1; mindeg++)
        {
        }
        if (next[k] != -1) last[next[k]] = -1;
        head[mindeg] = next[k];          /* remove k from degree list */
        elenk = elen[k];                  /* elenk = |Ek| */
        nvk = nv[k];                      /* # of nodes k represents */
        nel += nvk;                        /* nv[k] nodes of A eliminated */

        /* --- Garbage collection ------------------------------------------- */
        if (elenk > 0 && cnz + mindeg >= nzmax)
        {
            for (j = 0; j < n; j++)
            {
                if ((p = Co[j]) >= 0)      /* j is a live node or element */
                {
                    Co[j] = Ci[p];          /* save first entry of object */
                    Ci[p] = amd_flip(j);    /* first entry is now amd_flip(j) */
                }
            }
            for (q = 0, p = 0; p < cnz; ) /* scan all of memory */
            {
                if ((j = amd_flip(Ci[p++])) >= 0)  /* found object j */
                {
                    Ci[q] = Co[j];       /* restore first entry of object */
                    Co[j] = q++;          /* new pointer to object j */
                    for (k3 = 0; k3 < len[j] - 1; k3++) Ci[q++] = Ci[p++];
                }
            }
            cnz = q;                       /* Ci[cnz...nzmax-1] now free */
        }

        /* --- Construct new element ---------------------------------------- */
        dk = 0;
        nv[k] = -nvk;                     /* flag k as in Lk */
        p = Co[k];
        pk1 = (elenk == 0) ? p : cnz;      /* do in place if elen[k] == 0 */
        pk2 = pk1;
        for (k1 = 1; k1 <= elenk + 1; k1++)
        {
            if (k1 > elenk)
            {
                e = k;                     /* search the nodes in k */
                pj = p;                    /* list of nodes starts at Ci[pj]*/
                ln = len[k] - elenk;      /* length of list of nodes in k */
            }
            else
            {
                e = Ci[p++];              /* search the nodes in e */
                pj = Co[e];
                ln = len[e];              /* length of list of nodes in e */
            }
            for (k2 = 1; k2 <= ln; k2++)
            {
                i = Ci[pj++];
                if ((nvi = nv[i]) <= 0) continue; /* node i dead, or seen */
                dk += nvi;                 /* degree[Lk] += size of node i */
                nv[i] = -nvi;             /* negate nv[i] to denote i in Lk*/
                Ci[pk2++] = i;            /* place i in Lk */
                if (next[i] != -1) last[next[i]] = last[i];
                if (last[i] != -1)         /* remove i from degree list */
                {
                    next[last[i]] = next[i];
                }
                else
                {
                    head[degree[i]] = next[i];
                }
            }
            if (e != k)
            {
                Co[e] = amd_flip(k);      /* absorb e into k */
                w[e] = 0;                 /* e is now a dead element */
            }
        }
        if (elenk != 0) cnz = pk2;         /* Ci[cnz...nzmax] is free */
        degree[k] = dk;                   /* external degree of k - |Lk\i| */
        Co[k] = pk1;                      /* element k is in Ci[pk1..pk2-1] */
        len[k] = pk2 - pk1;
        elen[k] = -2;                     /* k is now an element */

        /* --- Find set differences ----------------------------------------- */
        mark = internal::cs_wclear<Index>(mark, lemax, w, n);  /* clear w if necessary */
        for (pk = pk1; pk < pk2; pk++)    /* scan 1: find |Le\Lk| */
        {
            i = Ci[pk];
            if ((eln = elen[i]) <= 0) continue;/* skip if elen[i] empty */
            nvi = -nv[i];                      /* nv[i] was negated */
            wnvi = mark - nvi;
            for (p = Co[i]; p <= Co[i] + eln - 1; p++)  /* scan Ei */
            {
                e = Ci[p];
                if (w[e] >= mark)
                {
                    w[e] -= nvi;          /* decrement |Le\Lk| */
                }
                else if (w[e] != 0)        /* ensure e is a live element */
                {
                    w[e] = degree[e] + wnvi; /* 1st time e seen in scan 1 */
                }
            }
        }

        /* --- Degree update ------------------------------------------------ */
        for (pk = pk1; pk < pk2; pk++)    /* scan2: degree update */
        {
            i = Ci[pk];                   /* consider node i in Lk */
            p1 = Co[i];
            p2 = p1 + elen[i] - 1;
            pn = p1;
            for (h = 0, d = 0, p = p1; p <= p2; p++)    /* scan Ei */
            {
                e = Ci[p];
                if (w[e] != 0)             /* e is an unabsorbed element */
                {
                    dext = w[e] - mark;   /* dext = |Le\Lk| */
                    if (dext > 0)
                    {
                        d += dext;         /* sum up the set differences */
                        Ci[pn++] = e;     /* keep e in Ei */
                        h += e;            /* compute the hash of node i */
                    }
                    else
                    {
                        Co[e] = amd_flip(k);  /* aggressive absorb. e->k */
                        w[e] = 0;             /* e is a dead element */
                    }
                }
            }
            elen[i] = pn - p1 + 1;        /* elen[i] = |Ei| */
            p3 = pn;
            p4 = p1 + len[i];
            assert(p4 <= cnz);
            for (p = p2 + 1; p < p4; p++) /* prune edges in Ai */
            {
                assert(p < cnz);
                j = Ci[p];
                if ((nvj = nv[j]) <= 0) continue; /* node j dead or in Lk */
                d += nvj;                  /* degree(i) += |j| */
                Ci[pn++] = j;             /* place j in node list of i */
                h += j;                    /* compute hash for node i */
            }
            if (d == 0)                     /* check for mass elimination */
            {
                Co[i] = amd_flip(k);      /* absorb i into k */
                nvi = -nv[i];
                dk -= nvi;                 /* |Lk| -= |i| */
                nvk += nvi;                /* |k| += nv[i] */
                nel += nvi;
                nv[i] = 0;
                elen[i] = -1;             /* node i is dead */
            }
            else
            {
                degree[i] = std::min<Index>(degree[i], d);   /* update degree(i) */
                Ci[pn] = Ci[p3];         /* move first node to end */
                Ci[p3] = Ci[p1];         /* move 1st el. to end of Ei */
                Ci[p1] = k;               /* add k as 1st element in of Ei */
                len[i] = pn - p1 + 1;     /* new len of adj. list of node i */
                h %= n;                    /* finalize hash of i */
                next[i] = hhead[h];      /* place i in hash bucket */
                hhead[h] = i;
                last[i] = h;              /* save hash of i in last[i] */
            }
        }                                   /* scan2 is done */
        degree[k] = dk;                   /* finalize |Lk| */
        lemax = std::max<Index>(lemax, dk);
        mark = internal::cs_wclear<Index>(mark + lemax, lemax, w, n);    /* clear w */

        /* --- Supernode detection ------------------------------------------ */
        for (pk = pk1; pk < pk2; pk++)
        {
            i = Ci[pk];
            if (nv[i] >= 0) continue;         /* skip if i is dead */
            h = last[i];                      /* scan hash bucket of node i */
            i = hhead[h];
            hhead[h] = -1;                    /* hash bucket will be empty */
            for (; i != -1 && next[i] != -1; i = next[i], mark++)
            {
                ln = len[i];
                eln = elen[i];
                for (p = Co[i] + 1; p <= Co[i] + ln - 1; p++) w[Ci[p]] = mark;
                jlast = i;
                for (j = next[i]; j != -1; ) /* compare i with all j */
                {
                    ok = (len[j] == ln) && (elen[j] == eln);
                    for (p = Co[j] + 1; ok && p <= Co[j] + ln - 1; p++)
                    {
                        if (w[Ci[p]] != mark) ok = 0;    /* compare i and j*/
                    }
                    if (ok)                     /* i and j are identical */
                    {
                        Co[j] = amd_flip(i);  /* absorb j into i */
                        nv[i] += nv[j];
                        nv[j] = 0;
                        elen[j] = -1;         /* node j is dead */
                        j = next[j];          /* delete j from hash bucket */
                        next[jlast] = j;
                    }
                    else
                    {
                        jlast = j;             /* j and i are different */
                        j = next[j];
                    }
                }
            }
        }

        /* --- Finalize new element------------------------------------------ */
        for (p = pk1, pk = pk1; pk < pk2; pk++)   /* finalize Lk */
        {
            i = Ci[pk];
            if ((nvi = -nv[i]) <= 0) continue;/* skip if i is dead */
            nv[i] = nvi;                      /* restore nv[i] */
            d = degree[i] + dk - nvi;         /* compute external degree(i) */
            d = std::min<Index>(d, n - nel - nvi);
            if (head[d] != -1) last[head[d]] = i;
            next[i] = head[d];               /* put i back in degree list */
            last[i] = -1;
            head[d] = i;
            mindeg = std::min<Index>(mindeg, d);       /* find new minimum degree */
            degree[i] = d;
            Ci[p++] = i;                      /* place i in Lk */
        }
        nv[k] = nvk;                      /* # nodes absorbed into k */
        if ((len[k] = p - pk1) == 0)         /* length of adj list of element k*/
        {
            Co[k] = -1;                   /* k is a root of the tree */
            w[k] = 0;                     /* k is now a dead element */
        }
        if (elenk != 0) cnz = p;           /* free unused space in Lk */
    }

    /* --- Postordering ----------------------------------------------------- */
    for (i = 0; i < n; i++) Co[i] = amd_flip(Co[i]);/* fix assembly tree */
    for (j = 0; j <= n; j++) head[j] = -1;
    for (j = n; j >= 0; j--)              /* place unordered nodes in lists */
    {
        if (nv[j] > 0) continue;          /* skip if j is an element */
        next[j] = head[Co[j]];          /* place j in list of its parent */
        head[Co[j]] = j;
    }
    for (e = n; e >= 0; e--)              /* place elements in lists */
    {
        if (nv[e] <= 0) continue;         /* skip unless e is an element */
        if (Co[e] != -1)
        {
            next[e] = head[Co[e]];      /* place e in list of its parent */
            head[Co[e]] = e;
        }
    }
    for (k = 0, i = 0; i <= n; i++)       /* postorder the assembly tree */
    {
        if (Co[i] == -1) k = internal::cs_tdfs<Index>(i, k, head, next, perm.indices().data(), w);
    }

    perm.indices().conservativeResize(n);

    delete[] W;
    if (deleteCo) delete[] Co;
    if (deleteCi) delete[] Ci;
}


void amd_minimum_degree_ordering(const SparceMatrixType& _C, PermutationMatrix<Dynamic, Dynamic, Index>& perm)
{
    // fast
    //amd_minimum_degree_ordering__separated(_C, perm);
    // fast+
    amd_minimum_degree_ordering__IndexObject(_C, perm);
}