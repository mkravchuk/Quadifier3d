
#pragma once

template <class T, bool willHaveDiagonalIndexes>
class SparceMatrixDirect
{
public:
    int rows;
    int cols;
    Is counts;
    Is indexes;
    Matrix<T, Dynamic, 1> values;
    SparceMatrixDirect(int rows, int cols)
    {
        this->rows = rows;
        this->cols = cols;
        counts = Is::Constant(cols, 0);
        indexes = Is::Constant(cols * 4, 0);
        values.resize(cols * 4); // avoid initialization since it will take a time for example for Complex4f
    }
    void increaseDiagonalCounts(int fid)
    {
        assert(willHaveDiagonalIndexes);
        //v0
        //indexes[fid * 4 + 0] = fid;
        //values[fid * 4 + 0] += T(1.f);
        //v1 - faster - we dont need to store value - we will remember counts in array 'indexes', and will use that stored 'count' value in method 'setSparseMatrix'
        indexes[fid * 4 + 0] += 1; // use this position in array for storing only counts of diagonal values (speed optimization)
    };
    void setValue(int fidRow, int fidCol, const T& value)
    {
        int count = counts[fidCol];
        count++;
        assert(count <= 3);
        counts[fidCol] = count;
        if (willHaveDiagonalIndexes)
        {
            indexes[fidCol * 4 + count] = fidRow;// first index reserved for diagonal index
            values[fidCol * 4 + count] = value;// first index reserved for diagonal value
        }
        else
        {
            indexes[fidCol * 4 + count - 1] = fidRow;
            values[fidCol * 4 + count - 1] = value;
        }
    };

    template <typename IndexType>
    void setSparseMatrix(SparseMatrix<T, ColMajor, IndexType>& q)
    {
        // create sparce matrix
        q.resize(rows, cols);
        int size = cols;

        IndexType* qp = q.outerIndexPtr();
        qp[0] = 0;
        // v0
        //for (IndexType i = 0; i < size; ++i)
        //    qp[i + 1] = qp[i] + counts[i] + 1;
        // v1 - fast, using direct pointers and remembering prev value
        int* pcounts = counts.data();
        int* pcountsEnd = counts.data() + size;
        if (pcounts < pcountsEnd)
        {
            IndexType prev = 0;
            do
            {
                qp++;
                IndexType current = prev + *pcounts + (willHaveDiagonalIndexes ? 1 : 0); // plus diagonal value, which must to have for each col
                *qp = current;
                prev = current;
                pcounts++;
            } while (pcounts < pcountsEnd);
        }

        // finalize index construction
        q.resizeNonZeros(q.outerIndexPtr()[size]);

        IndexType* qi = q.innerIndexPtr();
        T* qx = q.valuePtr();

        int* pindexes = indexes.data() - 4;
        T* pvalues = values.data() - 4;
        for (int i = 0; i < counts.size(); i++)
        {
            pindexes += 4;// move pointer on row-indexes to next col
            pvalues += 4; // move pointer on values to next col
            int valuesCount = counts(i) + (willHaveDiagonalIndexes ? 1 : 0); // plus diagonal value, which must to have for each col
            if (valuesCount == 0) continue;
            __m128i rowIndexes = Vec4i(pindexes);

            int diagonalCounts = rowIndexes.m128i_i32[0]; // extract temporaly stored value (it was done in method 'increaseDiagonalCounts' to achive better performance)
            T diagonalValue(willHaveDiagonalIndexes ? (0.f + diagonalCounts) : *pvalues); // create diagonal value, what we should have done in method 'increaseDiagonalCounts', but shift here to achive better performance
            if (willHaveDiagonalIndexes) rowIndexes.m128i_i32[0] = i; // store real value - diagonal index what is equal to row index
            __m128i sortedIndexes;


            if (utils::cpu::isSupportedSSE4)
            {
                //
                // v1 - using sse - much faster
                //
                if (valuesCount == 4) // most of the cols will have 4 values - 1 for diagonal and 3 for edges
                {
                    utils::sse::_4::sort4ints(rowIndexes, sortedIndexes);
                    auto si = sortedIndexes.m128i_i32;
                    Vec4i(rowIndexes).store(qi); // since we have to store all 4 indexes and they are already sorted, we can store them at one shot as a single 128 bit value
                    *(qx + 0) = (si[0] == 0) ? diagonalValue : pvalues[si[0]];
                    *(qx + 1) = (si[1] == 0) ? diagonalValue : pvalues[si[1]];
                    *(qx + 2) = (si[2] == 0) ? diagonalValue : pvalues[si[2]];
                    *(qx + 3) = (si[3] == 0) ? diagonalValue : pvalues[si[3]];

                    qi += 4;
                    qx += 4;
                }
                else if (valuesCount == 3)
                {
                    utils::sse::_4::sort3ints(rowIndexes, sortedIndexes);
                    auto si = sortedIndexes.m128i_i32;
                    *(qi + 0) = rowIndexes.m128i_i32[0];
                    *(qx + 0) = (si[0] == 0) ? diagonalValue : pvalues[si[0]];

                    *(qi + 1) = rowIndexes.m128i_i32[1];
                    *(qx + 1) = (si[1] == 0) ? diagonalValue : pvalues[si[1]];

                    *(qi + 2) = rowIndexes.m128i_i32[2];
                    *(qx + 2) = (si[2] == 0) ? diagonalValue : pvalues[si[2]];

                    qi += 3;
                    qx += 3;
                }
                else if (valuesCount == 2)
                {
                    utils::sse::_4::sort2ints(rowIndexes, sortedIndexes);
                    auto si = sortedIndexes.m128i_i32;
                    *(qi + 0) = rowIndexes.m128i_i32[0];
                    *(qx + 0) = (si[0] == 0) ? diagonalValue : pvalues[si[0]];

                    *(qi + 1) = rowIndexes.m128i_i32[1];
                    *(qx + 1) = (si[1] == 0) ? diagonalValue : pvalues[si[1]];

                    qi += 2;
                    qx += 2;

                }
                else if (valuesCount == 1)
                {
                    *(qi + 0) = rowIndexes.m128i_i32[0];
                    *(qx + 0) = diagonalValue;

                    qi += 1;
                    qx += 1;
                }
            }
            else
            {
                //
                // v0 - using std
                //
                auto sort_ints_std = [](__m128i& v, __m128i& sortedIndexes, int valuesCount)
                {
                    if (valuesCount == 0) return;
                    std::vector<int> a(valuesCount);
                    a[0] = v.m128i_i32[0];
                    if (valuesCount >= 2) a[1] = v.m128i_i32[1];
                    if (valuesCount >= 3) a[2] = v.m128i_i32[2];
                    if (valuesCount >= 4) a[3] = v.m128i_i32[3];
                    std::vector<unsigned int> indexes = utils::stdvector::sort_indexes(a);
                    sortedIndexes.m128i_i32[0] = indexes[0];
                    if (valuesCount >= 2) sortedIndexes.m128i_i32[1] = indexes[1];
                    if (valuesCount >= 3) sortedIndexes.m128i_i32[2] = indexes[2];
                    if (valuesCount >= 4) sortedIndexes.m128i_i32[3] = indexes[3];
                    v.m128i_i32[0] = a[indexes[0]];
                    if (valuesCount >= 2) v.m128i_i32[1] = a[indexes[1]];
                    if (valuesCount >= 3) v.m128i_i32[2] = a[indexes[2]];
                    if (valuesCount >= 4) v.m128i_i32[3] = a[indexes[3]];
                };
                sort_ints_std(rowIndexes, sortedIndexes, valuesCount);
                auto si = sortedIndexes.m128i_i32;

                if (valuesCount == 0) continue;
                *qi = rowIndexes.m128i_i32[0];
                *qx = (si[0] == 0) ? diagonalValue : pvalues[si[0]];
                qi++;
                qx++;
                valuesCount--;

                if (valuesCount == 0) continue;
                *qi = rowIndexes.m128i_i32[1];
                *qx = (si[1] == 0) ? diagonalValue : pvalues[si[1]];
                qi++;
                qx++;
                valuesCount--;

                if (valuesCount == 0) continue;
                *qi = rowIndexes.m128i_i32[2];
                *qx = (si[2] == 0) ? diagonalValue : pvalues[si[2]];
                qi++;
                qx++;
                valuesCount--;

                if (valuesCount == 0) continue;
                *qi = rowIndexes.m128i_i32[3];
                *qx = (si[3] == 0) ? diagonalValue : pvalues[si[3]];
                qi++;
                qx++;
            }
        }
    }
};
