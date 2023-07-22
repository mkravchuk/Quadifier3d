#pragma once
#include "CommonClassesShared.h"
#include "List__Dynamic.h"  
#include <vector>  



template <class T>
class CompactVectorVector
{
public:
    // minimum information of size 8 bytes
    struct CompactVectorVector_Info
    {
        int size;
        int positionInData;
    };
private:
    int& position(int vectorIndex)
    {
        return infoptr[vectorIndex].positionInData;
    }
    int position(int vectorIndex) const
    {
        return infoptr[vectorIndex].positionInData;
    }
    int _size;
    CommonClasses::List<CompactVectorVector_Info> Info; // size, positionInData  (stores elements in vector, positions of vector in data)
    CommonClasses::List<T> Data; // compacted data for each vector
    CompactVectorVector_Info* infoptr;
    T* dataptr;
    T dummy;

    void checkVectorIndex(int vectorIndex, std::string methodName) const
    {
        #ifdef DEBUG
        if (vectorIndex >= _size)
        {
            std::cout << "!!! Error:   CompactVectorVector::" << methodName << " - index is out of size:  vectorIndex=" << vectorIndex << ",   size=" << _size << std::endl;
            assert(vectorIndex < _size);
            return;
        }
        #endif
    }

    void RefreshPointers()
    {
        dataptr = Data.data();
        infoptr = Info.data();
    }

public:
    CompactVectorVector()
        : _size(0), infoptr(nullptr), dataptr(nullptr)
    {
        clear();
    }

    CompactVectorVector(const CompactVectorVector& a) // copy constructor (Rule of five)
        : _size(a.size()), Info(a.Info), Data(a.Data), infoptr(nullptr), dataptr(nullptr)
    {
        RefreshPointers();
    }

    CompactVectorVector& operator=(CompactVectorVector other)
    {
        std::swap(_size, other._size);
        std::swap(Info, other.Info);
        std::swap(Data, other.Data);
        std::swap(infoptr, other.infoptr);
        std::swap(dataptr, other.dataptr);
        return *this;
    }

    void InitFromVectors(const std::vector<std::vector<T>>& v)
    {
        resizeBegin(v.size(), true);
        auto info = InfoPOINTER();
        int totalSize = 0;
        for (int i = 0; i < v.size(); i++)
        {
            int size = v[i].size();
            info->size = size;
            info->positionInData = totalSize;
            totalSize += size;
            info++;
        }
        resizeEnd(totalSize); // pass totalSize since we done direct initialization

        T* next_write = dataptr;
        for (int i = 0; i < v.size(); i++)
        {
            const T* vbegin = v[i].data();
            const T* vend = vbegin + v[i].size();
            while (vbegin < vend)
            {
                *next_write = *vbegin;
                vbegin++;
                next_write++;
            }
        }
    }

    CompactVectorVector_Info* InfoPOINTER()
    {
        return infoptr;
    }
    T* DataPOINTER()
    {
        return dataptr;
    }
    int DataSize()
    {
        return Data.size();
    }

    /// Allocate memory for information need to keep data in vectors
    /// Call this method in a sequence: resizeBegin(), size(i)++, resize(k)=N, resizeEnd()
    /// 'skipZeroingInfo' - set to 'true' if all 'size' will be populated just after calling this method by setting direct value: size(i)=N, otherwise if you not sure - leave it as 'false' (speed optimization flag - avoiding zeroing memory will improve performance - especially important for big arrays)
    void resizeBegin(int newSize, bool skipZeroingInfo = false)
    {
        _size = newSize;
        Info.resize(newSize);
        if (!skipZeroingInfo)
        {
            //Info.setConstant({ 0,0,0,nullptr });
            Info.setZero();
        }
        Data.resize(0);
        RefreshPointers();
    }
    void clear()
    {
        resizeBegin(0, false);
    }
    int size() const
    {
        return _size;
    }
    int& size(int vectorIndex)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "size(int vectorIndex)");
        #endif
        return infoptr[vectorIndex].size;
    }

    int size(int vectorIndex) const
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "size(int vectorIndex)");
        #endif
        return infoptr[vectorIndex].size;
    }
    int capacity(int vectorIndex) const
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "capacity(int vectorIndex)");
        #endif
        int begin = infoptr[vectorIndex].positionInData;
        int end = (vectorIndex == size() - 1) ? Data.size() : infoptr[vectorIndex + 1].positionInData;
        return end - begin;
    }

    // allocate item and return reference to it, so now it is possible to set all properties of item
    T& allocate(int vectorIndex)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "allocate(int vectorIndex)");
        #endif
        CompactVectorVector_Info& info = infoptr[vectorIndex];
        int& vectorSize = info.size;
        #ifdef DEBUG
        int vectorCapacity = capacity(vectorIndex);
        if (vectorSize >= vectorCapacity)
        {
            std::cout << "!!! Error:   CompactVectorVector::allocate() - cannot allocate more elements in vector " << vectorIndex << " because it is out of capacity " << std::endl;
            assert(vectorSize < vectorCapacity && "CompactVectorVector::allocate() - cannot allocate more elements in vector");
            return dummy;
        }
        #endif
        vectorSize++;
        return dataptr[info.positionInData + vectorSize - 1];
    }
    // same as 'add' but using direct pointers to info and data - this method tryis to be faster from 'add' by avoiding unferences
    static __forceinline void add_direct(CompactVectorVector_Info* infoptr, T* dataptr, int vectorIndex, const T& value)
    {
        auto& info = infoptr[vectorIndex];
        dataptr[info.positionInData + info.size] = value;
        info.size++;
    }
    __forceinline void add(int vectorIndex, const T& value)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "add(int vectorIndex)");
        #endif
        auto& info = infoptr[vectorIndex];
        #ifdef DEBUG
        int vectorCapacity = capacity(vectorIndex);
        int vectorSize = info.size;
        if (vectorSize >= vectorCapacity)
        {
            std::cout << "!!! Error:   CompactVectorVector::add() - cannot add more elements in vector " << vectorIndex << " because it is out of capacity " << std::endl;
            assert(vectorSize < vectorCapacity && "CompactVectorVector::add() - cannot add more elements in vector");
            return;
        }
        #endif
        dataptr[info.positionInData + info.size] = value;
        info.size++;
    }
    void add2(int vectorIndex, const T& value1, const T& value2)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "add2(int vectorIndex)");
        #endif
        CompactVectorVector_Info& info = infoptr[vectorIndex];
        int& vectorSize = info.size;
        #ifdef DEBUG
        int vectorCapacity = capacity(vectorIndex);
        if (vectorSize + 2 > vectorCapacity)
        {
            std::cout << "!!! Error:   CompactVectorVector::add2() - cannot add more elements in vector " << vectorIndex << " because it is out of capacity " << std::endl;
            assert(vectorSize + 2 <= vectorCapacity && "CompactVectorVector::add2() - cannot add more elements in vector");
            return;
        }
        #endif
        dataptr[info.positionInData + vectorSize] = value1;
        dataptr[info.positionInData + vectorSize + 1] = value2;
        vectorSize += 2;
    }
    void add3(int vectorIndex, const T& value1, const T& value2, const T& value3)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "add3(int vectorIndex)");
        #endif
        CompactVectorVector_Info& info = infoptr[vectorIndex];
        int& vectorSize = info.size;
        #ifdef DEBUG
        int vectorCapacity = capacity(vectorIndex);
        if (vectorSize + 3 > vectorCapacity)
        {
            std::cout << "!!! Error:   CompactVectorVector::add3() - cannot add more elements in vector " << vectorIndex << " because it is out of capacity " << std::endl;
            assert(vectorSize + 3 <= vectorCapacity && "CompactVectorVector::add3() - cannot add more elements in vector");
            return;
        }
        #endif
        dataptr[info.positionInData + vectorSize] = value1;
        dataptr[info.positionInData + vectorSize + 1] = value2;
        dataptr[info.positionInData + vectorSize + 2] = value3;
        vectorSize += 3;
    }
    void add(int vectorIndex, const std::vector<T>& values)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "add(int vectorIndex, values)");
        #endif
        CompactVectorVector_Info& info = infoptr[vectorIndex];
        int& vectorSize = info.size;

        #ifdef DEBUG
        int vectorCapacity = capacity(vectorIndex);
        if (vectorSize + values.size() > vectorCapacity)
        {
            std::cout << "!!! Error:   CompactVectorVector::add() - cannot add more elements in vector " << vectorIndex << " because it is out of capacity " << std::endl;
            assert(vectorSize + values.size() <= vectorCapacity && "CompactVectorVector::add() - cannot add more elements in vector");
            return;
        }
        #endif

        T* writePtr = dataptr + info.positionInData + vectorSize;
        const T* readPtr = values.data();
        for (int i = 0; i < values.size(); i++)
        {
            *writePtr = *readPtr;
            writePtr++;
            readPtr++;
        }
        vectorSize += values.size();
    }
    void add_inReversedOrder(int vectorIndex, const T& value)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "add_inReversedOrder(int vectorIndex, const T& value)");
        #endif
        CompactVectorVector_Info& info = infoptr[vectorIndex];
        int& vectorSize = info.size;
        int vectorCapacity = capacity(vectorIndex);
        #ifdef DEBUG
        if (vectorSize >= vectorCapacity)
        {
            std::cout << "!!! Error:   CompactVectorVector::add_inReverseOrder() - cannot add more elements in vector " << vectorIndex << " because it is out of capacity " << std::endl;
            assert(vectorSize < vectorCapacity);
            return;
        }
        #endif
        vectorSize++;
        dataptr[info.positionInData + vectorCapacity - vectorSize] = value;
    }

    T& operator()(int vectorIndex, int valueIndex)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "(int vectorIndex, int valueIndex)");
        #endif
        #ifdef DEBUG
        int vectorSize = size(vectorIndex);
        if (valueIndex > vectorSize - 1)
        {
            std::cout << "!!! Error:   CompactVectorVector::get() - cannot get element from vector " << vectorIndex << " because it is out of scope:  vector size " << vectorSize << ", but requested index is " << valueIndex << std::endl;
            assert(valueIndex < vectorSize);
            return dummy;
        }
        #endif
        //return dataptr[position(vectorIndex) + valueIndex];
        return *pointer(vectorIndex, valueIndex);
    }

    const T& operator()(int vectorIndex, int valueIndex) const
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "(int vectorIndex, int valueIndex)");
        #endif
        #ifdef DEBUG
        int vectorSize = size(vectorIndex);
        if (valueIndex > vectorSize - 1)
        {
            std::cout << "!!! Error:   CompactVectorVector::get() - cannot get element from vector " << vectorIndex << " because it is out of scope:  vector size " << vectorSize << ", but requested index is " << valueIndex << std::endl;
            assert(valueIndex < vectorSize);
            return dummy;
        }
        #endif
        return *pointer(vectorIndex, valueIndex);
    }

    T* pointer(int vectorIndex)
    {
        //return &dataptr[position(vectorIndex)];
        return dataptr + position(vectorIndex);
    }
    const T* pointer(int vectorIndex) const
    {
        //return &dataptr[position(vectorIndex)];
        return dataptr + position(vectorIndex);
    }
    T* pointer(int vectorIndex, int valueIndex)
    {
        //return &dataptr[position(vectorIndex) + valueIndex];
        return pointer(vectorIndex) + valueIndex;
    }
    const T* pointer(int vectorIndex, int valueIndex) const
    {
        //return &dataptr[position(vectorIndex) + valueIndex];
        return pointer(vectorIndex) + valueIndex;
    }



    int find(int vectorIndex, T value) const
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "find(int vectorIndex, T value)");
        #endif
        const CompactVectorVector_Info& info = infoptr[vectorIndex];
        int vectorSize = info.size;
        const T* p = pointer(vectorIndex);
        for (int i = 0; i < vectorSize; i++)
        {
            if (*p == value) return i;
            ++p;
        }
        return -1;
    }
    bool exists(int vectorIndex, T value) const
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "exists(int vectorIndex, T value)");
        #endif
        const T* p = pointer(vectorIndex);
        const T* pEnd = p + size(vectorIndex);
        while (p < pEnd)
        {
            if (*p == value) return true;
            ++p;
        }
        return false;
    }

    void sort(int vectorIndex, std::function<bool(const T& a, const T& b)> predicate__a_less_from_b)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "sort(int vectorIndex, predicate__a_less_from_b)");
        #endif
        T* first = pointer(vectorIndex);
        T* end = first + size(vectorIndex);
        while (first < end)
        {
            T* best = first;
            T* next = first + 1;
            while (next < end)
            {
                if (predicate__a_less_from_b(*next, *best))
                {
                    best = next;
                }
                ++next;
            }

            if (best != first)
            {
                std::swap(*best, *first);
            }
            ++first;
        }
    }

    void remove(int vectorIndex, int valueIndex)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "remove(int vectorIndex, int valueIndex)");
        #endif
        int vectorSize = size(vectorIndex);
        #ifdef DEBUG
        if (valueIndex > vectorSize - 1)
        {
            std::cout << "!!! Error:   CompactVectorVector::remove() - cannot remove element from vector " << vectorIndex << " because it is out of scope:  vector size " << vectorSize << ", but requested index is " << valueIndex << std::endl;
            assert(valueIndex < vectorSize);
            return;
        }
        #endif

        T* first = pointer(vectorIndex);
        T* end = first + vectorSize - 1; // excluding last since we want to take 'iNext = i+1'
        T* replaced = first + valueIndex; //  set cursor to removed index
        while (replaced < end)
        {
            T* moved = replaced + 1;
            *replaced = *moved;
            ++replaced;
        }
        size(vectorIndex)--; // decrement size of vector
    }

    void remove(std::vector<int>& deleted_vectorIndexes, bool sorted = false)
    {
        if (deleted_vectorIndexes.size() == 0) return;

        if (!sorted) std::sort(deleted_vectorIndexes.begin(), deleted_vectorIndexes.end());
        CompactVectorVector<T> n;
        n.resizeBegin(size() - deleted_vectorIndexes.size(), false);
        int skiped = 0;
        int totalSize = 0;
        auto ninfo = n.InfoPOINTER();
        for (int i = 0; i < size(); i++)
        {
            if (binary_search(deleted_vectorIndexes.begin(), deleted_vectorIndexes.end(), i))
            {
                skiped++;
                continue;
            }
            ninfo->size = size(i);
            ninfo->positionInData = totalSize;
            totalSize += ninfo->size;
            ninfo++;
        }
        n.resizeEnd(totalSize);
        skiped = 0;
        T* pnew = n.DataPOINTER();
        T* pold = DataPOINTER();
        long long copysize = 0;
        for (int i = 0; i < size(); i++)
        {
            if (binary_search(deleted_vectorIndexes.begin(), deleted_vectorIndexes.end(), i))
            {
                if (copysize > 0) memcpy(pnew, pold, copysize * sizeof(T));
                pnew += copysize;
                pold = pointer(i + 1);
                copysize = 0;
                skiped++;
                continue;
            }
            copysize += size(i);
        }
        if (copysize > 0) memcpy(pnew, pold, copysize * sizeof(T));

        std::swap(_size, n._size);
        std::swap(Info, n.Info);
        std::swap(Data, n.Data);
        std::swap(infoptr, n.infoptr);
        std::swap(dataptr, n.dataptr);
        n.clear();
    }

    std::vector<std::vector<T>> items()
    {
        std::vector<std::vector<T>> res;
        res.reserve(size());
        for (int i = 0; i < size(); i++)
        {
            res.push_back(items(i));
        }
        return res;
    }

    std::vector<T> items(int vectorIndex)
    {
        #ifdef DEBUG
        checkVectorIndex(vectorIndex, "items(int vectorIndex)");
        #endif
        std::vector<T> res;
        int vectorSize = size(vectorIndex);
        res.reserve(vectorSize);
        T* first = pointer(vectorIndex);
        T* end = first + vectorSize;
        T* i = first;
        while (i < end)
        {
            res.push_back(*i);
            ++i;
        }
        return res;
    }

    // shrink capacity to size
    void Compact()
    {

    }

    /// Finalize setting sizes to all vectors - in this method
    /// 1) data for vectors will be allocated
    /// 2) size to each vectors will be cleared to 0, and capacity will be calculated dynamically based on 'positionInData'
    /// Only after this method this class is ready to use
    /// Call this method in a sequence: resizeBegin(), size(i)++, resize(k)=N, resizeEnd()
    ///
    /// pass 'totalSize' only if for each vector 'size' and 'positionInData' was initialized - this is for speed improvement
    /// Example of direct initialization:
    /// auto info = ngons.InfoPOINTER();
    /// int totalSize = 0;
    /// for (int i = 0; i < F.size(); i++)
    /// {
    ///     int size = F[i].size();
    ///     info->size = 0; // yes - set to zero
    ///     info->positionInData = totalSize;
    ///     totalSize += size;
    ///     info++;
    /// }
    /// ngons.resizeEnd(totalSize); // pass totalSize since we done direct initialization
    ///
    int resizeEnd(int totalSize = -1)
    {
        // set 'info.size',  'info.positionInData'
        if (totalSize == -1)
        {
            totalSize = 0;
            CompactVectorVector_Info* info = Info.data();
            if (info != nullptr)
            {
                CompactVectorVector_Info* info_end = info + size();
                CompactVectorVector_Info* info_end4 = info + size() - 4;
                while (info < info_end4)
                {
                    int size0 = (info + 0)->size;
                    int size1 = (info + 1)->size;
                    int size2 = (info + 2)->size;
                    int size3 = (info + 3)->size;

                    (info + 0)->size = 0;
                    (info + 0)->positionInData = totalSize;
                    (info + 1)->size = 0;
                    (info + 1)->positionInData = totalSize + size0;
                    (info + 2)->size = 0;
                    (info + 2)->positionInData = totalSize + size0 + size1;
                    (info + 3)->size = 0;
                    (info + 3)->positionInData = totalSize + size0 + size1 + size2;
                    info += 4;
                    totalSize += size0 + size1 + size2 + size3;
                }
                while (info < info_end)
                {
                    auto size = info->size;
                    info->size = 0;
                    info->positionInData = totalSize;
                    totalSize += size;
                    info++;
                }
            }
        }
        // resize 'Data'
        Data.resize(totalSize);
        RefreshPointers();
        return totalSize;
    }

    long long SizeOF() const
    {
        return sizeof(this)
            + Info.size() * sizeof(CompactVectorVector_Info)
            + Data.size() * sizeof(T);
    }

    static void run_example_of_usage()
    {
        std::cout << "*** CompactVectorVector  test *** " << std::endl;
        CompactVectorVector<int> c;
        c.resizeBegin(3);
        std::cout << "SizeOF() = " << c.SizeOF() << std::endl;
        c.size(0) = 3;
        c.size(1) = 4;
        c.size(2) = 5;
        c.resizeEnd();
        std::cout << "resizeEnd() " << std::endl;
        c.add(0, 1);
        c.add(0, 2);
        c.add(0, 3);
        c.add(1, 11);
        c.add(1, 12);
        c.add(1, 13);
        c.add(1, 14);
        c.add(2, 21);
        c.add(2, 22);
        c.add(2, 23);
        c.add(2, 24);
        c.add(2, 25);
        c.remove(2, 3);
        std::cout << "SizeOF() = " << c.SizeOF() << std::endl;
        for (int i = 0; i < c.size(); i++)
        {
            std::cout << "vector#" << i << "    values: ";
            for (int k = 0; k < c.size(i); k++)
            {
                std::cout << c(i, k);
                if (k != c.size(i) - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
        std::cout << "c.find(1, 14)   == " << c.find(1, 14) << std::endl;
        std::cout << "c.find(2, 14)   == " << c.find(2, 14) << std::endl;
        std::cout << "c.exists(1, 14) == " << c.exists(1, 14) << std::endl;
        std::cout << "c.exists(2, 14) == " << c.exists(2, 14) << std::endl;
        std::cout << std::endl;
    }
};

