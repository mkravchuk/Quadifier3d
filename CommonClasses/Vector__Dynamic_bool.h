#pragma once
#include "CommonClassesShared.h"
#include "MemoryDynamic.h"
#include "Vector__Fixed.h"  // must be included, because this class is specialization of Vector__Fixed
#include <intrin.h> //  for  method '__popcnt'
#include <string> 
#include <vector> 

namespace CommonClasses
{
    using boolBits_Type = int;
    constexpr int boolBitsCount = sizeof(boolBits_Type) * 8;
    constexpr int boolBits_SHIFT = 5;
    constexpr int boolBits_MASK = 31;

    // use this class if you want to update bits manually using data() instead of using [i]
    class IntBits
    {
    private:
        boolBits_Type& bits;
        int bitIndex;// index in data()
    public:
        IntBits(boolBits_Type& _bits, int _bitIndex)
            : bits(_bits), bitIndex(_bitIndex)
        {
        }
        #pragma region Convert to
        operator bool() const
        {
            return (bits >> bitIndex) & 1;
        }
        #pragma endregion 
        #pragma region Convert from
        CommonClassesINLINE IntBits& operator=(bool value)
        {
            if (value)
            {
                bits |= (boolBits_Type)1 << bitIndex;
            }
            else
            {
                bits &= ~((boolBits_Type)1 << bitIndex);
            }
            return *this;
        }
        #pragma endregion 
    };

    class Vector_Dynamic_bool_Info
    {
    public:
        Index bitsCount; // real size, because in MemoryDynamic we store size in ints and also alignet to 128 bits so we can use sse
        Vector_Dynamic_bool_Info()
        {
            setNewCount(0);
        }

        void setNewCount(Index _bitsCount)
        {
            bitsCount = _bitsCount;
        }
    };

    // Specialization - dynamic bool vector (speed optimization)       for help see more at https://stackoverflow.com/questions/39154014/how-to-conditionally-add-a-function-to-a-class-template
    //  size is always divisable by 512 bits (4 times __m128i or 16 times int) - so you can use data() without worying of ouf of index exception
    template <>
    class Vector<bool, X> : public MemoryDynamic<int, Vector_Dynamic_bool_Info>
    {
    private:
        //const Index bitsperindex = sizeof(boolBits_Type) * 8;
        Index size_to_allocatesize(Index size)
        {
            return (size / 512 + 1) * 16;  // allocate size divisable by 512 bits (4 times 128bits or 16 times int)
        }
        static Index bitsIsSetCount(int bits)
        {
            return __popcnt(bits);
        }
        static unsigned long long bitsIsSetCount(const __m128i& bits)
        {
            return __popcnt64(bits.m128i_i64[0]) + __popcnt64(bits.m128i_i64[1]);
        }
    public:
        #pragma region Constructors
        Vector() = default;
        Vector(Index size)
        {
            resize(size);
        }
        constexpr Index size() const
        {
            return m_info.bitsCount;
        }

        constexpr boolBits_Type* data()
        {
            return (boolBits_Type*)m_data;
        }
        constexpr boolBits_Type* data() const
        {
            return (boolBits_Type*)m_data;
        }
        constexpr __m128i* data128()
        {
            return (__m128i*)m_data;
        }
        constexpr const __m128i* data128() const
        {
            return (__m128i*)m_data;
        }

        #pragma endregion    

        #pragma region Dimension control
        void resize(Index size)
        {
            m_info.setNewCount(size);
            resize_internal(size_to_allocatesize(size));
        }
        void conservativeResize(Index size)
        {
            m_info.setNewCount(size);
            conservativeResize_internal(size_to_allocatesize(size));
        }
        #pragma endregion 

        #pragma region Read/Write (single value)
        CommonClassesINLINE IntBits operator()(Index index)
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif

            return IntBits(data()[index >> boolBits_SHIFT], index & boolBits_MASK);
        }
        CommonClassesINLINE bool operator()(Index index) const
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            // all 3 method works indentical and with same speed
            return ((data()[index >> boolBits_SHIFT]) >> (index & boolBits_MASK)) & 1;
            //return IntBits(data()[index >> boolBits_shift], index & boolBits_mask);
        }
        CommonClassesINLINE IntBits operator[](Index index)
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            return IntBits(data()[index >> boolBits_SHIFT], index & boolBits_MASK);
        }
        CommonClassesINLINE bool operator[](Index index) const
        {
            #if DEBUG
            assert(index >= 0 && index < size() && "index is out of range");
            #endif
            // all 3 method works indentical and with same speed
            return ((data()[index >> boolBits_SHIFT]) >> (index & boolBits_MASK)) & 1;
            //return IntBits(data()[index >> boolBits_shift], index & boolBits_mask);
        }
        #pragma endregion 

        #pragma region  Fill data (Math)
        void setConstant(bool value)
        {
            __m128i constValue;
            if (value)
            {
                constValue = _mm_set1_epi32(0xFFFFFFFF);
            }
            else
            {
                constValue = _mm_set1_epi8(0);
            }

            __m128i* pbitints = data128();
            const __m128i* pbitintsEnd128_4x = data128() + m_size / 4; // size is always divisable by 512 bits (4 times __m128i or 16 times int)

            // every 512 bits
            while (pbitints < pbitintsEnd128_4x) // size is always divisable by 512 bits (4 times __m128i)
            {
                //TODO can be used '_mm_stream_si128' if memory is aligned by 128 bit ( mem_addr must be aligned on a 16-byte boundary or a general-protection exception may be generated.)
                _mm_store_si128(pbitints + 0, constValue);
                _mm_store_si128(pbitints + 1, constValue);
                _mm_store_si128(pbitints + 2, constValue);
                _mm_store_si128(pbitints + 3, constValue);
                pbitints += 4;
            }
        }
        void setZero()
        {
            setConstant(false);
        }
        void setZero(Index size)
        {
            resize(size);
            setZero();
        }
        void setConstant(Index size, bool value)
        {
            resize(size);
            setConstant(value);
        }
        #pragma endregion

        #pragma region Convertions
        std::string toString() const
        {
            std::string s = "";
            for (Index i = 0; i < size(); i++)
            {
                if (i != 0) s += ", ";
                s += std::to_string((*this)[i]);
            }
            return s;
        }
        template<class T>
        void ToIndexes(T& indexes, int precalculatedIndexesCount = -1) const
        {
            if (precalculatedIndexesCount == -1) precalculatedIndexesCount = countOfValues(true);
            indexes.resize(precalculatedIndexesCount);
            int* write = &indexes[0];
            int sizeof_ = boolBitsCount;
            int uimax = size() / sizeof_;
            __m128i fids0123 = _mm_set_epi32(3, 2, 1, 0); // here we need pass parameters in reverse order
            for (int ui = 0; ui < uimax; ui++)
            {
                auto usedbits = data()[ui];

                if (usedbits == 0) continue;
                auto count = bitsIsSetCount(usedbits);
                if (count == boolBitsCount)
                {
                    int fid = ui * boolBitsCount;
                    for (int bitIndex = 0; bitIndex < boolBitsCount / 4; bitIndex++)
                    {
                        //v0
                        //*(write + 0) = fid + 0;
                        //*(write + 1) = fid + 1;
                        //*(write + 2) = fid + 2;
                        //*(write + 3) = fid + 3;
                        //v1 - using sse
                        __m128i fids = _mm_add_epi32(_mm_set1_epi32(fid), fids0123); // add 4 integers at once
                        _mm_storeu_si128((__m128i*)write, fids); // write 4 integers at once
                        write += 4;
                        fid += 4;
                    }
                }
                else
                {
                    int fid = ui * boolBitsCount;
                    for (int bitIndex = 0; bitIndex < boolBitsCount; bitIndex++)
                    {
                        if (CommonClasses::IntBits(usedbits, bitIndex))
                        {
                            *write = fid;
                            write++;
                        }
                        fid++;
                    }
                }
            }
            for (int fid = size() - size() % boolBitsCount; fid < size(); fid++)
            {
                if ((*this)[fid])
                {
                    *write = fid;
                    write++;
                }
            }
        }
        friend std::ostream& operator<<(std::ostream& os, const Vector<bool, X>& v)
        {
            os << v.toString();
            return os;
        }
        #pragma endregion

        #pragma region Static Methods
        static Vector<bool, X> Zero(Index size)
        {
            Vector<bool, X>  m(size);
            m.setZero();
            return m;
        }
        static Vector<bool, X> Constant(Index size, bool value)
        {
            Vector<bool, X>  m(size);
            m.setConstant(value);
            return m;
        }
        #pragma endregion

        #pragma region Math Agregators  (single value)
        Index sum() const
        {
            unsigned long long res = 0;
            int count512 = size() / 512;
            int count128 = size() / 128;
            const __m128i* pbitints = data128();
            const __m128i* pbitintsEnd128_4x = data128() + count512 * 4; // size is always divisable by 512 bits (4 times __m128i)
            // every 512 bits
            while (pbitints < pbitintsEnd128_4x) // size is always divisable by 512 bits (4 times __m128i)
            {
                res += bitsIsSetCount(_mm_loadu_si128(pbitints + 0));
                res += bitsIsSetCount(_mm_loadu_si128(pbitints + 1));
                res += bitsIsSetCount(_mm_loadu_si128(pbitints + 2));
                res += bitsIsSetCount(_mm_loadu_si128(pbitints + 3));
                pbitints += 4;
            }
            //every 128 bits
            const __m128i* pbitintsEnd128 = data128() + count128;
            while (pbitints < pbitintsEnd128)
            {
                res += bitsIsSetCount(_mm_loadu_si128(pbitints));
                pbitints++;
            }

            // last 128 bits
            for (int i = m_info.bitsCount - m_info.bitsCount % 128; i < m_info.bitsCount; i++)
            {
                if ((*this)[i]) res++;
            }

            return (Index)res;
        }
        Index countOfValues(bool testValue) const
        {
            Index res = sum();
            if (testValue == false)
            {
                return size() - res;
            }
            return res;
        }
        bool all(bool testValue) const
        {
            int count512 = size() / 512;
            int count128 = size() / 128;
            const __m128i* pbitints = data128();
            const __m128i* pbitintsEnd128_4x = data128() + count512 * 4; // size is always divisable by 512 bits (4 times __m128i)
            // every 512 bits
            unsigned long long all512 = testValue ? 512 : 0;
            while (pbitints < pbitintsEnd128_4x) // size is always divisable by 512 bits (4 times __m128i)
            {
                unsigned long long all = bitsIsSetCount(_mm_loadu_si128(pbitints + 0));
                all += bitsIsSetCount(_mm_loadu_si128(pbitints + 1));
                all += bitsIsSetCount(_mm_loadu_si128(pbitints + 2));
                all += bitsIsSetCount(_mm_loadu_si128(pbitints + 3));
                if (all != all512) return false;
                pbitints += 4;
            }
            //every 128 bits
            unsigned long long all128 = testValue ? 128 : 0;
            const __m128i* pbitintsEnd128 = data128() + count128;
            while (pbitints < pbitintsEnd128)
            {
                unsigned long long all = bitsIsSetCount(_mm_loadu_si128(pbitints));
                if (all != all128) return false;
                pbitints++;
            }

            // last 128 bits
            bool all1 = testValue ? true : false;
            for (int i = m_info.bitsCount - m_info.bitsCount % 128; i < m_info.bitsCount; i++)
            {
                if ((*this)[i] != all1) return false;
            }

            return true;
        }

        #pragma endregion

        // find first index for which data()[i]==testValue, or return -1 if not found
        int FindFirstIndex(bool testValue) const
        {
            int uimax = size() / boolBitsCount;
            if (testValue)
            {
                for (int ui = 0; ui < uimax; ui++)
                {
                    auto usedbits = data()[ui];
                    if (usedbits == 0) continue;
                    int fid = ui * boolBitsCount;
                    // v0 - simple
                    for (int bitIndex = 0; bitIndex < boolBitsCount; bitIndex++)
                    {
                        if (CommonClasses::IntBits(usedbits, bitIndex)) return fid + bitIndex;
                    }
                    // v1 - fast
                    //unsigned long bitpos;
                    //_BitScanForward(usedbits, bitpos); // always true, becase we check it before  'while (mask != 0)'
                    //return fid + bitpos;
                }
                for (int fid = size() - size() % boolBitsCount; fid < size(); fid++)
                {
                    if ((*this)[fid]) return fid;
                }
            }
            else
            {
                for (int ui = 0; ui < uimax; ui++)
                {
                    auto usedbits = data()[ui];
                    if (usedbits == 0xffffffff) continue;
                    int fid = ui * boolBitsCount;
                    for (int bitIndex = 0; bitIndex < boolBitsCount; bitIndex++)
                    {
                        if (!CommonClasses::IntBits(usedbits, bitIndex)) return fid + bitIndex;
                    }
                }
                for (int fid = size() - size() % boolBitsCount; fid < size(); fid++)
                {
                    if (!(*this)[fid]) return fid;
                }

            }
            return -1;
        }

    };
}



#ifdef COMMONCLASSES_DEFINE_SHORTCUTS
namespace CommonClasses
{
    #pragma region Define   VectorXb

    #define CommonClasses_MAKE_TYPEDEFS_DYNAMIC(Type, TypeSuffix)         \
    typedef Vector<Type, X> Vector##X##TypeSuffix;

    CommonClasses_MAKE_TYPEDEFS_DYNAMIC(bool, b)


        #undef CommonClasses_MAKE_TYPEDEFS_DYNAMIC

        #pragma endregion
}
#endif
