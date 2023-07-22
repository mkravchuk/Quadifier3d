#pragma once
#include <xmmintrin.h> //  for SSE
#include <emmintrin.h> //  for SSE
#include <smmintrin.h> //  for SSE 4.1


namespace utils
{
    namespace sse
    { 
        namespace _4
        {
            __forceinline float __vectorcall dot(__m128 v1, __m128 v2)
            {
                return _mm_dp_ps(v1, v2, 113).m128_f32[0];
            }

            #define SHUFFLE(i0,i1,i2,i3) (((i3) << 6) | ((i2) << 4) | ((i1) << 2) | ((i0)))

            //************************************
            //************************************
            // int
            //************************************
            //************************************
            // sorts 2 ints, and also high pair of ints (sort two pair of ints - low pair and high pair)
            template<int index0, int index1>
            __forceinline __m128i __vectorcall take2sortedInts(__m128i& v)
            {
                // we have number {a,b,c,d}
                // we have to sort two pairs {index0,index1} 
                auto _01 = _mm_shuffle_epi32(v, SHUFFLE(index0, index1, 2, 3)); // sort pairs (a,b) 
                auto _10 = _mm_shuffle_epi32(v, SHUFFLE(index1, index0, 2, 3)); // sort pairs (a,b) 
                auto cmp = _mm_cmplt_epi32(_01, _10);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 0, 2, 2));// copy pair with lowest first value
                return _mm_blendv_epi8(_10, _01, cmp); // contain 2 sorted pairs
            }

            // sorts 2 ints, and also high pair of ints (sort two pair of ints - low pair and high pair)
            __forceinline void __vectorcall sort2ints(__m128i& _0123)
            {
                // we have number {0,1,2,3}
                // we have to sort two pairs {0,1} and {2,3}
                auto _1032 = _mm_shuffle_epi32(_0123, SHUFFLE(1, 0, 3, 2)); // sort pairs (a,b) and (c,d)
                auto cmp = _mm_cmplt_epi32(_0123, _1032);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 0, 2, 2));// copy pair with lowest first value
                _0123 = _mm_blendv_epi8(_1032, _0123, cmp); // contain 2 sorted pairs
            }
            // sorts 2 ints, and also high pair of ints (sort two pair of ints - low pair and high pair)
            __forceinline void __vectorcall sort2ints(__m128i& v, __m128i& sortedIndexes)
            {
                sortedIndexes = _mm_set_epi32(3, 2, 1, 0);

                // we have number {a,b,c,d}
                // we have to sort two pairs {a,b} and {c,d}
                auto shuffled = _mm_shuffle_epi32(v, SHUFFLE(1, 0, 3, 2)); // sort pairs (a,b) and (c,d)
                auto shuffledI = _mm_shuffle_epi32(sortedIndexes, SHUFFLE(1, 0, 3, 2)); // sort pairs (a,b) and (c,d)
                auto cmp = _mm_cmplt_epi32(v, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 0, 2, 2));// copy pair with lowest first value
                v = _mm_blendv_epi8(shuffled, v, cmp); // contain 2 sorted pairs
                sortedIndexes = _mm_blendv_epi8(shuffledI, sortedIndexes, cmp); // contain 2 sorted pairs
            }

            __forceinline void __vectorcall sort4ints(__m128i& v)
            {
                // Step#1
                // we have number {a,b,c,d}
                // we have to sort two pairs {a,b} and {c,d}
                sort2ints(v);

                // Step#2
                // we have 2 sorted pairs, so 0 or 2 will be absolute minimum, and 1 or 3 absolute maximum
                // we have to move abolute minimum to 1 index and absolute maximum to 3 index
                auto shuffled = _mm_shuffle_epi32(v, SHUFFLE(2, 3, 0, 1)); // compare minimums and maximums for each pair
                auto cmp = _mm_cmplt_epi32(v, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 1, 0, 1));// copy min to 0 index, and maximum to 3 index, others leave as is
                v = _mm_blendv_epi8(shuffled, v, cmp); // contain {min, val, val, max}

                // Step#3
                // we have number {min, val, val, max}
                // we have to sort 2 values at middle
                shuffled = _mm_shuffle_epi32(v, SHUFFLE(0, 2, 1, 3)); // compare 1 and 2 indexes (middle values)
                cmp = _mm_cmplt_epi32(v, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 1, 1, 3));// copy lowest at 1 and highest at 2, 0 and 3 leave unchanged, since we already sort them in step 2
                v = _mm_blendv_epi8(shuffled, v, cmp); // contains sorted array
            }
            __forceinline void __vectorcall sort4ints(__m128i& v, __m128i& sortedIndexes)
            {
                // Step#1
                // we have number {a,b,c,d}
                // we have to sort two pairs {a,b} and {c,d}
                sort2ints(v, sortedIndexes);

                // Step#2
                // we have 2 sorted pairs, so 0 or 2 will be absolute minimum, and 1 or 3 absolute maximum
                // we have to move abolute minimum to 1 index and absolute maximum to 3 index
                auto shuffled = _mm_shuffle_epi32(v, SHUFFLE(2, 3, 0, 1)); // compare minimums and maximums for each pair
                auto shuffledI = _mm_shuffle_epi32(sortedIndexes, SHUFFLE(2, 3, 0, 1)); // sort pairs (a,b) and (c,d)
                auto cmp = _mm_cmplt_epi32(v, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 1, 0, 1));// copy min to 0 index, and maximum to 3 index, others leave as is
                v = _mm_blendv_epi8(shuffled, v, cmp); // contain {min, val, val, max}
                sortedIndexes = _mm_blendv_epi8(shuffledI, sortedIndexes, cmp); // contain 2 sorted pairs

                // Step#3
                // we have number {min, val, val, max}
                // we have to sort 2 values at middle
                shuffled = _mm_shuffle_epi32(v, SHUFFLE(0, 2, 1, 3)); // compare 1 and 2 indexes (middle values)
                shuffledI = _mm_shuffle_epi32(sortedIndexes, SHUFFLE(0, 2, 1, 3)); // sort pairs (a,b) and (c,d)
                cmp = _mm_cmplt_epi32(v, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 1, 1, 3));// copy lowest at 1 and highest at 2, 0 and 3 leave unchanged, since we already sort them in step 2
                v = _mm_blendv_epi8(shuffled, v, cmp); // contains sorted array
                sortedIndexes = _mm_blendv_epi8(shuffledI, sortedIndexes, cmp); // contain 2 sorted pairs
            }


            __forceinline void __vectorcall sort3ints(__m128i& _012)
            {
                // v0
                //_012.m128i_i32[3] = INT_MAX;
                //sort4ints(_012);



                //  v1
                //    auto _0000 = _mm_shuffle_epi32(_012, SHUFFLE(0, 0, 0, 0));
                //    auto _1122 = _mm_shuffle_epi32(_012, SHUFFLE(1, 1, 2, 2));
                //    auto cmp0 = _mm_cmpgt_epi32(_0000, _1122);
                //    int index0 = __popcnt(_mm_movemask_pd(_mm_castsi128_pd(cmp0)));

                //    auto _1111 = _mm_shuffle_epi32(_012, SHUFFLE(1, 1, 1, 1));
                //    auto _0022 = _mm_shuffle_epi32(_012, SHUFFLE(0, 0, 2, 2));
                //    auto cmp1 = _mm_cmpgt_epi32(_1111, _0022);
                //    auto index1 = __popcnt(_mm_movemask_pd(_mm_castsi128_pd(cmp1)));

                //    auto _2222 = _mm_shuffle_epi32(_012, SHUFFLE(2, 2, 2, 2));
                //    auto _0011 = _mm_shuffle_epi32(_012, SHUFFLE(0, 0, 1, 1));
                //    auto cmp2 = _mm_cmpgt_epi32(_2222, _0011);
                //    auto index2 = __popcnt(_mm_movemask_pd(_mm_castsi128_pd(cmp2)));


                // v2
                //auto _0011 = _mm_shuffle_epi32(_012, SHUFFLE(0, 0, 1, 1));
                //auto _1202 = _mm_shuffle_epi32(_012, SHUFFLE(1, 2, 0, 2));
                //auto cmp = _mm_cmpgt_epi32(_0011, _1202);

                //int mask = _mm_movemask_epi8(cmp);
                //constexpr int mask0 = (1 | (1 << 4));
                //constexpr int mask1 = mask0 << 8;

                //int index0 = __popcnt(mask &  mask0);
                //int index1 = __popcnt(mask &  mask1);
                //auto index2 = (0 + 1 + 2) - index0 - index1;  // since we know index0 and index1 - then index2 will be the last free index


                //__m128i _sorted;
                //_sorted.m128i_i32[index0] = _012.m128i_i32[0];
                //_sorted.m128i_i32[index1] = _012.m128i_i32[1];
                //_sorted.m128i_i32[index2] = _012.m128i_i32[2];
                //_012 = _sorted;



                //  v3
                // we have number {0,1,2, }
                // we have to sort two pairs {0,1} and {2}
                auto _1022 = _mm_shuffle_epi32(_012, SHUFFLE(1, 0, 2, 2)); // sort pairs (a,b) and (c)
                auto cmp = _mm_cmplt_epi32(_012, _1022);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 0, 2, 2));// copy pair with lowest first value
                _012 = _mm_blendv_epi8(_1022, _012, cmp); // contain 2 sorted pairs


                // Step#2
                // we have number {min, val, 2, }
                // we have to sort 2 values at middle
                auto shuffled = _mm_shuffle_epi32(_012, SHUFFLE(0, 2, 1, 3)); // compare 1 and 2 indexes (middle values)
                cmp = _mm_cmplt_epi32(_012, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 1, 1, 3));// copy lowest at 1 and highest at 2, 0 and 3 leave unchanged, since we already sort them in step 2
                _012 = _mm_blendv_epi8(shuffled, _012, cmp); // contains sorted array


                // Step#3
                // we have to sort two pairs {0,1} 
                shuffled = _mm_shuffle_epi32(_012, SHUFFLE(1, 0, 2, 2)); // sort pairs (a,b) 
                cmp = _mm_cmplt_epi32(_012, shuffled);
                cmp = _mm_shuffle_epi32(cmp, SHUFFLE(0, 0, 2, 2));// copy pair with lowest first value
                _012 = _mm_blendv_epi8(shuffled, _012, cmp); // contain 2 sorted pairs




            }

            __forceinline void __vectorcall sort3ints(__m128i& v, __m128i& sortedIndexes)
            {
                v.m128i_i32[3] = INT_MAX;
                sort4ints(v, sortedIndexes);
            }
            void testSort();
        }

        inline int bitsIsSetCount(int bits)
        {
            return __popcnt(bits);
        }
        inline int bitsIsSetCount(const __m128i& bits)
        {
            return (int)(__popcnt64(bits.m128i_i64[0]) + __popcnt64(bits.m128i_i64[1]));
        }


        //************************************
        //************************************
        // float
        //************************************
        //************************************

        inline __m128 __vectorcall min(__m128 a, __m128 b)
        {
            return _mm_min_ps(a, b);
        }
        inline __m128 __vectorcall max(__m128 a, __m128 b)
        {
            return _mm_max_ps(a, b);
        }
        inline float min(float a, float b)
        {
            // Branchless SSE min.
            _mm_store_ss(&a, _mm_min_ss(_mm_set_ss(a), _mm_set_ss(b)));
            return a;
        }
        inline float max(float a, float b)
        {
            // Branchless SSE max.
            _mm_store_ss(&a, _mm_max_ss(_mm_set_ss(a), _mm_set_ss(b)));
            return a;
        }
        inline float clamp(float val, float minval, float maxval)
        {
            // Branchless SSE clamp.
            // return minss( maxss(val,minval), maxval );
            _mm_store_ss(&val, _mm_min_ss(_mm_max_ss(_mm_set_ss(val), _mm_set_ss(minval)), _mm_set_ss(maxval)));
            return val;
        }
        inline __m128 __vectorcall add(__m128 a, __m128 b)
        {
            return _mm_add_ps(a, b);
        }
        inline __m128 __vectorcall sqrt(__m128 a)
        {
            return _mm_sqrt_ps(a);
        }
        inline __m128d __vectorcall sqrt(__m128d a)
        {
            return _mm_sqrt_pd(a);
        }
        inline float __vectorcall sqrt(float a)
        {
            return _mm_sqrt_ps(_mm_set1_ps(a)).m128_f32[0];
        }
        inline double __vectorcall sqrt(double a)
        {
            return _mm_sqrt_pd(_mm_set1_pd(a)).m128d_f64[0];
        }
        inline float __vectorcall sqrt_inverse_approximate(float a)
        {
            return _mm_rsqrt_ss(_mm_set1_ps(a)).m128_f32[0];
        }
        void sqrt(Ds& data);
        __m128 __vectorcall log(__m128 x);
        __m128 __vectorcall exp(__m128 x);
        __m128 __vectorcall sin(__m128 x);
        __m128 __vectorcall cos(__m128 x);
        void __vectorcall sincos(__m128 x, __m128& sin, __m128& cos);
    }
}