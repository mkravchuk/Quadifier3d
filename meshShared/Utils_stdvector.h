#pragma once


namespace utils
{

    // Helps methods for std::vector  (all 'vector' must be prefixed by 'std' because here, in utils namespace, we have defined namespace 'vector' - so without 'std' prefix will be name conflict and compilation error)
    namespace stdvector
    {
        template <typename T>
        inline string toString(const std::vector<T> &v)
        {
            string text = "[";
            for (int i = 0; i < v.size(); i++)
            {
                if (i != 0) text += ",";
                text += to_string(v[i]);
            }
            text += "]";
            return text;
        }

        template <typename T>
        inline bool exists(const std::vector<T> &v, T value)
        {
            return std::find(v.begin(), v.end(), value) != v.end();
        }

        template <typename T>
        inline bool exists_in_sorted_list(const std::vector<T> &v, T value)
        {
            //v1
            //auto lower = std::lower_bound(v.begin(), v.end(), value);
            //const bool found = (lower != v.end()) && *lower == value; // check that value has been found
            //return found;
            //v2
            return binary_search(v.begin(), v.end(), value);
        }


        template <typename T>
        inline int find_index_in_sorted_list(const std::vector<T> &v, T value)
        {
            auto lower = std::lower_bound(v.begin(), v.end(), value);
            // check that value has been found
            const bool found = lower != v.end() && *lower == value;
            return found
                ? distance(v.begin(), lower)// convert iterator to index
                : -1;                                  // -1 in case we didnt find searched value
        }


        template <typename T>
        inline T max_element(const std::vector<T> &v)
        {
            return *std::max_element(std::begin(v), std::end(v));
        }
        template <typename T>
        inline T min_element(const std::vector<T> &v)
        {
            return *std::min_element(std::begin(v), std::end(v));
        }



        template <typename T>
        inline void remove_at(std::vector<T> &v, int index)
        {
            assert(index <= v.size() - 1);
            if (index <= v.size() - 1)
            {
                v.erase(v.begin() + index);
            }
        }

        template <typename T>
        inline void remove_at(std::vector<T> &v, int index, int length)
        {
            //http://www.cplusplus.com/reference/list/list/erase/
            assert(length == 0 || index <= v.size() - 1);
            if (length != 0 && index <= v.size() - 1)
            {
                v.erase(v.begin() + index, v.begin() + index + length);
            }
        }

        template <typename T>
        inline bool same(const std::vector<T> &v1, const std::vector<T> &v2)
        {
            if (v1.size() != v2.size())
            {
                return false;
            }
            for (int i = 0; i < v1.size(); i++)
            {
                if (v1[i] != v2[i]) return false;
            }
            return true;
        }


        template <typename T>
        inline void sort(std::vector<T> &v, bool from_low_to_high = true)
        {
            if (from_low_to_high)
                sort(v.begin(), v.end());
            else
                sort(v.rbegin(), v.rend());
        }


        template<class T>
        void sort_parallel(T* data, int len, int threadsCount)
        {
            // Use grainsize instead of thread count so that we don't e.g.
            // spawn 4 threads just to sort 8 elements.
            if (len < threadsCount)
            {
                std::sort(data, data + len, std::less<T>());
            }
            else
            {
                auto future = std::async(sort_parallel<T>, data, len / 2, threadsCount);

                // No need to spawn another thread just to block the calling 
                // thread which would do nothing.
                //v1 - which is correct? see for more info https://codereview.stackexchange.com/questions/22744/multi-threaded-sort
                parallel_sort(data + len / 2, len / 2, threadsCount);
                //v2 - which is correct? see for more info https://codereview.stackexchange.com/questions/22744/multi-threaded-sort
                //parallel_sort(data + len / 2, len - len / 2, threadsCount);

                future.wait();

                std::inplace_merge(data, data + len / 2, data + len, std::less<T>());
            }
        }

        inline std::vector<unsigned> sort_indexes_custom(size_t vector_size, std::function<bool(unsigned int, unsigned int)> predicate)
        {

            // initialize original index locations
            std::vector<unsigned> idx(vector_size);
            iota(idx.begin(), idx.end(), 0);

            // sort indexes based on comparing values in v - v1
            //sort(idx.begin(), idx.end(), [&v](unsigned int i1, unsigned int i2)
            //{
            //    return predicate(i1, i2);
            //});

            // sort indexes based on comparing values in v - v2 (same but faster)
            sort(idx.begin(), idx.end(), predicate);

            return idx;
        }

        inline void sort_indexes_custom(std::vector<unsigned>& idx, size_t vector_size, std::function<bool(unsigned int, unsigned int)> predicate)
        {
            // resize idx
            if (idx.size() < vector_size)
            {
                idx.resize(vector_size);
            }

            // initialize original index locations
            iota(idx.begin(), idx.begin() + vector_size, 0);

            // sort indexes based on comparing values in v - v2
            sort(idx.begin(), idx.begin() + vector_size, predicate);
        }


        template <typename T>
        inline std::vector<unsigned> sort_indexes(const std::vector<T> &v)
        {
            //TODO use method sort_indexes_custom
            // initialize original index locations
            std::vector<unsigned> idx(v.size());
            iota(idx.begin(), idx.end(), 0);

            // sort indexes based on comparing values in v
            sort(idx.begin(), idx.end(), [&v](unsigned i1, unsigned i2)
            {
                return v[i1] < v[i2];
            });

            return idx;
        }

        template <typename T>
        inline std::vector<unsigned> sort_indexes_desc(const std::vector<T> &v)
        {

            //TODO use method sort_indexes_custom
            // initialize original index locations
            std::vector<unsigned> idx(v.size());
            iota(idx.begin(), idx.end(), 0);

            // sort indexes based on comparing values in v
            sort(idx.begin(), idx.end(), [&](unsigned i1, unsigned i2)
            {
                return v[i1] > v[i2];
            });

            return idx;

        }



        //Selects some elements from vector defined by dondition 'selector'
        template <typename T>
        inline std::vector<T> where(const std::vector<T> &v, std::function<bool(const T&)> selector)
        {
            std::vector<T> res;
            res.reserve(v.size());
            for (const T& e : v)
            {
                if (selector(e))
                {
                    res.push_back(e);
                }
            }
            res.shrink_to_fit();
            return res;
        }

        //Selects some data from vector
        template <typename T, typename TRes>
        inline std::vector<TRes> select(const std::vector<T>& v, std::function<TRes(const T&)> selector)
        {
            std::vector<TRes> res;
            res.reserve(v.size());
            for (const T&e : v)
            {
                res.push_back(selector(e));
            }
            res.shrink_to_fit();
            return res;
        }

        template <class ForwardIterator>
        ForwardIterator unique(ForwardIterator first, ForwardIterator last)
        {
            if (first == last) return last;

            ForwardIterator result = first;
            while (++first != last)
            {
                if (!(*result == *first))  // or: if (!pred(*result,*first)) for version (2)
                    *(++result) = *first;
            }
            return ++result;
        }

        template <typename T>
        inline void removeDuplicates(std::vector<T> &v, bool vectorIsAlreadySorted)
        {
            if (!vectorIsAlreadySorted)
            {
                sort(v);
            }
            std::vector<int>::iterator it;
            it = std::unique(v.begin(), v.end());
            v.resize(std::distance(v.begin(), it));
        }

        template <typename T>
        inline T summ(const std::vector<T> &v)
        {
            T res = 0;
            for (const T& e : v)
                res += e;
            return res;
        }


        template <typename T, typename TRes>
        inline T summ(const std::vector<T> &v, std::function<TRes(const T&)> predicate)
        {
            std::vector<TRes> s(select(v, predicate));
            return summ(s);
        }

        template <typename T>
        inline void append(std::vector<T> &v, const std::vector<T> &addedV)
        {
            v.insert(v.end(), addedV.begin(), addedV.end());
        }
    }

}