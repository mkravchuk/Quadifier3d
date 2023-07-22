#pragma once


namespace utils
{
    namespace num
    {
        __forceinline D sqrtsign(D value)
        {
            return (value < 0 ? -sqrt(-value) : sqrt(value));
        }
        D  sqrtFast(D a);
        D round(D d);

        template<typename T>
        void update_minimum(std::atomic<T>& minimum_value, T const& value) noexcept
        {
            T prev_value = minimum_value;
            while (prev_value > value &&
                !minimum_value.compare_exchange_strong(prev_value, value))
                ;
        };

        template<typename T>
        void update_maximum(std::atomic<T>& maximum_value, T const& value) noexcept
        {
            T prev_value = maximum_value;
            while (prev_value < value &&
                !maximum_value.compare_exchange_weak(prev_value, value))
                ;
        };
    }

}

