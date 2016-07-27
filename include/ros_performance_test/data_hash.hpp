#ifndef __DATA_HASH_HPP_INCLUDED__
#define __DATA_HASH_HPP_INCLUDED__

#include <numeric>
#include <iterator>

#include <boost/functional/hash.hpp>

template<typename Iterator>
uint32_t
data_hash_1(Iterator begin, Iterator end)
{
    uint32_t initial = 0;

    return (std::accumulate(begin, end, initial));
}

template<typename Iterator>
uint32_t
data_hash_2(Iterator begin, Iterator end)
{
    std::size_t h = boost::hash_range(begin, end);

    return (h % std::numeric_limits<uint32_t>::max());
}

#endif
