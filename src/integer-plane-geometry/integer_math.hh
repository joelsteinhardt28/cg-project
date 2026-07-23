#pragma once

#include <typed-geometry/feature/fixed_int.hh>

namespace ipg
{
using i32 = tg::i32;
using i64 = tg::i64;
using i128 = tg::i128;
using i192 = tg::i192;
using i256 = tg::i256;

template <int bits>
struct fixed_int_t
{
    // re-route to supported types
    using type = typename fixed_int_t<bits <= 32 ? 32 : (bits + 63) / 64 * 64>::type;
};

template <>
struct fixed_int_t<32>
{
    using type = i32;
};

template <>
struct fixed_int_t<64>
{
    using type = i64;
};

template <>
struct fixed_int_t<128>
{
    using type = i128;
};

template <>
struct fixed_int_t<192>
{
    using type = i192;
};

template <>
struct fixed_int_t<256>
{
    using type = i256;
};

template <int bits>
using fixed_int = typename fixed_int_t<bits>::type;

template <int bits_out, class A, class B>
fixed_int<bits_out> mul(A a, B b)
{
    // round to next 64bit
    auto constexpr words_out = (bits_out + 63) / 64;

    if constexpr (words_out == 1)
        return a * b;
    else if constexpr (sizeof(a) <= 8 && sizeof(b) <= 8)
        return tg::detail::imul<words_out>(tg::i64(a), tg::i64(b));
    else if constexpr (sizeof(a) <= 8)
        return tg::detail::imul<words_out>(tg::i64(a), b);
    else if constexpr (sizeof(b) <= 8)
        return tg::detail::imul<words_out>(a, tg::i64(b));
    else
        return tg::detail::imul<words_out>(a, b);
}

template <int w>
tg::fixed_int<w> abs(tg::fixed_int<w> const& x)
{
    return tg::detail::less_than_zero(x) ? -x : x;
}
inline tg::i64 abs(tg::i64 x) { return x < 0 ? -x : x; }
}
