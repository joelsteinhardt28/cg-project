#pragma once

#include <integer-plane-geometry/line.hh>
#include <integer-plane-geometry/plane.hh>

namespace ipg
{
/// returns true if the two planes are parallel (but not necessary at the same distance)
template <class geometry_t>
bool are_parallel(plane<geometry_t> const& p0, plane<geometry_t> const& p1)
{
    static constexpr int out_bits = 1 + 2 * geometry_t::bits_normal;

    // cross product
    auto const crossa = mul<out_bits>(p0.b, p1.c) - mul<out_bits>(p0.c, p1.b);
    auto const crossb = mul<out_bits>(p0.c, p1.a) - mul<out_bits>(p0.a, p1.c);
    auto const crossc = mul<out_bits>(p0.a, p1.b) - mul<out_bits>(p0.b, p1.a);

    // all zero
    return tg::is_zero(crossa) && tg::is_zero(crossb) && tg::is_zero(crossc);
}

template <class geometry_t>
bool are_parallel(plane<geometry_t> const& plane, line<geometry_t> const& line)
{
    static constexpr int max_bits = 2 + geometry_t::bits_normal + ipg::line<geometry_t>::bits_nn;
    // dot-product of plane normal and line direction
    auto const res = ipg::mul<max_bits>(plane.a, line.bc_cb) + //
                     ipg::mul<max_bits>(plane.b, line.ca_ac) + //
                     ipg::mul<max_bits>(plane.c, line.ab_ba);
    return tg::is_zero(res);
}
}
