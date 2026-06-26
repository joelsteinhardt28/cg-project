#pragma once

#include <integer-plane-geometry/plane.hh>

namespace ipg
{
template <class geometry_t>
tg::i8 classify(typename geometry_t::determinant_xxd_t const& x, //
                typename geometry_t::determinant_xxd_t const& y,
                typename geometry_t::determinant_xxd_t const& z,
                typename geometry_t::determinant_abc_t const& w,
                plane<geometry_t> const& s)
{
    // ld(3) = 2 summations plus maximal bits of multiplication.
    static constexpr int max_bits = 2 + geometry_t::bits_determinant_xxd + geometry_t::bits_normal;

    auto const d = (mul<max_bits>(x, s.a) + //
                    mul<max_bits>(y, s.b))
                   +                        //
                   (mul<max_bits>(z, s.c) + //
                    mul<max_bits>(w, s.d));

    return tg::sign(d) * tg::sign(w);
}

template <class geometry_t>
tg::i8 classify(point4<geometry_t> const& pt, plane<geometry_t> const& p)
{
    return classify(pt.x, pt.y, pt.z, pt.w, p);
}

template <class geometry_t>
tg::i8 classify(typename geometry_t::pos_t const& pt, plane<geometry_t> const& p)
{
    return tg::sign(signed_distance(p, pt));
}


/// classifies the bounding box relative to the plane
/// +1 -> completely on positive side
///  0 -> might intersect
/// -1 -> completely on negative side
/// (25 cycles)
template <class geometry_t>
tg::i8 classify(tg::iaabb3 const& bb, plane<geometry_t> const& pl)
{
    static_assert(geometry_t::bits_position <= 30, "only int pos allowed");
    // NOTE: all coordinates are multiplied by 2 so we can center properly
    static constexpr int bits = 1 + geometry_t::bits_plane_d;
    static_assert(bits <= 128, "should not be so high");

    CC_ASSERT(tg::abs(bb.min.x) <= (i64(1) << geometry_t::bits_position));
    CC_ASSERT(tg::abs(bb.min.y) <= (i64(1) << geometry_t::bits_position));
    CC_ASSERT(tg::abs(bb.min.z) <= (i64(1) << geometry_t::bits_position));

    CC_ASSERT(tg::abs(bb.max.x) <= (i64(1) << geometry_t::bits_position));
    CC_ASSERT(tg::abs(bb.max.y) <= (i64(1) << geometry_t::bits_position));
    CC_ASSERT(tg::abs(bb.max.z) <= (i64(1) << geometry_t::bits_position));

    auto const c = bb.min + bb.max;
    auto const s = bb.max - bb.min;

    auto d = pl.d << 1;
    d += mul<bits>(c.x, pl.a);
    d += mul<bits>(c.y, pl.b);
    d += mul<bits>(c.z, pl.c);

    auto const hn = mul<bits>(s.x, abs(pl.a)) + //
                    mul<bits>(s.y, abs(pl.b)) + //
                    mul<bits>(s.z, abs(pl.c));

    if (tg::detail::less_than_zero(hn + d))
        return -1;

    if (tg::detail::less_than_zero(hn - d))
        return 1;

    return 0;
}
}
