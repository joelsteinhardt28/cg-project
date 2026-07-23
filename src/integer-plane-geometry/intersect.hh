#pragma once

#include <integer-plane-geometry/integer_math.hh>
#include <integer-plane-geometry/line.hh>
#include <integer-plane-geometry/plane.hh>
#include <integer-plane-geometry/point.hh>

namespace ipg
{
template <class geometry_t>
bool intersect(plane<geometry_t> const& p, //
               plane<geometry_t> const& q,
               plane<geometry_t> const& r,
               point4<geometry_t>& subs)
{
    auto constexpr bits_det_2x2_xx = geometry_t::bits_normal * 2;
    auto constexpr bits_det_2x2_xd = geometry_t::bits_normal + geometry_t::bits_plane_d;
    auto constexpr bits_det_abc = geometry_t::bits_determinant_abc;
    auto constexpr bits_det_xxd = geometry_t::bits_determinant_xxd;

    auto const det_2x2_ab = mul<bits_det_2x2_xx>(p.a, q.b) - mul<bits_det_2x2_xx>(p.b, q.a);
    auto const det_2x2_ac = mul<bits_det_2x2_xx>(p.a, q.c) - mul<bits_det_2x2_xx>(p.c, q.a);
    auto const det_2x2_ad = mul<bits_det_2x2_xd>(p.a, q.d) - mul<bits_det_2x2_xd>(p.d, q.a);
    auto const det_2x2_bc = mul<bits_det_2x2_xx>(p.b, q.c) - mul<bits_det_2x2_xx>(p.c, q.b);
    auto const det_2x2_bd = mul<bits_det_2x2_xd>(p.b, q.d) - mul<bits_det_2x2_xd>(p.d, q.b);
    auto const det_2x2_cd = mul<bits_det_2x2_xd>(p.c, q.d) - mul<bits_det_2x2_xd>(p.d, q.c);

    auto const det_abc = mul<bits_det_abc>(det_2x2_ab, r.c) - //
                         mul<bits_det_abc>(det_2x2_ac, r.b) + //
                         mul<bits_det_abc>(det_2x2_bc, r.a);

    auto const det_abd = mul<bits_det_xxd>(det_2x2_ad, r.b) - //
                         mul<bits_det_xxd>(det_2x2_ab, r.d) - //
                         mul<bits_det_xxd>(det_2x2_bd, r.a);

    auto const det_acd = mul<bits_det_xxd>(det_2x2_ac, r.d) - //
                         mul<bits_det_xxd>(det_2x2_ad, r.c) + //
                         mul<bits_det_xxd>(det_2x2_cd, r.a);

    auto const det_bcd = mul<bits_det_xxd>(det_2x2_bd, r.c) - //
                         mul<bits_det_xxd>(det_2x2_cd, r.b) - //
                         mul<bits_det_xxd>(det_2x2_bc, r.d);

    subs.x = det_bcd;
    subs.y = det_acd;
    subs.z = det_abd;
    subs.w = det_abc;

    return !is_zero(det_abc);
}

template <class geometry_t>
point4<geometry_t> intersect(plane<geometry_t> const& p, //
                             plane<geometry_t> const& q,
                             plane<geometry_t> const& r)
{
    point4<geometry_t> pt;
    intersect(p, q, r, pt);
    return pt;
}

template <class geometry_t>
line<geometry_t> intersect(plane<geometry_t> const& pl0, plane<geometry_t> const& pl1)
{
    auto constexpr bits_nn = line<geometry_t>::bits_nn;
    auto constexpr bits_nd = line<geometry_t>::bits_nd;

    line<geometry_t> l;

    l.bc_cb = mul<bits_nn>(pl0.b, pl1.c) - mul<bits_nn>(pl0.c, pl1.b); // cross product x
    l.ca_ac = mul<bits_nn>(pl0.c, pl1.a) - mul<bits_nn>(pl0.a, pl1.c); // cross product y
    l.ab_ba = mul<bits_nn>(pl0.a, pl1.b) - mul<bits_nn>(pl0.b, pl1.a); // cross product z

    l.ad_da = mul<bits_nd>(pl0.a, pl1.d) - mul<bits_nd>(pl0.d, pl1.a);
    l.bd_db = mul<bits_nd>(pl0.b, pl1.d) - mul<bits_nd>(pl0.d, pl1.b);
    l.cd_dc = mul<bits_nd>(pl0.c, pl1.d) - mul<bits_nd>(pl0.d, pl1.c);

    return l;
}

template <class geometry_t>
void intersect(line<geometry_t> const& l, plane<geometry_t> const& p, point4<geometry_t>& r)
{
    auto constexpr bits_xxd = geometry_t::bits_determinant_xxd;
    auto constexpr bits_abc = geometry_t::bits_determinant_abc;

    r.x = mul<bits_xxd>(p.c, l.bd_db) - //
          mul<bits_xxd>(p.b, l.cd_dc) - //
          mul<bits_xxd>(p.d, l.bc_cb);

    r.y = mul<bits_xxd>(p.a, l.cd_dc) - //
          mul<bits_xxd>(p.c, l.ad_da) - //
          mul<bits_xxd>(p.d, l.ca_ac);

    r.z = mul<bits_xxd>(p.b, l.ad_da) - //
          mul<bits_xxd>(p.a, l.bd_db) - //
          mul<bits_xxd>(p.d, l.ab_ba);

    r.w = mul<bits_abc>(p.a, l.bc_cb) + //
          mul<bits_abc>(p.b, l.ca_ac) + //
          mul<bits_abc>(p.c, l.ab_ba);
}

template <class geometry_t>
point4<geometry_t> intersect(line<geometry_t> const& l, plane<geometry_t> const& p)
{
    point4<geometry_t> r;
    intersect(l, p, r);
    return r;
}
}
