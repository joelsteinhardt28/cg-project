#pragma once

#include <clean-core/array.hh>

#include <integer-plane-geometry/geometry.hh>

#include <typed-geometry/tg-lean.hh>

namespace ipg
{
/// a plane defined by its plane equation:
/// ax + by + cz + d = 0
template <class GeometryT>
struct plane
{
    using geometry_t = GeometryT;
    using normal_scalar_t = typename geometry_t::normal_scalar_t;
    using distance_t = typename geometry_t::plane_d_t;
    using pos_t = typename geometry_t::pos_t;
    using vec_t = tg::vec<3, typename geometry_t::pos_scalar_t>;

    normal_scalar_t a, b, c;
    distance_t d; // is plane equation d

    tg::vec<3, normal_scalar_t> normal() const { return {a, b, c}; }

    normal_scalar_t normal_comp(int axis) const { return (&a)[axis]; }
    normal_scalar_t& normal_comp(int axis) { return (&a)[axis]; }

    void translate(vec_t v)
    {
        d = d - mul<8 * sizeof(distance_t)>(a, v.x);
        d = d - mul<8 * sizeof(distance_t)>(b, v.y);
        d = d - mul<8 * sizeof(distance_t)>(c, v.z);
    }

    void compute_d_for(pos_t p)
    {
        d = mul<8 * sizeof(distance_t)>(-a, p.x) + //
            mul<8 * sizeof(distance_t)>(-b, p.y) + //
            mul<8 * sizeof(distance_t)>(-c, p.z);
    }

    plane inverted() const { return {-a, -b, -c, -d}; }

    static plane from_pos_normal(pos_t p, tg::vec<3, normal_scalar_t> n)
    {
        TG_ASSERT(tg::abs(n.x) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.y) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.z) <= (i64(1) << geometry_t::bits_normal));

        auto const d = mul<8 * sizeof(distance_t)>(-n.x, p.x) + //
                       mul<8 * sizeof(distance_t)>(-n.y, p.y) + //
                       mul<8 * sizeof(distance_t)>(-n.z, p.z);

        return {n.x, n.y, n.z, d};
    }

    static plane from_points(pos_t p0, pos_t p1, pos_t p2)
    {
        using hpos_t = tg::pos<3, normal_scalar_t>;

        // higher precision needed as the cross product can go up to 64 bit
        auto n = cross(hpos_t(p1) - hpos_t(p0), hpos_t(p2) - hpos_t(p0));

        TG_ASSERT(n != (tg::vec<3, normal_scalar_t>::zero));

        auto const f = tg::gcd(tg::gcd(tg::abs(n.x), tg::abs(n.y)), tg::abs(n.z));

        if (f > 1)
            n /= f;

        // these assertions only work as long as the normal is less than 64 bit (for now)
        TG_ASSERT(tg::abs(n.x) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.y) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.z) <= (i64(1) << geometry_t::bits_normal));

        // -dot(n, p0);
        auto const d = mul<8 * sizeof(distance_t)>(-n.x, p0.x) + //
                       mul<8 * sizeof(distance_t)>(-n.y, p0.y) + //
                       mul<8 * sizeof(distance_t)>(-n.z, p0.z);

        return {n.x, n.y, n.z, d};
    }

    static plane from_points_no_gcd(pos_t p0, pos_t p1, pos_t p2)
    {
        using hpos_t = tg::pos<3, normal_scalar_t>;

        // higher precision needed as the cross product can go up to 64 bit
        auto n = cross(hpos_t(p1) - hpos_t(p0), hpos_t(p2) - hpos_t(p0));

        // these assertions only work as long as the normal is less than 64 bit (for now)
        TG_ASSERT(tg::abs(n.x) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.y) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.z) <= (i64(1) << geometry_t::bits_normal));

        // -dot(n, p0);
        auto const d = mul<8 * sizeof(distance_t)>(-n.x, p0.x) + //
                       mul<8 * sizeof(distance_t)>(-n.y, p0.y) + //
                       mul<8 * sizeof(distance_t)>(-n.z, p0.z);

        return {n.x, n.y, n.z, d};
    }

    tg::dplane3 to_dplane() const
    {
        auto const il = 1. / length(tg::dvec3(a, b, c));
        return {{a * il, b * il, c * il}, -double(d) * il};
    }

    bool is_valid() const { return !tg::is_zero(a) || !tg::is_zero(b) || !tg::is_zero(c); }

    bool constexpr operator==(plane const& rhs) const { return a == rhs.a && b == rhs.b && c == rhs.c && d == rhs.d; }
    bool constexpr operator!=(plane const& rhs) const { return !operator==(rhs); }
};

template <class In, class GeometryT>
constexpr void introspect(In&& inspect, plane<GeometryT>& v)
{
    inspect(v.a, "a");
    inspect(v.b, "b");
    inspect(v.c, "c");
    inspect(v.d, "d");
}

/// positive in normal direction.
template <class geometry_t>
auto signed_distance(plane<geometry_t> const& plane, typename geometry_t::pos_t const& point)
{
    // dot of normal and point plus d
    return ipg::mul<geometry_t::bits_plane_d>(plane.a, point.x) + //
           ipg::mul<geometry_t::bits_plane_d>(plane.b, point.y) + //
           ipg::mul<geometry_t::bits_plane_d>(plane.c, point.z) + // bits_determinant_xxd
           plane.d;
}
}
