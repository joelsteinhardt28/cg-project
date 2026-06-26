#pragma once

#include <clean-core/assert.hh>

#include <typed-geometry/feature/fixed_int.hh>

#include <integer-plane-geometry/fwd.hh>
#include <integer-plane-geometry/geometry.hh>

namespace ipg
{
template <class GeometryT>
struct point4
{
    using geometry_t = GeometryT;
    using det_abc_t = typename geometry_t::determinant_abc_t;
    using det_xxd_t = typename geometry_t::determinant_xxd_t;

    det_xxd_t x;
    det_xxd_t y;
    det_xxd_t z;
    det_abc_t w;

    det_xxd_t& comp(int i) { return (&x)[i]; }
    det_xxd_t const& comp(int i) const { return (&x)[i]; }

    constexpr point4() = default;
    constexpr point4(tg::ipos3 p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
        w = 1;
    }

    bool is_valid() const { return !is_zero(w); }

    constexpr bool operator==(point4 const& rhs) const { return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w; }
    constexpr bool operator!=(point4 const& rhs) const { return x != rhs.x || y != rhs.y || z != rhs.z || w != rhs.w; }
};

template <class In, class GeometryT>
constexpr void introspect(In&& inspect, point4<GeometryT>& v)
{
    inspect(v.x, "x");
    inspect(v.y, "y");
    inspect(v.z, "z");
    inspect(v.w, "w");
}

// // pass-through to simplify generic code
inline tg::dpos3 to_dpos3(tg::dpos3 const& v) { return v; }
inline tg::dpos3 to_dpos3(tg::ipos3 const& v) { return tg::dpos3(v); }

template <class geometry_t>
tg::dpos3 to_dpos3(point4<geometry_t> const& pt)
{
    auto const x = double(pt.x);
    auto const y = double(pt.y);
    auto const z = double(pt.z);
    auto const w = double(pt.w);
    TG_ASSERT(!tg::is_zero(pt.w));
    TG_ASSERT(tg::is_finite(double(x / w)));
    TG_ASSERT(tg::is_finite(double(y / w)));
    TG_ASSERT(tg::is_finite(double(z / w)));
    return {x / w, y / w, z / w};
}

template <class geometry_t>
tg::pos3 to_pos3(point4<geometry_t> const& pt)
{
    auto const x = float(pt.x);
    auto const y = float(pt.y);
    auto const z = float(pt.z);
    auto const w = float(pt.w);
    TG_ASSERT(!tg::is_zero(pt.w));
    TG_ASSERT(tg::is_finite(float(x / w)));
    TG_ASSERT(tg::is_finite(float(y / w)));
    TG_ASSERT(tg::is_finite(float(z / w)));
    return {x / w, y / w, z / w};
}

template <class geometry_t>
tg::dpos3 to_dpos3_fast(point4<geometry_t> const& pt)
{
    auto const x = double(pt.x);
    auto const y = double(pt.y);
    auto const z = double(pt.z);
    auto const iw = 1 / double(pt.w);
    TG_ASSERT(!tg::is_zero(pt.w));
    TG_ASSERT(tg::is_finite(double(x * iw)));
    TG_ASSERT(tg::is_finite(double(y * iw)));
    TG_ASSERT(tg::is_finite(double(z * iw)));
    return {x * iw, y * iw, z * iw};
}

template <class geometry_t>
tg::pos3 to_pos3_fast(point4<geometry_t> const& pt)
{
    auto const x = float(pt.x);
    auto const y = float(pt.y);
    auto const z = float(pt.z);
    auto const iw = 1 / float(pt.w);
    TG_ASSERT(!tg::is_zero(pt.w));
    TG_ASSERT(tg::is_finite(float(x * iw)));
    TG_ASSERT(tg::is_finite(float(y * iw)));
    TG_ASSERT(tg::is_finite(float(z * iw)));
    return {x * iw, y * iw, z * iw};
}
}
