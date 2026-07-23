#pragma once

#include <typed-geometry/tg.hh>

#include <integer-plane-geometry/fwd.hh>
#include <integer-plane-geometry/integer_math.hh>

namespace ipg
{
template <int bits_pos_t, int bits_normal_t>
struct geometry
{
    static inline constexpr int bits_position = bits_pos_t;
    static inline constexpr int bits_normal = bits_normal_t;
    static inline constexpr int bits_edge = (bits_normal - 1) / 2;
    static inline constexpr int bits_plane_d = bits_position + bits_normal + 2;
    static inline constexpr int bits_determinant_abc = 3 * bits_normal + 3;
    static inline constexpr int bits_determinant_xxd = 2 * bits_normal + bits_plane_d + 3;

    using pos_scalar_t = fixed_int<bits_position>;
    using pos_t = tg::pos<3, pos_scalar_t>;
    using vec_t = tg::vec<3, pos_scalar_t>;
    using normal_scalar_t = fixed_int<bits_normal>;
    using plane_d_t = fixed_int<bits_plane_d>;
    using point4_t = point4<geometry>;
    using determinant_abc_t = fixed_int<bits_determinant_abc>;
    using determinant_xxd_t = fixed_int<bits_determinant_xxd>;
    using plane_t = plane<geometry>;
    using aabb_t = tg::aabb<3, pos_scalar_t>;

    static tg::dpos3 to_dpos3(pos_scalar_t const& a, pos_scalar_t const& b, pos_scalar_t const& c) { return {double(a), double(b), double(c)}; }
};

using geometry256_x64_n45 = geometry<64, 45>;
using geometry128_x32_n21 = geometry<32, 21>;
using geometry256_x48_n49 = geometry<48, 49>;
using geometry256_x27_n55 = geometry<27, 55>;
using geometry256_x26_n53 = geometry<26, 53>;
using geometry192_x19_n39 = geometry<19, 39>;
}
