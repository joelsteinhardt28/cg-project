#pragma once

#include <integer-plane-geometry/integer_math.hh>

namespace ipg
{
template <class geometry_t>
struct line
{
    static constexpr int bits_plane_n = geometry_t::bits_normal;
    static constexpr int bits_plane_d = geometry_t::bits_plane_d;

    static constexpr int bits_nn = 2 * bits_plane_n + 1;
    static constexpr int bits_nd = bits_plane_n + bits_plane_d + 1;

    using nn_t = ipg::fixed_int<bits_nn>;
    using nd_t = ipg::fixed_int<bits_nd>;

    // a b c d  - plane 0
    // a b c d  - plane 1

    // NOTE: do not change order!
    nn_t ab_ba;
    nn_t bc_cb;
    nn_t ca_ac;

    nd_t ad_da;
    nd_t bd_db;
    nd_t cd_dc;

    bool is_valid() const { return ab_ba != 0 || bc_cb != 0 || ca_ac != 0; }

    tg::vec<3, nn_t> direction() const { return {bc_cb, ca_ac, ab_ba}; }
};
}
