#pragma once

#include <integer-plane-geometry/line.hh>
#include <integer-plane-geometry/plane.hh>
#include <integer-plane-geometry/point.hh>

namespace ipg
{
/// generate any valid point on the plane
template <class geometry_t>
ipg::point4<geometry_t> any_point(ipg::plane<geometry_t> const& plane)
{
    ipg::point4<geometry_t> point;
    if (plane.a != 0) // intersection with x-axis
    {
        point.x = -plane.d;
        point.y = 0;
        point.z = 0;
        point.w = plane.a;
    }
    else if (plane.b != 0) // intersecion with y-axis
    {
        point.x = 0;
        point.y = -plane.d;
        point.z = 0;
        point.w = plane.b;
    }
    else // intersection with z-axis
    {
        point.x = 0;
        point.y = 0;
        point.z = -plane.d;
        point.w = plane.c;
    }
    return point;
}

/// generate any valid point on the line
template <class geometry_t>
ipg::point4<geometry_t> any_point(ipg::line<geometry_t> const& line)
{
    ipg::point4<geometry_t> p;
    if (line.bc_cb != 0) // x = 0
    {
        p.x = 0;
        p.y = line.cd_dc;
        p.z = -line.bd_db; //
        p.w = line.bc_cb;  //
    }
    if (line.ca_ac != 0) // y = 0
    {
        p.x = -line.cd_dc;
        p.y = 0;
        p.z = line.ad_da;
        p.w = line.ca_ac;
    }
    if (line.ab_ba != 0) // z = 0
    {
        p.x = line.bd_db;
        p.y = -line.ad_da;
        p.z = 0;
        p.w = line.ab_ba;
    }
    return p;
}
}
