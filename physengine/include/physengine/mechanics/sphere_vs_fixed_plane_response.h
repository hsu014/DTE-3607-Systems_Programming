#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H

#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline types::Vector3 computeImpactResponseSphereFixedPlane(
    [[maybe_unused]] types::Vector3 const& sphere_v,
    [[maybe_unused]] types::Vector3 const& fplane_n)
  {

    auto n = blaze::normalize(fplane_n);

    return sphere_v - 2 * blaze::inner(sphere_v, n) * n;
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
