#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H

#include "../bits/types.h"

// stl
#include <utility>

namespace dte3607::physengine::mechanics
{

  inline
    std::pair<types::Vector3, types::Vector3>
    computeImpactResponseSphereSphere(
      [[maybe_unused]] types::Point3 const&  s1_p,
      [[maybe_unused]] types::Vector3 const& s1_v,
      [[maybe_unused]] types::ValueType      s1_mass,
      [[maybe_unused]] types::Point3 const&  s2_p,
      [[maybe_unused]] types::Vector3 const& s2_v,
      [[maybe_unused]] types::ValueType      s2_mass)
  {

    types::Vector3 d = blaze::normalize(s2_p - s1_p);

    types::ValueType v1_d = blaze::inner(s1_v, d);
    types::ValueType v2_d = blaze::inner(s2_v, d);

    types::Vector3 v1_n = s1_v - v1_d * d;
    types::Vector3 v2_n = s2_v - v2_d * d;

    types::ValueType m = s1_mass + s2_mass;

    types::ValueType new_v1_d =
      (s1_mass - s2_mass) / m * v1_d + 2 * s2_mass / m * v2_d;

    types::ValueType new_v2_d =
      (s2_mass - s1_mass) / m * v2_d + 2 * s1_mass / m * v1_d;

    types::Vector3 new_v1 = v1_n + new_v1_d * d;
    types::Vector3 new_v2 = v2_n + new_v2_d * d;

    return {new_v1, new_v2};
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
