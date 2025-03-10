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
    return {};
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
