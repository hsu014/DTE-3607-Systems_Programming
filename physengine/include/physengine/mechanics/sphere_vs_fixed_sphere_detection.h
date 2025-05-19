#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_SPHERE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_SPHERE_DETECTION_H

#include "compute_trajectory.h"
#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::Vector3> detectCollisionSphereFixedSphere(
    [[maybe_unused]] types::Vector3 const&                s1_ds,
    [[maybe_unused]] types::Point3 const&                 s1_p,
    [[maybe_unused]] types::ValueType                     s1_r,
    [[maybe_unused]] types::Point3 const&                 s2_p,
    [[maybe_unused]] types::ValueType                     s2_r)
  {

    types::ValueType r = s1_r + s2_r;
    types::ValueType r_sqr = pow(r, 2);

    types::Vector3 Q = s2_p - s1_p;
    types::Vector3 R = - s1_ds;
    auto eps = 1e-12;

    auto QQ_inner = blaze::inner(Q, Q);
    auto RR_inner = blaze::inner(R, R);
    auto QR_inner = blaze::inner(Q, R);
    auto inside = pow(QR_inner, 2) - RR_inner * (QQ_inner - r_sqr); // Part inside square root


    if (QQ_inner - r_sqr < eps) {
      // Spheres are touching
      return std::nullopt;
    }

    if (RR_inner < eps) {
      // Spheres are moving (almost) in parallell
      return std::nullopt;
    }

    if (inside < 0) {
      // No collision
      return std::nullopt;
    }

    auto x = (-QR_inner - sqrt(pow(QR_inner, 2) - RR_inner * ( QQ_inner - r_sqr ))) / RR_inner;
    if (x<=0 or x>1) {
      // Collision outside of timestep
      return std::nullopt;
    }

    // types::Duration const t_col = utils::toDuration(x * time_1);
    // types::Vector3 ds = s1_ds * x;
    return s1_ds * x;   //s1_tc + t_col;
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_SPHERE_DETECTION_H
