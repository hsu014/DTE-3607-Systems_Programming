#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "compute_trajectory.h"
#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::HighResolutionTP> detectCollisionSphereSphere(
    [[maybe_unused]] types::HighResolutionTP const& s1_tc,
    [[maybe_unused]] types::Point3 const&           s1_p,
    [[maybe_unused]] types::ValueType               s1_r,
    [[maybe_unused]] types::Vector3 const&          s1_v,
    [[maybe_unused]] types::HighResolutionTP const& s2_tc,
    [[maybe_unused]] types::Point3 const&           s2_p,
    [[maybe_unused]] types::ValueType               s2_r,
    [[maybe_unused]] types::Vector3 const&          s2_v,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {

    if (s1_tc != s2_tc) {
      std::cout << "Spheres not synced in time, fix code! " << std::endl;
    }

    types::Duration time_1 = timestep - (s1_tc - t_0);
    types::Duration time_2 = timestep - (s2_tc - t_0);

    types::ValueType r = s1_r + s2_r;
    types::ValueType r_sqr = pow(r, 2);
    auto ds_1 = computeLinearTrajectory(s1_v, external_forces, time_1).first;
    auto ds_2 = computeLinearTrajectory(s2_v, external_forces, time_2).first;

    types::Vector3 Q = s2_p - s1_p;
    types::Vector3 R = ds_2 - ds_1;
    auto eps = 1e-5;

    auto QQ_inner = blaze::inner(Q, Q);
    auto RR_inner = blaze::inner(R, R);
    auto QR_inner = blaze::inner(Q, R);
    auto inside = pow(QR_inner, 2) - RR_inner * (QQ_inner - r_sqr); // Part inside square root


    if (QQ_inner - r_sqr < eps) {
      // Spheres are touching
      // std::cout << "Spheres are touching" << std::endl;
      return std::nullopt;
    }

    if (RR_inner < eps) {
      // Spheres are moving (almost) in parallell
      // std::cout << "Spheres are moving (almost) in parallell" << std::endl;

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

    types::Duration const t_col = utils::toDuration(x * time_1);
    return s1_tc + t_col;
  }


}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
