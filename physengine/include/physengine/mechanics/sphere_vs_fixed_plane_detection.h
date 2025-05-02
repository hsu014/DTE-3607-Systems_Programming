#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "compute_trajectory.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const&       sphere_tc,        // 'position' within timestep
    [[maybe_unused]] types::Point3 const&                 sphere_p,
    [[maybe_unused]] types::ValueType                     sphere_r,
    [[maybe_unused]] types::Vector3 const&                sphere_v,
    [[maybe_unused]] types::Point3 const&                 fplane_q,
    [[maybe_unused]] types::Vector3 const&                fplane_n,
    [[maybe_unused]] types::Vector3 const&                external_forces,
    [[maybe_unused]] types::HighResolutionTP const&       t_0,              // timestep start
    [[maybe_unused]] types::Duration                      timestep,
    [[maybe_unused]] std::optional<types::Vector3> const& sphere_ds = std::nullopt)
  {

    types::Duration time = timestep - (sphere_tc - t_0);
    if (time < types::Duration(0)) {    // Negative time, sphere_tc is after timestep
      std::cout << "Negative time interval " << std::endl;
      return std::nullopt;
    }

    types::Vector3 n = blaze::normalize(fplane_n);
    types::Vector3 ds;
    types::Point3 s = fplane_q + sphere_r * n;
    types::Point3 d = s - sphere_p;

    if (sphere_ds.has_value()) {
      ds = sphere_ds.value();
    }
    else {
      ds = computeLinearTrajectory(sphere_v, external_forces, time).first;
    }

    auto q = blaze::inner(d, n);        // Q = q * n
    auto r = blaze::inner(ds, n);       // R = r * n;
    auto eps = 1e-12;

    if (std::abs(q) < eps) {
      // Sphere is touching the plane
      return std::nullopt;
    }
    if (std::abs(r) < eps) {
      // Sphere is moving parallel to the plane
      return std::nullopt;
    }

    auto x = q / r;
    if (x<=0 or x>1) {
      // Collision outside of timestep
      return std::nullopt;
    }

    types::Duration const t_col = utils::toDuration(x * time);
    return sphere_tc + t_col;
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
