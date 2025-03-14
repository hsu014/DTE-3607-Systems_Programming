#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "compute_trajectory.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,        // 'position' within timestep
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_v,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,              // timestep start
    [[maybe_unused]] types::Duration                timestep)
  {

    types::Duration time = timestep - (sphere_tc - t_0);
    if (time < types::Duration(0)) {    // Negative time, sphere_tc is after timestep
      std::cout << "Negative time interval " << std::endl;
      return std::nullopt;
    }

    auto n = blaze::normalize(fplane_n);
    auto ds = computeLinearTrajectory(sphere_v, external_forces, time).first;
    auto s = fplane_q + sphere_r * n;
    auto d = s - sphere_p;

    auto q = blaze::inner(d, n);        // Q = q * n
    auto r = blaze::inner(ds, n);       // R = r * n;
    auto eps = 1e-5;

    if (std::abs(q) < eps) {            // Sphere is touching the plane
      std::cout << "Something wierd happened (Hit corner?) " << std::endl;
      // return t_0;
      return std::nullopt;

      // types::Duration offset = types::Duration(100);
      // auto offset_ds = computeLinearTrajectory(sphere_v, external_forces, offset).first;

      // return detectCollisionSphereFixedPlane(
      //   sphere_tc - offset,
      //   sphere_p - offset_ds,
      //   sphere_r,
      //   sphere_v,
      //   fplane_q,
      //   fplane_n,
      //   external_forces,
      //   t_0,
      //   timestep);


      //return std::nullopt;
    }
    if (std::abs(r) < eps) {            // Sphere is moving parallel to the plane
      std::cout << "Moving parallel to plane" << std::endl;
      return std::nullopt;
    }

    auto x = q / r;
    if (x<=0 or x>1) {                  // Collision outside of timestep
      return std::nullopt;
    }

    auto const tp = utils::toDuration(x * time);
    return t_0 + tp;
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
