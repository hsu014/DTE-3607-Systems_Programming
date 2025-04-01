#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_3_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_3_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../mechanics/sphere_vs_sphere_detection.h"
#include "../mechanics/sphere_vs_sphere_response.h"
#include "../mechanics/compute_trajectory.h"
#include <set>
#include <vector>
#include <algorithm>


namespace dte3607::physengine::solver_dev::level2_3
{

  struct Params {
    types::Vector3 const          F;                // External forces (environment)
    types::HighResolutionTP const t_0;
    types::Duration const         dt;               // Timestep (system)
  };



  struct SphereGeomDataBlock {
    types::Point3                 p;
    types::ValueType const        r;
    types::Vector3                v;
    types::HighResolutionTP       t_c;
  };
  using SphereGeomData = std::vector<SphereGeomDataBlock>;



  struct InfPlaneGeomDataBlock {
    types::Point3 const           p;
    types::Vector3 const          n;
  };
  using InfPlaneGeomData = std::vector<InfPlaneGeomDataBlock>;



  struct IntersectDetProcDataBlock {
    size_t                        sphere1_id;       // Sphere 1 id
    std::optional<size_t>         sphere2_id;       // Sphere 2 id
    std::optional<size_t>         plane_id;         // Inf plane id
    types::HighResolutionTP       col_tp;           // Intersection time
  };
  using IntersectDetProcData = std::vector<IntersectDetProcDataBlock>;



  void detectCollision(Params&               params,
                       IntersectDetProcData& intersections,
                       SphereGeomData&       spheres,
                       InfPlaneGeomData&     planes,
                       size_t                cur_s_id,
                       std::set<size_t>&     exclude_sphere_idx,
                       std::set<size_t>&     exclude_plane_idx)
  {

    IntersectDetProcData collisions;
    auto const sphere = spheres[cur_s_id];

    // Collision with other spheres
    for (size_t s_id=0; s_id < spheres.size(); s_id++ ) {
      if (exclude_sphere_idx.contains(s_id)) continue;

      auto const sphere2 = spheres[s_id];

      types::HighResolutionTP t2 = sphere.t_c;
      types::HighResolutionTP t1 = sphere2.t_c;
      types::HighResolutionTP t_min = std::max(t1, t2);   // Starting timestamp to detect collision between spheres.

      // 'Move' spheres forward until tc = t_min. Temporary, not actual simulation
      auto ds_1 = mechanics::computeLinearTrajectory(
                    sphere.v,
                    params.F,
                    t_min - sphere.t_c).first;
      auto new_s1_p = sphere.p + ds_1;

      auto ds_2 = mechanics::computeLinearTrajectory(
                    sphere2.v,
                    params.F,
                    t_min - sphere2.t_c).first;
      auto new_s2_p = sphere2.p + ds_2;

      auto tc = mechanics::detectCollisionSphereSphere(
        t_min,
        new_s1_p,
        sphere.r,
        sphere.v,
        t_min,
        new_s2_p,
        sphere2.r,
        sphere2.v,
        params.F,
        params.t_0,
        params.dt
        );

      if (tc.has_value()) {               // Add possible collision
        intersections.push_back({
          cur_s_id,
          s_id,
          std::nullopt,
          tc.value()
        });
      }
    }

    // Collision with other planes
    for (size_t p_id=0; p_id < planes.size(); p_id++) {
      if (exclude_plane_idx.contains(p_id)) continue;

      auto plane = planes[p_id];
      auto tc = mechanics::detectCollisionSphereFixedPlane(
        sphere.t_c,
        sphere.p,
        sphere.r,
        sphere.v,
        plane.p,
        plane.n,
        params.F,
        params.t_0,
        params.dt
        );

      if (tc.has_value()) {               // Add possible collision
        intersections.push_back({
          cur_s_id,
          std::nullopt,
          p_id,
          tc.value()
        });
      }
    }
  }



  void detectAllCollisions(Params&               params,
                           IntersectDetProcData& intersections,
                           SphereGeomData&       spheres,
                           InfPlaneGeomData&     planes)
  {

    for (size_t s_id=0; s_id < spheres.size(); s_id++) {
      std::set<size_t> exclude_sphere_idx{s_id};
      std::set<size_t> exclude_plane_idx;

      detectCollision(params,
                      intersections,
                      spheres,
                      planes,
                      s_id,
                      exclude_sphere_idx,
                      exclude_plane_idx);

    }
  }



  void sortAndReduce(IntersectDetProcData& intersections) {

    std::ranges::sort(intersections, [](auto& intersection1, auto& intersection2){
      return intersection1.col_tp < intersection2.col_tp;
    });

    std::set<size_t> visited;
    std::erase_if(intersections, [&visited](auto& intersection){
      if (visited.contains(intersection.sphere1_id)) {
        return true;
      }
      else if (intersection.sphere2_id.has_value() && visited.contains(intersection.sphere2_id.value())) {
        return true;
      }
      else {
        visited.insert(intersection.sphere1_id);
        if (intersection.sphere2_id.has_value()) {
          visited.insert(intersection.sphere2_id.value());
        }
        return false;
      }
    });

    // Reverse intersections. Pop first collision from end of vector
    std::reverse(intersections.begin(), intersections.end());
  }



  void handleCollision(Params&               params,
                       IntersectDetProcData& intersections,
                       SphereGeomData&       spheres,
                       InfPlaneGeomData&     planes)
  {

    auto collision = intersections.back();
    intersections.pop_back();

    size_t                        s1_id = collision.sphere1_id;
    std::optional<size_t>         s2_id = collision.sphere2_id;
    std::optional<size_t>         p_id = collision.plane_id;

    types::Vector3                ds1;
    std::optional<types::Vector3> ds2;
    types::Vector3                new_v1;
    std::optional<types::Vector3> new_v2;

    std::set<size_t>              exclude_sphere_idx{s1_id};
    std::set<size_t>              exclude_plane_idx{};

    bool                          two_sphere_col = s2_id.has_value();

    //  Simulate Object
    ds1 = mechanics::computeLinearTrajectory(
            spheres[s1_id].v,
            params.F,
            collision.col_tp - spheres[s1_id].t_c
            ).first;

    if (two_sphere_col) {
      ds2 = mechanics::computeLinearTrajectory(
              spheres[s2_id.value()].v,
              params.F,
              collision.col_tp - spheres[s2_id.value()].t_c
              ).first;
    }

    // ImpactResponse
    if (two_sphere_col) {
      auto s1 = spheres[s1_id];
      auto s2 = spheres[s2_id.value()];
      auto [v1, v2] = mechanics::computeImpactResponseSphereSphere(
        s1.p + ds1,
        s1.v,
        1.0,
        s2.p + ds2.value(),
        s2.v,
        1.0);

      new_v1 = v1;
      new_v2 = v2;
    }
    else {
      new_v1 = mechanics::computeImpactResponseSphereFixedPlane(
        spheres[s1_id].v,
        planes[p_id.value()].n
        );
    }

    // UpdateCacheProperties
    spheres[s1_id].p += ds1;
    spheres[s1_id].v = new_v1;
    spheres[s1_id].t_c = collision.col_tp;

    if (two_sphere_col) {
      spheres[s2_id.value()].p += ds2.value();
      spheres[s2_id.value()].v = new_v2.value();
      spheres[s2_id.value()].t_c = collision.col_tp;
      exclude_sphere_idx.insert(s2_id.value());
    }
    else {
      exclude_plane_idx.insert(p_id.value());
    }

    // Detect further collision. Add to collisions
    detectCollision(
      params,
      intersections,
      spheres,
      planes,
      s1_id,
      exclude_sphere_idx,
      exclude_plane_idx);

    if (two_sphere_col) {
      detectCollision(
        params,
        intersections,
        spheres,
        planes,
        s2_id.value(),
        exclude_sphere_idx,
        exclude_plane_idx);
    }
  }



  void simulateObjects(Params&         params,
                       SphereGeomData& spheres)
  {

    for (size_t i = 0; i<spheres.size(); i++) {

      types::Duration timestep = (params.t_0 + params.dt) - spheres[i].t_c;
      // types::Duration timestep = params.dt - (spheres[i].t_c - params.t_0);

      auto ds = mechanics::computeLinearTrajectory(
                  spheres[i].v,
                  params.F,
                  timestep
                  ).first;

      spheres[i].p += ds;
    }
  }



  std::set<size_t> activeSpheres(IntersectDetProcData& intersections){

    std::set<size_t> active;
    for (auto col : intersections) {
      active.insert(col.sphere1_id);
      if (col.sphere2_id.has_value()){
        active.insert(col.sphere2_id.value());
      }
    }

    return active;
  }



  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve(Fixture_T&         scenario,
             types::NanoSeconds timestep)
  {

    Params params = {
      scenario.getGravity(),
      types::HighResolutionClock::now(),
      timestep
    };

    SphereGeomData   spheres;
    InfPlaneGeomData planes;

    auto sphere_idx = scenario.nonFixedSphereRBs();
    auto plane_idx = scenario.fixedInfPlaneRBs();

    // Spheres
    for (size_t i = 0; i < sphere_idx.size(); i++) {

      size_t idx = sphere_idx[i];
      spheres.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbSphereRadius(idx),
        scenario.rbSphereVelocity(idx),
        params.t_0
      });
    }

    // Planes
    for (size_t i = 0; i < plane_idx.size(); i++) {

      size_t idx = plane_idx[i];
      planes.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbPlaneNormal(idx)
      });
    }

    IntersectDetProcData intersections;

    detectAllCollisions(params, intersections, spheres, planes);
    sortAndReduce(intersections);

    while (!intersections.empty()) {
      handleCollision(params, intersections, spheres, planes);

      std::set<size_t> active_before = activeSpheres(intersections);

      sortAndReduce(intersections);

      std::set<size_t> active_after = activeSpheres(intersections);

      bool new_collisions_added = false;
      // New collision for spheres in active_before, but not in active_after
      for (auto id : active_before){
        if (!active_after.contains(id)){
          new_collisions_added = true;
          std::set<size_t> exclude_sphere_idx{id};
          std::set<size_t> exclude_plane_idx{};

          detectCollision(
            params,
            intersections,
            spheres,
            planes,
            id,
            exclude_sphere_idx,
            exclude_plane_idx);
        }
      }
      if (new_collisions_added) sortAndReduce(intersections);
    }

    simulateObjects(params, spheres);

    // Update scenario
    for (size_t i = 0; i<spheres.size(); i++) {
      size_t sphere_id = sphere_idx[i];

      scenario.setGlobalFramePosition(sphere_id, spheres[i].p);
    }
  }
}   // namespace dte3607::physengine::solver_dev::level2_3


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_3_H
// [](){}
