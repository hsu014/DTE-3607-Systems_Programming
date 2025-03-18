#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_2_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../mechanics/compute_trajectory.h"
#include <set>
#include <vector>
#include <algorithm>


namespace dte3607::physengine::solver_dev::level2_2
{

  struct Params {
    types::Vector3 const F;                 // External forces (environment)
    types::HighResolutionTP const t_0;
    types::Duration const dt;               // Timestep (system)
  };



  struct SphereGeomDataBlock {
    types::Point3 p;
    types::ValueType const r;
    types::Vector3 v;
    types::HighResolutionTP t_c;
  };
  using SphereGeomData = std::vector<SphereGeomDataBlock>;



  struct InfPlaneGeomDataBlock {
    types::Point3 const p;
    types::Vector3 const n;
  };
  using InfPlaneGeomData = std::vector<InfPlaneGeomDataBlock>;



  struct IntersectDetProcDataBlock {
    size_t sphere_id;                     // Sphere id
    size_t plane_id;                      // Inf plane data
    types::HighResolutionTP col_tp;       // Intersection time
  };
  using IntersectDetProcData = std::vector<IntersectDetProcDataBlock>;



  void detectCollisions(Params&               params,
                        IntersectDetProcData& intersections,
                        SphereGeomData&       spheres,
                        InfPlaneGeomData&     planes)
  {

    for (size_t s_id=0; s_id < spheres.size(); s_id++) {

      auto sphere = spheres[s_id];
      for (size_t p_id=0; p_id < planes.size(); p_id++) {

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
            s_id,
            p_id,
            tc.value()
          });
        }
      }
    }
  }



  void detectCollision(Params&               params,
                       IntersectDetProcData& intersections,
                       SphereGeomDataBlock&  sphere,
                       size_t                s_id,
                       InfPlaneGeomData&     planes,
                       std::set<size_t>&     exclude_plane_idx)
  {

    IntersectDetProcData collisions;

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
        collisions.push_back({
          s_id,
          p_id,
          tc.value()
        });
      }
    }

    if (collisions.empty()) return;       // No further collisions found

    // Use first possible collision
    std::ranges::sort(collisions, [](auto& collision1, auto& collision2){
      return collision1.col_tp < collision2.col_tp;
    });

    intersections.push_back(collisions[0]);

    // Resort intersections. NB! intersections is reversed from detectCollisions()
    std::ranges::sort(intersections, [](auto& intersection1, auto& intersection2){
      return intersection1.col_tp > intersection2.col_tp;
    });
  }



  void sortAndReduce([[maybe_unused]]IntersectDetProcData& intersections) {

    std::ranges::sort(intersections, [](auto& intersection1, auto& intersection2){
      return intersection1.col_tp < intersection2.col_tp;
    });

    std::set<size_t> visited;
    std::erase_if(intersections, [&visited](auto& intersection){
      if (visited.contains(intersection.sphere_id)) {
        return true;
      }
      else {
        visited.insert(intersection.sphere_id);
        return false;
      }
    });
  }



  void handleCollision(Params&                    params,
                       IntersectDetProcData&      collisions,
                       SphereGeomData&            spheres,
                       InfPlaneGeomData&          planes)
  {

    auto collision = collisions.back();
    collisions.pop_back();

    auto s_id = collision.sphere_id;
    auto p_id = collision.plane_id;

    //  Simulate Object
    auto ds = mechanics::computeLinearTrajectory(
                spheres[s_id].v,
                params.F,
                collision.col_tp - spheres[s_id].t_c
                ).first;

    // ImpactResponse
    auto new_v = mechanics::computeImpactResponseSphereFixedPlane(
      spheres[s_id].v,
      planes[p_id].n
      );

    // UpdateCacheProperties
    spheres[s_id].p += ds;
    spheres[s_id].v = new_v;
    spheres[s_id].t_c = collision.col_tp;

    // Detect further collision. Add to collisions
    std::set<size_t> exclude_plane_idx{p_id};
    detectCollision(
      params,
      collisions,
      spheres[s_id],
      s_id,
      planes,
      exclude_plane_idx);
  }



  void simulateObjects(Params&           params,
                       SphereGeomData&   spheres)
  {

    for (size_t i = 0; i<spheres.size(); i++) {

      types::Duration timestep = (params.t_0 + params.dt) - spheres[i].t_c;
      auto ds = mechanics::computeLinearTrajectory(
                  spheres[i].v,
                  params.F,
                  timestep
                  ).first;

      spheres[i].p += ds;
    }
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

    SphereGeomData spheres;
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
    detectCollisions(params, intersections, spheres, planes);

    sortAndReduce(intersections);

    // Reverse intersections. Pop first collision from end of vector
    std::reverse(intersections.begin(), intersections.end());

    while (!intersections.empty()) {
      handleCollision(params, intersections, spheres, planes);
    }

    simulateObjects(params, spheres);

    // Update scenario
    for (size_t i = 0; i<spheres.size(); i++) {
      size_t sphere_id = sphere_idx[i];

      scenario.setGlobalFramePosition(sphere_id, spheres[i].p);
    }
  }
}   // namespace dte3607::physengine::solver_dev::level2_2


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_2_H
