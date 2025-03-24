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
      size_t                      sphere1_id;       // Sphere 1 id
      std::optional<size_t>       sphere2_id;       // Sphere 2 id
      std::optional<size_t>       plane_id;         // Inf plane data
      types::HighResolutionTP     col_tp;           // Intersection time
    };
  using IntersectDetProcData = std::vector<IntersectDetProcDataBlock>;



  void printIntersections(Params& params, IntersectDetProcData& intersections) {
    // ~~Print content of intersections:~~
    std::set<size_t> active;

    for (auto col : intersections) {
      std::cout << "S1: " << col.sphere1_id;
      active.insert(col.sphere1_id);
      if (col.sphere2_id.has_value()) {
        std::cout << ", S2: " << col.sphere2_id.value();
        active.insert(col.sphere2_id.value());
      }
      else {
        std::cout << ", Pl: " << col.plane_id.value();
      }
      std::cout << ", T: " << col.col_tp - params.t_0 << std::endl;
    }
    std::cout << "Active spheres: ";
    for (auto i : active) std::cout << i << " ";
    std::cout << std::endl << std::endl;
  }



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

    // For sphere. Vector of current spheres in the future?

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
        // collisions.push_back({
        //   cur_s_id,
        //   s_id,
        //   std::nullopt,
        //   tc.value()
        // });

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
        // collisions.push_back({
        //   cur_s_id,
        //   std::nullopt,
        //   p_id,
        //   tc.value()
        // });

        intersections.push_back({
          cur_s_id,
          std::nullopt,
          p_id,
          tc.value()
        });
      }
    }

    // std::cout << "Possible collisions:" << std::endl;
    // printIntersections(params, collisions);

    // if (collisions.empty()) return;

    // // Use first possible collision
    // std::ranges::sort(collisions, [](auto& collision1, auto& collision2){
    //   return collision1.col_tp < collision2.col_tp;
    // });

    // intersections.push_back(collisions.front());
  }



  void detectCollisions(Params&               params,
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



  void sortAndReduce(IntersectDetProcData& intersections,
                     std::set<size_t>& new_collisions) {

    std::ranges::sort(intersections, [](auto& intersection1, auto& intersection2){
      return intersection1.col_tp < intersection2.col_tp;
    });

    std::set<size_t> visited;
    // std::set<size_t> new_collisions;
    /*
     * Add check for second sphere id
     * Done?
     */
    std::erase_if(intersections, [&visited, &new_collisions](auto& intersection){
      if (visited.contains(intersection.sphere1_id)) {
        return true;
      }
      else if (intersection.sphere2_id.has_value() && visited.contains(intersection.sphere2_id.value())) {
        if (!visited.contains(intersection.sphere1_id)) {
          new_collisions.insert(intersection.sphere1_id);
        }
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

    // auto& sp1 = spheres[s1_id].p;
    // if ((std::abs(sp1[0]) > 9) || (std::abs(sp1[1]) > 9) || (std::abs(sp1[2]) > 9) ){
    //   std::cout << "Sphere " << s1_id+planes.size() << " outside planes (Before collision)" << std::endl;
    //   std::cout << "Pos: " << sp1[0] << ", " << sp1[1] << ", " << sp1[2] << std::endl;
    // }

    //  Simulate Object
    ds1 = mechanics::computeLinearTrajectory(
            spheres[s1_id].v,
            params.F,
            collision.col_tp - spheres[s1_id].t_c
            ).first;

    // spheres[s1_id].p += ds1;

    // types::Duration dt1 = collision.col_tp - spheres[s1_id].t_c;
    // if (utils::toDtScalar(dt1) < 0) std::cout << "dt1: " << dt1 << std::endl;

    if (two_sphere_col) {
      ds2 = mechanics::computeLinearTrajectory(
              spheres[s2_id.value()].v,
              params.F,
              collision.col_tp - spheres[s2_id.value()].t_c
              ).first;

      // spheres[s2_id.value()].p += ds2.value();

      // std::cout << "Ball vs ball collision" << std::endl;
      // types::Duration dt2 = collision.col_tp - spheres[s2_id.value()].t_c;
      // if (utils::toDtScalar(dt2) < 0) std::cout << "dt2: " << dt2 << std::endl;
    }
    // else {
    //   ds2 = std::nullopt;      // Not needed!
    //   // std::cout << "Ball vs plane collision" << std::endl;
    // }

    // ImpactResponse
    if (two_sphere_col) {   // Ball vs ball collision
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
    else {                    // Ball vs fixed plane collision
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

    std::set<size_t> temp{};
    sortAndReduce(intersections, temp);

    int counter = 0;
    while (!intersections.empty()) {
      handleCollision(params, intersections, spheres, planes);

      // detectCollisions(params, intersections, spheres, planes); // testing

      std::set<size_t> new_collisions{};
      sortAndReduce(intersections, new_collisions);

      // Test new collisions:
      if (!new_collisions.empty()) {
        for (size_t id : new_collisions) {
          std::cout << "New collision: " << id << std::endl;
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
        sortAndReduce(intersections, temp); // Only sort?
      }

      std::cout << "After handleCollisions() number " << ++counter << std::endl;
      // printIntersections(params, intersections);


      // ~~Print position if sphere is outside of planes~~
      // for (size_t i=0; i < spheres.size(); i++ ) {
      //   auto sp = spheres[i].p;
      //   auto lim = 10 - spheres[i].r;
      //   if (std::abs(sp[0])>lim || std::abs(sp[1])>lim || std::abs(sp[1])>lim ) {
      //     std::cout << std::endl;
      //     std::cout << "After collision: " << std::endl;
      //     std::cout << "Sphere " << i /*+planes.size()*/ << " outside plane" << std::endl;
      //     std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] << std::endl;
      //   }
      // }

    }

    // std::cout << "After:" << std::endl;
    // detectCollisions(params, intersections, spheres, planes); // Remaining collisions after handleCollision done

    // for (size_t i=0; i < intersections.size(); i++ ) {
    //   std::cout << "Collision for sphere " << intersections[i].sphere1_id/* + planes.size()*/ << std::endl;
    //   if (intersections[i].sphere2_id.has_value()) {
    //     std::cout << "Collision* for sphere " << intersections[i].sphere2_id.value()/* + planes.size()*/ << std::endl;
    //   }
    // }

    // printIntersections(params, intersections);

    simulateObjects(params, spheres);

    // std::cout << std::endl;
    // std::cout << "After simulateObjects " << std::endl;
    // for (size_t i=0; i < spheres.size(); i++ ) {
    //   auto sp = spheres[i].p;
    //   if (std::abs(sp[0])>9 || std::abs(sp[1])>9 || std::abs(sp[1])>9 ) {
    //     std::cout << "Sphere " << i /*+planes.size()*/ << " outside plane" << std::endl;
    //     std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] << std::endl;
    //   }
    // }
    // std::cout << std::endl;

    // Update scenario
    for (size_t i = 0; i<spheres.size(); i++) {
      size_t sphere_id = sphere_idx[i];

      scenario.setGlobalFramePosition(sphere_id, spheres[i].p);
    }
  }
}   // namespace dte3607::physengine::solver_dev::level2_3


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_3_H
// [](){}
