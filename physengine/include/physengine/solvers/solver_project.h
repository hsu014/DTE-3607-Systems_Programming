#ifndef DTE3607_PHYSENGINE_SOLVER_PROJECT_H
#define DTE3607_PHYSENGINE_SOLVER_PROJECT_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../utils/type_conversion.h"
#include "../mechanics/sphere_vs_fixed_sphere_detection.h"

namespace dte3607::physengine::solver_dev::project
{

  bool loopPath = true;

  struct Params {
    types::Duration const         dt;               // Timestep (system)
    types::ValueType const        v_max;            // Allowed movement speed
    std::vector<types::Point3>    path;
  };



  struct SphereGeomDataBlock {
    types::Point3                 p;
    types::ValueType const        r;
    size_t                        next_goal;
    bool                          raised;
  };
  using SphereGeomData = std::vector<SphereGeomDataBlock>;



  struct StaticSphereGeomDataBlock {
    types::Point3                 p;
    types::ValueType const        r;
  };
  using StaticSphereGeomData = std::vector<StaticSphereGeomDataBlock>;



  struct MovementDataBlock {
    size_t                        sphere1_id;
    types::Point3                 destination;
  };
  using MovementData = std::vector<MovementDataBlock>;



  std::optional<types::Vector3>
  detectCollision(types::Vector3 ds,
                  SphereGeomDataBlock& sphere,
                  StaticSphereGeomData& static_spheres)
  {

    std::vector<types::Vector3> collisions;

    for (size_t s_id=0; s_id < static_spheres.size(); s_id++){
      StaticSphereGeomDataBlock static_sphere = static_spheres[s_id];

      std::optional<types::Vector3> collision = mechanics::detectCollisionSphereFixedSphere(
        ds,
        sphere.p,
        sphere.r,
        static_sphere.p,
        static_sphere.r);

      if (collision.has_value()) {
        collisions.push_back(collision.value());
      }

    }

    if (collisions.empty()) {
      return std::nullopt;
    }
    else {
      // sort collisions, return shortest collision ds
      std::ranges::sort(collisions, [](auto& col1, auto&col2){
        return blaze::length(col1) < blaze::length(col2);
      });

      return collisions.front();
    }

  }



  void raiseSphere(SphereGeomDataBlock& sphere) {
    sphere.raised = true;
    sphere.p[1] += 2 * sphere.r;
  }



  void lowerSphere(SphereGeomDataBlock& sphere) {
    sphere.raised = false;
    sphere.p[1] -= 2 * sphere.r;
  }



  void attemptLower(Params&               params,
                    SphereGeomDataBlock&  sphere,
                    StaticSphereGeomData& static_spheres,
                    types::ValueType&     dist_remaining,
                    size_t                prev_goal_id) {

    std::optional<types::Vector3> collision = detectCollision(  // collision down
      {0, -2 * sphere.r, 0},
      sphere,
      static_spheres);

    if (!collision.has_value()) {
      lowerSphere(sphere);

      // backtrack to find first availible position for sphere to lower into.
      if (prev_goal_id < 0) return;

      types::Point3 prev_goal = params.path[prev_goal_id];
      types::Vector3 ds = prev_goal - sphere.p;

      collision = detectCollision(ds, sphere, static_spheres);
      if (!collision.has_value()) {
        return;
      }

      ds = collision.value();
      dist_remaining += blaze::length(ds);
      sphere.p += ds;

    }

  }



  void moveToGoal(Params&               params,
                  SphereGeomDataBlock&  sphere,
                  StaticSphereGeomData& static_spheres,
                  types::Point3&        goal,
                  types::ValueType&     dist_to_goal,
                  types::ValueType&     dist_remaining) {

    types::Point3 cur_goal = goal;
    if (sphere.raised) {
      cur_goal[1] += 2 * sphere.r;
    }

    types::Vector3                ds = cur_goal - sphere.p;
    std::optional<types::Vector3> collision = detectCollision(ds, sphere, static_spheres);

    if (collision.has_value()) {
      ds = collision.value();
      dist_remaining -= blaze::length(ds);
      sphere.p += ds;
      raiseSphere(sphere);

      return;
    }
    else {
      dist_remaining -= dist_to_goal;
      sphere.p = cur_goal;
      sphere.next_goal += 1;
    }

    // attempt to lower
    if(sphere.raised) {
      attemptLower(
        params,
        sphere,
        static_spheres,
        dist_remaining,
        sphere.next_goal - 2);
    }

  }



  void moveTowardsGoal(Params&               params,
                       SphereGeomDataBlock&  sphere,
                       StaticSphereGeomData& static_spheres,
                       types::Point3&        goal,
                       types::ValueType&     dist_remaining) {

    types::Point3 cur_goal = goal;
    if (sphere.raised) {
      cur_goal[1] += 2 * sphere.r;
    }

    types::Vector3 ds = blaze::normalize(cur_goal - sphere.p) * dist_remaining;
    std::optional<types::Vector3> collision = detectCollision(ds, sphere, static_spheres);

    if (collision.has_value()) {
      ds = collision.value();
      dist_remaining -= blaze::length(ds);
      sphere.p += ds;
      raiseSphere(sphere);
    }
    else{
      sphere.p += ds;
    }

    if(sphere.raised) {
      attemptLower(
        params,
        sphere,
        static_spheres,
        dist_remaining,
        sphere.next_goal - 1);
    }
  }



  void moveSphere(Params&               params,
                  SphereGeomData&       spheres,
                  StaticSphereGeomData& static_spheres,
                  size_t                s_id)
  {

    SphereGeomDataBlock& sphere = spheres[s_id];

    types::Point3    goal;
    types::ValueType dist_to_goal;
    types::ValueType dist_remaining = params.v_max * utils::toDtScalar(params.dt);

    while(true) {

      if (sphere.next_goal >= params.path.size()) { // Path complete

        if (loopPath) {
          sphere.next_goal = 0;
        }
        return;
      }

      goal = params.path[sphere.next_goal];

      if (sphere.raised) {
        types::Point3 raised_goal = goal;
        raised_goal[1] += 2 * sphere.r;
        dist_to_goal = blaze::length(raised_goal - sphere.p);
      }
      else{
        dist_to_goal = blaze::length(goal - sphere.p);
      }

      if (dist_remaining >= dist_to_goal) {
        moveToGoal(
          params,
          sphere,
          static_spheres,
          goal,
          dist_to_goal,
          dist_remaining);
      }
      else {
        moveTowardsGoal(
          params,
          sphere,
          static_spheres,
          goal,
          dist_remaining);

        break;
      }
    }
  }



  template <concepts::SolverFixtureProject Fixture_T>
  void solve(Fixture_T&         scenario,
             types::NanoSeconds timestep)
  {

    Params params = {
      timestep,
      scenario.getMaxSpeed(),
      scenario.getPath()
    };

    SphereGeomData spheres;
    StaticSphereGeomData static_spheres;

    auto sphere_idx = scenario.nonFixedSphereRBs();
    auto static_sphere_idx = scenario.fixedSphereRBs();

    // Spheres
    for (size_t i = 0; i < sphere_idx.size(); i++) {
      size_t idx = sphere_idx[i];

      spheres.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbSphereRadius(idx),
        scenario.rbSphereNextGoal(idx),
        scenario.rbSphereIsRaised(idx)
      });
    }

    // Static spheres
    for (size_t i = 0; i < static_sphere_idx.size(); i++){
      size_t idx = static_sphere_idx[i];

      static_spheres.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbSphereRadius(idx)
      });
    }

    for (size_t s_id=0; s_id < spheres.size(); s_id++) {
      moveSphere(params, spheres, static_spheres, s_id);
    }

    // Update scenario
    for (size_t i = 0; i<spheres.size(); i++) {
      size_t sphere_id = sphere_idx[i];

      scenario.setGlobalFramePosition(sphere_id, spheres[i].p);
      scenario.setSphereNextGoal(sphere_id, spheres[i].next_goal);
      scenario.setSphereRaisedState(sphere_id, spheres[i].raised);
    }

  }

}   // namespace dte3607::physengine::solver_dev::project


#endif   // DTE3607_PHYSENGINE_SOLVER_PROJECT_H
