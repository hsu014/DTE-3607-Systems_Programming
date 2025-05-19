#ifndef DTE3607_PHYSENGINE_SOLVER_PROJECT_H
#define DTE3607_PHYSENGINE_SOLVER_PROJECT_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../utils/type_conversion.h"
#include "../mechanics/sphere_vs_fixed_sphere_detection.h"

namespace dte3607::physengine::solver_dev::project
{

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
    // sphere.p += types::Vector3{0, 2 * sphere.r, 0};
  }



  void lowerSphere(SphereGeomDataBlock& sphere) {
    sphere.raised = false;
    // sphere.p -= types::Vector3{0, 2 * sphere.r, 0};
  }



  void moveSphere(Params&              params,
                  SphereGeomData&      spheres,
                  StaticSphereGeomData static_spheres,
                  size_t               s_id)
  {

    SphereGeomDataBlock& sphere = spheres[s_id];

    types::Point3    goal;
    types::ValueType dist_to_goal;
    types::ValueType dist_remaining = params.v_max * utils::toDtScalar(params.dt);
    types::Vector3   ds;

    std::optional<types::Vector3> collision;

    while(true) {

      if (sphere.next_goal >= params.path.size()) {
        // std::cout << "Path complete\n" << std::endl;
        return;
      }

      if (sphere.raised) {
        // std::cout << "Collision\n" << std::endl;
        return;
      }

      goal = params.path[sphere.next_goal];
      dist_to_goal = blaze::length(goal - sphere.p);

      // Move to next goal
      if (dist_remaining >= dist_to_goal) {
        ds = goal - sphere.p;
        collision = detectCollision(ds, sphere, static_spheres);

        if (collision.has_value()) {
          ds = collision.value();
          dist_remaining -= blaze::length(ds);
          sphere.p += ds;
          raiseSphere(sphere);
        }
        else {
          std::cout << "Goal reached: " << sphere.next_goal << std::endl;
          dist_remaining -= dist_to_goal;
          sphere.p = goal;
          sphere.next_goal += 1;
          std::cout << "New goal: " << sphere.next_goal << std::endl;          
        }
      }
      // Move towards next goal
      else {
        ds = blaze::normalize(goal - sphere.p) * dist_remaining;
        collision = detectCollision(ds, sphere, static_spheres);

        if (collision.has_value()) {
          ds = collision.value();
          dist_remaining -= blaze::length(ds);
          sphere.p += ds;
          raiseSphere(sphere);
        }
        else{
          sphere.p += ds;
        }

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
