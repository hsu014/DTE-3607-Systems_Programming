#ifndef DTE3607_PHYSENGINE_SOLVER_PROJECT_H
#define DTE3607_PHYSENGINE_SOLVER_PROJECT_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../utils/type_conversion.h"

namespace dte3607::physengine::solver_dev::project
{

  struct Params {
    types::Vector3 const          F;                // External forces (environment)
    types::HighResolutionTP const t_0;
    types::Duration const         dt;               // Timestep (system)
    types::ValueType const        v_max;            // Allowed movement speed
    std::vector<types::Point3>    path;
  };



  struct SphereGeomDataBlock {
    types::Point3                 p;
    types::ValueType const        r;
    types::HighResolutionTP       t_c;
    size_t                        next_goal;
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



  // void findMovement([[maybe_unused]]Params&         params,
  //                   [[maybe_unused]]SphereGeomData& spheres,
  //                   [[maybe_unused]]size_t          s_id,
  //                   [[maybe_unused]]MovementData&   movement)
  // {

  //   types::Point3 goal = params.path[spheres[s_id].next_goal];
  //   movement.push_back({s_id, goal});
  // }



  // void findInitialMovement([[maybe_unused]]Params&         params,
  //                          [[maybe_unused]]SphereGeomData& spheres,
  //                          [[maybe_unused]]MovementData&   movement)
  // {

  //   for (size_t s_id=0; s_id < spheres.size(); s_id++) {
  //     findMovement(params, spheres, s_id, movement);
  //   }
  // }



  // void handleMovement([[maybe_unused]]Params&         params,
  //                     [[maybe_unused]]SphereGeomData& spheres,
  //                     [[maybe_unused]]MovementData&   movement)
  // {

  // }







  void moveSphere([[maybe_unused]]Params&         params,
                  [[maybe_unused]]SphereGeomData& spheres,
                  [[maybe_unused]]size_t          s_id)
  {

    SphereGeomDataBlock& sphere = spheres[s_id];

    types::Point3    goal;
    types::ValueType dist_to_goal;
    types::ValueType dist_remaining = params.v_max * utils::toDtScalar(params.dt);

    while(true) {

      if (sphere.next_goal >= params.path.size()) {
        std::cout << "Path complete\n" << std::endl;
        return;
      }

      goal = params.path[sphere.next_goal];
      dist_to_goal = blaze::length(goal - sphere.p);

      // std::cout << "position: \n" << sphere.p << std::endl;
      // std::cout << "goal " << sphere.next_goal << "\n" << goal << std::endl;

      if (dist_remaining >= dist_to_goal) { // Move to next goal
        std::cout << "Goal reached: " << sphere.next_goal << std::endl;
        dist_remaining -= dist_to_goal;
        sphere.p = goal;
        sphere.next_goal += 1;
        std::cout << "New goal: " << sphere.next_goal << std::endl;

        // goal = params.path[sphere.next_goal];
        // dist_to_goal = blaze::length(goal - sphere.p);

        // std::cout << "position: \n" << sphere.p << std::endl;
        // std::cout << "dist remaining: " << dist_remaining << std::endl;
      }
      else {                                // Move towards goal

        types::Point3 path = goal - sphere.p;
        types::Point3 ds = blaze::normalize(path) * dist_remaining;

        // std::cout << "path: \n" << path << std::endl;
        // std::cout << "ds: \n" << ds << std::endl;

        sphere.p += ds;

        // std::cout << "position: \n" << sphere.p << std::endl;

        break;
      }

    }
    // std::cout << "After loop\n\n\n" << std::endl;
  }




  /* Want to do:
   *
   * for each sphere:
   *   define next goal point
   *   while time remaining:
   *     - look for collision
   *     if collision found:
   *       move to collision point
   *       raise shpere up
   *     else:
   *       move toward goal
   *     move sphere forward in time
   *     set next goal
   *
   */



  template <concepts::SolverFixtureProject Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {

    std::cout << "Using solver" << std::endl;

    Params params = {
      {0,0,0},
      types::HighResolutionClock::now(),
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
        params.t_0,
        scenario.rbSphereNextGoal(idx)
      });
    }

    // std::cout << "Path size:      " << params.path.size() << std::endl;
    // std::cout << "Spheres:        " << spheres.size() << std::endl;
    // std::cout << "Static Spheres: " << static_spheres.size() << std::endl;

    // Static spheres
    for (size_t i = 0; i < static_sphere_idx.size(); i++){

      size_t idx = static_sphere_idx[i];

      static_spheres.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbSphereRadius(idx)
      });
    }

    //MovementData movements;

    for (size_t s_id=0; s_id < spheres.size(); s_id++) {
      moveSphere(params, spheres, s_id);
    }

    // Update scenario
    for (size_t i = 0; i<spheres.size(); i++) {
      size_t sphere_id = sphere_idx[i];

      scenario.setGlobalFramePosition(sphere_id, spheres[i].p);
      scenario.setSphereNextGoal(sphere_id, spheres[i].next_goal);
    }















  }

}   // namespace dte3607::physengine::solver_dev::level4


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL4_H
