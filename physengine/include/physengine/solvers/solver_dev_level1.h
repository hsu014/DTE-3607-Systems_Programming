#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/compute_trajectory.h"


namespace dte3607::physengine::solver_dev::level1
{

  struct Params {
    types::Vector3 F;                 // External forces (environment)
    types::Duration dt;               // Timestep (system)
  };


  struct CacheProcDataBlock {
    types::Vector3 in_p;              // Position
    types::Vector3 in_v;              // Velocity
    types:: Vector3 out_ds;           // Trajectory
  };
  using CacheProcData = std::vector<CacheProcDataBlock>;


  struct SimProcDataBlock{
    types:: Vector3 in_p;             // Position
    types:: Vector3 in_ds;            // Trajectory
    types:: Vector3 out_p;            // NewPosition
  };
  using SimProcData = std::vector<SimProcDataBlock>;


  void computeCache(CacheProcData& data, Params const &params) {

    auto const proc_kernel = [&params](auto& data) {
      auto const& [F, dt] = params;
      auto& [pos, vel, out_ds] = data;
      out_ds = mechanics::computeLinearTrajectory(vel, F, dt).first;
    };
    std::ranges::for_each(data, proc_kernel);
  }


  void computeSimulation(SimProcData& data) {

    auto const proc_kernel = [](auto& data){
      auto& [p, ds, out_p] = data;
      out_p = p + ds;
    };
    std::ranges::for_each(data,proc_kernel);
  }


  template <concepts::SolverFixtureLevel1 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {

    Params params = {
      scenario.getGravity(),
      timestep
    };
    CacheProcData cache;
    SimProcData data;

    auto no_rbs = scenario.noRigidBodies();

    // Prepare cache
    for (int rid = 0; rid < no_rbs; rid++) {
      cache.push_back({
        scenario.globalFramePosition(rid),
        scenario.globalFrameVelocity(rid)
      });
    }
    computeCache(cache, params);

    // Simulate
    for (int rid = 0; rid < no_rbs; rid++) {
      data.push_back({
        scenario.globalFramePosition(rid),
        cache[rid].out_ds
      });
    }
    computeSimulation(data);

    // Return values to fixture
    for (int rid = 0; rid < no_rbs; rid++) {
      scenario.setGlobalFramePosition(
        rid,
        data[rid].out_p
        );
    }
  }








}   // namespace dte3607::physengine::solver_dev::level1


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
