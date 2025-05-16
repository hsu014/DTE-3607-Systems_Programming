#ifndef DTE3607_PHYSENGINE_FIXTURE_API_H
#define DTE3607_PHYSENGINE_FIXTURE_API_H


#include "solvers/solver_project.h"
#include "bits/types.h"
#include "bits/concepts.h"



namespace dte3607::physengine::api
{

  // SOLVER
  template <concepts::SolverFixtureProject Fixture_T>
  void solve(Fixture_T& scenario, types::NanoSeconds timestep)
  {
    // Call another solver !_!
    solver_dev::project::solve(scenario, timestep);
  }
}


#endif // DTE3607_PHYSENGINE_FIXTURE_API_H
