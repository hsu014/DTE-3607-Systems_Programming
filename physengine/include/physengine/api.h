#ifndef DTE3607_PHYSENGINE_FIXTURE_API_H
#define DTE3607_PHYSENGINE_FIXTURE_API_H


#include "solvers/solver_dev_level2_3.h"
#include "bits/types.h"
#include "bits/concepts.h"



namespace dte3607::physengine::api
{

  // SOLVER
  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve(Fixture_T& scenario, types::NanoSeconds timestep)
  {
    // Call another solver !_!
    solver_dev::level2_3::solve(scenario, timestep);
  }
}


#endif // DTE3607_PHYSENGINE_FIXTURE_API_H
