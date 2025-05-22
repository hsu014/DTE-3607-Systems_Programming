// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <chrono>

#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>
#include "test_setup.h"
#include <physengine/solvers/solver_dev_level3.h>


using namespace dte3607::physengine;
using namespace std::chrono_literals;


namespace dte3607::benchmarking::dev3 {

  types::ValueType sph_r = 0.99;
  types::NanoSeconds dt = 16ms; //16ms;
  int num_iterations = 10;


  struct GoldBenchmarkF1 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF1() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 1, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF2 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF2() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 2, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF3 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF3() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
      sphere_data = setup::createSphereData(10, 3, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF4 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF4() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 4, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF5 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF5() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 5, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF6 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF6() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 6, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF7 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF7() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 7, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF8 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF8() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 8, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF9 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF9() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>>
        sphere_data = setup::createSphereData(10, 9, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };



  struct GoldBenchmarkF10 : benchmark::Fixture {

    using TestFixture = fixtures::FixtureLevel3;
    std::unique_ptr<TestFixture> m_scenario;

    using benchmark::Fixture::Fixture;
    ~GoldBenchmarkF10() override {}

    void SetUp(benchmark::State const&) final
    {
      m_scenario = std::make_unique<TestFixture>();
      m_scenario->setGravity({0, -9.81, 0});
      m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});
      m_scenario->createFixedInfPlane({0, -1, 0}, {0, 22, 0});

      std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
      sphere_data = setup::createSphereData(10, 10, 10);

      for (auto const& sd : sphere_data) {
        m_scenario->createSphere(sph_r, std::get<0>(sd), std::get<1>(sd));
      }

    }
    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };

}   // namespace dte3607::benchmarking::dev3








// Qualify predefined fixtures
using namespace dte3607::benchmarking::dev3;


BENCHMARK_DEFINE_F(GoldBenchmarkF1, test_10x1x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF2, test_10x2x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF3, test_10x3x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF4, test_10x4x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF5, test_10x5x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF6, test_10x6x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF7, test_10x7x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF8, test_10x8x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF9, test_10x9x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}



BENCHMARK_DEFINE_F(GoldBenchmarkF10, test_10x10x10_spheres)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    solver_dev::level3::solve(*m_scenario, dt);
}








BENCHMARK_REGISTER_F(GoldBenchmarkF1, test_10x1x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF2, test_10x2x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF3, test_10x3x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF4, test_10x4x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF5, test_10x5x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF6, test_10x6x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF7, test_10x7x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF8, test_10x8x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF9, test_10x9x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(GoldBenchmarkF10, test_10x10x10_spheres)->Iterations(num_iterations)->Unit(benchmark::kMillisecond);



BENCHMARK_MAIN();
