#include <physengine/solvers/solver_dev_level3.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;

bool test1 = false;
bool test2 = false;
bool test3 = true;
bool test4 = false;
bool test5 = false;



/////////////////
/// \brief TEST
///
///


struct SolverDevLevel3_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, -9.82, 0});

    // make plane
    m_scenario->createFixedInfPlane(
      {0, 1, 0}, {0, 0, 0}, m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.001, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef());
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel3_Fixture001, Test001)
{
  std::cout << "Test 1" << std::endl;
  if (!test1) GTEST_SKIP();
  solver_dev::level3::solve(*m_scenario, 10s);

  auto pos = m_scenario->globalFramePosition(m_the_s_rid);
  std::cout << "Pos: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Resting);
}





struct SolverDevLevel3_Fixture001_5 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, -9.81, 0});

    // make plane
    m_scenario->createFixedInfPlane(
      {0, 1, 0}, {0, 0, 0}, m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.001, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef());
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel3_Fixture001_5, Test001)
{
std::cout << "Test 1.5 (Test 1 with shorter timestep)" << std::endl;
if (!test1) GTEST_SKIP();
solver_dev::level3::solve(*m_scenario, 16ms);

auto pos = m_scenario->globalFramePosition(m_the_s_rid);
std::cout << "Pos: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Resting);
}





struct SolverDevLevel3_Fixture002 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({10, -9.82, 0});

    // make plane
    m_scenario->createFixedInfPlane(
      {0, 1, 0}, {0, 0, 0}, m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.001, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef());
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel3_Fixture002, Test001)
{
  std::cout << "Test 2" << std::endl;
  if (!test2) GTEST_SKIP();
  solver_dev::level3::solve(*m_scenario, 10s);

  auto pos = m_scenario->globalFramePosition(m_the_s_rid);
  std::cout << "Pos: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Sliding);
}




struct SolverDevLevel3_Fixture003 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_p_rid;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, -9.82, 0});

    // make plane
    m_the_p_rid = m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.000, 0}, types::RBState::Resting);

    // Attach sphere to plane
    m_scenario->addAttached(m_the_s_rid, m_the_p_rid);

    // make sphere 2
    m_scenario->createSphere(
      0.2, {0, -5, 0}, {-.5, 2.000, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef(), 10.);
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel3_Fixture003, Test001)
{
  std::cout << "Test 3" << std::endl;
  if (!test3) GTEST_SKIP();
  solver_dev::level3::solve(*m_scenario, 10s);
  // EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Free); // Wrong???
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Sliding);
}




struct SolverDevLevel3_Fixture004 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_p_rid;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_the_p_rid = m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.000, 0}, types::RBState::Resting);

    // Attach sphere to plane
    m_scenario->addAttached(m_the_s_rid, m_the_p_rid);

    // make sphere 2
    m_scenario->createSphere(
      0.2, {5, 0, 0}, {-1.5, .5, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef(), 10.);
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel3_Fixture004, Test001)
{
  std::cout << "Test 4" << std::endl;
  if (!test4) GTEST_SKIP();
  solver_dev::level3::solve(*m_scenario, 2s);
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Free);
}




struct SolverDevLevel3_Fixture005 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_p_rid;
  size_t                       m_the_s_rid;
  size_t                       m_the_s2_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_the_p_rid = m_scenario->createFixedInfPlane(
      {0, 1, 0}, {0, 0, 0}, m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {10, 0, 0}, {-5, 1, 0}, types::RBState::Sliding);

    // make sphere 2
    m_the_s2_rid = m_scenario->createSphere(
      // 0.2, {0, 0, 0}, {-5, 1, 0}, types::RBState::Resting, // Wrong?
      0.2, {0, 0, 0}, {0, 0.2, 0}, types::RBState::Resting,
      m_scenario->rbSphereMaxFrictionCoef(), 10000.);

    // Attach spheres to plane
    m_scenario->addAttached(m_the_s_rid, m_the_p_rid);
    m_scenario->addAttached(m_the_s2_rid, m_the_p_rid);
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel3_Fixture005, Test001)
{
  std::cout << "Test 5" << std::endl;
  if (!test5) GTEST_SKIP();
  solver_dev::level3::solve(*m_scenario, 2s);
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Free);
}
