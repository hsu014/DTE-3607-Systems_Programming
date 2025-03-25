#include <physengine/solvers/solver_dev_level2_2.h>
#include <physengine/solvers/solver_dev_level2_3.h>
// #include <physengine/solvers/solver_dev_level2_3_old.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>


// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>
#include <memory>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///


struct SolverDevLevel2Step1_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step1_Fixture001, Test001)
{
  std::cout << "Test 1! " << std::endl;
  solver_dev::level2_2::solve(*m_scenario, 1s);

  auto no_rbs = m_scenario->noRigidBodies();
  for( size_t rid = 0; rid < no_rbs; ++rid )
  {
    // Ask for global frame position of object nr. i
    auto const pos = m_scenario->globalFramePosition(rid);
    // std::cout << "Pos[0]: " << pos[0] << std::endl;
    EXPECT_LE(pos[0], 10);
  }
}




struct SolverDevLevel2Step1_Fixture002 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane(s)
    // yz+ plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

    // yz- plane
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step1_Fixture002, Test001)
{
  std::cout << "Test 2! " << std::endl;
  solver_dev::level2_2::solve(*m_scenario, 1s);

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const d  = blaze::evaluate(sp - (pp + blaze::normalize(pn)*sr));
      // std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;
      EXPECT_GE(blaze::inner(pn,d),0);
    }
  }
}




struct SolverDevLevel2Step2_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step2_Fixture001, Test001)
{
  std::cout << "Test 3! " << std::endl;
  solver_dev::level2_2::solve(*m_scenario, 1s);

  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    auto const sp = m_scenario->globalFramePosition(s_rid);
    // std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;
    EXPECT_GT(sp[0], -10);
    EXPECT_LT(sp[0], 10);
  }
}



/*
// struct SolverDevLevel2Step2_Fixture002 : ::testing::Test {

//   using TestFixture = fixtures::FixtureLevel2;
//   std::unique_ptr<TestFixture> m_scenario;

//   using ::testing::Test::Test;

//   void SetUp() final
//   {
//     // Create Fixture
//     m_scenario = std::make_unique<TestFixture>();


//     // Set external forces
//     m_scenario->setGravity({0,0,0});


//     // make plane
//     m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

//     m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

//     m_scenario->createFixedInfPlane({0, 0, -1}, {0, 0, 10});

//     m_scenario->createFixedInfPlane({0, 0, 1}, {0, 0, -10});


//     // make sphere
//     m_scenario->createSphere(1.0, {100, 0, 100}, {0, 0, 0});
//   }
//   void TearDown() final { m_scenario.release(); }
// };


// TEST_F(SolverDevLevel2Step2_Fixture002, Test001)
// {
//   solver_dev::level2_2::solve(*m_scenario, 1s);

//   // Expect to be inbetween the planes
//   for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
//     for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

//       std::cout << "Running test 4! " << std::endl;
//       auto const pn = m_scenario->rbPlaneNormal(p_rid);
//       auto const pp = m_scenario->globalFramePosition(p_rid);
//       auto const sp = m_scenario->globalFramePosition(s_rid);
//       auto const sr = m_scenario->rbSphereRadius(s_rid);
//       auto const& d = sp - pp;
//       auto const pnd = blaze::inner(pn,d) - sr;
//       std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;
//       EXPECT_TRUE(pnd > 0);
//     }
//   }
// }
*/




struct SolverDevLevel2Step2_Fixture003 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});
    m_scenario->createFixedInfPlane({0, 0, -1}, {0, 0, 10});
    m_scenario->createFixedInfPlane({0, 0, 1}, {0, 0, -10});
    m_scenario->createFixedInfPlane({0, -1, 0}, {0, 10, 0});
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, -10, 0});

    // Set up spheres
    auto const p = types::Vector3{-2, -2, -2};

    // Randomize selection of normal
    auto const v_min = types::Vector3{-10, 5, -20};
    auto const v_max = types::Vector3{ 20, 10, 10};

    std::random_device                     r;
    std::default_random_engine             g(r());
    std::uniform_real_distribution<types::ValueType> x_dist(v_min[0], v_max[0]);
    std::uniform_real_distribution<types::ValueType> y_dist(v_min[1], v_max[1]);
    std::uniform_real_distribution<types::ValueType> z_dist(v_min[2], v_max[2]);

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x) {
      for (auto y = 0; y < 3; ++y) {
        for (auto z = 0; z < 3; ++z) {

          auto const v = types::Vector3{x_dist(g), y_dist(g), z_dist(g)};
          sphere_data.emplace_back(
            v, p + types::Vector3{2. * x, 2. * y, 2. * z});
        }
      }
    }

    for (auto const& sd : sphere_data) {
      m_scenario->createSphere(1.0, std::get<0>(sd), std::get<1>(sd));

      // std::cout << "m_scenario->createSphere(0.9, {" << std::get<0>(sd)[0] << ", " << std::get<0>(sd)[1] << ", " << std::get<0>(sd)[2] << "}, "; // v
      // std::cout << " {" << std::get<1>(sd)[0] << ", " << std::get<1>(sd)[1] << ", " << std::get<1>(sd)[2] << "} );" << std::endl; // p
    }
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step2_Fixture003, Test001)
{
  std::cout << "Test 5! " << std::endl;
  solver_dev::level2_3::solve(*m_scenario, 1000ms);

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const& d = sp - pp;
      auto const pnd = blaze::inner(pn,d) - sr;
      if (pnd < 0.0) {
        std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;
        std::cout  << "Sphere: " << s_rid << "    pnd: " << pnd <<  std::endl << std::endl;
      }
      EXPECT_GT(pnd, 0);
    }
  }
}




// Specific case of test 5
struct SolverDevLevel2Step2_Fixture004 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // Make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});
    m_scenario->createFixedInfPlane({0, 0, -1}, {0, 0, 10});
    m_scenario->createFixedInfPlane({0, 0, 1}, {0, 0, -10});
    m_scenario->createFixedInfPlane({0, -1, 0}, {0, 10, 0});
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, -10, 0});

    // Set up spheres
    m_scenario->createSphere(0.9, {13.2747, 7.71821, 2.3973},  {-2, -2, -2} );
    m_scenario->createSphere(0.9, {7.84091, 5.4148, -17.0569},  {-2, -2, 0} );
    m_scenario->createSphere(0.9, {-0.0345602, 6.24655, 4.76254},  {-2, -2, 2} );
    m_scenario->createSphere(0.9, {16.5811, 5.58608, -8.8269},  {-2, 0, -2} );
    m_scenario->createSphere(0.9, {15.1732, 8.67646, -12.4931},  {-2, 0, 0} );
    m_scenario->createSphere(0.9, {0.741096, 8.07127, 6.68489},  {-2, 0, 2} );
    m_scenario->createSphere(0.9, {8.13768, 5.35436, -10.3766},  {-2, 2, -2} );
    m_scenario->createSphere(0.9, {12.2849, 8.7848, -15.204},  {-2, 2, 0} );
    m_scenario->createSphere(0.9, {15.8895, 8.0556, 7.10997},  {-2, 2, 2} );
    m_scenario->createSphere(0.9, {-1.6776, 5.82968, 4.8973},  {0, -2, -2} );
    m_scenario->createSphere(0.9, {-8.27168, 5.83577, -5.54859},  {0, -2, 0} );
    m_scenario->createSphere(0.9, {16.5587, 9.00887, -1.23641},  {0, -2, 2} );
    m_scenario->createSphere(0.9, {11.666, 5.63942, 0.0924569},  {0, 0, -2} );
    m_scenario->createSphere(0.9, {13.1824, 8.77258, 0.610019},  {0, 0, 0} );
    m_scenario->createSphere(0.9, {-6.54629, 9.53051, -13.5187},  {0, 0, 2} );
    m_scenario->createSphere(0.9, {-1.24066, 8.04538, 2.09348},  {0, 2, -2} );
    m_scenario->createSphere(0.9, {15.399, 5.3056, -13.4107},  {0, 2, 0} );
    m_scenario->createSphere(0.9, {-2.18282, 9.81262, -7.52723},  {0, 2, 2} );
    m_scenario->createSphere(0.9, {-4.96062, 6.54013, -9.47521},  {2, -2, -2} );
    m_scenario->createSphere(0.9, {6.92941, 7.44851, 6.94175},  {2, -2, 0} );
    m_scenario->createSphere(0.9, {11.1125, 6.09522, 7.84873},  {2, -2, 2} );
    m_scenario->createSphere(0.9, {9.96694, 5.56824, -8.47706},  {2, 0, -2} );
    m_scenario->createSphere(0.9, {-3.71993, 8.90286, -8.00741},  {2, 0, 0} );
    m_scenario->createSphere(0.9, {15.0349, 7.59546, -17.9487},  {2, 0, 2} );
    m_scenario->createSphere(0.9, {0.950062, 5.55888, -5.20937},  {2, 2, -2} );
    m_scenario->createSphere(0.9, {9.1452, 7.52006, -13.9678},  {2, 2, 0} );
    m_scenario->createSphere(0.9, {4.92318, 5.43744, -13.6422},  {2, 2, 2} );
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step2_Fixture004, Test001)
{
  std::cout << "Test 6! " << std::endl;
  solver_dev::level2_3::solve(*m_scenario, 600ms);
  std::cout << std::endl;

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const& d = sp - pp;
      auto const pnd = blaze::inner(pn,d) - sr;
      if (pnd < 0.0) {
        std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] << std::endl;
        std::cout << "Sphere: " << s_rid - 6 << "    pnd: " << pnd << std::endl << std::endl;
      }
      EXPECT_GT(pnd, 0);
    }
  }
}




/*
 * Test ball collisions:
 */
struct SolverDevLevel2Step3_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // Set up spheres
    m_scenario->createSphere(1.0, {30, 0, 0}, {-30, -0.1, 0});
    m_scenario->createSphere(1.0, {-30, 0, 0}, {30, 0.1, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step3_Fixture001, Test001)
{
  std::cout << "Test 7! " << std::endl;
  solver_dev::level2_3::solve(*m_scenario, 1s);

  // Expect spheres to not be at x = 0
  // Expect sphere to be at y > |0.1|
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    auto const sp = m_scenario->globalFramePosition(s_rid);
    // std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;

    EXPECT_GT(std::abs(sp[0]), 0.00001);
    EXPECT_GT(std::abs(sp[1]), 0.1);
  }
}




struct SolverDevLevel2Step3_Fixture002 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // Make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

    // Set up spheres
    m_scenario->createSphere(1.0, {21, 0, 0}, {-5, 0, 0});
    m_scenario->createSphere(1.0, {-21, 0, 0}, {5, 0, 0});

  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step3_Fixture002, Test001)
{
  std::cout << "Test 8! " << std::endl;
  solver_dev::level2_3::solve(*m_scenario, 1s);

  // Expect balls to not be at x = 0
  // Expect balls to be between planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    auto const sp = m_scenario->globalFramePosition(s_rid);
    auto const sr = m_scenario->rbSphereRadius(s_rid);

    // std::cout << "Pos(id " << s_rid-2 << "): " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;

    EXPECT_GT(std::abs(sp[0]), 0.00001);
    EXPECT_LE(std::abs(sp[0]), 10 - sr);
  }
}




struct SolverDevLevel2Step3_Fixture003 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // Make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});
    m_scenario->createFixedInfPlane({0, -1, 0}, {0, 10, 0});
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, -10, 0});

    // Set up spheres
    m_scenario->createSphere(0.9, {-11, 9.74219, 0}, {2, 2, 0} );
    m_scenario->createSphere(0.9, {18, 9.16423, 0}, {0, -2, 0});
    m_scenario->createSphere(0.9, {15, 7.56572, 0}, {-2, 0, 0} );
    m_scenario->createSphere(0.9, {19, 7.41067, 0}, {-2, -2, 0} );
    m_scenario->createSphere(0.9, {19, 7.41067, 0}, {2, -2, 0} );



  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step3_Fixture003, Test001)
{
  std::cout << "Test 9! " << std::endl;
  solver_dev::level2_3::solve(*m_scenario, 10s);

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const& d = sp - pp;
      auto const pnd = blaze::inner(pn,d) - sr;
      if (pnd < 0.0) {
        std::cout << "Pos: " << sp[0] << ", " << sp[1] << ", " << sp[2] <<std::endl;
        std::cout  << "Sphere: " << s_rid << "    pnd: " << pnd <<  std::endl << std::endl;
      }
      EXPECT_GT(pnd, 0);
    }
  }
}









//struct SolverDevLevel2Step3_TestFixture : ::testing::Test {

//  using ::testing::Test::Test;
//  ~SolverDevLevel2Step3_TestFixture() override {}

//  void SetUp() final
//  {
////    m_scenario_fixture = ...;
//  }
//  void TearDown() final {}

//  fixtures::FixtureLevel2 m_scenario;
//};



//TEST_F(SolverDevLevel2Step3_TestFixture, Test001)
//{
//  solver_dev::level2_2::solve(m_scenario, 16ms);

//  EXPECT_TRUE(false);
//}

