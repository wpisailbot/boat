#include "gtest/gtest.h"
#include <gflags/gflags.h>

#include "util/node.h"
#include "control/line_plan.h"
#include "control/util.h"
#include "util/testing.h"

namespace sailbot {
namespace control {
namespace testing {

void TryTurnCost(double startheading, double endheading, double winddir,
                 double expcost, double expdstart, double expdend,
                 const char *desc) {
  expcost *= LinePlan::kTurnCost;
  expdstart *= LinePlan::kTurnCost;
  expdend *= LinePlan::kTurnCost;
  double cost, dstart, dend;
  LinePlan::TurnCost(startheading, endheading, winddir, &cost, &dstart, &dend);
  EXPECT_NEAR(expcost, cost, 1e-6) << desc << ": Incorrect cost";
  EXPECT_NEAR(expdstart, dstart, 1e-6) << desc << ": Incorrect dcostdstart";
  EXPECT_NEAR(expdend, dend, 1e-6) << desc << ": Incorrect dcostdend";
}

TEST(LinePlanUtilTest, TurnCostTest) {
  constexpr double kT = LinePlan::kTackCost;
  // startheading, endheading, winddir, expcost, expdstart, expdend, desc
  TryTurnCost(0.0, 1.0, 0.0, 1.0, -2.0, 2.0, "Basic conditions, no upwind");
  TryTurnCost(0.0, -1.0, 0.0, 1.0, 2.0, -2.0, "Reverse direction, no upwind");
  double expbasecost = 1.0 + (kT - 1.0) * M_PI / 4.0;
  double expcost = expbasecost * expbasecost;
  double dcoeff = 2.0 * expbasecost;
  TryTurnCost(0.0, 1.0, M_PI, expcost, -dcoeff * kT, dcoeff,
              "Out of irons, 1 rad turn");
  double kT2 = kT * kT;
  TryTurnCost(0.0, 0.5, M_PI, 0.25 * kT2, -kT2, kT2,
              "Upwind solely in irons");
  TryTurnCost(2.0, 2.5, 2.0 - M_PI, 0.25 * kT2, -kT2, kT2,
              "Upwind solely in irons, nonzero start");
  expbasecost = 2.0 + (kT - 1.0) * M_PI / 2.0;
  expcost = expbasecost * expbasecost;
  TryTurnCost(-1.0, 1.0, M_PI, expcost, -2.0 * expbasecost, 2.0 * expbasecost,
              "Complete tack");
}

void TryCrossFinishCost(double alpha, double expcost, double expdalpha,
                        const char *desc) {
  double cost, dalpha;
  LinePlan::CrossFinishCost(alpha, &cost, &dalpha);
  EXPECT_EQ(expcost, cost) << desc << ": Incorrect cost";
  EXPECT_EQ(expdalpha, dalpha) << desc << ": Incorrect dcostdalpha";
}

TEST(LinePlanUtilTest, CrossFinishTest) {
  TryCrossFinishCost(0.0, 1.0, -4.0, "Basic alpha=0");
  TryCrossFinishCost(0.5, 0.0, 0.0, "At min cost");
  TryCrossFinishCost(1.0, 1.0, 4.0, "At alpha=1");
}

TEST(LinePlanUtilTest, SingleLineCostTest) {
  // For these, things start to get more complicated and trying to guess at the
  // true resulting cost is a bit countrproductive. Instead, I will check that
  // the cost behaves roughly how we would expect when inputs are tweaked.

  // Cost should increase slightly when the startpt is moved farther out.
  // For this, use conditions where we are on a beam reach,
  // just bisecting the finish line and continuing straight on.
  double cost1, cost2, dcostdalpha;
  Eigen::Vector2d startline(0, 1), endline(2, 1), startpt(1, 0), nextpt(1, 2);
  double alpha = 0.5;
  double winddir = 0.0;
  double preheading = M_PI_2;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost1, &dcostdalpha, nullptr);
  startpt.y() -= 1.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha, nullptr);
  startpt.y() += 1.0;
  EXPECT_LT(cost1, cost2)
      << "Moving the start point farther away should increase cost";

  // Cost should also increase when lengths stay the same but
  // the middle angle change increases.
  startpt << 0.0, 1.0;
  preheading = 0.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha, nullptr);
  startpt << 1.0, 0.0;
  preheading = M_PI_2;
  EXPECT_LT(cost1, cost2)
      << "Increasing required turn should increase cost";
  // And changing the initial angle:
  preheading = 0.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha, nullptr);
  preheading = M_PI_2;
  EXPECT_LT(cost1, cost2)
      << "Increasing required initial turn should increase cost";

  // And increase if we are forced to travel upwind.
  winddir = -M_PI_2 - 0.2;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha, nullptr);
  winddir = 0.0;
  EXPECT_LT(cost1, cost2)
      << "Going more upwind should increase cost";

  // And we should get sane derivatives out of alpha.
  double eps = 0.0001;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha + eps, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha, nullptr);
  LOG(INFO) << "Cost1: " << cost1 << " cost2: " << cost2;
  EXPECT_NEAR(dcostdalpha, (cost2 - cost1) / eps, 0.005)
      << "Numeric and analytic derivatives should be close";
}

void TryStraightLineCost(double len, double heading, double winddir,
                         double expcost, double expdlen, double expdheading,
                         const char *desc) {
  double cost, dlen, dheading;
  bool viable;
  LinePlan::StraightLineCost(len, heading, winddir, &cost, &dlen, &dheading,
                             &viable);
  EXPECT_EQ(expcost, cost) << desc << ": Incorrect cost";
  EXPECT_EQ(expdlen, dlen) << desc << ": Incorrect costdlen";
  EXPECT_EQ(expdheading, dheading) << desc << ": Incorrect costdheading";
  EXPECT_EQ(std::abs(util::norm_angle(heading - winddir + M_PI)) <
                LinePlan::kSailableReach,
            !viable)
      << desc << ": incorrect viability";

  double dcost;
  double eps = 1e-5;
  LinePlan::StraightLineCost(len + eps, heading, winddir, &dcost, &dlen,
                             &dheading, nullptr);
  EXPECT_NEAR(dlen, (dcost - cost) / eps, 0.001) << desc
                                                 << ": Bad empirical dlen";

  LinePlan::StraightLineCost(len, heading + eps, winddir, &dcost, &dlen,
                             &dheading, nullptr);
  EXPECT_NEAR(dheading, (dcost - cost) / eps, 0.001)
      << desc << ": Bad empirical dheading";
}

TEST(LinePlanUtilTest, StraightLineTest) {
  constexpr double kL = LinePlan::kLengthCost;
  constexpr double kU = LinePlan::kUpwindCost;

  double expcost = kL;
  double expdlen = kL;
  double expdheading = 0.0;
  TryStraightLineCost(1.0, 0.0, 0.0, expcost, expdlen, expdheading,
                      "Downwind leg");

  double heading = 0.5;
  double len = 2.0;
  expdlen = kL * (1.0 + kU * (1.0 - (1.0 - heading) * (1.0 - heading)));
  expcost = expdlen * len;
  // d(1 - (1 - h)^2) / dh = 2 * (1 - h)
  expdheading = -kL * 2.0 * kU * (1.0 - heading) * len;
  TryStraightLineCost(len, heading, M_PI, expcost, expdlen, expdheading,
                      "Upwind leg");

  heading *= -1.0;
  expdheading *= -1.0;
  TryStraightLineCost(len, heading, M_PI, expcost, expdlen, expdheading,
                      "Upwind leg, other side");
}

TEST(LinePlanUtilTest, LinePairCostTest) {
  // Same general idea as the SingleLineCostTest, don't worry too much about
  // exact numbers.

  // Initial conditions involve just going straight on a beam reach.
  Eigen::Vector2d startpt(0.0, 0.0), endpt(2.0, 2.0), turnpt(1.0, 1.0),
      dcostdturn;
  double preheading = M_PI_4, postheading = M_PI_4, winddir = 0.0;
  // Use cost for the nominal conditions, dcost for variations:
  double cost, dcost;

  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &cost, &dcostdturn, nullptr);

  // First, just check that empirically measured derivatives match the actual
  // derivatives.
  double eps = 1e-4, tol = 1e-3;
  LinePlan::LinePairCost(startpt, endpt, turnpt + Eigen::Vector2d(eps, 0.0),
                         preheading, postheading, winddir, &dcost, &dcostdturn,
                         nullptr);
  EXPECT_NEAR(dcostdturn.x(), (dcost - cost) / eps, tol)
      << "X partial doesn't match numeric/analytic";

  LinePlan::LinePairCost(startpt, endpt, turnpt + Eigen::Vector2d(0.0, eps),
                         preheading, postheading, winddir, &dcost, &dcostdturn,
                         nullptr);
  EXPECT_NEAR(dcostdturn.y(), (dcost - cost) / eps, tol)
      << "Y partial doesn't match numeric/analytic";

  // Moving endpt farther out should increase costs
  endpt *= 2.0;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn, nullptr);
  endpt /= 2.0;
  EXPECT_LT(cost, dcost) << "Increasing segment lengths should increase cost";

  // Shifting endpt to create angle should increase cost.
  endpt << 0.0, 2.0;
  postheading = 3.0 * M_PI_4;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn, nullptr);
  endpt << 2.0, 2.0;
  postheading = M_PI_4;
  EXPECT_LT(cost, dcost) << "Creating elbow should increase cost";

  // Adjusting postheading should incur a cost:
  postheading *= -1.0;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn, nullptr);
  postheading *= -1.0;
  EXPECT_LT(cost, dcost) << "Adding turn at end should increase cost";

  // Adjusting preheading should incur a cost:
  preheading *= -1.0;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn, nullptr);
  preheading *= 1.0;
  EXPECT_LT(cost, dcost) << "Adding turn at beginning should increase cost";

  // Forcing us to go through the wind should increase cost:
  winddir = M_PI;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn, nullptr);
  winddir = 0.0;
  EXPECT_LT(cost, dcost) << "Going upwind should increase cost";
}

TEST(LinePlanUtilTest, ObstacleCostTest) {
  constexpr double kCost = LinePlan::kObstacleCost;
  std::vector<Polygon> obstacles;
  // First, should produce 0 cost with no obstacles:
  double cost;
  Point dcostdstart;
  LinePlan::ObstacleCost({0, 0}, {1, 1}, obstacles, &cost, &dcostdstart);
  EXPECT_EQ(0, cost) << "Should have no cost on no obstacles";
  EXPECT_EQ(Point(0.0, 0.0), dcostdstart)
      << "Should have zero derivative on no obstacles";

  // Create single, simple line obstacle
  obstacles.push_back(Polygon({{2.0, 0.0}, {2.0, 2.0}}));

  // A line of length one distance 1.0 from the obstacle should
  // produce a cost of nearly e^{-1}.
  LinePlan::ObstacleCost({1.0, 0.0}, {1.0, 1.0}, obstacles, &cost,
                         &dcostdstart);
  double expcost = kCost * std::exp(-1.0);
  double tol = 5e-4;
  EXPECT_NEAR(expcost, cost, tol);
  EXPECT_NEAR(-expcost, dcostdstart.y(), tol);
  // x derivative should be around half of the derivative of exp(-x):
  EXPECT_NEAR(expcost / 2.0, dcostdstart.x(), tol);

  // Check that integrating over line not of length 1 works:
  obstacles.clear();
  obstacles.push_back(Polygon({{2.0, 0.0}, {2.0, 5.0}}));
  LinePlan::ObstacleCost({1.0, 0.0}, {1.0, 2.5}, obstacles, &cost,
                         &dcostdstart);
  expcost = kCost * std::exp(-1.0) * 2.5;
  EXPECT_NEAR(expcost, cost, tol);
  EXPECT_NEAR(-expcost / 2.5, dcostdstart.y(), tol);
  // x derivative should be around half of the derivative of exp(-x):
  EXPECT_NEAR(expcost / 2.0, dcostdstart.x(), tol);
}

// Run a set of tests on BackPass, for situations when we are only using one
// point--really more of a test of the derivative returned by SingleLineCost
TEST(LinePlanUtilTest, BackPassTestOnePoint) {
  // First, just check that a basic beam reach in what should be optimal
  // position doesn't change substantially:
  std::vector<Point> tackpts({{0.0, 0.0}});
  std::vector<Polygon> obstacles; // Start with none.
  double alpha = 0.5;
  std::pair<Point, Point> gate({{-1.0, 10.0}, {1.0, 10.0}});
  double cur_yaw = M_PI_2;
  Point nextpt(0.0, 20.0); // We should just continue straight through the gate.

  // For comparisons:
  std::vector<Point> orig_tackpts = tackpts;
  double orig_alpha = alpha;
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha, nullptr, nullptr);
  EXPECT_EQ(orig_tackpts[0], tackpts[0])
      << "BackPass should never touch the first point in tackpts";
  EXPECT_EQ(orig_alpha, alpha) << "In theory, should make no change to alpha";
  alpha = 0.5;

  // Now, for kicks, try with endpt on the goal line:
  nextpt << 0.0, 10.0;
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha, nullptr, nullptr);
  EXPECT_EQ(orig_tackpts[0], tackpts[0])
      << "BackPass should never touch the first point in tackpts";
  EXPECT_EQ(orig_alpha, alpha) << "In theory, should make no change to alpha";
  alpha = 0.5;

  // Slight upwind should result in us being pushed downwind.
  // By putting nextpt on the gate, we ignore the cost of that line, which is
  // relevant because otherwise going more downind on the first leg will make
  // you go more upwind on the second.
  nextpt << 0.0, 10.0;
  LinePlan::BackPass(gate, nextpt, /*winddir=*/-1.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha, nullptr, nullptr);
  EXPECT_LT(orig_alpha, alpha) << "Expected alpha to increase to move downwind";
  alpha = 0.5;

  // A small difference in our current yaw should cause a tweak to the overall
  // trajectory:
  cur_yaw = M_PI_4;
  nextpt << 0.0, 50.0; // Make very far away; otherwise any changes in initial
                       // turn will be counteracted by the second turn.
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha, nullptr, nullptr);
  EXPECT_LT(orig_alpha, alpha) << "Expected alpha to increase to turn less";
  alpha = 0.5;

  // A deviation in alpha should be corrected:
  alpha = 0.45;
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha, nullptr, nullptr);
  EXPECT_LT(orig_alpha, alpha)
      << "Expected alpha to increase to pass through center of the gate";
  alpha = 0.5;

  // Presence of an obstacle should shift the line:
  obstacles.push_back(Polygon({{3.0, 5.0}, {4.0, 13.0}, {2.5, 12.0}}));
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha, nullptr, nullptr);
  EXPECT_GT(orig_alpha, alpha)
      << "Expected alpha to decrease to avoid obstacle";
  alpha = 0.5;
}

// Similar to above, but now exercising multiple-point stuff (i.e., exercising
// LinePairCost)
class BackPassTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    orig_tackpts = tackpts;
    orig_alpha = alpha;
  }
  virtual void TearDown() {}

  void BackPass(double wind_dir, double step) {
    LinePlan::BackPass(gate, nextpt, wind_dir, obstacles, cur_yaw, step,
                       &tackpts, &alpha, nullptr, nullptr);
  }

  std::vector<Point> tackpts{{0.0, 0.0}, {10.0, 0.0}};
  std::vector<Polygon> obstacles; // Start with none.
  double alpha = 0.5;
  std::pair<Point, Point> gate{{15.0, -1.0}, {15.0, 1.0}};
  double cur_yaw = 0.0;
  Point nextpt{30.0, 0.0}; // We should just continue straight through the gate.

  // For comparisons:
  std::vector<Point> orig_tackpts = tackpts;
  double orig_alpha = alpha;
}; // class BackPassTest

TEST_F(BackPassTest, DownwindAtEquilibrium) {

  // Start with stable conditions
  BackPass(0.0, 1.0);
  EXPECT_EQ(orig_tackpts[0], tackpts[0])
      << "BackPass should never touch the first point in tackpts";
  EXPECT_EQ(orig_tackpts[1], tackpts[1])
      << "Middle point should've been stable straight downwind";
  EXPECT_EQ(orig_alpha, alpha)
      << "alpha should've been stable under straight downind condition";
}

TEST_F(BackPassTest, ObstacleAdjustsPath) {
  // Obstacle below the path should create elbow:
  obstacles.push_back(
      Polygon({{7.0, -5.0}, {8.0, -10.0}, {12.0, -10.0}, {7.0, -5.0}}));
  BackPass(0.0, 1.0);
  EXPECT_LT(orig_tackpts[1].y(), tackpts[1].y())
      << "Middle point should've moved up to avoid obstacle";
  EXPECT_LT(orig_alpha, alpha)
      << "alpha should've moved up to avoid osbtacle";
}

TEST_F(BackPassTest, UpwindPathTurnsDown) {
  // Going upwind should result in us tacking; however, because this optimizeer
  // is gradient descent based, a single step should just send us more downwind.
  tackpts[1].y() += 0.1;
  // Bring in nextpt so it doesn't affect the optimization.
  nextpt << 15.0, 0.0;
  orig_tackpts = tackpts;
  BackPass(M_PI - 0.1, 1e-3);
  EXPECT_LT(orig_tackpts[1].y(), tackpts[1].y())
      << "Middle point should've moved up to cause tack";
  // Because of the simple gradient descent nature of the BackPass function, we
  // will end up just pushing this more downwind.
  EXPECT_LT(orig_alpha, alpha) << "alpha should've moved up to avoid upwind";
}

TEST_F(BackPassTest, PreseededUpwindPathWorks) {
  // Going upwind should result in us tacking; however, because this optimizeer will get suck on a
  nextpt << 15.0, 0.0;
  tackpts[1].y() += 3.0;
  orig_tackpts = tackpts;
  BackPass(M_PI, 1.0);
  EXPECT_LT(orig_tackpts[1].y(), tackpts[1].y())
      << "Middle point should've moved up to cause tack";
  EXPECT_GT(orig_alpha, alpha)
      << "alpha should've moved up to avoid upwind";
}

TEST_F(BackPassTest, UpwindMultipointTack) {
  // Going upwind should induce tacking, but with many points.
  nextpt << 15.0, 0.0;
  tackpts = {{0.0, 0.0}, {5.0, 1.0}, {10.0, -1.0}};
  orig_tackpts = tackpts;
  BackPass(M_PI, 1.0);
  EXPECT_LT(orig_tackpts[1].y(), tackpts[1].y())
      << "First point should've moved up a tick";
  EXPECT_GT(orig_tackpts[2].y(), tackpts[2].y())
      << "Second point should've moved down a tick";
}

TEST_F(BackPassTest, OutOfAlignmentInitial) {
  // If we have a standard, downwind conditions, should correct towards center
  tackpts[1].y() = 1.0;
  orig_tackpts = tackpts;
  alpha = 0.3;
  orig_alpha = alpha;
  BackPass(0.0, 1.0);
  EXPECT_GT(orig_tackpts[1].y(), tackpts[1].y())
      << "Middle point should've moved down to straighten out";
  EXPECT_LT(orig_alpha, alpha)
      << "alpha should've moved towards center";
}

class GenerateHypothesesTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  Point startpt{0.0, 0.0};
  Point endpt{10.0, 10.0};
  double winddir = 0.0;

  std::vector<std::vector<Point>> GenerateHypotheses(int Npts) {
    std::vector<std::vector<Point>> paths;
    LinePlan::GenerateHypotheses(startpt, endpt, winddir, Npts, &paths);
    return paths;
  }

  // Return whether the given tack is a starboard tack
  bool IsStarboard(Point start, Point end) {
    return util::norm_angle(util::atan2(end - start) - winddir) < 0.0;
  }
};

TEST_F(GenerateHypothesesTest, ZeroPointPath) {
  auto pts = GenerateHypotheses(0);
  // pts should contain one path that is a vector of length 1.
  ASSERT_EQ(1, pts.size()) << "should've returned exactly one path";
  ASSERT_EQ(1, pts[0].size()) << "the path should contain exactly one point";
  EXPECT_EQ(startpt, pts[0][0])
      << "The one point on the path should be startpt";
}

TEST_F(GenerateHypothesesTest, OnePointDownwind) {
  auto pts = GenerateHypotheses(1);
  ASSERT_EQ(1, pts.size());
  ASSERT_EQ(2, pts[0].size());
  EXPECT_EQ(startpt, pts[0][0]);
  EXPECT_EQ(0.5 * (startpt + endpt), pts[0][1]);
}

TEST_F(GenerateHypothesesTest, OnePointStraightUpwind) {
  winddir = -3.0 * M_PI_4;
  auto pts = GenerateHypotheses(1);
  ASSERT_EQ(3, pts.size());
  ASSERT_EQ(2, pts[0].size());
  ASSERT_EQ(2, pts[1].size());
  ASSERT_EQ(2, pts[2].size());

  EXPECT_EQ(0.5 * (startpt + endpt), pts[0][1]);
  // Tacks should be of equal length
  EXPECT_EQ((pts[1][1] - pts[1][0]).norm(), (endpt - pts[1][1]).norm());
  EXPECT_EQ((pts[2][1] - pts[2][0]).norm(), (endpt - pts[2][1]).norm());
  // And check that points are on the expected side of the tack
  EXPECT_TRUE(IsStarboard(pts[1][0], pts[1][1]));
  EXPECT_FALSE(IsStarboard(pts[2][0], pts[2][1]));
}

// Test that it generates something reasonable for not straight upwind paths
TEST_F(GenerateHypothesesTest, OnePointSlightlyUpwind) {
  winddir = M_PI;
  auto pts = GenerateHypotheses(1);
  ASSERT_EQ(3, pts.size());
  ASSERT_EQ(2, pts[0].size());
  ASSERT_EQ(2, pts[1].size());
  ASSERT_EQ(2, pts[2].size());

  EXPECT_EQ(0.5 * (startpt + endpt), pts[0][1]);
  // Tacks should be of different lengths, and in a different order for the two
  // options:
  EXPECT_GT((pts[1][1] - pts[1][0]).norm(), (endpt - pts[1][1]).norm());
  EXPECT_LT((pts[2][1] - pts[2][0]).norm(), (endpt - pts[2][1]).norm());
  // Long legs and short legs should be equal
  EXPECT_NEAR((pts[1][1] - pts[1][0]).norm(), (endpt - pts[2][1]).norm(), 1e-5);
  EXPECT_NEAR((pts[2][1] - pts[2][0]).norm(), (endpt - pts[1][1]).norm(), 1e-5);
  // And check that points are on the expected side of the tack
  EXPECT_TRUE(IsStarboard(pts[1][0], pts[1][1]));
  EXPECT_FALSE(IsStarboard(pts[2][0], pts[2][1]));
  EXPECT_FALSE(IsStarboard(pts[1][1], endpt));
  EXPECT_TRUE(IsStarboard(pts[2][1], endpt));
}

// Test reaosnableness for multi-point paths
TEST_F(GenerateHypothesesTest, TwoPointSlightlyUpwind) {
  winddir = M_PI;
  auto pts = GenerateHypotheses(2);
  ASSERT_EQ(3, pts.size());
  ASSERT_EQ(3, pts[0].size());
  ASSERT_EQ(3, pts[1].size());
  ASSERT_EQ(3, pts[2].size());

  EXPECT_EQ(0.75 * startpt + 0.25 * endpt, pts[0][1]);
  EXPECT_EQ(0.25 * startpt + 0.75 * endpt, pts[0][2]);
  double longlen = 2.0 * (pts[1][1] - pts[1][0]).norm();
  double shortlen = 2.0 * (pts[2][1] - pts[2][0]).norm();
  // Check leg lengths:
  EXPECT_NEAR(longlen, (endpt - pts[1][2]).norm() * 2.0, 1e-5);
  EXPECT_NEAR(shortlen, (pts[1][2] - pts[1][1]).norm(), 1e-5);
  EXPECT_NEAR(longlen, (pts[1][1] - pts[1][0]).norm() * 2.0, 1e-5);
  EXPECT_NEAR(shortlen, (endpt - pts[2][2]).norm() * 2.0, 1e-5);
  EXPECT_NEAR(longlen, (pts[2][2] - pts[2][1]).norm(), 1e-5);
  EXPECT_NEAR(shortlen, (pts[2][1] - pts[2][0]).norm() * 2.0, 1e-5);
  // And check that points are on the expected side of the tack
  EXPECT_TRUE(IsStarboard(pts[1][0], pts[1][1]));
  EXPECT_FALSE(IsStarboard(pts[2][0], pts[2][1]));
  EXPECT_FALSE(IsStarboard(pts[1][1], pts[1][2]));
  EXPECT_TRUE(IsStarboard(pts[2][1], pts[2][2]));
  EXPECT_TRUE(IsStarboard(pts[1][2], endpt));
  EXPECT_FALSE(IsStarboard(pts[2][2], endpt));
}

TEST_F(GenerateHypothesesTest, ThreePointSlightlyUpwind) {
  winddir = M_PI;
  auto pts = GenerateHypotheses(3);
  ASSERT_EQ(3, pts.size());
  ASSERT_EQ(4, pts[0].size());
  ASSERT_EQ(4, pts[1].size());
  ASSERT_EQ(4, pts[2].size());

  EXPECT_LT(((5.0 * startpt + endpt) / 6.0 - pts[0][1]).norm(), 1e-6);
  EXPECT_EQ((startpt + endpt) / 2.0, pts[0][2]);
  EXPECT_EQ((startpt + 5.0 * endpt) / 6.0, pts[0][3]);
  double longlen = 2.0 * (pts[1][1] - pts[1][0]).norm();
  double shortlen = 2.0 * (pts[2][1] - pts[2][0]).norm();
  // Check leg lengths:
  EXPECT_NEAR(longlen, (pts[1][1] - pts[1][0]).norm() * 2.0, 1e-5);
  EXPECT_NEAR(shortlen, (pts[1][2] - pts[1][1]).norm(), 1e-5);
  EXPECT_NEAR(longlen, (pts[1][3] - pts[1][2]).norm(), 1e-5);
  EXPECT_NEAR(shortlen, (endpt - pts[1][3]).norm() * 2.0, 1e-5);

  EXPECT_NEAR(shortlen, (pts[2][1] - pts[2][0]).norm() * 2.0, 1e-5);
  EXPECT_NEAR(longlen, (pts[2][2] - pts[2][1]).norm(), 1e-5);
  EXPECT_NEAR(shortlen, (pts[2][3] - pts[2][2]).norm(), 1e-5);
  EXPECT_NEAR(longlen, (endpt - pts[2][3]).norm() * 2.0, 1e-5);
  // And check that points are on the expected side of the tack
  EXPECT_TRUE(IsStarboard(pts[1][0], pts[1][1]));
  EXPECT_FALSE(IsStarboard(pts[1][1], pts[1][2]));
  EXPECT_TRUE(IsStarboard(pts[1][2], pts[1][3]));
  EXPECT_FALSE(IsStarboard(pts[1][3], endpt));

  EXPECT_FALSE(IsStarboard(pts[2][0], pts[2][1]));
  EXPECT_TRUE(IsStarboard(pts[2][1], pts[2][2]));
  EXPECT_FALSE(IsStarboard(pts[2][2], pts[2][3]));
  EXPECT_TRUE(IsStarboard(pts[2][3], endpt));
}

// Test the actual optimization, now that all the basis is out of the way.
class OptimizerTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  void OptimizeTacks() {
    orig_tackpts = tackpts;
    orig_alpha = alpha;
    LinePlan::OptimizeTacks(gate, nextpt, winddir, obstacles, cur_yaw, &tackpts,
                            &alpha, nullptr, nullptr);
  }

  void FindPath() {
    orig_tackpts = tackpts;
    orig_alpha = alpha;
    LinePlan::FindPath(startpt, gate, nextpt, winddir, obstacles, cur_yaw,
                       &tackpts, &alpha);
  }

  Point startpt{0.0, 0.0};
  std::pair<Point, Point> gate{{20.0, -3.0}, {20.0, 3.0}};
  Point nextpt{40.0, 0.0};
  double winddir = 0.0, cur_yaw = 0.0;
  std::vector<Polygon> obstacles;
  std::vector<Point> tackpts{{0.0, 0.0}, {9.0, 0.0}};
  double alpha = 0.5;
  std::vector<Point> orig_tackpts;
  double orig_alpha;
}; // class OptimizerTest

// Confirm that a basic run downwind will work, given a bit of perturbation at
// the start.
TEST_F(OptimizerTest, DownwindStabilizes) {
  alpha = 0.7;
  tackpts[1] << 9.0, 5.0;
  OptimizeTacks();
  EXPECT_NEAR(0.5, alpha, 1e-1) << "Alpha should've approached 0.5";
  EXPECT_NEAR(tackpts[1].y(), 0.0, 4.9)
      << "Center point should've moved to x-axis";
}

// Test full optimizer in trivial conditions
TEST_F(OptimizerTest, DownwindFullOpt) {
  alpha = 0.0;
  tackpts.clear();
  FindPath();
  EXPECT_NEAR(0.5, alpha, 1e-1) << "Alpha should've approached 0.5";
  EXPECT_EQ(1.0, tackpts.size());
  LOG(INFO) << "tackpts: ";
  for (const auto &pt : tackpts) {
    LOG(INFO) << pt.transpose();
  }
}

// Test full optimizer with a basic obstacle
TEST_F(OptimizerTest, DownwindObstacleOpt) {
  alpha = 0.0;
  tackpts.clear();
  // Add obstacle below bath; should result in at least one extra point
  // and alpha being moved up:
  obstacles.push_back(Polygon({{11.0, 0.0}, {9.0, 0.0}, {10.0, -1.0}}));
  FindPath();
  EXPECT_LT(0.5, alpha) << "Alpha should've increased above 0.5";
  EXPECT_LE(2.0, tackpts.size())
      << "Should have at least one tack point to avoid obstacles";
  LOG(INFO) << "alpha: " << alpha << " tackpts: ";
  for (const auto &pt : tackpts) {
    LOG(INFO) << pt.transpose();
  }
}

// Test obstacle-free upwind leg:
TEST_F(OptimizerTest, UpwindBasicOpt) {
  winddir = M_PI;
  alpha = 0.0;
  tackpts.clear();
  FindPath();
  EXPECT_EQ(2.0, tackpts.size()) << "Should only need one tack on upwind";
  LOG(INFO) << "alpha: " << alpha << " tackpts: ";
  for (const auto &pt : tackpts) {
    LOG(INFO) << pt.transpose();
  }
}

// Test upwind leg, forcing more tacks due to obstacles:
TEST_F(OptimizerTest, UpwindForceTacks) {
  winddir = M_PI;
  alpha = 0.0;
  tackpts.clear();
  // Top and bottom barriers, should necessitate just 2 tacks
  obstacles.push_back(
      Polygon({{0.0, 10.0}, {20.0, 10.0}, {20.0, 50.0}, {0.0, 50.0}}));
  obstacles.push_back(
      Polygon({{0.0, -10.0}, {0.0, -50.0}, {20.0, -50.0}, {20.0, -10.0}}));
  FindPath();
  EXPECT_LE(3.0, tackpts.size()) << "Should need multiple tacks to avoid shore";
  LOG(INFO) << "alpha: " << alpha << " tackpts: ";
  for (const auto &pt : tackpts) {
    LOG(INFO) << pt.transpose();
  }
}

// Example with funny business:
// Reminder: Line ~490 (second calculation of StraightLineCost in
// SingleLineCost) should be commented out.
TEST_F(OptimizerTest, DISABLED_Example) {
  gate.first << 100.0, -3.0;
  gate.second << 100.0, 3.0;
  winddir = 3.0;
//  winddir = M_PI_4;
  alpha = 0.0;
  tackpts.clear();
  // Top and bottom small boxes:
  obstacles.push_back(
      Polygon({{15.0, 15.0}, {30.0, 15.0}, {30.0, 150.0}, {15.0, 150.0}}));
  obstacles.push_back(
      Polygon({{70.0, -15.0}, {70.0, -150.0}, {85.0, -150.0}, {85.0, -15.0}}));
  obstacles.push_back(
      Polygon({{45.0, 5.0}, {45.0, -5.0}, {55.0, -5.0}, {55.0, 5.0}}));
//  obstacles.push_back(
//      Polygon({{45.0, 0.0}, {50.0, -5.0}, {55.0, 0.0}, {50.0, 5.0}}));
  FindPath();
  LOG(INFO) << "alpha: " << alpha << " tackpts: ";
  for (const auto &pt : tackpts) {
    LOG(INFO) << pt.transpose();
  }
}

}  // namespace testing
}  // namespace control
}  // namespace sailbot
