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
  double cost, dstart, dend;
  LinePlan::TurnCost(startheading, endheading, winddir, &cost, &dstart, &dend);
  EXPECT_EQ(expcost, cost) << desc << ": Incorrect cost";
  EXPECT_EQ(expdstart, dstart) << desc << ": Incorrect dcostdstart";
  EXPECT_EQ(expdend, dend) << desc << ": Incorrect dcostdend";
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
                           nextpt, winddir, &cost1, &dcostdalpha);
  startpt.y() -= 1.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha);
  startpt.y() += 1.0;
  EXPECT_LT(cost1, cost2)
      << "Moving the start point farther away should increase cost";

  // Cost should also increase when lengths stay the same but
  // the middle angle change increases.
  startpt << 0.0, 1.0;
  preheading = 0.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha);
  startpt << 1.0, 0.0;
  preheading = M_PI_2;
  EXPECT_LT(cost1, cost2)
      << "Increasing required turn should increase cost";
  // And changing the initial angle:
  preheading = 0.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha);
  preheading = M_PI_2;
  EXPECT_LT(cost1, cost2)
      << "Increasing required initial turn should increase cost";

  // And increase if we are forced to travel upwind.
  winddir = -M_PI_2 - 0.2;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha);
  winddir = 0.0;
  EXPECT_LT(cost1, cost2)
      << "Going more upwind should increase cost";

  // And we should get sane derivatives out of alpha.
  double eps = 0.0001;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha + eps, preheading,
                           nextpt, winddir, &cost2, &dcostdalpha);
  LOG(INFO) << "Cost1: " << cost1 << " cost2: " << cost2;
  EXPECT_NEAR(dcostdalpha, (cost2 - cost1) / eps, 0.005)
      << "Numeric and analytic derivatives should be close";
}

void TryStraightLineCost(double len, double heading, double winddir,
                         double expcost, double expdlen, double expdheading,
                         const char *desc) {
  double cost, dlen, dheading;
  LinePlan::StraightLineCost(len, heading, winddir, &cost, &dlen, &dheading);
  EXPECT_EQ(expcost, cost) << desc << ": Incorrect cost";
  EXPECT_EQ(expdlen, dlen) << desc << ": Incorrect costdlen";
  EXPECT_EQ(expdheading, dheading) << desc << ": Incorrect costdheading";

  double dcost;
  double eps = 1e-3;
  LinePlan::StraightLineCost(len + eps, heading, winddir, &dcost, &dlen,
                             &dheading);
  EXPECT_NEAR(dlen, (dcost - cost) / eps, 0.001) << desc
                                                 << ": Bad empirical dlen";

  LinePlan::StraightLineCost(len, heading + eps, winddir, &dcost, &dlen,
                             &dheading);
  EXPECT_NEAR(dheading, (dcost - cost) / eps, 0.001)
      << desc << ": Bad empirical dheading";
}

TEST(LinePlanUtilTest, StraightLineTest) {
  constexpr double kL = LinePlan::kLengthCost;

  double expcost = kL;
  double expdlen = kL;
  double expdheading = 0.0;
  TryStraightLineCost(1.0, 0.0, 0.0, expcost, expdlen, expdheading,
                      "Downwind leg");

  double heading = 0.5;
  expcost = kL / heading;
  expdlen = kL / heading;
  expdheading = -kL / (heading * heading);
  TryStraightLineCost(1.0, heading, M_PI, expcost, expdlen, expdheading,
                      "Upwind leg");

  heading *= -1.0;
  expdheading *= -1.0;
  TryStraightLineCost(1.0, heading, M_PI, expcost, expdlen, expdheading,
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
                         winddir, &cost, &dcostdturn);

  // First, just check that empirically measured derivatives match the actual
  // derivatives.
  double eps = 1e-4, tol = 1e-3;
  LinePlan::LinePairCost(startpt, endpt, turnpt + Eigen::Vector2d(eps, 0.0),
                         preheading, postheading, winddir, &dcost, &dcostdturn);
  EXPECT_NEAR(dcostdturn.x(), (dcost - cost) / eps, tol)
      << "X partial doesn't match numeric/analytic";

  LinePlan::LinePairCost(startpt, endpt, turnpt + Eigen::Vector2d(0.0, eps),
                         preheading, postheading, winddir, &dcost, &dcostdturn);
  EXPECT_NEAR(dcostdturn.y(), (dcost - cost) / eps, tol)
      << "Y partial doesn't match numeric/analytic";

  // Moving endpt farther out should increase costs
  endpt *= 2.0;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn);
  endpt /= 2.0;
  EXPECT_LT(cost, dcost) << "Increasing segment lengths should increase cost";

  // Shifting endpt to create angle should increase cost.
  endpt << 0.0, 2.0;
  postheading = 3.0 * M_PI_4;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn);
  endpt << 2.0, 2.0;
  postheading = M_PI_4;
  EXPECT_LT(cost, dcost) << "Creating elbow should increase cost";

  // Adjusting postheading should incur a cost:
  postheading *= -1.0;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn);
  postheading *= -1.0;
  EXPECT_LT(cost, dcost) << "Adding turn at end should increase cost";

  // Adjusting preheading should incur a cost:
  preheading *= -1.0;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn);
  preheading *= 1.0;
  EXPECT_LT(cost, dcost) << "Adding turn at beginning should increase cost";

  // Forcing us to go through the wind should increase cost:
  winddir = M_PI;
  LinePlan::LinePairCost(startpt, endpt, turnpt, preheading, postheading,
                         winddir, &dcost, &dcostdturn);
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
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_EQ(orig_tackpts[0], tackpts[0])
      << "BackPass should never touch the first point in tackpts";
  EXPECT_EQ(orig_alpha, alpha) << "In theory, should make no change to alpha";
  alpha = 0.5;

  // Now, for kicks, try with endpt on the goal line:
  nextpt << 0.0, 10.0;
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha);
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
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_LT(orig_alpha, alpha) << "Expected alpha to increase to move downwind";
  alpha = 0.5;

  // A small difference in our current yaw should cause a tweak to the overall
  // trajectory:
  cur_yaw = M_PI_4;
  nextpt << 0.0, 50.0; // Make very far away; otherwise any changes in initial
                       // turn will be counteracted by the second turn.
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_LT(orig_alpha, alpha) << "Expected alpha to increase to turn less";
  alpha = 0.5;

  // A deviation in alpha should be corrected:
  alpha = 0.45;
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_LT(orig_alpha, alpha)
      << "Expected alpha to increase to pass through center of the gate";
  alpha = 0.5;

  // Presence of an obstacle should shift the line:
  obstacles.push_back(Polygon({{3.0, 5.0}, {4.0, 13.0}, {2.5, 12.0}}));
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_GT(orig_alpha, alpha)
      << "Expected alpha to decrease to avoid obstacle";
  alpha = 0.5;
}

// Similar to above, but now exercising multiple-point stuff (i.e., exercising
// LinePairCost)
TEST(LinePlanUtilTest, BackPassTestMultiPoint) {
  std::vector<Point> tackpts({{0.0, 0.0}, {10.0, 0.0}});
  std::vector<Polygon> obstacles; // Start with none.
  double alpha = 0.5;
  std::pair<Point, Point> gate({{15.0, -1.0}, {15.0, 1.0}});
  double cur_yaw = 0.0;
  Point nextpt(30.0, 0.0); // We should just continue straight through the gate.

  // For comparisons:
  std::vector<Point> orig_tackpts = tackpts;
  double orig_alpha = alpha;

  // Start with stable conditions
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_EQ(orig_tackpts[0], tackpts[0])
      << "BackPass should never touch the first point in tackpts";
  EXPECT_EQ(orig_tackpts[1], tackpts[1])
      << "Middle point should've been stable straight downwind";
  EXPECT_EQ(orig_alpha, alpha)
      << "alpha should've been stable under straight downind condition";
  alpha = 0.5;

  // Obstacle below the path should create elbow:
  obstacles.push_back(
      Polygon({{7.0, -5.0}, {8.0, -10.0}, {12.0, -10.0}, {7.0, -5.0}}));
  LinePlan::BackPass(gate, nextpt, /*winddir=*/0.0, obstacles, cur_yaw,
                     /*step=*/1.0, &tackpts, &alpha);
  EXPECT_LT(orig_tackpts[1].y(), tackpts[1].y())
      << "Middle point should've moved up to avoid obstacle";
  EXPECT_LT(orig_alpha, alpha)
      << "alpha should've moved up to avoid osbtacle";
  alpha = 0.5;
}

}  // namespace testing
}  // namespace control
}  // namespace sailbot
