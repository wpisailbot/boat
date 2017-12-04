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
  TryTurnCost(0.0, 1.0, 0.0, 1.0, -1.0, 1.0, "Basic conditions, no upwind");
  TryTurnCost(0.0, -1.0, 0.0, 1.0, 1.0, -1.0, "Reverse direction, no upwind");
  double expcost = 1.0 + (kT - 1.0) * M_PI / 4.0;
  TryTurnCost(0.0, 1.0, M_PI, expcost, -kT, 1.0, "Out of irons, 1 rad turn");
  TryTurnCost(0.0, 0.5, M_PI, kT * 0.5, -kT, kT, "Upwind solely in irons");
  TryTurnCost(2.0, 2.5, 2.0 - M_PI, kT * 0.5, -kT, kT,
              "Upwind solely in irons, nonzero start");
  expcost = 2.0 + (kT - 1.0) * M_PI / 2.0;
  TryTurnCost(-1.0, 1.0, M_PI, expcost, -1.0, 1.0, "Complete tack");
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
  Eigen::Vector2d startline(1, 0), endline(1, 2), startpt(0, 1), nextpt(2, 1);
  double alpha = 0.5;
  double winddir = 0.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, nextpt, winddir,
                           &cost1, &dcostdalpha);
  startpt.y() -= 1.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, nextpt, winddir,
                           &cost2, &dcostdalpha);
  startpt.y() += 1.0;
  EXPECT_LT(cost1, cost2)
      << "Moving the start point farther away should increase cost";

  // Cost should also increase when lengths stay the same but
  // the angle change increases.
  startpt << 1.0, 0.0;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, nextpt, winddir,
                           &cost2, &dcostdalpha);
  startpt << 0.0, 1.0;
  EXPECT_LT(cost1, cost2)
      << "Increasing required turn should increase cost";

  // And increase if we are forced to travel upwind.
  winddir = M_PI - 0.2;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha, nextpt, winddir,
                           &cost2, &dcostdalpha);
  winddir = 0.0;
  EXPECT_LT(cost1, cost2)
      << "Going more upwind should increase cost";

  // And we should get sane derivatives out of alpha.
  double eps = 0.0001;
  LinePlan::SingleLineCost(startline, endline, startpt, alpha + eps, nextpt,
                           winddir, &cost2, &dcostdalpha);
  EXPECT_NEAR(dcostdalpha, (cost2 - cost1) / eps, 0.001)
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

}  // namespace testing
}  // namespace control
}  // namespace sailbot
