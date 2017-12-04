#pragma once
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "math/polygon.h"
#include <Eigen/Core>
#include <mutex>
#include "gtest/gtest_prod.h"

namespace sailbot {
namespace control {

namespace testing {
class LinePlanUtilTest_TurnCostTest_Test;
class LinePlanUtilTest_CrossFinishTest_Test;
class LinePlanUtilTest_SingleLineCostTest_Test;
class LinePlanUtilTest_StraightLineTest_Test;
class LinePlanUtilTest_LinePairCostTest_Test;
void TryTurnCost(double startheading, double endheading, double winddir,
                 double expcost, double expdstart, double expdend,
                 const char *desc);
void TryCrossFinishCost(double alpha, double expcost, double expdalpha,
                        const char *desc);
void TryStraightLineCost(double len, double heading, double winddir,
                         double expcost, double expdlen, double, const char *);
} // namespace testing

class LinePlan : public Node {
 public:
  LinePlan();

  void Iterate() override;
 private:
  constexpr static float dt = 2.0;
  // Cost, used in TurnCost, of traversing the upwind no-go zones
  // relative to typical turns.
  constexpr static float kTackCost = 5.0;
  // Cost per unit length of a line, cost / meter
  constexpr static float kLengthCost = 0.01;

  // None of these functions below account for obstacles.
  static void SingleLineCost(const Eigen::Vector2d &startline,
                             const Eigen::Vector2d &endline,
                             const Eigen::Vector2d &startpt, double alpha,
                             const Eigen::Vector2d &nextpt, double winddir,
                             double *cost, double *dcostdalpha);
  static void LinePairCost(const Eigen::Vector2d &startpt,
                           const Eigen::Vector2d &endpt,
                           const Eigen::Vector2d &turnpt, double preheading,
                           double postheading, double winddir, double *cost,
                           Eigen::Vector2d *dcostdturnpt);
  static void CrossFinishCost(double alpha, double *cost, double *dcostdalpha);
  static void TurnCost(double startheading, double endheading, double winddir,
                       double *cost, double *dcostdstart, double *dscostdend);
  static void StraightLineCost(double len, double heading, double winddir,
                               double *cost, double *dcostdlen,
                               double *dcostdheading);

  // The current reference point as the origin for the meters reference frame.
  // For doing coordinate conversions, +latitude=+y, +longitude=+x, so it's
  // just a matter of scaling indivdual axes.
  // lonlat_ref_.x() = longitude corresponding to 0 meters x, some for lat/y.
  // Unless otherwise specified, all values in this entire class are in
  // the meters system, rather than lat/lon.
  Point lonlat_ref_;

  // Various representations of obstacles:
  // The generally constant representation of obstacles of lon/lat coordinate
  // pairs instead of meters.
  std::vector<Polygon> obstacles_lonlat_;
  std::vector<Polygon> obstacles_;

  FRIEND_TEST(testing::LinePlanUtilTest, TurnCostTest);
  FRIEND_TEST(testing::LinePlanUtilTest, CrossFinishTest);
  FRIEND_TEST(testing::LinePlanUtilTest, SingleLineCostTest);
  FRIEND_TEST(testing::LinePlanUtilTest, StraightLineTest);
  FRIEND_TEST(testing::LinePlanUtilTest, LinePairCostTest);
  friend void testing::TryTurnCost(double, double, double, double, double,
                                   double, const char *);
  friend void testing::TryCrossFinishCost(double, double, double, const char *);
  friend void testing::TryStraightLineCost(double len, double heading,
                                           double winddir, double expcost,
                                           double expdlen, double,
                                           const char *);
};

}  // control
}  // sailbot
