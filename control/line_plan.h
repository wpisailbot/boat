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
class LinePlanUtilTest_ObstacleCostTest_Test;
class LinePlanUtilTest_BackPassTestOnePoint_Test;
class LinePlanUtilTest_BackPassTestMultiPoint_Test;
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
  typedef Eigen::Vector2d Vector2d;

  LinePlan();

  void Iterate() override;

  /**
   * Return the meters-based coordinate frame value for the longitude/latitude
   * from lonlat.
   */
  Point LonLatToFrame(const Point &lonlat) {
    return lonlat_scale_.cwiseProduct(lonlat - lonlat_ref_);
  }

  // Reset _all_ of our meters-based using newlonlatref as the origin
  // longitude/latitude and basis for scaling.
  void ResetRef(const Point &newlonlatref);

 private:
  constexpr static float dt = 2.0;
  // Cost, used in TurnCost, of traversing the upwind no-go zones
  // relative to typical turns.
  constexpr static float kTackCost = 5.0;
  // The relative cost of being near obstacles.
  // If kObstacleCost = 1, then the cost will be integral e^-x dl with
  // x the distance to a given obstacle, l being the line we are
  // following.
  constexpr static float kObstacleCost = 100.0;
  // Cost per unit length of a line, cost / meter
  constexpr static float kLengthCost = 0.01;
  // Default length of a gate, in meters.
  constexpr static float kGateWidth = 10.0;

  // None of these functions below account for obstacles.
  static void SingleLineCost(const Vector2d &startline, const Vector2d &endline,
                             const Vector2d &startpt, double alpha,
                             double preheading, const Vector2d &nextpt,
                             double winddir, double *cost, double *dcostdalpha);
  static void LinePairCost(const Vector2d &startpt,
                           const Vector2d &endpt,
                           const Vector2d &turnpt, double preheading,
                           double postheading, double winddir, double *cost,
                           Vector2d *dcostdturnpt);
  static void CrossFinishCost(double alpha, double *cost, double *dcostdalpha);
  static void TurnCost(double startheading, double endheading, double winddir,
                       double *cost, double *dcostdstart, double *dscostdend);
  static void StraightLineCost(double len, double heading, double winddir,
                               double *cost, double *dcostdlen,
                               double *dcostdheading);

  // Computes the cost associated with the line segment defined by (start, end)
  // and computes the derivatives of the cost with respect to the start of the
  // line.
  static void ObstacleCost(const Vector2d &start,
                           const Vector2d &end,
                           const std::vector<Polygon> &obstacles, double *cost,
                           Vector2d *dcostdstart);
  // Same as the similarly named functions above, but account for obstacles.
  static void SingleLineCost(const Vector2d &startline, const Vector2d &endline,
                             const Vector2d &startpt, double alpha,
                             double preheading, const Vector2d &nextpt,
                             double winddir,
                             const std::vector<Polygon> &obstacles,
                             double *cost, double *dcostdalpha);
  static void LinePairCost(const Vector2d &startpt,
                           const Vector2d &endpt,
                           const Vector2d &turnpt, double preheading,
                           double postheading, double winddir,
                           const std::vector<Polygon> &obstacles, double *cost,
                           Vector2d *dcostdturnpt);

  // Perform a backwards pass to update tackpts/alpha
  static void BackPass(const std::pair<Vector2d, Vector2d> &gate,
                       const Vector2d &nextpt, double winddir,
                       const std::vector<Polygon> &obstacles, double cur_yaw,
                       double step, std::vector<Vector2d> *tackpts,
                       double *alpha);

  // Optimizes tackpts/alpha. You should seed tackpts/alpha with
  // some sane initial values.
  void OptimizeTacks(const std::pair<Vector2d, Vector2d> &gate,
                     const Vector2d &nextpt, double winddir,
                     const std::vector<Polygon> &obstacles, double cur_yaw,
                     std::vector<Vector2d> *tackpts, double *alpha);

  /**
   * Update the transformations on the obstacles, assuming some change to either
   * obstacles_lonlat_ or to the transformations.
   */
  void UpdateObstacles();

  /**
   * Update the waypoints_ from the waypoints_lonlat_ vector.
   */
  void UpdateWaypoints();

  std::mutex data_mutex_;

  // Potentially useful state information
  std::atomic<double> yaw_;
  std::atomic<double> wind_dir_;
  Point boat_pos_;

  // The current reference point as the origin for the meters reference frame.
  // For doing coordinate conversions, +latitude=+y, +longitude=+x, so it's
  // just a matter of scaling indivdual axes.
  // lonlat_ref_.x() = longitude corresponding to 0 meters x, some for lat/y.
  // Unless otherwise specified, all values in this entire class are in
  // the meters system, rather than lat/lon.
  Point lonlat_ref_;
  // Stores the current scaling to convert from degrees lat/lon to meters.
  Point lonlat_scale_; // meters / degree lon/lat

  // Various representations of obstacles:
  // The generally constant representation of obstacles of lon/lat coordinate
  // pairs instead of meters.
  std::vector<Polygon> obstacles_lonlat_;
  std::vector<Polygon> obstacles_;

  // Goal waypoints
  // TODO(james): Make mode adjust to actually handle all reasonable inputs.
  std::vector<Point> waypoints_lonlat_;
  // Store ``waypoints'' as pairs of points representing the line we must cross.
  // The order of the pair in theory could matter, although currently it should
  // not.
  // Nominally, the first value in the pair shall be the ``true'' waypoint.
  // In the algorithms, the path shall always be planned under
  // the assumption that we shall be following a path from our
  // current position, to waypoints_[0]... and terminated at waypoints_.back()
  // At any given instant, we are straing to sail to waypoint next_waypoint_.
  std::vector<std::pair<Point, Point>> waypoints_;
  std::atomic<int> next_waypoint_{0};

  FRIEND_TEST(testing::LinePlanUtilTest, TurnCostTest);
  FRIEND_TEST(testing::LinePlanUtilTest, CrossFinishTest);
  FRIEND_TEST(testing::LinePlanUtilTest, SingleLineCostTest);
  FRIEND_TEST(testing::LinePlanUtilTest, StraightLineTest);
  FRIEND_TEST(testing::LinePlanUtilTest, LinePairCostTest);
  FRIEND_TEST(testing::LinePlanUtilTest, ObstacleCostTest);
  FRIEND_TEST(testing::LinePlanUtilTest, BackPassTestOnePoint);
  FRIEND_TEST(testing::LinePlanUtilTest, BackPassTestMultiPoint);
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
