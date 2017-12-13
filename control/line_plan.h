#pragma once
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "math/polygon.h"
#include <Eigen/Core>
#include <mutex>

#define TEST_FUN(test_case_name, test_name)\
  test_case_name##_##test_name##_Test
#define FRIEND_TEST_FUN(test_case_name, test_name)\
  friend class TEST_FUN(test_case_name, test_name);

namespace sailbot {
namespace control {

namespace testing {
class TEST_FUN(LinePlanUtilTest, TurnCostTest);
class TEST_FUN(LinePlanUtilTest, CrossFinishTest);
class TEST_FUN(LinePlanUtilTest, SingleLineCostTest);
class TEST_FUN(LinePlanUtilTest, StraightLineTest);
class TEST_FUN(LinePlanUtilTest, LinePairCostTest);
class TEST_FUN(LinePlanUtilTest, ObstacleCostTest);
class TEST_FUN(LinePlanUtilTest, BackPassTestOnePoint);
class BackPassTest;
class GenerateHypothesesTest;
class OptimizerTest;
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
  // data_mutex_ MUST already be locked going into this.
  void ResetRef(const Point &newlonlatref);

 private:
  constexpr static float dt = 1.0;
  // Cost, used in TurnCost, of traversing the upwind no-go zones
  // relative to typical turns.
  constexpr static float kTackCost = 5.0;
  // Relative weight of turn costs:
  constexpr static float kTurnCost = 0.3;
  // Scalar affecting cost associated with running upwind:
  constexpr static float kUpwindCost = 300.0;
  // The nearest to the wind that we physically can sail:
  constexpr static float kSailableReach = M_PI_4;
  // The relative cost of being near obstacles.
  // If kObstacleCost = 1, then the cost will be integral e^-x dl with
  // x the distance to a given obstacle, l being the line we are
  // following.
  constexpr static float kObstacleCost = 100.0;
  // Cost per unit length of a line, cost / meter
  constexpr static float kLengthCost = 0.1;
  // Default length of a gate, in meters.
  constexpr static float kGateWidth = 20.0;

  // None of these functions below account for obstacles.
  static void SingleLineCost(const Vector2d &startline, const Vector2d &endline,
                             const Vector2d &startpt, double alpha,
                             double preheading, const Vector2d &nextpt,
                             double winddir, double *cost, double *dcostdalpha,
                             bool *viable);
  static void LinePairCost(const Vector2d &startpt, const Vector2d &endpt,
                           const Vector2d &turnpt, double preheading,
                           double postheading, double winddir, double *cost,
                           Vector2d *dcostdturnpt, bool *viable);
  static void CrossFinishCost(double alpha, double *cost, double *dcostdalpha);
  static void TurnCost(double startheading, double endheading, double winddir,
                       double *cost, double *dcostdstart, double *dscostdend);
  static void StraightLineCost(double len, double heading, double winddir,
                               double *cost, double *dcostdlen,
                               double *dcostdheading, bool *viable);

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
                             double *cost, double *dcostdalpha, bool *viable);
  static void LinePairCost(const Vector2d &startpt, const Vector2d &endpt,
                           const Vector2d &turnpt, double preheading,
                           double postheading, double winddir,
                           const std::vector<Polygon> &obstacles, double *cost,
                           Vector2d *dcostdturnpt, bool *viable);

  // Perform a backwards pass to update tackpts/alpha
  static void BackPass(const std::pair<Vector2d, Vector2d> &gate,
                       const Vector2d &nextpt, double winddir,
                       const std::vector<Polygon> &obstacles, double cur_yaw,
                       double step, std::vector<Vector2d> *tackpts,
                       double *alpha, double *finalcost, bool *viable);

  // Optimizes tackpts/alpha. You should seed tackpts/alpha with
  // some sane initial values.
  static void OptimizeTacks(const std::pair<Vector2d, Vector2d> &gate,
                            const Vector2d &nextpt, double winddir,
                            const std::vector<Polygon> &obstacles,
                            double cur_yaw, std::vector<Vector2d> *tackpts,
                            double *alpha, double *finalcost, bool *viable);

  // Provides a seeding of points to use for OptimizeTacks.
  // For beam reaches/downwinds, seeds with straight line path.
  // For upwind or near-upwind paths will produce three hypotheses:
  //   1) Straight line path, for sanity
  //   2) Evenly spaced close reachs, starting with a starboard tack
  //   3) Evenly spaced close reachs, starting with a port tack
  // startpt: Starting point/current boat position
  // endpt: Ending point (middle of gate)
  // winddir: Wind direction
  // Npts: Number of points to generate along path, excluding startpt/endpt.
  //   Must be nonnegative; if 0, then this isn't really doing anything useful.
  // paths: vector of paths; paths will contain Npts+1 points, where the first
  //   element will always be startpt.
  static void GenerateHypotheses(const Vector2d &startpt, const Vector2d &endpt,
                                 double winddir, int Npts,
                                 std::vector<std::vector<Vector2d>> *paths);

  // Actually put GenerateHypotheses and OptimizeTacks together to
  // produce a final path.
  static void FindPath(const Vector2d &startpt,
                       const std::pair<Vector2d, Vector2d> &gate,
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

  /**
   * Increment next_waypoint_ as appropriate.
   * Must of data_mutex_ locked BEFORE being called.
   */
  void UpdateWaypointInc();

  /**
   * Figure out what the next heading should be
   */
  double GetGoalHeading();

  std::mutex data_mutex_;

  // Potentially useful state information
  std::atomic<double> yaw_;
  std::atomic<double> wind_dir_;
  // Positions in meters reference system
  Point boat_pos_;
  Point boat_pos_lonlat_;
  Point prev_boat_pos_;
  Point prev_boat_pos_lonlat_;

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

  msg::HeadingCmd *heading_msg_;
  ProtoQueue<msg::HeadingCmd> heading_cmd_;

  FRIEND_TEST_FUN(testing::LinePlanUtilTest, TurnCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, CrossFinishTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, SingleLineCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, StraightLineTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, LinePairCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, ObstacleCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, BackPassTestOnePoint);
  friend class testing::BackPassTest;
  friend class testing::GenerateHypothesesTest;
  friend class testing::OptimizerTest;
  friend void testing::TryTurnCost(double, double, double, double, double,
                                   double, const char *);
  friend void testing::TryCrossFinishCost(double, double, double, const char *);
  friend void testing::TryStraightLineCost(double len, double heading,
                                           double winddir, double expcost,
                                           double expdlen, double,
                                           const char *);
};

#undef FRIEND_TEST_FUN
#undef TEST_FUN

}  // control
}  // sailbot
