#pragma once
// See http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
// This only seems to be an issue on the BBB, not normal laptops.
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
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
class TEST_FUN(LinePlanUtilTest, MakeGateFromPointsTest);
class BackPassTest;
class GenerateHypothesesTest;
class OptimizerTest;
void TryTurnCost(double startheading, double endheading, double winddir,
                 double expcost, double expdstart, double expdend,
                 const char *desc);
void TryCrossFinishCost(double alpha, double expcost, double expdalpha,
                        const char *desc);
void TryStraightLineCost(double len, double heading, double winddir,
                         bool is_real, double expcost, double expdlen, double,
                         const char *);
} // namespace testing

class LinePlan : public Node {
 public:
  typedef Eigen::Vector2d Vector2d;

  LinePlan();

  ~LinePlan() {
    delete pathpoints_lonlat_msg_;
  }

  void Iterate() override;

  /**
   * Return the meters-based coordinate frame value for the longitude/latitude
   * from lonlat.
   */
  Point LonLatToFrame(const Point &lonlat) {
    return lonlat_scale_.cwiseProduct(lonlat - lonlat_ref_);
  }
  Point FrameToLonLat(const Point &frame) {
    return frame.cwiseQuotient(lonlat_scale_) + lonlat_ref_;
  }

  // Reset _all_ of our meters-based using newlonlatref as the origin
  // longitude/latitude and basis for scaling.
  // data_mutex_ MUST already be locked going into this.
  void ResetRef(const Point &newlonlatref);

 private:
  constexpr static float dt = 2.0;
  // Cost, used in TurnCost, of traversing the upwind no-go zones
  // relative to typical turns.
  constexpr static float kTackCost = 5.0;
  // Relative weight of turn costs:
  constexpr static float kTurnCost = 0.3;
  // Scalar affecting cost associated with running upwind:
  constexpr static float kUpwindCost = 300.0;
  // Scalar for the cost associated with an overall upwind stretch,
  // on the assumption that we would end up adding tacking later.
  constexpr static float kUpwindApproxCost = 0 * 2.0;
  // The nearest to the wind that we physically can sail:
  constexpr static float kSailableReach = M_PI / 6.0;
  // The relative cost of being near obstacles.
  // If kObstacleCost = 1, then the cost will be integral e^-x dl with
  // x the distance to a given obstacle, l being the line we are
  // following.
  constexpr static float kObstacleCost = 100;
  // Cost per unit length of a line, cost / meter
  constexpr static float kLengthCost = 1.0;
  // Default length of a gate, in meters.
  constexpr static float kGateWidth = 20.0;
  // Amount to weight the very first turn relative
  // to the later turns (provides some hysteresis):
  constexpr static float kPreTurnScale = 200.0;
  // Maximum number of turn points
//  constexpr static int kMaxLegpts = 6;
  constexpr static int kMaxTotalpts = 6;
  constexpr static int kMaxLegs = 1;
  // Weighting for how near we get to the "preferred"
  // distance from the waypoint
  constexpr static float kAlphaCrossCost = 5.0;

  // None of these functions below account for obstacles.
  static void SingleLineCost(const Vector2d &startline, const Vector2d &endline,
                             const Vector2d &startpt, double alpha,
                             double preheading, const Vector2d &nextpt,
                             double winddir, double *cost, double *dcostdalpha,
                             bool *viable);
  static void LinePairCost(const Vector2d &startpt, const Vector2d &endpt,
                           const Vector2d &turnpt, double preheading,
                           double postheading, double winddir,
                           double scale_pre_cost, double *cost,
                           Vector2d *dcostdturnpt, bool *viable);
  static void CrossFinishCost(double alpha, double *cost, double *dcostdalpha);
  static void TurnCost(double startheading, double endheading, double winddir,
                       double *cost, double *dcostdstart, double *dscostdend);
  static void StraightLineCost(double len, double heading, double winddir,
                               bool is_real, double *cost, double *dcostdlen,
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
                           double scale_pre_cost,
                           const std::vector<Polygon> &obstacles, double *cost,
                           Vector2d *dcostdturnpt, bool *viable);

  // Perform a backwards pass to update tackpts/alpha
  static void BackPass(const std::pair<Vector2d, Vector2d> &gate,
                       const Vector2d &nextpt, double winddir,
                       const std::vector<Polygon> &obstacles, double cur_yaw,
                       double step, bool prescale,
                       std::vector<Vector2d> *tackpts, double *alpha,
                       double *finalcost, bool *viable);

  // Optimizes tackpts/alpha. You should seed tackpts/alpha with
  // some sane initial values.
  static void OptimizeTacks(const std::pair<Vector2d, Vector2d> &gate,
                            const Vector2d &nextpt, double winddir,
                            const std::vector<Polygon> &obstacles,
                            double cur_yaw, std::vector<Vector2d> *tackpts,
                            double *alpha, double *finalcost, bool *viable);

  static void
  OptimizeMultipleTacks(const std::vector<std::pair<Vector2d, Vector2d>> &gates,
                        const Vector2d &nextpt, double winddir,
                        const std::vector<Polygon> &obstacles, double cur_yaw,
                        std::vector<std::vector<Vector2d>> *tacks,
                        std::vector<double> *alphas, double *finalcost,
                        bool *viable);

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
                                 std::vector<std::vector<Vector2d>> *paths,
                                 bool clear_paths = true);

  // Actually put GenerateHypotheses and OptimizeTacks together to
  // produce a final path.
  static void FindPath(const Vector2d &startpt,
                       const std::pair<Vector2d, Vector2d> &gate,
                       const Vector2d &nextpt, double winddir,
                       const std::vector<Polygon> &obstacles, double cur_yaw,
                       std::vector<Vector2d> *tackpts, double *alpha);

  static void
  FindMultiplePath(const Vector2d &startpt,
                   const std::vector<std::pair<Vector2d, Vector2d>> &gates,
                   const Vector2d &nextpt, double winddir,
                   const std::vector<Polygon> &obstacles, double cur_yaw,
                   std::vector<std::vector<Vector2d>> *tacks,
                   std::vector<double> *alphas);

  // Compute the gate attached to a waypoint given the waypoints before and
  // after it. If either the first or last waypoint overlaps with the gate
  // waypoint, create gate with middle waypoint in middle of the gate. If all
  // three waypoints are the same (or extremely near), then align gate with
  // X-axis.
  static std::pair<Point, Point> MakeGateFromWaypoints(const Point &gatept,
                                                       const Point &prevpt,
                                                       const Point &nextpt);

  /**
   * Update the transformations on the obstacles, assuming some change to either
   * obstacles_lonlat_ or to the transformations.
   */
  void UpdateObstacles();
  // Do processing for handler
  void ReceiveObstacles(const msg::Obstacles &msg);

  /**
   * Update the waypoints_ from the waypoints_lonlat_ vector.
   */
  void UpdateWaypoints();
  // Do processing for handler
  void ReceiveWaypoints(const msg::WaypointList &msg);

  /**
   * Increment next_waypoint_ as appropriate.
   * Must of data_mutex_ locked BEFORE being called.
   */
  void UpdateWaypointInc();

  /**
   * Figure out what the next heading should be
   */
  double GetGoalHeading();

  /**
   * Perform waypoint+obstacle initialization
   * Should only be done in initialization, as this
   * will use dynamic memory allocation (not that all the std::vector's
   * in this whole class wouldn't cause more issue...) and, more importantly,
   * File I/O
   * Should be called AFTER we register all of our handlers.
   */
  void ReadWaypointsAndObstacles();

  std::mutex data_mutex_;

  // Potentially useful state information
  std::atomic<double> yaw_;
  std::atomic<double> wind_dir_;
  std::atomic<int> crossed_cnt_{0};
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
  msg::WaypointList* pathpoints_lonlat_msg_;
  ProtoQueue<msg::WaypointList> pathpoints_queue_;
  // Store ``waypoints'' as pairs of points representing the line we must cross.
  // The order of the pair in theory could matter, although currently it should
  // not.
  // Nominally, the first value in the pair shall be the ``true'' waypoint.
  // In the algorithms, the path shall always be planned under
  // the assumption that we shall be following a path from our
  // current position, to waypoints_[0]... and terminated at waypoints_.back()
  // At any given instant, we are straing to sail to waypoint next_waypoint_.
  std::vector<std::pair<Point, Point>> waypoints_;
  // Whether given waypoints are gates or buoys:
  std::vector<bool> are_gates_;
  std::atomic<int> next_waypoint_{0};
  // Whether to repeat waypoints when we finish them.
  std::atomic<bool> repeat_waypoints_{false};

  msg::HeadingCmd *heading_msg_;
  ProtoQueue<msg::HeadingCmd> heading_cmd_;

  msg::TackerState *state_msg_;
  ProtoQueue<msg::TackerState> state_queue_;

  std::atomic<int> tack_mode_{msg::ControlMode_TACKER_LINE_PLAN};

  FRIEND_TEST_FUN(testing::LinePlanUtilTest, TurnCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, CrossFinishTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, SingleLineCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, StraightLineTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, LinePairCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, ObstacleCostTest);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, BackPassTestOnePoint);
  FRIEND_TEST_FUN(testing::LinePlanUtilTest, MakeGateFromPointsTest);
  friend class testing::BackPassTest;
  friend class testing::GenerateHypothesesTest;
  friend class testing::OptimizerTest;
  friend void testing::TryTurnCost(double, double, double, double, double,
                                   double, const char *);
  friend void testing::TryCrossFinishCost(double, double, double, const char *);
  friend void testing::TryStraightLineCost(double len, double heading,
                                           double winddir, bool is_real,
                                           double expcost, double expdlen,
                                           double, const char *);
};

#undef FRIEND_TEST_FUN
#undef TEST_FUN

}  // control
}  // sailbot
