// See http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
// This only seems to be an issue on the BBB, not normal laptops.
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include "gtest/gtest.h"
#include <gflags/gflags.h>

#include "simple.h"
#include "adaptive.h"
#include "line_plan.h"
#include "util/node.h"
#include "ui/server.h"
#include "sim/sim_inter.h"
#include "control/simple.h"
#include "control/line_tacking.h"
#include "control/waypoint_manager.h"
#include "sensor/state_estimator.h"
#include "sim/csv_logging.h"
#include "util/testing.h"

DEFINE_string(csv_file, "sim/python/basic_sim_data.csv", "File to save CSV data to");

namespace sailbot {
namespace testing {

class SimpleControlTest : public TestWrapper {
 protected:
  SimpleControlTest() :
#define LOG_VECTOR(path, name)                                                 \
  { path ".x", name " X" }                                                     \
  , {path ".y", name " Y"}, { path ".z", name " Z" }
#define TRUE_STATE "sim_true_boat_state"
    csv_logger_({
         {TRUE_STATE".internal.sail", "Sail"},                // 0
         {TRUE_STATE".internal.rudder", "Rudder"},            // 1
         {TRUE_STATE".pos.x", "Boat X"},                      // 2
         {TRUE_STATE".pos.y", "Boat Y"},                      // 3
         {TRUE_STATE".vel.x", "Boat Vel X"},                  // 4
         {TRUE_STATE".vel.y", "Boat Vel Y"},                  // 5
         {TRUE_STATE".orientation.w", "Quat W"},              // 6
         {TRUE_STATE".orientation.x", "Quat X"},              // 7
         {TRUE_STATE".orientation.y", "Quat Y"},              // 8
         {TRUE_STATE".orientation.z", "Quat Z"},              // 9
         {"wind.x", "Wind X"},                                // 10
         {"wind.y", "Wind Y"},                                // 11
         LOG_VECTOR("sim_debug.fs", "Sail Force"),            // 12-14
         LOG_VECTOR("sim_debug.fr", "Rudder Force"),          // 15-17
         LOG_VECTOR("sim_debug.fk", "Keel Force"),            // 18-20
         LOG_VECTOR("sim_debug.fh", "Hull Force"),            // 21-23
         LOG_VECTOR("sim_debug.fnet", "Net Force"),           // 24-26
         LOG_VECTOR("sim_debug.taus", "Sail Torque"),         // 27-29
         LOG_VECTOR("sim_debug.taur", "Rudder Torque"),       // 30-32
         LOG_VECTOR("sim_debug.tauk", "Keel Torque"),         // 33-35
         LOG_VECTOR("sim_debug.tauh", "Hull Torque"),         // 36-38
         LOG_VECTOR("sim_debug.tauright", "Righting Moment"), // 39-41
         LOG_VECTOR("sim_debug.taunet", "Net Torque"),        // 42-44
         {"heading_cmd.heading", "Heading Goal"},             // 45
        }, FLAGS_csv_file, .1)
#undef TRUE_STATE
#undef LOG_VECTOR
      //,simple_ctrl_(true)
      {

    threads_.emplace_back(&WebSocketServer::Run, &server_);
    threads_.emplace_back(&CsvLogger::Run, &csv_logger_);
    threads_.emplace_back(&sim::SimulatorNode::Run, &sim_node_);
//    threads_.emplace_back(&control::SimpleControl::Run, &simple_ctrl_);
    threads_.emplace_back(&control::AdaptiveControl::Run, &adaptive_ctrl_);
    //threads_.emplace_back(&control::LineTacker::Run, &tacker_);
    threads_.emplace_back(&control::LinePlan::Run, &line_plan_);
    threads_.emplace_back(&control::WaypointManager::Run, &manager_);
    threads_.emplace_back(&control::StateEstimator::Run, &state_estimator_);

    ProtoQueue<msg::ControlMode> mode_queue("control_mode", true);
    msg::ControlMode mode_msg;
    mode_msg.set_winch_mode(msg::ControlMode_MODE_AUTO);
    mode_msg.set_rudder_mode(msg::ControlMode_MODE_AUTO);
    mode_msg.set_tacker(msg::ControlMode_TACKER_LINE_PLAN);
    mode_queue.send(&mode_msg);
  }

  ~SimpleControlTest() {
    for (auto &t : threads_) {
      t.join();
    }
  }

  std::vector<std::thread> threads_;
  CsvLogger csv_logger_;
  WebSocketServer server_;
  sim::SimulatorNode sim_node_{-1};
  //control::SimpleControl simple_ctrl_;
  control::AdaptiveControl adaptive_ctrl_;
  //control::LineTacker tacker_;
  control::LinePlan line_plan_;
  control::WaypointManager manager_;
  control::StateEstimator state_estimator_;
};

namespace {
double ToLat(double y) {
  return y / 111015. + 38.9816688;
}
double ToLon(double x) {
  return x / 86647. - 76.47591338;
}
void SetWaypoint(msg::Waypoint* p, double x, double y) {
  p->set_x(ToLon(x));
  p->set_y(ToLat(y));
}
}  // namespace

TEST_F(SimpleControlTest, NavigationChallenge) {
  FLAGS_v = 1;
  msg::WaypointList waypoints;
  msg::Waypoint* p1 = waypoints.add_points();
  msg::Waypoint* p2 = waypoints.add_points();
  msg::Waypoint* p3 = waypoints.add_points();
  msg::Waypoint* p4 = waypoints.add_points();
  SetWaypoint(p1, 0, 0);
  SetWaypoint(p2, 15, -10);
  SetWaypoint(p3, 15, 10);
  SetWaypoint(p4, 0, 0);
  ProtoQueue<msg::WaypointList> way_q("waypoints", true);
  way_q.send(&waypoints);

  msg::Obstacles obstacles;
  msg::WaypointList *obs1 = obstacles.add_polygons();
  SetWaypoint(obs1->add_points(), 16, 3);
  SetWaypoint(obs1->add_points(), 16, -3);
  SetWaypoint(obs1->add_points(), 25, -3);
  SetWaypoint(obs1->add_points(), 25, 3);
  sim_node_.set_wind(0 * M_PI / 4, 3.5);
  ProtoQueue<msg::Obstacles> obs_q("planner_obstacles", true);
  obs_q.send(&obstacles);

  Sleep(75);
  ASSERT_TRUE(true);
}

TEST_F(SimpleControlTest, Square) {
  msg::WaypointList waypoints;
  msg::Waypoint* p1 = waypoints.add_points();
  msg::Waypoint* p2 = waypoints.add_points();
  msg::Waypoint* p3 = waypoints.add_points();
  msg::Waypoint* p4 = waypoints.add_points();
  msg::Waypoint* p5 = waypoints.add_points();
  SetWaypoint(p1, 0, 0);
  SetWaypoint(p2, 0, 100);
  SetWaypoint(p3, -300, 100);
  SetWaypoint(p4, -300, 0);
  SetWaypoint(p5, 0, 0);
  ProtoQueue<msg::WaypointList> way_q("waypoints", true);
  way_q.send(&waypoints);

  msg::Obstacles obstacles;
  msg::WaypointList *obs1 = obstacles.add_polygons();
  SetWaypoint(obs1->add_points(), 50, 500);
  SetWaypoint(obs1->add_points(), -350, 500);
  SetWaypoint(obs1->add_points(), -350, 115);
  SetWaypoint(obs1->add_points(), 50, 115);
  msg::WaypointList *obs2 = obstacles.add_polygons();
  SetWaypoint(obs2->add_points(), 50, -15);
  SetWaypoint(obs2->add_points(), -350, -15);
  SetWaypoint(obs2->add_points(), -350, -400);
  SetWaypoint(obs2->add_points(), 50, -400);
  ProtoQueue<msg::Obstacles> obs_q("planner_obstacles", true);
  obs_q.send(&obstacles);

  sim_node_.set_wind(0, 3);
  Sleep(150);
  ASSERT_TRUE(true);
}

TEST_F(SimpleControlTest, StartInIrons) {
  msg::WaypointList waypoints;
  msg::Waypoint* p1 = waypoints.add_points();
  msg::Waypoint* p2 = waypoints.add_points();
  p1->set_x(0);
  p1->set_y(0);
  p2->set_x(15000);
  p2->set_y(0);
  ProtoQueue<msg::WaypointList> way_q("waypoints", true);
  way_q.send(&waypoints);
  sim_node_.set_wind(M_PI, 6);
  Sleep(20);
  EXPECT_TRUE(true);
 // std::cerr << "x: " << sim_node_->get_x() << std::endl;
 // std::cerr << "x: " << tacker_->cur_pos_.x << std::endl;
//  EXPECT_GT(sim_node_->get_x(), 50);
  //EXPECT_TRUE(false);
}

}  // testing
}  // sailbot
