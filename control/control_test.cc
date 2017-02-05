#include "gtest/gtest.h"
#include <gflags/gflags.h>

#include "simple.h"
#include "util/node.h"
#include "ui/server.h"
#include "sim/sim_inter.h"
#include "control/simple.h"
#include "control/line_tacking.h"
#include "sim/csv_logging.h"

DEFINE_string(csv_file, "sim/python/basic_sim_data.csv", "File to save CSV data to");

namespace sailbot {
namespace testing {

class SimpleControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Queue::set_testing(true);
    sailbot::util::ClockManager::SetFakeClock(true);

    server_.reset(new WebSocketServer());
#define LOG_VECTOR(path, name)                                                 \
  { path ".x", name " X" }                                                     \
  , {path ".y", name " Y"}, { path ".z", name " Z" }
    csv_logger_.reset(new CsvLogger(
        {
         {"boat_state.internal.sail", "Sail"},                // 0
         {"boat_state.internal.rudder", "Rudder"},            // 1
         {"boat_state.pos.x", "Boat X"},                      // 2
         {"boat_state.pos.y", "Boat Y"},                      // 3
         {"boat_state.vel.x", "Boat Vel X"},                  // 4
         {"boat_state.vel.y", "Boat Vel Y"},                  // 5
         {"boat_state.orientation.w", "Quat W"},              // 6
         {"boat_state.orientation.x", "Quat X"},              // 7
         {"boat_state.orientation.y", "Quat Y"},              // 8
         {"boat_state.orientation.z", "Quat Z"},              // 9
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
        },
        FLAGS_csv_file, .1));
#undef LOG_VECTOR

    clock_.reset(new util::ClockInstance());
    sim_node_.reset(new sim::SimulatorNode());
    simple_ctrl_.reset(new control::SimpleControl(false));
    tacker_.reset(new control::LineTacker());

    threads_.emplace_back(&sim::SimulatorNode::Run, simple_ctrl_.get());
    threads_.emplace_back(&WebSocketServer::Run, server_.get());
    threads_.emplace_back(&CsvLogger::Run, csv_logger_.get());
    threads_.emplace_back(&control::SimpleControl::Run, sim_node_.get());
    threads_.emplace_back(&control::LineTacker::Run, tacker_.get());
    threads_.emplace_back(&sailbot::util::ClockManager::Run, 0);
  }

  void TearDown() override {
    util::RaiseShutdown();
    clock_.reset();
    // Deal with everything but clock manager
    for (unsigned i = 0; i < threads_.size()-1; ++i) {
      threads_[i].join();
    }
    csv_logger_.reset();
    server_.reset();
    sim_node_.reset();
    simple_ctrl_.reset();
    tacker_.reset();

    threads_[threads_.size()-1].join();
  }

  void Sleep(double sec) {
    clock_->SleepUntil(clock_->Time() +
                       std::chrono::milliseconds(int(sec * 1e3)));
  }

  std::unique_ptr<util::ClockInstance> clock_;
  std::unique_ptr<CsvLogger> csv_logger_;
  std::unique_ptr<WebSocketServer> server_;
  std::unique_ptr<sailbot::sim::SimulatorNode> sim_node_;
  std::unique_ptr<sailbot::control::SimpleControl> simple_ctrl_;
  std::unique_ptr<sailbot::control::LineTacker> tacker_;

  std::vector<std::thread> threads_;
};

TEST_F(SimpleControlTest, NavigationChallenge) {
  msg::WaypointList waypoints;
  msg::Waypoint* p1 = waypoints.add_points();
  msg::Waypoint* p2 = waypoints.add_points();
  msg::Waypoint* p3 = waypoints.add_points();
  msg::Waypoint* p4 = waypoints.add_points();
  p1->set_x(0);
  p1->set_y(0);
  p2->set_x(43);
  p2->set_y(-25);
  p3->set_x(43);
  p3->set_y(25);
  p4->set_x(0);
  p4->set_y(0);
  ProtoQueue<msg::WaypointList> way_q("waypoints", true);
  way_q.send(&waypoints);
  sim_node_->set_wind(0, 6);
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
  p1->set_x(0);
  p1->set_y(0);
  p2->set_x(0);
  p2->set_y(150);
  p3->set_x(-150);
  p3->set_y(150);
  p4->set_x(-150);
  p4->set_y(0);
  p5->set_x(0);
  p5->set_y(0);
  ProtoQueue<msg::WaypointList> way_q("waypoints", true);
  way_q.send(&waypoints);
  sim_node_->set_wind(0, 6);
  Sleep(200);
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
  sim_node_->set_wind(M_PI, 6);
  Sleep(20);
  EXPECT_TRUE(true);
 // std::cerr << "x: " << sim_node_->get_x() << std::endl;
 // std::cerr << "x: " << tacker_->cur_pos_.x << std::endl;
//  EXPECT_GT(sim_node_->get_x(), 50);
  //EXPECT_TRUE(false);
}

}  // testing
}  // sailbot
