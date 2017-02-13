#include "gtest/gtest.h"
#include <gflags/gflags.h>

#include "util/node.h"
#include "control/line_tacking.h"
#include "control/util.h"
#include "util/testing.h"

namespace sailbot {
namespace control {
namespace testing {

class GoalReceiverTestNode : public Node {
 public:
  GoalReceiverTestNode() : Node(-1), tol_{1e-3} {
    RegisterHandler<msg::HeadingCmd>("heading_cmd",
                                     [this](const msg::HeadingCmd &cmd) {
      std::unique_lock<std::mutex> lck(mutex_);
      actual_ = cmd;
      EXPECT_EQ(expected_.has_heading(), actual_.has_heading());
      if (expected_.has_heading() && actual_.has_heading()) {
        EXPECT_LT(
            std::abs(util::norm_angle(actual_.heading() - expected_.heading())),
            tol_.load());
      }
    });
  }
  void set_expected(float e) {
    std::unique_lock<std::mutex> lck(mutex_);
    expected_.set_heading(e);
  }
  void clear_expected() {
    std::unique_lock<std::mutex> lck(mutex_);
    expected_.clear_heading();
  }
  void set_tolerance(float t) {
    tol_ = t;
  }
 private:
  void Iterate() override {}
  std::mutex mutex_;
  std::atomic<float> tol_;
  msg::HeadingCmd expected_;
  msg::HeadingCmd actual_;
};

class LineTackerTest : public ::sailbot::testing::TestWrapper {
 protected:
  LineTackerTest() {
    threads_.emplace_back(&LineTacker::Run, &tacker_);
    threads_.emplace_back(&GoalReceiverTestNode::Run, &receiver_);
  }

  ~LineTackerTest() {
    for (std::thread &t : threads_) {
      t.join();
    }
  }

  void SetupWaypoints(float x, float y) {
    msg::WaypointList waypoints;
    msg::Waypoint* p1 = waypoints.add_points();
    msg::Waypoint* p2 = waypoints.add_points();
    p1->set_x(0);
    p1->set_y(0);
    p2->set_x(x);
    p2->set_y(y);
    ProtoQueue<msg::WaypointList> way_q("waypoints", true);
    way_q.send(&waypoints);

    ProtoQueue<msg::Vector3f> wind_q("wind", true);
    msg::Vector3f wind_msg;
    wind_msg.set_x(0);
    wind_msg.set_y(-1);
    wind_msg.set_z(0);
    wind_q.send(&wind_msg);
  }

  LineTacker tacker_;
  GoalReceiverTestNode receiver_;
  std::vector<std::thread> threads_;
};

TEST_F(LineTackerTest, DoesNothingOnNoData) {
  Sleep(0.1);
}

TEST_F(LineTackerTest, GoesStraightBeamReach) {
  SetupWaypoints(100, 0);
  receiver_.set_expected(0);
  Sleep(0.1);
}

TEST_F(LineTackerTest, GoesToReasonableTack) {
  SetupWaypoints(0, 100);
  receiver_.set_expected(0.8);
  receiver_.set_tolerance(0.2);
  Sleep(0.1);
}

}  // namespace testing
}  // namespace control
}  // namespace sailbot
