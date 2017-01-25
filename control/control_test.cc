#include "gtest/gtest.h"

#include "simple.h"
#include "util/node.h"
#include "sim/sim_inter.h"
#include "control/simple.h"
#include "control/line_tacking.h"

namespace sailbot {
namespace testing {

class SimpleControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Queue::set_testing(true);
    sailbot::util::ClockManager::SetFakeClock(true);

    clock_.reset(new util::ClockInstance());
    sim_node_.reset(new sim::SimulatorNode());
    simple_ctrl_.reset(new control::SimpleControl());
    tacker_.reset(new control::LineTacker());

    threads_.emplace_back(&sailbot::util::ClockManager::Run, 0);
    threads_.emplace_back(&sim::SimulatorNode::Run, simple_ctrl_.get());
    threads_.emplace_back(&control::SimpleControl::Run, sim_node_.get());
    threads_.emplace_back(&control::LineTacker::Run, tacker_.get());
  }

  void TearDown() override {
    util::RaiseShutdown();
    clock_.reset();
    for (auto& thread : threads_) {
      util::RaiseShutdown();
      thread.join();
    }
  }

  void Sleep(double sec) {
    clock_->SleepUntil(clock_->Time() +
                       std::chrono::nanoseconds(int(sec * 1e9)));
  }

  std::unique_ptr<util::ClockInstance> clock_;
  std::unique_ptr<sailbot::sim::SimulatorNode> sim_node_;
  std::unique_ptr<sailbot::control::SimpleControl> simple_ctrl_;
  std::unique_ptr<sailbot::control::LineTacker> tacker_;

  std::vector<std::thread> threads_;
};

TEST_F(SimpleControlTest, Basic) {
  sim_node_->set_wind(-M_PI / 2, 4);
  Sleep(1);
  ASSERT_TRUE(true);
  ASSERT_TRUE(true);
}

}  // testing
}  // sailbot
