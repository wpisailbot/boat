#include "gtest/gtest.h"
#include <gflags/gflags.h>

#include "util/node.h"
#include "control/line_tacking.h"

namespace sailbot {
namespace control {
namespace testing {

class GoalReceiverTestNode : public Node {
 public:
  GoalReceiverTestNode() : Node(0) {
    RegisterHandler<msg::HeadingCmd>("heading_cmd",
                                     [this](const msg::HeadingCmd &cmd) {
      actual_ = cmd;
      EXPECT_EQ(expected_.has_heading(), actual_.has_heading());
    });
  }
 private:
  void Iterate() override {}
  msg::HeadingCmd expected_;
  msg::HeadingCmd actual_;
};

class LineTackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    util::CancelShutdown();
    Queue::set_testing(true);
    sailbot::util::ClockManager::SetFakeClock(true, true);

    tacker_.reset(new LineTacker());
    receiver_.reset(new GoalReceiverTestNode());
    threads_.emplace_back(&sailbot::util::ClockManager::Run, 0);
    threads_.emplace_back(&LineTacker::Run, tacker_.get());
  }
  void TearDown() override {
    util::RaiseShutdown();
    for (std::thread &t : threads_) {
      t.join();
    }
    tacker_.reset();
  }

  std::unique_ptr<LineTacker> tacker_;
  std::unique_ptr<GoalReceiverTestNode> receiver_;
  std::vector<std::thread> threads_;
};

TEST_F(LineTackerTest, DoesNothingOnNoData) {
  // As of yet, should not have received any data.
}

}  // namespace testing
}  // namespace control
}  // namespace sailbot
