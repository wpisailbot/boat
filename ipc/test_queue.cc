#include "test_queue.h"
#include "util/clock.h"
#include <string.h>
#include <iostream>

namespace sailbot {
namespace testing {

std::shared_timed_mutex TestQueue::map_lock_;
std::map<std::string, TestQueue::QueueData> TestQueue::conditions_;

void TestQueue::send(const void *msg, size_t size) {
  std::unique_lock<std::shared_timed_mutex> lck_(map_lock_);
  QueueData &data = conditions_[name_];
  data.inc++;
  if (!data.data) {
    data.data = std::make_unique<uint8_t[]>(max_size_);
  }
  memcpy(data.data.get(), msg, std::min(size, max_size_));
  data.data_len = std::min(size, max_size_);
  data.cond.notify_all();
}

bool TestQueue::receive(void *msg, size_t size, size_t &rcvd) {
  std::unique_lock<std::shared_timed_mutex> lck_(map_lock_);
  // Construct in place if needed.
  if (conditions_[name_].inc == last_inc_) {
    // Wait for the data...
    while (conditions_[name_].inc == last_inc_ && !util::IsShutdown()) {
      conditions_[name_].cond.wait_for(lck_, std::chrono::milliseconds(50));
    }
  }
  rcvd = std::min(size, conditions_[name_].data_len);
  memcpy(msg, conditions_[name_].data.get(), rcvd);
  last_inc_ = conditions_[name_].inc;
  return !util::IsShutdown();
}

}  // testing
}  // sailbot
