#include "test_queue.h"
#include "util/clock.h"
#include <string.h>
#include <iostream>

namespace sailbot {
namespace testing {

std::mutex TestQueue::map_lock_;
std::map<std::string, TestQueue::QueueData> TestQueue::conditions_;

void TestQueue::send(const void *msg, size_t size) {
  std::unique_lock<std::mutex> lck_(map_lock_);
  QueueData &data = conditions_[name_];
  data.inc++;
  if (!data.data) {
    // If the buffer hasn't been initialized yet, do so.
    data.data = std::make_unique<uint8_t[]>(max_size_);
  }
  // Does copy: Assumes that data.data has a size of at
  // least this->max_size_.
  memcpy(data.data.get(), msg, std::min(size, max_size_));
  data.data_len = std::min(size, max_size_);
  // Let everyone know that the message has been received
  data.cond.notify_all();
  data.cond.wait_for(lck_, std::chrono::milliseconds(50), [this, &data]() {
    if (util::IsShutdown())
      return true;
    for (const int n : data.users) {
      if (n >= 0 && n < data.inc) {
        return false;
      }
    }
    return true;
  });
}

bool TestQueue::receive(void *msg, size_t size, size_t &rcvd) {
  std::unique_lock<std::mutex> lck_(map_lock_);
  if (last_inc_ == 0) conditions_[name_].users[idx_] = 0;

  // Wait for data if we haven't yet received a message
  // newer than the last one received
  //
  if (conditions_[name_].inc == last_inc_) {
    // Wait for the data, until we get something
    // or the process shuts down.
    while (conditions_[name_].inc == last_inc_ && !util::IsShutdown()) {
      conditions_[name_].cond.wait_for(lck_, std::chrono::milliseconds(50));
    }
  }
  conditions_[name_].users[idx_] = conditions_[name_].inc;
  conditions_[name_].cond.notify_all();
  if (conditions_[name_].data) {
    rcvd = std::min(size, conditions_[name_].data_len);
    memcpy(msg, conditions_[name_].data.get(), rcvd);
    last_inc_ = conditions_[name_].inc;
  } else {
    // We should never received a message with invalid
    // data unless we have just shut down.
    last_inc_ = conditions_[name_].inc;
    return false;
  }
  return !util::IsShutdown();
}

}  // testing
}  // sailbot
