#pragma once
#include <map>
#include <condition_variable>
#include <shared_mutex>
#include <mutex>
#include <vector>
#include <atomic>
#include <string>

namespace sailbot {
namespace testing {

// Class implementing a duplicate of the Boost message_queue
// which only allows communication within a single process.
// This is helpful in cases where you are testing and
// want repeatable behavior that will not be influenced
// by outside processes.
// Each individual TestQueue corresponds with a single queue which can be
// sent/received from.
class TestQueue {
 public:
  // Construct TestQueue on a certain name with a certain
  // size buffer.
  TestQueue(const std::string &name, size_t max_size)
      : name_(name), max_size_(max_size) {
    // Increment the number of instances accessing this
    // queue name. Note that the operator[] on std::map
    // will create an entry at name_ if it does not
    // exist already.
    std::unique_lock<std::mutex> lck_(map_lock_);
    conditions_[name_].cnt++;
    idx_ = conditions_[name_].users.size();
    conditions_[name_].users.push_back(-1);
  }
  ~TestQueue() {
    std::unique_lock<std::mutex> lck_(map_lock_);
    // Decrement counter and remove if we are the last queue.
    conditions_[name_].cnt--;
    conditions_[name_].users[idx_] = -1;
    if (conditions_[name_].cnt.load() == 0) {
      conditions_.erase(name_);
    }
  }
  // Send a message with content msg and of size size.
  void send(const void *msg, size_t size);
  // Wait until we receive a message.
  // msg: Message content buffer
  // size: Max number of bytes to write to msg
  // rcvd: Fills in the number of bytes actually written to msg
  // Return Value: true if successful, false if not
  // Returns early if the util::IsShutdown() flag has been
  // set
  bool receive(void *msg, size_t size, size_t &rcvd);
 private:
  struct QueueData {
    // Condition variable to notify all readers
    // when a new message arrives
    std::condition_variable cond;
    // Actual raw data of most recent message
    std::unique_ptr<uint8_t[]> data;
    // Number of active TestQueue instances using this queue
    std::atomic<int> cnt{0};

    std::vector<int> users;

    // Size of data
    size_t data_len = 0;
    // Index that is incremented for each new message on queue
    int inc = 0;
  };
  // Lock for conditions_
  static std::mutex map_lock_;
  // Map with data for each active queue.
  static std::map<std::string, QueueData> conditions_;

  // Name of the queue we are reading on.
  std::string name_;
  // Maximum size of sent/received messages
  const size_t max_size_;
  // last_inc_ is the index (corresponds with inc)
  // received on the queue.
  // Assumes that last_inc_ is called in only one thread.
  int last_inc_ = 0;
  int idx_ = -1; // Index in users vector
};

}  // testing
}  // sailbot
