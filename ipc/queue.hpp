// Author: James Kuszmaul <jabukuszmaul@gmail.com>
// Contains Queue and ProtoQueue<T>, which uses Queue to send back and forth
// protobuf messages.
#pragma once

#include "message_queue.hpp"
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <cstring>
#include <array>
#include <mutex>

#include "glog/logging.h"
#include "google/protobuf/arena.h"

#include "util/msg.pb.h"
#include "util/clock.h"
#include "test_queue.h"

namespace sailbot {

// TODO(james): Fix documentation.

/**
 * @brief A wrapper for the boost message_queue class.
 *
 * Keeps track of a boost message_queue and IPC semaphore, in order
 * to track when everyone is done using the message_queue. Unfortunately,
 * due to a tendency to Ctrl-C processes, the semaphore rarely gets
 * properly decremented and so some other implementation should be considered.
 */
class Queue {
 public:
  /**
   * @brief Construct a Queue, initializing shared memory as necessary.
   *
   * @param name The name to use for the memory mapped file for the queue.
   * @param writer Whether or not this queue will be sending messages.
   * @param queue_len The maximum number of messages allowed in the queue.
   * @param msg_size The mazimum size that a message will be.
   */
  Queue(const char *name, bool writer, size_t queue_len = 10,
        size_t msg_size = 1024);

  /**
   * @brief Cleans up shared memory as necessary.
   */
  ~Queue();

  /**
   * @brief Append a message to the queue.
   *
   * See the boost::interprocess::message_queue documentation for more
   *   information; this function just calls message_queue::send.
   *
   * @param buffer The object to append to the queue.
   */
  void send(const void *buffer, size_t size);

  /**
   * @brief Retrieve the next message from the queue, blocking until available.
   *
   * See the boost::interprocess::message_queue documentation for more
   *   information; this function just calls message_queue::receive.
   *
   * @param buffer The buffer to fill the the new message.
   */
  bool receive(void *buffer, size_t size, size_t &rcvd);

  /**
   * @brief removes everything in shared memory. Call when doing global init.
   *
   * If you don't call this and the various pieces of shared memory already are
   * initialized, then things may get awkward.
   */
  static void remove(const char *name);

  static void set_testing(bool testing) { testing_ = testing; }

 private:
  //! The name to be used for the shared memory queue itself.
  char name_[128];

  //! Whether or not this queue will be doing any writing.
  bool writer_;

  //! Whether or not we are testing (ie, use SHM or TestQueue)
  static bool testing_;

  //! @brief The boost queue object to use.
  //!
  //! This is a pointer so as to allow us to destroy queue_ before calling
  //! message_queue::remove().
  boost::interprocess::message_queue *queue_;

  //! @brief Semaphore to keep track of number of Queue objects using the queue.
  boost::interprocess::named_semaphore *semaphore_;

  //! The testing implementaiton (for use instead of the boost stuff).
  std::unique_ptr<testing::TestQueue> test_impl_;
};

/**
 * Message queue for sending protobufs back and forth; deals with serializing
 * and parsing the protobufs for you.
 */
template <typename T>
class ProtoQueue {
 public:
  // TODO(james): Parameterize number of messages.
  // @param name name of the queue to use.
  ProtoQueue(const char *name, bool writer)
      : impl_(name, writer, 10 /*number of messages*/, BUF_SIZE) {
    arena_settings_.start_block_size = 10000;
    arena_settings_.max_block_size = 0;
    arena_.reset(new google::protobuf::Arena(arena_settings_));
    msg_header_ = google::protobuf::Arena::CreateMessage<msg::LogEntry>(arena_.get());
    receive_header_ =
        google::protobuf::Arena::CreateMessage<msg::LogEntry>(arena_.get());
    field_ = msg_header_->GetDescriptor()->FindFieldByName(name);
    if (field_ == nullptr) {
      LOG(FATAL) << "Queue \"" << name << "\" does not exist in the LogEntry proto...";
    }
    field_content_ = msg_header_->GetReflection()->MutableMessage(msg_header_, field_);
  }

  void send(const T *msg);
  bool receive(T *msg);
 private:
  Queue impl_;
  msg::LogEntry *msg_header_;
  msg::LogEntry *receive_header_;
  const google::protobuf::FieldDescriptor *field_;
  google::protobuf::Message* field_content_;
  std::mutex receive_mutex_;

  // TODO(james): Maybe a separate/cleaner arena for all queues?
  google::protobuf::ArenaOptions arena_settings_;
  ::std::unique_ptr<google::protobuf::Arena> arena_;

  //! Buffer to be used when writing out to queues.
  static constexpr size_t BUF_SIZE = 1024;
  char buffer_[BUF_SIZE];
};

template <typename T>
void ProtoQueue<T>::send(const T *msg) {
  field_content_->CopyFrom(*msg);
  auto time = util::monotonic_clock::now().time_since_epoch();
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(time);
  auto nsecs = std::chrono::nanoseconds(time - secs);
  msg_header_->mutable_time()->set_seconds(secs.count());
  msg_header_->mutable_time()->set_nanos(nsecs.count());
  if (msg_header_->SerializeToArray(buffer_, BUF_SIZE)) {
    impl_.send(buffer_, msg_header_->ByteSize());
  } else {
    LOG(FATAL) << "Failed to serialize message";
  }
}

template <typename T>
bool ProtoQueue<T>::receive(T* msg) {
  std::unique_lock<std::mutex> lck(receive_mutex_);
  size_t rcvd;
  if (impl_.receive(buffer_, BUF_SIZE, rcvd)) {
    receive_header_->ParseFromArray(buffer_, rcvd);
    msg->CopyFrom(receive_header_->GetReflection()->GetMessage(*receive_header_, field_));
    return true;
  }
  return false;
}

} // namespace sailbot
