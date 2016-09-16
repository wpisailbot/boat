// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <cstring>
#include <array>

#include "glog/logging.h"

namespace sailbot {

// TODO(james): Fix documentation.

/**
 * @brief A wrapper for the boost boost message_queue class.
 *
 * @tparam T The object being passed through the queue. Should be a protobuf.
 *
 * The primary reasons for this wrapper are:
 * - Automatically deal with cleaning up shared memory.
 * - Slightly clean up the message_queue interface for our uses.
 *
 * **Method of operation:**
 *
 * In order to keep track of when to completely destroy the shared memory,
 * we choose to assume that there will never be an period when no processes
 * are using the shared memory. The boost libraries do not make such an
 * assumption and so using the boost libraries the user has to keep track
 * of when to remove the shared memory.
 *
 * In order to keep track of this, we use a semaphore to keep
 * track of the number of instances, across all processes, of Queues using
 * the shared memory queue in question.
 */
class Queue {
 public:
  /**
   * @brief Construct a Queue, initializing shared memory as necessary.
   *
   * @param name The name to use for the memory mapped file for the queue.
   * @param queue_len The maximum number of messages allowed in the queue.
   */
  Queue(const char *name, size_t queue_len=10, size_t msg_size=128);

  /**
   * @brief Cleans up shared memory as necessary.
   */
  ~Queue();

  /**
   * @brief Append a message to the queue.
   *
   * See the boost::interprocess::message_queue documentation for more
   *   information; this function just calls message_queue::send after
   *   serializing the protobuf.
   *
   * @param buffer The object to append to the queue.
   * @param priority The priority of the message within the queue (used to jump
   *   the queue).
   */
  void send(const void *buffer, size_t size, unsigned int priority=0);

  /**
   * @brief Retrieve the next message from the queue, blocking until available.
   *
   * See the boost::interprocess::message_queue documentation for more
   *   information; this function just calls message_queue::receive.
   *
   * @param buffer The buffer to fill the the new message.
   * @param priority The priority of the received message.
   */
  void receive(void *buffer, size_t size, size_t &rcvd, unsigned int &priority);
  void receive(void *buffer, size_t size, size_t &rcvd) {
    unsigned p;
    receive(buffer, size, rcvd, p);
  }

  /**
   * @brief removes everything in shared memory. Call when doing global init.
   *
   * If you don't call this and the various pieces of shared memory already are
   * initialized, then things will get awkward.
   */
  static void remove(const char *name);

 private:
  //! The name to be used for the shared memory queue itself.
  const char *name_;

  //! @brief The boost queue object to use.
  //!
  //! This is a pointer so as to allow us to destroy queue_ before calling
  //! message_queue::remove().
  boost::interprocess::message_queue *queue_;

  //! @brief Semaphore to keep track of number of Queue objects using the queue.
  boost::interprocess::named_semaphore *semaphore_;
};


template <typename T>
class ProtoQueue {
 public:
  ProtoQueue(const char *name) : impl_(name, 10, BUF_SIZE) {}

  void send(const T *msg);
  void receive(T *msg);
 private:
  Queue impl_;

  //! Buffer to be used when writing out to queues.
  static constexpr size_t BUF_SIZE = 128;
  char buffer_[BUF_SIZE];
};

template <typename T>
void ProtoQueue<T>::send(const T *msg) {
  if (msg->SerializeToArray(buffer_, BUF_SIZE)) {
    impl_.send(buffer_, msg->ByteSize(), 0);
  } else {
    LOG(FATAL) << "Failed to serialize message";
  }
}

template <typename T>
void ProtoQueue<T>::receive(T* msg) {
  size_t rcvd;
  impl_.receive(buffer_, BUF_SIZE, rcvd);
  msg->ParseFromArray(buffer_, rcvd);
}

} // namespace sailbot
