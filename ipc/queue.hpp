// Author: James Kuszmaul <jabukuszmaul@gmail.com>
// Contains Queue and ProtoQueue<T>, which uses Queue to send back and forth
// protobuf messages.
#pragma once

#include "message_queue.hpp"
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <cstring>
#include <array>

#include "glog/logging.h"

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
   * @param queue_len The maximum number of messages allowed in the queue.
   * @param msg_size The mazimum size that a message will be.
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
  void receive(void *buffer, size_t size, size_t &rcvd);

  /**
   * @brief removes everything in shared memory. Call when doing global init.
   *
   * If you don't call this and the various pieces of shared memory already are
   * initialized, then things may get awkward.
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

/**
 * Message queue for sending protobufs back and forth; deals with serializing
 * and parsing the protobufs for you.
 */
template <typename T>
class ProtoQueue {
 public:
  // TODO(james): Parameterize number of messages.
  // @param name name of the queue to use.
  ProtoQueue(const char *name)
      : impl_(name, 10 /*number of messages*/, BUF_SIZE) {}

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
    impl_.send(buffer_, msg->ByteSize());
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
