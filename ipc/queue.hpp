// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <cstring>
#include <array>

namespace sailbot {
/**
 * @brief A wrapper for the boost boost message_queue class.
 *
 * @tparam T The object being passed through the struct. This should be
 *   Plain-Old-Data (eg, a C-struct of basic types, or builtin types).
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
template <typename T>
class Queue {
 public:
  /**
   * @brief Construct a Queue, initializing shared memory as necessary.
   *
   * @param name The name to use for the memory mapped file for the queue.
   * @param queue_len The maximum number of messages allowed in the queue.
   */
  Queue(const char *name, size_t queue_len=10);

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
   * @param priority The priority of the message within the queue (used to jump
   *   the queue).
   */
  void send(const T *buffer, unsigned int priority=0);

  /**
   * @brief Retrieve the next message from the queue, blocking until available.
   *
   * See the boost::interprocess::message_queue documentation for more
   *   information; this function just calls message_queue::send.
   *
   * @param buffer The buffer to fill the the new message.
   * @param priority The priority of the received message.
   */
  void receive(T *buffer, unsigned int &priority);
  void receive(T *buffer) { unsigned p; receive(buffer, p); }

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
Queue<T>::Queue(const char *name, size_t queue_len) : name_(name) {
  namespace bst = boost::interprocess;

  semaphore_ = new bst::named_semaphore(bst::open_or_create, name, 0/*Initial Value*/);
  semaphore_->post();
  queue_ =
      new bst::message_queue(bst::open_or_create, name_, queue_len, sizeof(T));
}

template <typename T>
Queue<T>::~Queue() {
  // We need to delete the queue_ before calling bst::message_queue::remove().
  delete queue_;

  semaphore_->wait();

  // Check if the semaphore has reached zero and, if so, delete it.
  if (!semaphore_->try_wait()) {
    delete semaphore_;
    remove(name_);
  }
  else {
    // If the try_wait succeeded, then re-increment the semaphore_.
    semaphore_->post();
  }
}

template <typename T>
void Queue<T>::send(const T *buffer, unsigned int priority) {
  queue_->send(buffer, sizeof(T), priority);
}

template <typename T>
void Queue<T>::receive(T *buffer, unsigned int &priority) {
  size_t recvd_size;
  queue_->receive(buffer, sizeof(T), recvd_size, priority);
  if (recvd_size != sizeof(T)) return; // TODO: error.
}

template <typename T>
void Queue<T>::remove(const char *name) {
  namespace bst = boost::interprocess;
  bst::named_semaphore::remove(name);
  bst::message_queue::remove(name);
}
} // namespace sailbot
