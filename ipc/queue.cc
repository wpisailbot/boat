#include "queue.hpp"

namespace sailbot {

Queue::Queue(const char *name, bool writer, size_t queue_len, size_t msg_size)
    : writer_(writer) {
  strncpy(name_, name, sizeof(name_));
  namespace bst = boost::interprocess;

  semaphore_ = new bst::named_semaphore(bst::open_or_create, name_, 0/*Initial Value*/);
  if (writer) {
    queue_ =
        new bst::message_queue(bst::open_or_create, name_, queue_len, msg_size);
    semaphore_->post();
  } else {
    // Wait for a writer to start up before doing anything.
    semaphore_->wait();
    semaphore_->post();
    queue_ = new bst::message_queue(bst::open_only, name_);
  }
}

Queue::~Queue() {
  // We need to delete the queue_ before calling bst::message_queue::remove().
  delete queue_;

  if (writer_) {
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
}

void Queue::send(const void *msg, size_t size) {
  if (!writer_) {
    LOG(FATAL) << "You said you wouldn't be writing on this queue.";
  }
  queue_->send(msg, size);
}

void Queue::receive(void *msg, size_t size, size_t &rcvd) {
  queue_->receive(msg, size, rcvd);
}

void Queue::remove(const char *name) {
  namespace bst = boost::interprocess;
  bst::named_semaphore::remove(name);
  bst::message_queue::remove(name);
}

} // namespace sailbot
