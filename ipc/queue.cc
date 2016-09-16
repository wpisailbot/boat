#include "queue.hpp"

namespace sailbot {

Queue::Queue(const char *name, size_t queue_len, size_t msg_size) : name_(name) {
  namespace bst = boost::interprocess;

  semaphore_ = new bst::named_semaphore(bst::open_or_create, name, 0/*Initial Value*/);
  semaphore_->post();
  queue_ =
      new bst::message_queue(bst::open_or_create, name_, queue_len, msg_size);
}

Queue::~Queue() {
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

void Queue::send(const void *msg, size_t size) {
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
