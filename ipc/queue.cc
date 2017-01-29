#include "queue.hpp"
//#include <boost/date_time/microsec_time_clock.hpp>

namespace sailbot {

bool Queue::testing_ = false;

Queue::Queue(const char *name, bool writer, size_t queue_len, size_t msg_size)
    : writer_(writer) {
  strncpy(name_, name, sizeof(name_));

  if (!testing_) {
    namespace bst = boost::interprocess;

    semaphore_ = new bst::named_semaphore(bst::open_or_create, name_, 0/*Initial Value*/);
    if (writer) {
      queue_ =
          new bst::message_queue(bst::open_or_create, name_, queue_len, msg_size);
      semaphore_->post();
    } else {
      // Wait for a writer to start up before doing anything.
      while (!util::IsShutdown()) {
        boost::posix_time::ptime time(
            boost::date_time::microsec_clock<
                boost::posix_time::ptime>::universal_time() +
            boost::posix_time::millisec(1000));
        if (semaphore_->timed_wait(time)) {
          semaphore_->post();
          queue_ = new bst::message_queue(bst::open_only, name_);
          break;
        }
      }
    }
  } else {
    // Testing
    test_impl_.reset(new testing::TestQueue(name_, msg_size));
  }
}

Queue::~Queue() {
  if (!testing_) {
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
}

void Queue::send(const void *msg, size_t size) {
  if (!writer_) {
    LOG(FATAL) << "You said you wouldn't be writing on this queue.";
  }
  if (!testing_) {
    queue_->send(msg, size);
  } else {
    test_impl_->send(msg, size);
  }
}

bool Queue::receive(void *msg, size_t size, size_t &rcvd) {
  if (!testing_) {
    return queue_->timed_receive(
        msg, size, rcvd, boost::date_time::microsec_clock<
                             boost::posix_time::ptime>::universal_time() +
                             boost::posix_time::milliseconds(1000));
  } else {
    return test_impl_->receive(msg, size, rcvd);
  }
}

void Queue::remove(const char *name) {
  namespace bst = boost::interprocess;
  bst::named_semaphore::remove(name);
  bst::message_queue::remove(name);
}

} // namespace sailbot
