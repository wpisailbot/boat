#include "queue.hpp"

#include "ipc/queue_test_msg.pb.h"

#include <iostream>
#include <unistd.h>

#include "gtest/gtest.h"

namespace sailbot {
namespace test {

namespace bst = boost::interprocess;

class QueueTest : public ::testing::Test {
 public:
  const char *name = "test_queue";

  void TearDown() {

    // Test to see if the queue has managed to stick around after destruction.
    // This will throw an exception if name already exists.
    {
      bst::message_queue(bst::create_only, name, 10, 1);
      bst::named_semaphore(bst::create_only, name, 0);
    }
    Queue::remove(name);
    /*
    bst::message_queue::remove(name);
    bst::message_queue::remove(name);
    */
  }
};

TEST_F(QueueTest, UpDown) {
  ProtoQueue<QueueTestMsg> data(name);
  QueueTestMsg send, rcv;
  send.set_foo(10);
  rcv.set_foo(99);
  data.send(&send);
  data.receive(&rcv);
  EXPECT_EQ(rcv.foo(), send.foo());
}

#if 0
TEST_F(QueueTest, DISABLED_Priority) {
  Queue<int> data(name);
  int one=1, two=2, three=3;
  // We well send one, then two, then three.
  // three will be sent with a priority of 1.
  // We will do a receive before sending three and then two after.
  // The expected order of receipt is one, three, two.
  data.send(&one);
  data.send(&two);
  unsigned priority;
  int rcv;
  data.receive(&rcv, priority);
  EXPECT_EQ(rcv, one);
  EXPECT_EQ(priority, 0u);
  data.send(&three, 1);
  data.receive(&rcv, priority);
  EXPECT_EQ(rcv, three);
  EXPECT_EQ(priority, 1u);
  data.receive(&rcv, priority);
  EXPECT_EQ(rcv, two);
  EXPECT_EQ(priority, 0u);
}
#endif

TEST_F(QueueTest, MultipleProcess) {
  pid_t pid = fork();
  // Now in separate processes.
  QueueTestMsg send;
  send.set_foo(10);
  if (pid == 0) {
    // Child process; receive.
    {
      ProtoQueue<QueueTestMsg> data(name);
      QueueTestMsg rcv;
      data.receive(&rcv);
      ASSERT_EQ(send.foo(), rcv.foo());
    }
    exit(0);
  } else if (pid > 0) {
    // Parent process
    ProtoQueue<QueueTestMsg> data(name);
    data.send(&send);
    // Don't conclude till child exits.
    // Otherwise, data may be destroyed before the child starts.
    int status; // TODO: Don't ignore.
    waitpid(pid, &status, 0);
  } else {
    FAIL() << "Failed to fork.";
  }
}

} // namespace test
} // namespace sasilbot
