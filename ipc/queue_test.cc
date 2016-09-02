#include "queue.hpp"

#include <iostream>
#include <unistd.h>

#include "gtest/gtest.h"

namespace sailbot {
namespace test {

struct Data {
  double d;
  int i;
  char c;
};

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
    Queue<void>::remove(name);
    /*
    bst::message_queue::remove(name);
    bst::message_queue::remove(name);
    */
  }
};

TEST_F(QueueTest, UpDown) {
  Queue<Data> data(name);
  Data send, rcv;
  send.i = 10;
  rcv.i = 99;
  data.send(&send);
  data.receive(&rcv);
  EXPECT_TRUE(rcv.i == send.i);
}

TEST_F(QueueTest, Priority) {
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

TEST_F(QueueTest, MultipleProcess) {
  pid_t pid = fork();
  // Now in separate processes.
  Data send = {2.5, 10, 'c'};
  if (pid == 0) {
    // Child process; receive.
    {
      Queue<Data> data(name);
      Data rcv = {7.5, 440, '!'};
      data.receive(&rcv);
      ASSERT_EQ(send.d, rcv.d);
    }
    exit(0);
  } else if (pid > 0) {
    // Parent process
    Queue<Data> data(name);
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
