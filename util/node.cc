#include "node.h"

namespace sailbot {

Node::Node(float loop_period) : period_(loop_period), loop_(period_) {
  arena_settings_.start_block_size = 10000;
  arena_settings_.max_block_size = 0;
  arena_.reset(new google::protobuf::Arena(arena_settings_));
}

Node::~Node() {
  for (auto& thread : threads_) {
    thread.join();
  }
}

void Node::Run() {
  if (period_ < 0) {
    return;
  }
  while (!util::IsShutdown()) {
    if (period_ > 0) {
      loop_.WaitForNext();
    }
    if (util::IsShutdown()) {
      return;
    }
    Iterate();
  }
  loop_.Done();
}

}  // sailbot
