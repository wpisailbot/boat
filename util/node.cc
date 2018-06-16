#include "node.h"

namespace sailbot {

Node::Node(float loop_period) : period_(loop_period), loop_(period_) {
  // Arenas allocate memory in blocks; we want to
  // create one large initial block and then zero further blocks.
  arena_settings_.start_block_size = 10000;
  // Set max_block_size to prevent creation of more blocks
  arena_settings_.max_block_size = 0;
  arena_.reset(new google::protobuf::Arena(arena_settings_));
}

Node::~Node() {
  for (auto& thread : threads_) {
    thread.join();
  }
}

void Node::Run() {
  while (period_ >= 0 && !util::IsShutdown()) {
    if (period_ > 0) {
      loop_.WaitForNext();
    }
    if (util::IsShutdown()) {
      break;
    }
    Iterate();
  }
  loop_.Done();
}

}  // sailbot
