#include "node.h"

namespace sailbot {

Node::Node(float loop_period)
    : loop_(loop_period) {
  arena_settings_.start_block_size = 10000;
  arena_settings_.max_block_size = 0;
  arena_.reset(new google::protobuf::Arena(arena_settings_));
}

void Node::Run() {
  // TODO(james): Shutdown cleanly.
  while (true) {
    loop_.WaitForNext();
    Iterate();
  }
}


}  // sailbot
