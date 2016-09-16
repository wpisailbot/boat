#include "node.h"

namespace sailbot {

Node::Node(float loop_period)
    : loop_({.tv_sec = int(loop_period),
             .tv_nsec = long(1e9 * fmod(loop_period, 1))}) {
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
