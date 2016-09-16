// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include "clock.h"
#include <functional>
#include <memory>
#include <math.h>
#include "ipc/queue.hpp"
#include <thread>

#include "google/protobuf/arena.h"

namespace sailbot {

class Node {
 public:
  Node(float loop_period);

  void Run();

 protected:
  template <typename T>
  void RegisterHandler(const char *queue_name,
                       ::std::function<void(const T &)> callback);

  // Note: If you create too many messages, you will run out of Arena space
  // and end up with something bad happening. Reuse allocated messages.
  template <typename T>
  T *AllocateMessage();

  //time Time();

  virtual void Iterate() = 0;

 private:
  template <typename T>
  void RunHandlerCaller(const char *queue_name,
                        ::std::function<void(const T &)> callback);
  util::Loop loop_;

  google::protobuf::ArenaOptions arena_settings_;
  ::std::unique_ptr<google::protobuf::Arena> arena_;
};

template <typename T>
void Node::RegisterHandler(const char *queue_name,
                           ::std::function<void(const T &)> callback) {
  ::std::thread handler(&Node::RunHandlerCaller<T>, this, queue_name, callback);
  handler.detach();
}

template <typename T>
T *Node::AllocateMessage() {
  return google::protobuf::Arena::CreateMessage<T>(arena_.get());
}

template <typename T>
void Node::RunHandlerCaller(const char *queue_name,
                            ::std::function<void(const T &)> callback) {
  ProtoQueue<T> q(queue_name);
  T* buffer = AllocateMessage<T>();
  // TODO(james): Shutdown cleanly.
  while (true) {
    q.receive(buffer);
    callback(*buffer);
  }
}

}  // sailbot
