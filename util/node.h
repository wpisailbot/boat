// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include "clock.h"
#include <functional>
#include <memory>
#include <math.h>
#include "ipc/queue.hpp"
#include <thread>
#include <atomic>

#include "google/protobuf/arena.h"

namespace sailbot {

/**
 * A class that all processes should inherit from, should handle everything.
 * For now, mostly sticks to making it even easier to use the Loop interface and
 * to register handlers to watch queues.
 */
class Node {
 public:
  Node(float loop_period);
  ~Node();

  virtual void Run();
 protected:
  template <typename T>
  void RegisterHandler(const char *queue_name,
                       ::std::function<void(const T &)> callback);

  // Note: If you create too many messages, you will run out of Arena space
  // and end up with something bad happening. Reuse allocated messages.
  template <typename T>
  T *AllocateMessage();

  timespec Time() {
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return time;
  }

  virtual void Iterate() {}

 private:
  // Note: takes ownership of queue.
  template <typename T>
  void RunHandlerCaller(std::unique_ptr<char[]> queue_name,
                        ::std::function<void(const T &)> callback);
  double period_;

  google::protobuf::ArenaOptions arena_settings_;
  std::unique_ptr<google::protobuf::Arena> arena_;

  std::vector<std::thread> threads_;
  std::unique_ptr<std::thread> run_thread_;
};

template <typename T>
void Node::RegisterHandler(const char *queue_name,
                           ::std::function<void(const T &)> callback) {
  size_t len = strlen(queue_name)+1;
  std::unique_ptr<char[]> name = std::make_unique<char[]>(len);
  strncpy(name.get(), queue_name, len);
  threads_.emplace_back(&Node::RunHandlerCaller<T>, this, std::move(name),
                        callback);
}

template <typename T>
T *Node::AllocateMessage() {
  return google::protobuf::Arena::CreateMessage<T>(arena_.get());
}

template <typename T>
void Node::RunHandlerCaller(std::unique_ptr<char[]> queue_name,
                            ::std::function<void(const T &)> callback) {
  ProtoQueue<T> q(queue_name.get(), false);
  T* buffer = AllocateMessage<T>();
  // TODO(james): Shutdown cleanly; currently won't handle shutdown while
  // waiting on queue receive.
  while (!util::IsShutdown()) {
    if (q.receive(buffer) && !util::IsShutdown()) {
      callback(*buffer);
    }
  }
}

}  // sailbot
