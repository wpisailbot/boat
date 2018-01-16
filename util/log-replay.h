#pragma once

#include "logger.h"
#include "util/node.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <map>
#include <functional>

namespace sailbot {

/**
 * LogReplay notes:
 * -Don't care about Arena allocation.
 */

class LogReplay {
 public:
  LogReplay();
  void Run();
  void SetHandler(const std::string &queue,
                  std::function<void(msg::LogEntry *, char *, int *)> handler) {
    queue_process_[queue] = handler;
  }
 private:
  void Init();
  int ReadLogMessage(msg::LogEntry* msg, char *buf, size_t buf_len);
  util::ClockInstance clock_;
  std::ifstream input_;
  std::map<std::string, std::unique_ptr<Queue>> queues_;
  // Stores a set of functions to process the queues before sending them out.
  // Meant for handling changes to queue structures for backwards compatibility.
  std::map<std::string, std::function<void(msg::LogEntry *, char *, int *)>>
      queue_process_;
};

}  // sailbot
