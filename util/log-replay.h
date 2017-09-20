#pragma once

#include "logger.h"
#include "util/node.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <map>

namespace sailbot {

/**
 * LogReplay notes:
 * -Don't care about Arena allocation.
 */

class LogReplay {
 public:
  LogReplay(const std::map<std::string, std::string> &rename,
            bool default_ignore = false);
  void Run();
 private:
  void Init();
  int ReadLogMessage(msg::LogEntry* msg, char *buf, size_t buf_len);
  util::ClockInstance clock_;
  std::ifstream input_;
  std::map<std::string, std::unique_ptr<Queue>> queues_;
  std::map<std::string, std::string> rename_;
  const bool default_ignore_;
};

}  // sailbot
