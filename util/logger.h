#pragma once

#include "node.h"
#include "ipc/queue.hpp"
#include <fcntl.h>
#include <fstream>
#include <mutex>

namespace sailbot {

class Logger : public Node {
 public:
  // TODO(james): figure out more reasonable Node period.
  Logger();

  ~Logger();

  static constexpr size_t MAX_BUF = 1024;
 private:
  void Iterate() {
    std::unique_lock<std::mutex> lck(out_lock_);
    out_.flush();
  };
  void RegisterLogHandler(const char* info);
  void RunLogHandler(const char* info);
  std::ofstream out_;
  std::mutex out_lock_;

  std::vector<std::thread> threads_;
};

void ReadFile(const char *name);

}  // sailbot
