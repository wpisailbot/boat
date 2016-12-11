#pragma once

#include "node.h"
#include "ipc/queue.hpp"
#include <fcntl.h>
#include <fstream>
#include <mutex>

namespace sailbot {

class Logger : public Node {
 public:
  // TODO(james): Parameterize file name, figure out more reasonable Node
  // period.
  Logger() : Node(1.), out_("/tmp/logfilename") {
    uint64_t start_time = util::monotonic_clock::now().time_since_epoch().count();
    out_.write((const char*)&start_time, 8);
    msg::LogEntry tmp;
    const google::protobuf::Descriptor* desc = tmp.GetDescriptor();
    for (int i = 0; i < desc->field_count(); ++i) {
      RegisterLogHandler(desc->field(i)->lowercase_name().c_str());
    }
  }

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
