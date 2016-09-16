#pragma once

#include "node.h"
#include "ipc/queue.hpp"
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <fcntl.h>
#include <mutex>

namespace sailbot {

struct QueueInfo {
  const char *name;
  int id;
};

static std::vector<QueueInfo> queue_index = {{"ping", 1311}, {"ping", 1312}};

class Logger : public Node {
 public:
  Logger() : Node(1.), out_(open("/tmp/logfilename", O_CREAT | O_WRONLY)) {
    out_.SetCloseOnDelete(true);
    for (const QueueInfo& i : queue_index) {
      RegisterLogHandler(i);
    }
  }

  static constexpr size_t MAX_BUF = 128;
 private:
  void Iterate() {
    std::unique_lock<std::mutex> lck(out_lock_);
    out_.Flush();
  };
  void RegisterLogHandler(const QueueInfo& info);
  void RunLogHandler(const QueueInfo& info);
  google::protobuf::io::FileOutputStream out_;
  std::mutex out_lock_;
};

void ReadFile(const char *name);

}  // sailbot
