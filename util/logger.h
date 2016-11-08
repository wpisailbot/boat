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

// Once we start creating logs, we can not change what messages correspond
// to which number (the number is arbitrary, it is just used for identification
// in the message).
// TODO(james): Put this in a better place
static std::vector<QueueInfo> queue_index = {{"ping", 1311}, {"ping", 1312}};

class Logger : public Node {
 public:
  // TODO(james): Parameterize file name, figure out more reasonable Node
  // period.
  Logger() : Node(1.), out_(open("/tmp/logfilename", O_CREAT | O_WRONLY)) {
    out_.SetCloseOnDelete(true);
    for (const QueueInfo& i : queue_index) {
      RegisterLogHandler(i.name);
    }
  }

  static constexpr size_t MAX_BUF = 128;
 private:
  void Iterate() {
    std::unique_lock<std::mutex> lck(out_lock_);
    out_.Flush();
  };
  void RegisterLogHandler(const char* info);
  void RunLogHandler(const char* info);
  google::protobuf::io::FileOutputStream out_;
  std::mutex out_lock_;
};

void ReadFile(const char *name);

}  // sailbot
