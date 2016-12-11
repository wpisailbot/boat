#include "logger.h"
#include "ipc/queue.hpp"
#include "util/msg.pb.h"
#include <fstream>

namespace sailbot {

/**
 * Log file format:
 * First 8 bytes: little-endian start time in nanoseconds since epoch.
 * All following bytes are the following, repeated until EOF:
 * 2 little-endian bytes for length of message
 * the above number of data bytes, to be parsed as a msg::LogEntry protobuf.
 */

Logger::~Logger() {
  for (auto &thread : threads_) {
    thread.detach();
  }
}

void Logger::RegisterLogHandler(const char *name) {
  threads_.emplace_back(&Logger::RunLogHandler, this, name);
}

void Logger::RunLogHandler(const char *name) {
  Queue q(name, false);
  char buf[MAX_BUF];
  size_t rcvd;
  // TODO(james): Figure out how to make log message be written in order (by time)--
  // currently messages for each individual queue will be in order, but messages
  // from different queues could arrive at different times.
  while (!util::IsShutdown()) {
    q.receive(buf, MAX_BUF, rcvd);
    uint16_t n = rcvd;
    std::unique_lock<std::mutex> lck(out_lock_);
    out_.write((const char*)&n, 2);
    out_.write(buf, rcvd);
  }
}

void ReadFile(const char *name) {
  std::ifstream file(name);
  msg::LogEntry entry;
  msg::PingMsg foo;
  while (!file.eof()) {
    uint16_t len;
    file.read((char*)&len, 2);
    if (file.eof()) break;
    char buf[Logger::MAX_BUF];
    file.read(buf, len);
    if (file.eof()) break;
    entry.ParseFromArray(buf, len);
    std::vector<const google::protobuf::FieldDescriptor*> fields;
    entry.GetReflection()->ListFields(entry, &fields);
    for (const auto field : fields) {
      if (field->lowercase_name() == "ping") {
        foo = entry.ping();
        LOG(INFO) << foo.a();
      }
    }
  }
}

}  // sailbot
