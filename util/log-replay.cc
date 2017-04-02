#include "log-replay.h"
#include <fstream>
#include "util/msg.pb.h"
#include <vector>
#include "gflags.h"

DEFINE_string(input_log, "/tmp/logfilename",
              "The log file to read from when replaying");

namespace sailbot {

LogReplay::LogReplay() : input_(FLAGS_input_log) { Init(); }

void LogReplay::Init() {
  util::ClockManager::SetFakeClock(true);
  uint64_t start_time;
  input_.read((char*)&start_time, 8);
  if (input_.eof()) {
    LOG(WARNING) << "Log already ran out";
    util::RaiseShutdown();
  }
  std::thread clock_thread(&util::ClockManager::Run, start_time);
  CHECK(util::monotonic_clock::next_wakeup() == 0)
      << "All nodes should be initialized before running them.";
  clock_thread.detach();
}

void LogReplay::Run() {
  msg::LogEntry entry;
  char msg_buffer[Logger::MAX_BUF];
  util::monotonic_clock::time_point cur_time = clock_.Time();
  util::monotonic_clock::time_point msg_time = cur_time;
  std::string queue_name;
  while (!util::IsShutdown()) {
    int n = ReadLogMessage(&entry, msg_buffer, Logger::MAX_BUF);
    if (n < 0) {
      util::RaiseShutdown();
      return;
    }
    std::vector<const google::protobuf::FieldDescriptor*> fields;
    entry.GetReflection()->ListFields(entry, &fields);
    CHECK_LE(fields.size(), 2) << "We only support log entries with a timestamp + ONE message";
    for (const auto field : fields) {
      if (field->lowercase_name() == "time") {
        msg_time =
            util::monotonic_clock::time_point(util::monotonic_clock::duration(
                entry.time().seconds() * 1000000000 + entry.time().nanos()));
      } else {
        queue_name = field->lowercase_name();
        if (queues_.count(queue_name) == 0) {
          queues_[queue_name].reset(new Queue(queue_name.c_str(), true));
        }
      }
    }
    if (msg_time > cur_time) {
      // Wait for the time to come...
      clock_.SleepUntil(msg_time);
      cur_time = clock_.Time();
    }
    if (queue_name.size() > 0)
      queues_[queue_name]->send(msg_buffer, n);
  }
}

// Return length of buffer read. If less than 0, a problem.
int LogReplay::ReadLogMessage(msg::LogEntry *msg, char *buf, size_t buf_len) {
  uint16_t len;
  input_.read((char*)&len, 2);
  if (input_.eof()) {
    LOG(WARNING) << "EOF on log input file";
    return -1;
  }
  CHECK(len < buf_len) << "Trying to read an excessively large message. buf_len = " << buf_len;
  input_.read(buf, len);
  if (input_.eof()) {
    LOG(WARNING) << "EOF on log input file";
    return -1;
  }
  msg->ParseFromArray(buf, len);
  return len;
}

}  // sailbot
