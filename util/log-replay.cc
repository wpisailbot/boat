#include "log-replay.h"
#include <fstream>
#include "util/msg.pb.h"
#include <vector>
#include "gflags.h"

DEFINE_string(input_log, "/tmp/logfilename",
              "The log file to read from when replaying");

namespace sailbot {

LogReplay::LogReplay(const std::map<std::string, std::string> &rename,
                     bool default_ignore)
    : input_(FLAGS_input_log), rename_(rename),
      default_ignore_(default_ignore) {
  Init();
}

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
          std::string qname = default_ignore_ ? "" : queue_name;
          if (rename_.count(queue_name)) {
            qname = rename_[queue_name];
          }
          if (qname.size() > 0) {
            queues_[queue_name].reset(new Queue(qname.c_str(), true));
          }
        }
      }
    }
    if (msg_time > cur_time) {
      // Wait for the time to come...
      clock_.SleepUntil(msg_time);
      cur_time = clock_.Time();
    }
    // Handle moving the fields around within the LogEntry prot
    if (rename_.count(queue_name) > 0 && rename_[queue_name] != queue_name) {
      const google::protobuf::FieldDescriptor *new_field =
          entry.GetDescriptor()->FindFieldByName(rename_[queue_name]);
      const google::protobuf::FieldDescriptor *old_field =
          entry.GetDescriptor()->FindFieldByName(queue_name);
      if (old_field == nullptr || new_field == nullptr) {
        LOG(FATAL) << "Queue \"" << rename_[queue_name] << "\" or \""
                   << queue_name
                   << "\" does not exist in the LogEntry proto...";
      }
      google::protobuf::Message *field_content =
          entry.GetReflection()->MutableMessage(&entry, new_field);
      field_content->CopyFrom(
          entry.GetReflection()->GetMessage(entry, old_field));
      entry.GetReflection()->ClearField(&entry, old_field);
      entry.SerializeToArray(msg_buffer, Logger::MAX_BUF);
      n = entry.ByteSize();
    }

    if (queue_name.size() > 0 && queues_[queue_name]) {
      queues_[queue_name]->send(msg_buffer, n);
    }
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
