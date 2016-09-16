#include "logger.h"
#include "ipc/queue.hpp"
#include "util/msg.pb.h"
#include <fstream>

namespace sailbot {

void Logger::RegisterLogHandler(const QueueInfo &info) {
  ::std::thread handler(&Logger::RunLogHandler, this, info);
  handler.detach();
}

void Logger::RunLogHandler(const QueueInfo& info) {
  Queue q(info.name);
  char buf[MAX_BUF];
  size_t rcvd;
  LogEntry *entry = AllocateMessage<LogEntry>();
  entry->set_id(info.id);
  // TODO(james): Shutdown cleanly.
  while (true) {
    q.receive(buf, MAX_BUF, rcvd);
    timespec t = Time();
    entry->mutable_time()->set_seconds(t.tv_sec);
    entry->mutable_time()->set_nanos(t.tv_nsec);
    entry->set_data(buf, rcvd);

    std::unique_lock<std::mutex> lck(out_lock_);
    uint16_t len_write = entry->ByteSize();
    while (true) {
      void *data;
      int data_size;
      out_.Next(&data, &data_size);
      if (data_size == 0) continue;
      if (data_size == 1) {
        out_.BackUp(1);
      }
      *(uint16_t*)data = len_write;
      out_.BackUp(data_size - 2);
      break;
    }
    entry->SerializeToZeroCopyStream(&out_);
  }
}

void ReadFile(const char *name) {
  std::ifstream file(name);
  LogEntry entry;
  PingMsg foo;
  while (!file.eof()) {
    int16_t len;
    file.read((char*)&len, 2);
    if (file.eof()) break;
    char buf[Logger::MAX_BUF];
    file.read(buf, len);
    if (file.eof()) break;
    entry.ParseFromArray(buf, len);
    if (entry.id() == 1311) {
      foo.ParseFromString(entry.data());
      LOG(INFO) << foo.a();
    }
  }
}

}  // sailbot
