#include "logger.h"
#include "ipc/queue.hpp"
#include "util/msg.pb.h"
#include <fstream>

namespace sailbot {

void Logger::RegisterLogHandler(const char *name) {
  ::std::thread handler(&Logger::RunLogHandler, this, name);
  handler.detach();
}

void Logger::RunLogHandler(const char *name) {
  Queue q(name);
  char buf[MAX_BUF];
  size_t rcvd;
  LogEntry *entry = AllocateMessage<LogEntry>();
  const google::protobuf::FieldDescriptor *field =
      entry->GetDescriptor()->FindFieldByName(name);
  google::protobuf::Message *sub_msg;
  // TODO(james): Shutdown cleanly.
  while (true) {
    q.receive(buf, MAX_BUF, rcvd);
    timespec t = Time();
    entry->mutable_time()->set_seconds(t.tv_sec);
    entry->mutable_time()->set_nanos(t.tv_nsec);
    sub_msg = entry->GetReflection()->MutableMessage(entry, field);
    sub_msg->ParseFromArray(buf, rcvd);

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
