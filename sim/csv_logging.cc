#include "csv_logging.h"

#include "ipc/queue.hpp"
#include "util/msg.pb.h"

namespace sailbot {

CsvLogger::CsvLogger(std::vector<std::pair<std::string, std::string>> data,
                     std::string fname, float dt)
    : Node(dt), file_(fname) {
  std::map<std::string, std::vector<std::string>> fields;
  file_ << "#Time";
  for (const auto& d : data) {
    std::string msg_name = d.first.substr(0, d.first.find('.'));
    fields[msg_name].push_back(d.first);
    data_indices_[d.first] = data_.size();
    data_.push_back(0);
    file_ << "," << d.second;
  }
  file_ << std::endl;
  for (const auto& p : fields) {
    threads_.emplace_back(&CsvLogger::ProcessInput, this, p.first.c_str(), p.second);
  }
}

CsvLogger::~CsvLogger() {
  for (auto& thread : threads_) {
    thread.detach();
  }
}

void CsvLogger::ProcessInput(const char* name,
                             std::vector<std::string> fields) {
  Queue q(name, false);
  size_t BUF_LEN = 256;
  char buf[BUF_LEN];
  size_t rcvd;
  msg::LogEntry *entry = AllocateMessage<msg::LogEntry>();
  while (!util::IsShutdown()) {
    // TODO(james): Shutdown while receiving from queue.
    q.receive(buf, BUF_LEN, rcvd);
    entry->ParseFromArray(buf, rcvd);
    for (const auto& item : fields) {
      double val = GetField(*entry, item);
      std::unique_lock<std::mutex> lck(data_mutex_);
      data_[data_indices_[item]] = val;
    }
  }
}

double CsvLogger::GetField(const msg::LogEntry& msg, const std::string& field) {
  size_t n = 0;
  size_t next = field.find('.');
  const google::protobuf::Message* submsg = &msg;
  while (next != field.npos) {
    const google::protobuf::FieldDescriptor* f =
        submsg->GetDescriptor()->FindFieldByName(field.substr(n, next-n));
    if (f == nullptr) {
      LOG(FATAL) << "Didn't find field \"" << field.substr(0, next)
                 << "\" in message";
    }
    submsg = &submsg->GetReflection()->GetMessage(*submsg, f);
    n = next+1;
    next = field.find('.', n);
  }
  n = field.rfind('.')+1;
  const google::protobuf::FieldDescriptor* val_field =
      submsg->GetDescriptor()->FindFieldByName(field.substr(n));
  if (val_field == nullptr) {
    LOG(FATAL) << "Didn't find field \"" << field << "\" in message";
  }
  const google::protobuf::Reflection& reflect = *submsg->GetReflection();
  switch (val_field->type()) {
    case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
      return reflect.GetDouble(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_FLOAT:
      return reflect.GetFloat(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_BOOL:
      return reflect.GetBool(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_ENUM:
      return reflect.GetEnumValue(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_INT32:
    case google::protobuf::FieldDescriptor::TYPE_SINT32:
    case google::protobuf::FieldDescriptor::TYPE_SFIXED32:
      return reflect.GetInt32(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_INT64:
    case google::protobuf::FieldDescriptor::TYPE_SFIXED64:
    case google::protobuf::FieldDescriptor::TYPE_SINT64:
      return reflect.GetInt64(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_UINT32:
    case google::protobuf::FieldDescriptor::TYPE_FIXED32:
      return reflect.GetUInt32(*submsg, val_field);
    case google::protobuf::FieldDescriptor::TYPE_UINT64:
    case google::protobuf::FieldDescriptor::TYPE_FIXED64:
      return reflect.GetUInt64(*submsg, val_field);
    default:
      return 0;
  }
}

void CsvLogger::Iterate() {
  size_t BUF_LEN = 1024;
  char buf[BUF_LEN];
  int index = 0;
  index += snprintf(
      buf, BUF_LEN, "%f,",
      std::chrono::nanoseconds(util::monotonic_clock::now().time_since_epoch())
              .count() /
          1e9);
  {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (const auto &item : data_) {
      index += snprintf(buf+index, BUF_LEN-index, "%f,", item);
    }
  }
  buf[index-1] = '\n'; // Overwrite last comma
  file_.write(buf, index);
}

}  // sailbot
