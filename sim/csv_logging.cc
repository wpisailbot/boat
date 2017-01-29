#include "csv_logging.h"

#include "ipc/queue.hpp"
#include "util/msg.pb.h"
#include "util/proto_util.h"

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
    threads_.emplace_back(&CsvLogger::ProcessInput, this, p.first,
                          p.second);
  }
}

CsvLogger::~CsvLogger() {
  for (auto& thread : threads_) {
    thread.join();
  }
}

void CsvLogger::ProcessInput(const std::string name,
                             std::vector<std::string> fields) {
  Queue q(name.c_str(), false);
  size_t BUF_LEN = 1024;
  char buf[BUF_LEN];
  size_t rcvd;
  msg::LogEntry *entry = AllocateMessage<msg::LogEntry>();
  while (!util::IsShutdown()) {
    // TODO(james): Shutdown while receiving from queue.
    if (!q.receive(buf, BUF_LEN, rcvd)) {
      continue;
    }
    if (util::IsShutdown())
      return;
    entry->ParseFromArray(buf, rcvd);
    for (const auto& item : fields) {
      double val = GetField(*entry, item);
      std::unique_lock<std::mutex> lck(data_mutex_);
      data_.at(data_indices_.at(item)) = val;
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
  return util::GetProtoNumberField(val_field, submsg);
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
