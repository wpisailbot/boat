#include "server.h"
#include "util/msg.pb.h"
#include "util/proto_util.h"
#include <google/protobuf/util/json_util.h>
#include <functional>
#include <cstring>

namespace sailbot {

WebSocketServer::WebSocketServer(int port) : port_(port) {
  msg::LogEntry tmp;
  const google::protobuf::Descriptor* desc = tmp.GetDescriptor();
  std::unique_lock<std::mutex> send_lck(send_map_mutex_);
  for (int i = 0; i < desc->field_count(); ++i) {
    std::string name = desc->field(i)->lowercase_name();
    RegisterQueueHandler(name);
    send_msgs_[name] = std::make_unique<Queue>(name.c_str(), true);
  }

  hub_.onMessage(std::bind(&WebSocketServer::ProcessSocket, this,
                           std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3, std::placeholders::_4));
}

WebSocketServer::~WebSocketServer() {
  for (auto &thread : threads_) {
    thread.join();
  }
  hub_.getDefaultGroup<uWS::SERVER>().close();
  usleep(1000000);
}

void WebSocketServer::Run() {
  hub_.listen(port_);
  //threads_.emplace_back([this]() { hub_.run(); });
  std::thread t([this]() { hub_.run(); });
  t.detach();
}

void WebSocketServer::RegisterQueueHandler(const std::string name) {
  threads_.emplace_back(&WebSocketServer::ProcessQueue, this, name);
}

void WebSocketServer::ProcessQueue(const std::string name) {
  Queue q(name.c_str(), false);
  size_t BUF_LEN = 1024;
  char buf[BUF_LEN];
  size_t rcvd;
  msg::LogEntry entry;
  while (!util::IsShutdown()) {
    // TODO(james): Shutdown while receiving from queue.
    if (!q.receive(buf, BUF_LEN, rcvd)) {
      continue;
    }
    if (util::IsShutdown())
      return;
    std::unique_lock<std::mutex> lck(map_mutex_);
    entry.ParseFromArray(buf, rcvd);
    msgs_[name] = &entry;
  }
  msgs_[name] = nullptr;
}

// Message format: JSON object. For each object member, if it is null,
// we will return JSON with the current values for everything.
// If a value is provided for the member, then it must be an object
// and the member name must correspond to a queue (ie, no periods).
// Note: For simplicity, there should be no extra whitespace.
void WebSocketServer::ProcessSocket(uWS::WebSocket<uWS::SERVER> ws,
                                     char *message, size_t length,
                                     uWS::OpCode opcode) {
  std::string msg(message, length);
  // First, get rid of starting/trailing "{" "}".
  if (msg.front() == '{') {
    msg.erase(msg.begin());
  }
  if (msg.back() == '}') {
    msg.pop_back();
  }
  std::string out_str =
      "{\"time\":" +
      std::to_string(util::monotonic_clock::now().time_since_epoch().count() *
                     1e-9) +
      ",";
  while (msg.find(':') != msg.npos) {
    size_t start_name = msg.find('"') + 1;
    size_t end_name = msg.find('"', start_name);
    std::string name = msg.substr(start_name, end_name - start_name);
    msg.erase(0, msg.find(':') + 1);
    // Count opening [, {
    int open_cnt = 0;
    bool in_quote = false;
    size_t end_val = 0;
    for (size_t i = 0; i < msg.size(); ++i) {
      char c = msg[i];
      end_val = i;
      if (c == '"') {
        in_quote = !in_quote;
        continue;
      } else if (!in_quote) {
        if (c == '[' || c == '{') {
          ++open_cnt;
        } else if (c == ']' || c == '}') {
          --open_cnt;
        }
        if (open_cnt < 0 || (c == ',' && open_cnt == 0)) {
          break;
        }
      }
    }
    // If we ran out of string:
    if (open_cnt > 0) {
      LOG(WARNING) << "Invalid JSON in queue " << name << ": " << msg;
      return;
    } else if (open_cnt == 0 && msg[end_val] != ',') {
      ++end_val;
    }

    std::string val = msg.substr(0, end_val);
    msg.erase(0, end_val);

    if (val == "null") {
      out_str += GetCurrentVal(name);
    } else {
      if (name.find('.') == name.npos) {
        msg::LogEntry tmp;
        const google::protobuf::Descriptor *descriptor = tmp.GetDescriptor();
        const google::protobuf::FieldDescriptor *field =
            descriptor->FindFieldByName(name);
        if (field == nullptr) {
          LOG(WARNING) << "Unable to find queue " << name;
          continue;
        }
        google::protobuf::Message *submsg =
            tmp.GetReflection()->MutableMessage(&tmp, field);
        if (!google::protobuf::util::JsonStringToMessage(val, submsg).ok()) {
          LOG(WARNING) << "Unable to parse string \"" << val << "\" to queue \""
                       << name << "\"";
        } else {
          std::unique_lock<std::mutex> lck(send_map_mutex_);
          char buf[1024];
          tmp.SerializeToArray(buf, 1024);
          // TODO(james): fill out current time in tmp.
          send_msgs_[name]->send(buf, tmp.ByteSize());
        }
      } else {
        LOG(WARNING) << "Attempting to set individual field of queue " << name
                     << " val: " << val;
      }
    }
  }
  out_str.back() = '}'; // Overwrite Ending comma.
  ws.send(out_str.c_str(), out_str.size(), opcode);
}

std::string WebSocketServer::GetCurrentVal(const std::string &msg) {
  size_t index = 0;
  size_t next = msg.find('.');
  std::unique_lock<std::mutex> lck(map_mutex_);
  const google::protobuf::Message* submsg = msgs_[msg.substr(0, next)];
  if (submsg == nullptr) {
    LOG(WARNING) << "No data yet on " << msg.substr(0, next);
    return "";
  }

  while ((next = msg.find('.', index)) != msg.npos) {
    const std::string& msg_name = msg.substr(index, next-index);
    const google::protobuf::FieldDescriptor* f =
        submsg->GetDescriptor()->FindFieldByName(msg_name);
    if (f == nullptr) {
      LOG(WARNING) << "Unable to locate field \"" << msg_name << "\" in \""
                   << msg << "\"";
      return "";
    }

    submsg = &submsg->GetReflection()->GetMessage(*submsg, f);

    index = next + 1;
  }
  const google::protobuf::FieldDescriptor* val_field =
      submsg->GetDescriptor()->FindFieldByName(msg.substr(index));

  if (val_field == nullptr) {
    LOG(WARNING) << "Didn't find field \"" << msg << "\" in message";
    return "";
  }

  const google::protobuf::Reflection* reflect = submsg->GetReflection();
  std::string out_str;
  switch (val_field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      google::protobuf::util::MessageToJsonString(
          reflect->GetMessage(*submsg, val_field), &out_str);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      out_str = reflect->GetString(*submsg, val_field);
      break;
    default: // A Number
      out_str = std::to_string(util::GetProtoNumberField(val_field, submsg));
      break;
  }

  out_str = "\"" + msg + "\":" + out_str + ",";
  return out_str;
}

}  // namespace sailbot
