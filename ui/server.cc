#include "server.h"
#include "util/msg.pb.h"
#include "util/proto_util.h"
#include <functional>
#include <cstring>

namespace sailbot {

WebSocketServer::WebSocketServer(int port) : port_(port) {
  msg::LogEntry tmp;
  const google::protobuf::Descriptor* desc = tmp.GetDescriptor();
  for (int i = 0; i < desc->field_count(); ++i) {
    RegisterQueueHandler(desc->field(i)->lowercase_name());
  }

  hub_.onMessage(std::bind(&WebSocketServer::ProcessSocket, this,
                           std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3, std::placeholders::_4));
}

WebSocketServer::~WebSocketServer() {
  hub_.getDefaultGroup<uWS::SERVER>().close();
  for (auto &thread : threads_) {
    thread.join();
  }
}

void WebSocketServer::Run() {
  hub_.listen(port_);
  threads_.emplace_back([this]() { hub_.run(); });
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

void WebSocketServer::ProcessSocket(uWS::WebSocket<uWS::SERVER> ws,
                                     char *message, size_t length,
                                     uWS::OpCode opcode) {
  std::string msg(message, length);
  size_t index = 0;
  size_t next = msg.find('.');
  std::unique_lock<std::mutex> lck(map_mutex_);
  const google::protobuf::Message* submsg = msgs_[msg.substr(0, next)];
  if (submsg == nullptr) {
    LOG(WARNING) << "No data yet on " << msg.substr(0, next);
    return;
  }

  while ((next = msg.find('.', index)) != msg.npos) {
    const std::string& msg_name = msg.substr(index, next-index);
    const google::protobuf::FieldDescriptor* f =
        submsg->GetDescriptor()->FindFieldByName(msg_name);
    if (f == nullptr) {
      LOG(WARNING) << "Unable to locate field \"" << msg_name << "\" in \""
                   << msg << "\"";
      return;
    }

    submsg = &submsg->GetReflection()->GetMessage(*submsg, f);

    index = next + 1;
  }
  const google::protobuf::FieldDescriptor* val_field =
      submsg->GetDescriptor()->FindFieldByName(msg.substr(index));

  if (val_field == nullptr) {
    LOG(WARNING) << "Didn't find field \"" << msg << "\" in message";
    return;
  }

  const google::protobuf::Reflection* reflect = submsg->GetReflection();
  std::string out_str;
  switch (val_field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      out_str = reflect->GetMessage(*submsg, val_field).DebugString();
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      out_str = reflect->GetString(*submsg, val_field);
      break;
    default: // A Number
      out_str = std::to_string(util::GetProtoNumberField(val_field, submsg));
      break;
  }

  out_str = "{\"" + msg + "\":" + out_str + "}";
  ws.send(out_str.c_str(), out_str.size(), opcode);
}

}  // namespace sailbot
