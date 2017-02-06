#pragma once

#include "util/node.h"
#include "uWS.h"
#include <mutex>
#include <map>
#include <string>

namespace sailbot {

class WebSocketServer {
 public:
  WebSocketServer(int port=3000);
  ~WebSocketServer();
  void Run();

 private:
  void RegisterQueueHandler(const std::string name);
  void ProcessQueue(const std::string name);
  void ProcessSocket(uWS::WebSocket<uWS::SERVER> ws, char *message,
                     size_t length, uWS::OpCode opcode);
  std::string GetCurrentVal(const std::string &msg);

  uWS::Hub hub_;
  const int port_;
  std::mutex map_mutex_;
  std::map<std::string, const msg::LogEntry*> msgs_;
  std::mutex send_map_mutex_;
  std::map<std::string, std::unique_ptr<Queue>> send_msgs_;
  std::vector<std::thread> threads_;
};

}  // namespace sailbot
