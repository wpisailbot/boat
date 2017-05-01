#pragma once
#include <vector>
#include <fstream>
#include <map>
#include <atomic>
#include <google/protobuf/message.h>

#include "util/node.h"

namespace sailbot {

class CsvLogger : public Node {
 public:
  CsvLogger(std::vector<std::pair<std::string, std::string>> data,
            std::string fname, float dt = 0.01);

  ~CsvLogger();

  void Stop() { running_ = false; }
  void Start() { running_ = true; }

 private:
  void Iterate() override;

  void ProcessInput(const std::string name, std::vector<std::string> fields);

  double GetField(const msg::LogEntry &msg, const std::string &field);

  std::mutex data_mutex_;
  std::vector<double> data_;
  std::map<std::string, int> data_indices_;
  std::atomic<bool> running_{true};

  std::ofstream file_;

  std::vector<std::thread> threads_;
};

}  // sailbot
