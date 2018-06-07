#pragma once

#include "util/msg.pb.h"

namespace sailbot {
namespace util {

double GetProtoNumberField(const google::protobuf::FieldDescriptor *field,
                           const google::protobuf::Message *msg);
double GetProtoNumberFieldById(const google::protobuf::Message* msg, int num);

// Returns true on success
bool ReadProtoFromFile(const char *fname, google::protobuf::Message *proto);
bool WriteProtoToFile(const char *fname,
                      const google::protobuf::Message &proto);

}  // namespace util
}  // namespace sailbot
