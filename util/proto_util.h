#pragma once

#include "util/msg.pb.h"

namespace sailbot {
namespace util {

double GetProtoNumberField(const google::protobuf::FieldDescriptor *field,
                           const google::protobuf::Message *msg);
double GetProtoNumberFieldById(const google::protobuf::Message* msg, int num);

}  // namespace util
}  // namespace sailbot
