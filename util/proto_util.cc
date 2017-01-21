#include "proto_util.h"
#include "glog/logging.h"
#include <google/protobuf/message.h>

namespace sailbot {
namespace util {

double GetProtoNumberField(const google::protobuf::FieldDescriptor *field,
                           const google::protobuf::Message *msg) {
  const google::protobuf::Reflection* reflect = msg->GetReflection();
  switch (field->type()) {
    case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
      return reflect->GetDouble(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_FLOAT:
      return reflect->GetFloat(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_BOOL:
      return reflect->GetBool(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_ENUM:
      return reflect->GetEnumValue(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_INT32:
    case google::protobuf::FieldDescriptor::TYPE_SINT32:
    case google::protobuf::FieldDescriptor::TYPE_SFIXED32:
      return reflect->GetInt32(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_INT64:
    case google::protobuf::FieldDescriptor::TYPE_SFIXED64:
    case google::protobuf::FieldDescriptor::TYPE_SINT64:
      return reflect->GetInt64(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_UINT32:
    case google::protobuf::FieldDescriptor::TYPE_FIXED32:
      return reflect->GetUInt32(*msg, field);
    case google::protobuf::FieldDescriptor::TYPE_UINT64:
    case google::protobuf::FieldDescriptor::TYPE_FIXED64:
      return reflect->GetUInt64(*msg, field);
    default:
      return 0;
  }
}

double GetProtoNumberFieldById(const google::protobuf::Message* msg, int num) {
  const google::protobuf::Descriptor* descriptor = msg->GetDescriptor();
  const google::protobuf::FieldDescriptor* field = descriptor->FindFieldByNumber(num);
  if (field == nullptr) {
    LOG(ERROR) << "Failed to get field";
    return 0;
  }
  return GetProtoNumberField(field, msg);
}

}  // namespace util
}  // namespace sailbot
