#include "proto_util.h"
#include "glog/logging.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

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

// Returns true on success
bool ReadProtoFromFile(const char *fname, google::protobuf::Message *proto) {
  if (fname == nullptr) {
    return false;
  }
  int fd = open(fname, O_RDONLY);
  if (fd < 0) {
    PLOG(WARNING) << "Failed to open file " << fname;
    return false;
  }
  google::protobuf::io::FileInputStream finput(fd);
  finput.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Parse(&finput, proto)) {
    LOG(WARNING) << "Failed to parse protobuf";
    return false;
  }
  return true;
}

// Returns true on success
bool WriteProtoToFile(const char *fname, const google::protobuf::Message &proto) {
  if (fname == nullptr) {
    return false;
  }
  int fd = open(fname, O_WRONLY | O_CREAT,
                S_IROTH | S_IWUSR | S_IRUSR | S_IWOTH | S_IRGRP | S_IWGRP);
  if (fd < 0) {
    PLOG(WARNING) << "Failed to open file " << fname;
    return false;
  }
  google::protobuf::io::FileOutputStream foutput(fd);
  foutput.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(proto, &foutput)) {
    LOG(WARNING) << "Failed to parse protobuf";
    return false;
  }
  return true;
}

}  // namespace util
}  // namespace sailbot
