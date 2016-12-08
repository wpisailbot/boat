#include "can.h"
#include "util/node.h"
#include "pgn.h"
#include "canboat-pgn.h"
#include <linux/can.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>

namespace sailbot {
namespace can {

namespace {
  void SetProtoNumberField(google::protobuf::Message* msg, int field_num, double val) {
    VLOG(2) << "Setting field to " << val;
    const google::protobuf::Descriptor* descriptor = msg->GetDescriptor();
    const google::protobuf::Reflection* reflect = msg->GetReflection();
    const google::protobuf::FieldDescriptor* field = descriptor->FindFieldByNumber(field_num);
    if (field == nullptr) {
      LOG(ERROR) << "Failed to find field in protobuf message.";
      return;
    }
    switch (field->type()) {
      case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
        reflect->SetDouble(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_FLOAT:
        reflect->SetFloat(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_BOOL:
        reflect->SetBool(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_ENUM:
        reflect->SetEnumValue(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_INT32:
      case google::protobuf::FieldDescriptor::TYPE_SINT32:
      case google::protobuf::FieldDescriptor::TYPE_SFIXED32:
        reflect->SetInt32(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_INT64:
      case google::protobuf::FieldDescriptor::TYPE_SFIXED64:
      case google::protobuf::FieldDescriptor::TYPE_SINT64:
        reflect->SetInt64(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_UINT32:
      case google::protobuf::FieldDescriptor::TYPE_FIXED32:
        reflect->SetUInt32(msg, field, val);
        break;
      case google::protobuf::FieldDescriptor::TYPE_UINT64:
      case google::protobuf::FieldDescriptor::TYPE_FIXED64:
        reflect->SetUInt64(msg, field, val);
        break;
      default:
        break;
    }
    return;
  }
} // namespace

CanNode::CanNode() : Node(0/**/), s_(socket(PF_CAN, SOCK_RAW, CAN_RAW)) {
  // Iterate over all the PGNs we might be receiving:
  for (unsigned i = 0; i < pgnListSize; ++i) {
    const Pgn *p = &pgnList[i];
    char queue_name[16]; // TODO: Figure this out.
    snprintf(queue_name, 16, "can%u", p->pgn);
    msgs_[p->pgn] =
        CANMessage(p, queue_name, AllocateMessage<msg::can::CANMaster>());
  }
  PCHECK(s_ >= 0) << "Socket open failed";
  ifreq ifr;
  sockaddr_can addr;

  strcpy(ifr.ifr_name, "can0"); // TODO(james): Settable name.
  PCHECK(ioctl(s_, SIOCGIFINDEX, &ifr) == 0) << "ioctl IFINDEX set failed";
  int block = 1; // We want blocking I/O.
  PCHECK(ioctl(s_, FIONBIO, &ifr, &block) == 0) << "ioctl set to nonblocking failed";

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  PCHECK(bind(s_, (sockaddr*)&addr, sizeof(addr)) == 0) << "bind failed";
}

void CanNode::Iterate() {
  can_frame frame;
  int n = read(s_, &frame, sizeof(can_frame));
  PCHECK(n > 0) << "Read error";

  CANID id = RetrieveID(frame.can_id);
  uint32_t pgn = GetPGN(id);
  size_t dlc = frame.can_dlc;
  uint8_t *data = frame.data;
  size_t data_start = 0;
  CANMessage *msg = &msgs_[pgn];
  VLOG(2) << "Reading frame: pgn: " << pgn << " dlc: " << dlc;
  if (msg->is_long_) {
    uint8_t index = data[0] & 0x1F;
    uint8_t base = data[0] & 0xE0;
    data_start = index * 7 - 1;
    --dlc;
    ++data;
    if (index == 0) {
      // First packet in message--ignore next byte because it is just the length.
      --dlc;
      ++data;
      msg->base_ = base;
    } else {
      if (msg->base_ != base) {
        LOG(WARNING) << "Missed CAN frame";
      }
    }
  }
  for (unsigned i = 0; i < dlc; ++i) {
    msg->data_[data_start+i] = data[i];
  }

  // Check if we are at the end of the message and send it.
  bool at_end = (!msg->is_long_) || (data_start + dlc >= msg->len_);
  if (at_end) { DecodeAndSend(msg); }
}

void CanNode::DecodeAndSend(const CANMessage* msg) {
  //TODO(james): Repeating fields?
  size_t cur_bit = 0;
  const uint8_t *data = msg->data_.get();
  const google::protobuf::Descriptor* descriptor = msg->store_msg_->GetDescriptor();
  const google::protobuf::Reflection* reflection = msg->store_msg_->GetReflection();
  const google::protobuf::FieldDescriptor *field_desc =
      descriptor->FindFieldByNumber(msg->pgn->pgn);
  google::protobuf::Message *pgn_msg = reflection->MutableMessage(msg->store_msg_, field_desc);
  if (pgn_msg == nullptr) {
    LOG(WARNING) << "No message for pgn " << msg->pgn->pgn;
    return;
  }
  VLOG(2) << "Message name: " << msg->pgn->description;
  for (int i = 0; msg->pgn->fieldList[i].name /*ie, f exists*/; ++i) {
    const Field *f = msg->pgn->fieldList + i;
    VLOG(2) << "Setting field " << f->name;
    if (f->resolution > 0) {
      SetProtoNumberField(pgn_msg, i+1, ExtractNumberField(f, data, cur_bit));
    } else if (f->resolution < 0) {
      double res = f->resolution;
      if (cur_bit % 8 != 0 || f->size % 8 != 0) {
        LOG(ERROR) << "Unable to read field because expected byte alignment.";
      } else if (res == RES_LATITUDE || res == RES_LONGITUDE) {
        double val = 0;
        if (f->size == 32) {
          int32_t short_val;
          memcpy(&short_val, data + (cur_bit / 8), 4);
          val = (double)short_val * RES_LAT_LONG;
        } else if (f->size == 64) {
          int32_t int_val;
          memcpy(&int_val, data + (cur_bit / 8), 8);
          val = (double)int_val * RES_LAT_LONG_64;
        }
        SetProtoNumberField(pgn_msg, i+1, val);
      } else if (res == RES_DATE) { // Date = days since epoch
        uint16_t days;
        memcpy(&days, data + (cur_bit / 8), 2);
        SetProtoNumberField(pgn_msg, i+1, days);
      } else if (res == RES_TIME) { // time = seconds since midnight
        uint32_t secs;
        memcpy(&secs, data + (cur_bit / 8), 4);
        SetProtoNumberField(pgn_msg, i+1, secs);
      } else {
        // For now, assume we don't care about anything else (which is incorrect).
      }
    }
    cur_bit += f->size;
  }
  msg->queue_->send(msg->store_msg_);
}

double CanNode::ExtractNumberField(const Field* f, const uint8_t *data, size_t start_bit) {
  int64_t retval = 0;
  size_t bits_left = f->size;
  data += start_bit / 8;
  start_bit %= 8;
  int64_t maxval = 0;

  while (bits_left) {
    uint8_t byte_bits = std::min(8 - start_bit, bits_left);
    size_t end_gap = 8 - byte_bits + start_bit;
    uint8_t mask = ((uint16_t)0xFF >> start_bit) << end_gap;
    retval <<= byte_bits;
    retval += (mask & *data) >> end_gap;

    maxval <<= byte_bits;
    maxval += mask >> end_gap;

    bits_left -= byte_bits;
    start_bit = 0;
    ++data;
  }

  if (f->hasSign) {
    maxval >>= 1;
    retval |= ~maxval;
  }
  return (double)retval * f->resolution;
}
}  // namespace sailbot
}  // namespace can
