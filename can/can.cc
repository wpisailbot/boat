#include "can.h"
#include "util/node.h"
#include "util/proto_util.h"
#include "pgn.h"
#include "canboat-pgn.h"
#include <linux/can.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>

namespace sailbot {
namespace can {

namespace {
  bool IsReserved(int64_t val, int64_t maxval) {
    return ((maxval >= 15) && ((maxval - val) <= 2)) ||
           ((maxval > 1) && ((maxval - val) <= 1));
  }
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
  msg::can::CANMaster *tmp = AllocateMessage<msg::can::CANMaster>();
  const google::protobuf::Descriptor *descriptor = tmp->GetDescriptor();
  for (unsigned i = 0; i < pgnListSize; ++i) {
    const Pgn *p = &pgnList[i];
    int pgn = p->pgn;
    if (!descriptor->FindFieldByNumber(pgn)) {
      // The message isn't going to be sent out, so don't process it.
      continue;
    }
    char queue_name[16]; // TODO: Figure this out.
    snprintf(queue_name, 16, "can%u", pgn);
    msgs_[pgn] =
        CANMessage(p, queue_name, AllocateMessage<msg::can::CANMaster>());

    RegisterHandler<msg::can::CANMaster>(
        queue_name, [this, pgn](const msg::can::CANMaster &msg) {
          SendMessage(msg, pgn);
        });
  }
  PCHECK(s_ >= 0) << "Socket open failed";
  ifreq ifr;
  sockaddr_can addr;

  strcpy(ifr.ifr_name, "can0"); // TODO(james): Settable name.
  PCHECK(ioctl(s_, SIOCGIFINDEX, &ifr) == 0) << "ioctl IFINDEX set failed";
  /*
  int block = 1; // We want blocking I/O.
  PCHECK(ioctl(s_, FIONBIO, &ifr, &block) == 0) << "ioctl set to nonblocking failed";
  */

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  PCHECK(setsockopt(s_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                    sizeof(timeout)) == 0)
      << "Failed to set timeout on can socket receive";

  int flags = fcntl(s_, F_GETFL, 0);
  PCHECK(flags != -1) << "Failed to receive flags";
  flags &= ~O_NONBLOCK;
  PCHECK(fcntl(s_, F_SETFL, flags) != -1) << "Failed to set fnctl flags";

  PCHECK(bind(s_, (sockaddr*)&addr, sizeof(addr)) == 0) << "bind failed";
}

void CanNode::Iterate() {
  can_frame frame;
  while (!util::IsShutdown()) {
    int n = read(s_, &frame, sizeof(can_frame));
    if (n < 0) {
      if (errno == EAGAIN) {
        PLOG(ERROR) << "Timeout on read--not getting any CAN data";
      } else {
        PLOG(FATAL) << "Read failed for some reason";
      }
    } else {
      break;
    }
  }

  CANID id = RetrieveID(frame.can_id);
  uint32_t pgn = GetPGN(id);
  size_t dlc = frame.can_dlc;
  uint8_t *data = frame.data;
  size_t data_start = 0;
  if (msgs_.count(pgn) == 0) {
    VLOG(1) << "Ignoring CAN message with pgn " << pgn;
    return;
  }
  CANMessage *msg = &msgs_[pgn];
  VLOG(2) << "Reading frame: can_id: " << frame.can_id << " pgn: " << pgn
          << " dlc: " << dlc << " source: " << (int)id.source;
  VLOG(2) << "Last bytE: " << (int)frame.data[8];
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
  char debugbuf[128];
  size_t debug_size = 128;
  for (size_t i = 0; i < msg->len_; ++i) {
    debug_size -=
        snprintf(debugbuf + 128 - debug_size, debug_size, "0x%x ", data[i]);
  }
  VLOG(3) << "Data: " << debugbuf;
  VLOG(2) << "Message name: " << msg->pgn->description;
  for (int i = 0; msg->pgn->fieldList[i].name /*ie, f exists*/; ++i) {
    const Field *f = msg->pgn->fieldList + i;
    VLOG(2) << "Setting field " << f->name;
    if (f->resolution > 0) {
      int64_t max_val = 0;
      int64_t val = ExtractNumberField(f, data, cur_bit, &max_val);
      if (IsReserved(val, max_val)) {
        VLOG(3) << "Field not filled out.";
      } else {
        double fval = (double)val * f->resolution;
        SetProtoNumberField(pgn_msg, i + 1, fval);
      }
    } else if (f->resolution < 0) {
      double res = f->resolution;
      if (cur_bit % 8 != 0 || f->size % 8 != 0) {
        LOG(ERROR) << "Unable to read field because expected byte alignment.";
      } else if (res == RES_LATITUDE || res == RES_LONGITUDE) {
        double val = 0;
        bool reserved = false;
        if (f->size == 32) {
          int32_t short_val;
          memcpy(&short_val, data + (cur_bit / 8), 4);
          val = (double)short_val * RES_LAT_LONG;
          int64_t max = uint32_t(-1);
          max >>= 1;
          reserved = IsReserved(short_val, max);
        } else if (f->size == 64) {
          int32_t int_val;
          memcpy(&int_val, data + (cur_bit / 8), 8);
          val = (double)int_val * RES_LAT_LONG_64;
          int64_t max = -1 ^ ((int64_t)1 << 63);
          reserved = IsReserved(int_val, max);
        }
        if (!reserved) SetProtoNumberField(pgn_msg, i+1, val);
        else VLOG(3) << "Lat/lon field reserved";
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
  msg->store_msg_->set_outgoing(false);
  msg->queue_->send(msg->store_msg_);
}

int64_t CanNode::ExtractNumberField(const Field *f, const uint8_t *data,
                                    size_t start_bit, int64_t *maxval) {
  int64_t retval = 0;
  size_t bits_left = f->size;
  data += start_bit / 8;
  start_bit %= 8;
  *maxval = 0;
  size_t magnitude = 0;

  while (bits_left) {
    uint8_t byte_bits = std::min(8 - start_bit, bits_left);
    size_t end_gap = 8 - byte_bits - start_bit;
    uint8_t mask = ((uint16_t)0xFF >> start_bit) << end_gap;
    uint64_t cur_val = (uint64_t(mask & *data) >> end_gap) << magnitude;
    retval += cur_val;

    *maxval <<= byte_bits;
    *maxval += mask >> end_gap;

    bits_left -= byte_bits;
    magnitude += byte_bits;
    start_bit = 0;
    ++data;
  }

  if (f->hasSign) {
    *maxval >>= 1;
    if (retval >> (magnitude-1)) retval |= ~*maxval;
  }
  return retval;
}

void CanNode::SendMessage(const msg::can::CANMaster & msg, const int pgn) {
  if (!msg.outgoing()) {
    // If the message isn't outgoing, than we probably received it, so don't
    // send it out again.
    return;
  }
  const CANMessage *msg_info = &msgs_[pgn];
  can_frame frame;
  if (ConstructMessage(msg, msg_info, &frame)) {
    PCHECK(write(s_, &frame, sizeof(can_frame)) > 0) << "Write failed";
  }
}

bool CanNode::ConstructMessage(const msg::can::CANMaster &msg,
                               const CANMessage *msg_info, can_frame *frame) {
  const int pgn = msg_info->pgn->pgn;
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  const google::protobuf::Reflection* reflection = msg.GetReflection();
  const google::protobuf::FieldDescriptor *field_desc =
      descriptor->FindFieldByNumber(pgn);
  VLOG(3) << "Sending message with pgn " << pgn;
  if (field_desc == nullptr) {
    LOG(ERROR) << "Unable to find message for PGN " << pgn;
    return false;
  }
  const google::protobuf::Message &pgn_msg =
      reflection->GetMessage(msg, field_desc);

  frame->can_dlc = 0;
  if (msg_info->pgn->size > 8) {
    LOG(ERROR) << "Sending multi-packet messages is currently unsupported";
  }
  for (int i = 0; msg_info->pgn->fieldList[i].name /*ie, f exists*/; ++i) {
    const Field *f = msg_info->pgn->fieldList + i;
    if (f->size % 8 != 0) {
      LOG(ERROR) << "Attempting to send message with non-byte-aligned fields";
      return false;
    }
    // For now, only work with fields that are simple numbers and have byte alignment.
    if (f->resolution > 0) {
      double x = util::GetProtoNumberFieldById(&pgn_msg, i+1);
      uint64_t n_wire = x / f->resolution;
      VLOG(2) << "n_wire: " << n_wire;
      memcpy((void*)(frame->data + frame->can_dlc), (void*)&n_wire, f->size / 8);
    } else {
      LOG(WARNING) << "Attempting to write field with undefined resolution";
    }
    frame->can_dlc += f->size / 8;
  }

  CANID can_id;
  SetPGN(&can_id, pgn);
  can_id.priority = 0x2; // TODO(james): Parametrize to allow variation
  can_id.source = 100; // TODO(james): Use real source ID.
  frame->can_id = ConstructID(can_id);
  VLOG(2) << "ID: " << frame->can_id << " DLC: " << (int)frame->can_dlc
          << " data: " << (int)frame->data[0] << ", " << (int)frame->data[1]
          << ", " << (int)frame->data[2] << ", " << (int)frame->data[3] << ", "
          << (int)frame->data[4];
  for (; frame->can_dlc < 8; frame->can_dlc++) {
    frame->data[frame->can_dlc] = 0xFF;
  }
  VLOG(2) << "CANID: DP: " << (int)can_id.DP;
  return true;
}

}  // namespace sailbot
}  // namespace can
