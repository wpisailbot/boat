#include "gtest/gtest.h"
#include "can.h"
#include "pgn.h"

namespace sailbot {
namespace can {
namespace testing {

TEST(PgnTest, EncodeDecodeCanID) {
  CANID canid;
  canid.source = 0x15;
  SetPGN(&canid, 127251);
  canid.priority = 0x5;
  uint32_t raw = ConstructID(canid);
  EXPECT_EQ(0x95F11315, raw);
  CANID back = RetrieveID(raw);
  EXPECT_EQ(canid.source, back.source);
  EXPECT_EQ(canid.priority, back.priority);
  EXPECT_EQ(127251, GetPGN(back));
}

class CanTest : public ::testing::Test {
 protected:
   CanTest()
       : pgn(&pgnList[3] /*Vessel Heading*/) {
   }

  void SetUp() override {
    util::CancelShutdown();
    Queue::set_testing(true);

    msg.reset(new CanNode::CANMessage(pgn, "can127251", &queue_msg));
    EXPECT_EQ(false, msg->is_long_);
    EXPECT_EQ(127251, msg->pgn->pgn);
    EXPECT_EQ(5, msg->len_);
  }

  void TearDown() override {
    msg.reset();
  }

  const Pgn *pgn;
  msg::can::CANMaster queue_msg;
  std::unique_ptr<CanNode::CANMessage> msg;

  Field field = {.name = "FooBar",
                 .size = 8,
                 .resolution = 1,
                 .hasSign = true};

  int64_t ExtractNumberField(const Field *f, const uint8_t *data, size_t start_bit,
                        int64_t *maxval) {
    return CanNode::ExtractNumberField(f, data, start_bit, maxval);
  }

  void DecodeAndSend(const CanNode::CANMessage *msg) {
    CanNode::DecodeAndSend(msg);
  }
  bool ConstructMessage(const msg::can::CANMaster &msg,
                        const CanNode::CANMessage *msg_info, can_frame *frame) {
    return CanNode::ConstructMessage(msg, msg_info, frame);
  }
};

TEST_F(CanTest, ExtractSigned1ByteAligned0Field) {
  int8_t goal = -10;
  field.size = 8;
  field.hasSign = true;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, (uint8_t *)&goal, 0, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(127, maxval);
}

TEST_F(CanTest, ExtractUnsigned1ByteAligned0Field) {
  uint8_t goal = 240;
  field.size = 8;
  field.hasSign = false;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, &goal, 0, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(255, maxval);
}

TEST_F(CanTest, ExtractUnsignedMultiByteAligned0Field) {
  uint16_t goal = 4320;
  field.size = 16;
  field.hasSign = false;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, (uint8_t *)&goal, 0, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(65535, maxval);
}

TEST_F(CanTest, ExtractUnsignedSubByteAligned0Field) {
  uint8_t goal = 8;
  uint8_t data = (goal << 3) + 0x7;
  field.size = 5;
  field.hasSign = false;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, &data, 0, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(31, maxval);
}

TEST_F(CanTest, ExtractUnsignedSubByteUnalignedField) {
  uint8_t goal = 8;
  uint8_t data = (0x7 << 5) + goal;
  field.size = 5;
  field.hasSign = false;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, &data, 3, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(31, maxval);
}

TEST_F(CanTest, ExtractUnsignedMultiByteUnalignedField) {
  uint16_t goal = 0xAB0F;
  uint8_t data[4] = {(uint8_t)(0xF8 | goal), (uint8_t)(goal >> 3),
                     (uint8_t)((goal >> 8) & 0xF8)};
  field.size = 16;
  field.hasSign = false;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, data, 5, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(65535, maxval);
}

TEST_F(CanTest, ExtractSignedMultiByteUnalignedField) {
  int16_t goal = -0x3B0F;
  uint8_t data[4] = {(uint8_t)(0xF8 | goal), (uint8_t)(goal >> 3),
                     (uint8_t)((goal >> 8) & 0xF8)};
  field.size = 16;
  field.hasSign = true;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, data, 5, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(32767, maxval);
}

TEST_F(CanTest, ExtractSignedMultiByteUnalignedFieldOffset) {
  int16_t goal = -0x3B0F;
  uint8_t data[6] = {0xFF, 0xFF, (uint8_t)(0xF8 | goal), (uint8_t)(goal >> 3),
                     (uint8_t)((goal >> 8) & 0xF8)};
  field.size = 16;
  field.hasSign = true;
  int64_t maxval = -1;
  int64_t val = ExtractNumberField(&field, data, 21, &maxval);
  EXPECT_EQ(goal, val);
  EXPECT_EQ(32767, maxval);
}

TEST_F(CanTest, EncodeDecodeBasicMessage) {
  msg::can::CANMaster input_queue_msg;
  msg::can::RateOfTurn *rate_of_turn = input_queue_msg.mutable_rate_turn();
  rate_of_turn->set_sid(0x23);
  rate_of_turn->set_rate(1.2398);

  FLAGS_v = 2;
  can_frame frame;
  EXPECT_TRUE(ConstructMessage(input_queue_msg, msg.get(), &frame));

  // TODO(james): Figur eout proper behavior for padding CAN message.
  // Only the first three bytes of this are used.
  EXPECT_EQ(8, frame.can_dlc);
  CANID id = RetrieveID(frame.can_id);
  EXPECT_EQ(1, id.__eff_ident);
  EXPECT_EQ(100, id.source);
  EXPECT_EQ(2, id.priority);
  EXPECT_EQ(127251, GetPGN(id));

  int32_t int_rate = rate_of_turn->rate() / RES_HIRES_ROTATION;
  uint8_t goal_data[8] = {0x23, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(goal_data+1, &int_rate, 4);
  for (int i = 0; i < 8; ++i) {
    std::cerr << "i: " << i << std::endl;
    EXPECT_EQ(goal_data[i], frame.data[i]);
    msg->data_[i] = frame.data[i];
  }

  DecodeAndSend(msg.get());
  EXPECT_EQ(rate_of_turn->sid(), msg->store_msg_->rate_turn().sid());
  EXPECT_EQ(rate_of_turn->rate(), msg->store_msg_->rate_turn().rate());
}

}  // testing
}  // can
}  // sailbot
