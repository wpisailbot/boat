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
  Field field = {.name = "FooBar",
                 .size = 8,
                 .resolution = 1,
                 .hasSign = true};
  int64_t ExtractNumberField(const Field *f, const uint8_t *data, size_t start_bit,
                        int64_t *maxval) {
    return CanNode::ExtractNumberField(f, data, start_bit, maxval);
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

}  // testing
}  // can
}  // sailbot
