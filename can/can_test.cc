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
};

}  // testing
}  // can
}  // sailbot
