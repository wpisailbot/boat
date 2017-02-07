#include "proto_util.h"

#include "gtest/gtest.h"
#include "util/proto_util_test.pb.h"

namespace sailbot {
namespace util {
namespace testing {

class ProtoUtilTest : public ::testing::Test {
 public:
   sailbot::msg::testing::EveryTimeMessage msg_;

  void SetAllFields() {
    msg_.mutable_submsg();
    msg_.set_double_(1.1);
    msg_.set_float_(2.2);
    msg_.set_int32_(100);
    msg_.set_int64_(400);
    msg_.set_uint32_(313);
    msg_.set_uint64_(1001);
    msg_.set_sint32_(-1001);
    msg_.set_sint64_(-505);
    msg_.set_fixed32_(99);
    msg_.set_fixed64_(87);
    msg_.set_bool_(true);
    msg_.set_string_("foobar");
    msg_.set_bytes_("barfoo");
  }

  void CheckAllFields() {
    EXPECT_EQ(0, GetProtoNumberFieldById(&msg_, 1)); // submsg
    EXPECT_EQ(msg_.has_double_() ? msg_.double_() : 0, GetProtoNumberFieldById(&msg_, 2)); // double
    EXPECT_EQ(msg_.has_float_() ? msg_.float_() : 0, GetProtoNumberFieldById(&msg_, 3)); // float
    EXPECT_EQ(msg_.has_int32_() ? msg_.int32_() : 0, GetProtoNumberFieldById(&msg_, 4)); // int32
    EXPECT_EQ(msg_.has_int64_() ? msg_.int64_() : 0, GetProtoNumberFieldById(&msg_, 5)); // int64
    EXPECT_EQ(msg_.has_uint32_() ? msg_.uint32_() : 0, GetProtoNumberFieldById(&msg_, 6)); // uint32
    EXPECT_EQ(msg_.has_uint64_() ? msg_.uint64_() : 0, GetProtoNumberFieldById(&msg_, 7)); // uint64
    EXPECT_EQ(msg_.has_sint32_() ? msg_.sint32_() : 0, GetProtoNumberFieldById(&msg_, 8)); // sint32
    EXPECT_EQ(msg_.has_sint64_() ? msg_.sint64_() : 0, GetProtoNumberFieldById(&msg_, 9)); // sint64
    EXPECT_EQ(msg_.has_fixed32_() ? msg_.fixed32_() : 0, GetProtoNumberFieldById(&msg_, 10)); // fixed32
    EXPECT_EQ(msg_.has_fixed64_() ? msg_.fixed64_() : 0, GetProtoNumberFieldById(&msg_, 11)); // fixed64
    EXPECT_EQ(msg_.has_bool_() ? msg_.bool_() : 0, GetProtoNumberFieldById(&msg_, 12)); // bool
    EXPECT_EQ(0, GetProtoNumberFieldById(&msg_, 13)); // string
    EXPECT_EQ(0, GetProtoNumberFieldById(&msg_, 14)); // bytes
  }
};

TEST_F(ProtoUtilTest, DontFill) {
  CheckAllFields();
}

TEST_F(ProtoUtilTest, FillFields) {
  SetAllFields();
  CheckAllFields();
}

}  // namespace testing
}  // namespace util
}  // namespace sailbot
