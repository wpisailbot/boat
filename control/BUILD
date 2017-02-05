package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_proto_library(
  name="actuator_cmd_proto",
  srcs=["actuator_cmd.proto"],
  deps=["@protobuf//:cc_wkt_protos"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
cc_library(
  name="simple_control",
  srcs=["simple.cc"],
  hdrs=["simple.h"],
  deps=[":actuator_cmd_proto", "//util:node", "//sim:sim_physics"],
)
cc_library(
  name="line_tacking",
  srcs=["line_tacking.cc"],
  hdrs=["line_tacking.h"],
  deps=[":actuator_cmd_proto", "//util:node", "//sim:sim_physics"],
)
cc_library(
  name="elqr_lib",
  srcs=["elqr_routing.cc"],
  hdrs=["elqr_routing.h"],
  deps=["//sim:sim_util"],
)
cc_binary(
  name="elqr_bin",
  deps=[":elqr_lib"],
)
cc_test(
  name="simple_control_test",
  srcs=["control_test.cc"],
  deps=[":simple_control",
        "line_tacking",
        "@gtest//:main",
        "//sim:sim_inter",
        "//ui:server",
        "//sim:csv_logger"],
)
