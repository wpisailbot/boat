package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_proto_library(
  name="sbus_proto",
  srcs=["sbus.proto"],
  deps=["@protobuf//:cc_wkt_protos"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
cc_library(
  name="sbus",
  srcs=["read-sbus.cc"],
  hdrs=["read-sbus.h"],
  deps=[":sbus_proto", "//util:node"],
)
cc_library(
  name="state_estimator",
  srcs=["state_estimator.cc"],
  hdrs=["state_estimator.h"],
  deps=["//control:util",
        "//can:can_proto",
        "//control:actuator_cmd_proto",
        "//util:node"],
)
cc_binary(
  name="state_estimator_main",
  srcs=["state_estimator_main.cc"],
  deps=[":state_estimator"],
)
cc_binary(
  name="sbus-test-run",
  srcs=["sbus-testing.cc"],
  deps=[":sbus"],
)
