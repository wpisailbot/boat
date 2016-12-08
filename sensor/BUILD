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
cc_binary(
  name="sbus-test-run",
  srcs=["sbus-testing.cc"],
  deps=[":sbus"],
)
