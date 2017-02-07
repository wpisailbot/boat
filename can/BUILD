package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_proto_library(
  name="can_proto",
  srcs=["can.proto"],
  deps=["@protobuf//:cc_wkt_protos"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
cc_library(
  name="can",
  srcs=["can.cc"],
  hdrs=["can.h", "canboat-pgn.h", "pgn.h"],
  deps=[":can_proto", "//util:proto_util", "//util:node"],
)
cc_test(
  name="can_test",
  srcs=["can_test.cc"],
  deps=["@gtest//:main",
        ":can"],
)
cc_binary(
  name="can-dump",
  srcs=["can-dump.cc"],
  deps=[":can"],
)
cc_binary(
  name="can-ping",
  srcs=["can-ping.cc"],
  deps=[":can"],
)
