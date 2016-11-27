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
  deps=[":actuator_cmd_proto", "//util:node"],
)
