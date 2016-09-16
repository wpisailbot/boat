load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="clock",
  srcs=["clock.cc"],
  hdrs=["clock.h"],
  linkopts = ["-lglog"],
)
cc_library(
  name="node",
  srcs=["node.cc"],
  hdrs=["node.h"],
  deps = [
    ":clock",
    "//ipc:queue",
    ":msg_proto",
    "@protobuf//:protobuf",
  ]
)
cc_test(
  name="clock_test",
  srcs=["clock_test.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "@gtest//:main",
    ":clock",
  ],
  flaky = True,
)
cc_proto_library(
  name="msg_proto",
  srcs=["msg.proto"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)

cc_binary(
  name="pong",
  srcs=["pong.cc"],
  deps = [
    ":node",
    "//ipc:queue",
    ":msg_proto",
  ],
)
cc_binary(
  name="ping",
  srcs=["ping.cc"],
  deps = [
    ":node",
    "//ipc:queue",
    ":msg_proto",
  ],
)
