package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="clock",
  srcs=["clock.cc"],
  hdrs=["clock.h"],
  #linkopts = ["-lglog -lgflags"],
  deps = ["@glog//:glog",
          "@gflags//:gflags"],
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
cc_library(
  name="logger",
  srcs=["logger.cc"],
  hdrs=["logger.h"],
  deps = [
    ":node",
    "//ipc:queue",
    ":msg_proto",
    "@protobuf//:protobuf",
  ]
)
cc_library(
  name="log_replay",
  srcs=["log-replay.cc"],
  hdrs=["log-replay.h"],
  deps = [
    ":node",
    ":logger",
    "//ipc:queue",
    ":msg_proto",
    ":log_deprecation",
    "@gflags//:gflags",
  ],
)
cc_library(
  name="log_deprecation",
  srcs=["log_deprecation.cc"],
  hdrs=["log_deprecation.h"],
  deps = [
    ":msg_proto",
    ":logger",
  ],
)
cc_library(
  name="testing",
  srcs=["testing.cc"],
  hdrs=["testing.h"],
  deps = [
    "@gtest//:main",
    "//ipc:queue",
    ":clock",
  ],
)
cc_binary(
  name="logger_main",
  srcs=["logger_main.cc"],
  deps=[":logger"],
)
cc_binary(
  name="log_reader_main",
  srcs=["log_reader_main.cc"],
  deps=[":logger"],
)
cc_test(
  name="clock_test",
  srcs=["clock_test.cc"],
  deps = [
    "@gtest//:main",
    ":clock",
  ],
  flaky = True,
)
cc_proto_library(
  name="msg_proto",
  srcs=["msg.proto"],
  deps=["@protobuf//:cc_wkt_protos",
        "//control:actuator_cmd_proto",
        "//sim:sim_debug_proto",
        "//sensor:sbus_proto",
        "//can:can_proto",
        "//rigid_wing:rigid_wing_proto",
        "//ipc:queue_test_msg",],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
cc_library(
  name="proto_util",
  srcs=["proto_util.cc"],
  hdrs=["proto_util.h"],
  deps = [":msg_proto",
          "@glog//:glog",
          "@gflags//:gflags"],
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
cc_binary(
  name="log_replay_test",
  srcs=["log-replay-main.cc"],
  deps=[":log_replay", "//ui:server", "//sim:csv_logger"],
)
cc_binary(
  name="logsim_main",
  srcs=["logsim_main.cc"],
  deps=[":log_replay",
        "//ui:server",
        "//sim:csv_logger",
        "//sim:sim_inter",
        "//sensor:state_estimator",
        ],
)

cc_proto_library(
  name="proto_util_test_proto",
  srcs=["proto_util_test.proto"],
  deps=["@protobuf//:cc_wkt_protos"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
cc_test(
  name="proto_util_test",
  srcs=["proto_util_test.cc"],
  deps = [
    "@gtest//:main",
    ":proto_util",
    ":proto_util_test_proto",
  ],
)
