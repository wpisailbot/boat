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
  name="util",
  srcs=["util.cc"],
  hdrs=["util.h"],
  deps=["//sim:sim_debug_proto", "@eigen//:eigen"],
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
cc_test(
  name="line_tacking_test",
  srcs=["line_tacking_test.cc"],
  deps=[":line_tacking",
        "@gtest//:main",
        "//util:testing"],
)
cc_test(
  name="util_test",
  srcs=["util_test.cc"],
  deps=[":util", "@gtest//:main"],
)
cc_library(
  name="elqr_lib",
  srcs=["elqr_routing.cc"],
  hdrs=["elqr_routing.h"],
  deps=[":util"],
)
cc_binary(
  name="elqr_bin",
  deps=[":elqr_lib"],
)
cc_library(
  name="rudder_controller",
  srcs=["rudder.cc"],
  hdrs=["rudder.h", "//control/python:rudder_gains.h"],
  deps=["@eigen//:eigen", "//util:node"],
)
cc_test(
  name="control_test",
  srcs=["control_test.cc"],
  timeout="eternal",
  deps=[":simple_control",
        ":line_tacking",
        "//sensor:state_estimator",
        "//util:testing",
        "//sim:sim_inter",
        "//ui:server",
        "//sim:csv_logger"],
)
