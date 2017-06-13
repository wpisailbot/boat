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
  deps=["//rigid_wing:rigid_wing_proto", ":actuator_cmd_proto", "//util:node", "//sim:sim_physics"],
)
cc_library(
  name="arduino_motors",
  srcs=["arduino_motors.cc"],
  hdrs=["arduino_motors.h"],
  deps=[":actuator_cmd_proto", "//util:node"],
)
cc_binary(
  name="arduino_main",
  srcs=["arduino_motors_main.cc"],
  deps=[":arduino_motors"],
)
cc_library(
  name="scamp",
  srcs=["scamp.cc"],
  hdrs=["scamp.h"],
  deps=[":actuator_cmd_proto", "//util:node"],
)
cc_binary(
  name="scamp_main",
  srcs=["scamp_main.cc"],
  deps=[":scamp"],
)
cc_library(
  name="waypoint_manager",
  srcs=["waypoint_manager.cc"],
  hdrs=["waypoint_manager.h"],
  deps=[":actuator_cmd_proto", "//util:node", ":util"],
)
cc_binary(
  name="waypoint_manager_main",
  srcs=["waypoint_manager_main.cc"],
  deps=[":waypoint_manager"],
)
cc_binary(
  name="simple_control_main",
  srcs=["simple_main.cc"],
  deps=[":simple_control"],
)
cc_library(
  name="line_tacking",
  srcs=["line_tacking.cc"],
  hdrs=["line_tacking.h"],
  deps=[":actuator_cmd_proto", "//util:node", "//sim:sim_physics"],
)
cc_binary(
  name="line_tacking_main",
  srcs=["line_tacking_main.cc"],
  deps=[":line_tacking"],
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
cc_binary(
  name="rudder_controller_main",
  srcs=["rudder_main.cc"],
  deps=[":rudder_controller"],
)
cc_test(
  name="control_test",
  srcs=["control_test.cc"],
  timeout="eternal",
  deps=[":simple_control",
        ":line_tacking",
        ":rudder_controller",
        ":waypoint_manager",
        "//sensor:state_estimator",
        "//util:testing",
        "//sim:sim_inter",
        "//ui:server",
        "//sim:csv_logger"],
)
