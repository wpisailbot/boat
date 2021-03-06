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
  deps=["//rigid_wing:rigid_wing_proto",
        ":actuator_cmd_proto",
        "//util:node",
        "//util:proto_util",
        "//sim:sim_physics"],
)
cc_library(
  name="ballast_control",
  srcs=["ballast.cc"],
  hdrs=["ballast.h"],
  deps=[":actuator_cmd_proto", "//util:node", ":util"],
)
cc_library(
  name="adaptive_control",
  srcs=["adaptive.cc"],
  hdrs=["adaptive.h"],
  deps=["//rigid_wing:rigid_wing_proto", ":actuator_cmd_proto", "//util:node", "//sim:sim_physics"],
)
cc_library(
  name="line_plan",
  srcs=["line_plan.cc"],
  hdrs=["line_plan.h"],
  deps=[":actuator_cmd_proto",
        "//util:node",
        "//util:proto_util",
        "//math:polygon",
        "//sim:sim_physics"],
)
cc_library(
  name="rudderTest",
  srcs=["rudderTest.cc"],
  hdrs=["rudderTest.h", "simple.h"],
  deps=["//util:node", ":util"],
)
cc_library(
  name="scamp",
  srcs=["scamp.cc"],
  hdrs=["scamp.h"],
  deps=[":actuator_cmd_proto",
        ":util",
        "//util:proto_util",
        "//util:node"],
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
cc_binary(
  name="ballast_control_main",
  srcs=["ballast_main.cc"],
  deps=[":ballast_control"],
)
cc_binary(
  name="adaptive_control_main",
  srcs=["adaptive_main.cc"],
  deps=[":adaptive_control"],
)
cc_binary(
  name="line_plan_main",
  srcs=["line_plan_main.cc"],
  deps=[":line_plan"],
)
#cc_library(
#  name="ballastTest",
#  srcs=["ballastTest.cc"],
#  hdrs=["ballastTest.h", "simple.h"],
#  deps=["//util:node", ":util"],
#)
#cc_binary(
#  name="ballastTest_main",
#  srcs=["ballastTest_main.cc"],
#  deps=[":ballastTest"],
#)
cc_binary(
  name="rudderTest_main",
  srcs=["rudderTest_main.cc"],
  deps=[":rudderTest"],
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
cc_test(
  name="line_plan_test",
  srcs=["line_plan_test.cc"],
  deps=[":line_plan",
        "//util:testing",
        "@gtest//:main"],
)
cc_test(
  name="control_test",
  srcs=["control_test.cc"],
  timeout="eternal",
  deps=[":simple_control",
        ":adaptive_control",
        #":line_tacking",
        ":line_plan",
        ":waypoint_manager",
        "//sensor:state_estimator",
        "//util:testing",
        "//sim:sim_inter",
        "//ui:server",
        "//sim:csv_logger"],
)
