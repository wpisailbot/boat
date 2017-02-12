package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_proto_library(
  name="sim_debug_proto",
  srcs=["sim_debug.proto"],
  deps=["//control:actuator_cmd_proto"],
  protoc="@protobuf//:protoc",
  default_runtime="@protobuf//:protobuf",
)
cc_library(
  name="sim_physics",
  srcs=["sim_physics.cc"],
  hdrs=["sim_physics.h"],
  deps=["//control:actuator_cmd_proto",
        "//util:clock",
        "//control:util",
        ":sim_debug_proto",
        "//ipc:queue",
        ],
)
cc_library(
  name="fake_airmar",
  srcs=["fake_airmar.cc"],
  hdrs=["fake_airmar.h"],
  deps=["//can:can_proto",
        "//control:actuator_cmd_proto",
        "//util:node",
        "//control:util"],
)
cc_library(
  name="fake_internal_sensors",
  srcs=["fake_internal_sensors.cc"],
  hdrs=["fake_internal_sensors.h"],
  deps=["//control:actuator_cmd_proto", "//util:node"],
)
cc_library(
  name="sim_inter",
  srcs=["sim_inter.cc"],
  hdrs=["sim_inter.h"],
  deps=[":sim_physics",
        "//control:actuator_cmd_proto",
        ":fake_airmar",
        ":fake_internal_sensors",
        "//util:node"],
)
cc_library(
  name="csv_logger",
  srcs=["csv_logging.cc"],
  hdrs=["csv_logging.h"],
  deps=["//util:node", "//util:proto_util"],
)
cc_binary(
  name="basic_sim_main",
  srcs=["basic_sim_main.cc"],
  deps=[":sim_inter", ":csv_logger", "//control:simple_control", "//control:line_tacking"],
)
