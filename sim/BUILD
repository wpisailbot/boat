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
  name="sim_util",
  srcs=["util.cc"],
  hdrs=["util.h"],
  deps=[":sim_debug_proto", "@eigen//:eigen"],
)
cc_library(
  name="sim_physics",
  srcs=["sim_physics.cc"],
  hdrs=["sim_physics.h"],
  deps=["//control:actuator_cmd_proto",
        "//util:clock",
        ":sim_util",
        ":sim_debug_proto",
        "//ipc:queue",
        ],
)
cc_library(
  name="sim_inter",
  srcs=["sim_inter.cc"],
  hdrs=["sim_inter.h"],
  deps=[":sim_physics","//control:actuator_cmd_proto","//util:node"],
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
  deps=[":sim_inter", ":csv_logger", "//control:simple_control"],
)
