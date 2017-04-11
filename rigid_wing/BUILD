package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_proto_library(
  name="rigid_wing_proto",
  srcs=["rigid_wing.proto"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
cc_library(
  name="rigid_wing",
  srcs=["rigid_wing.cc"],
  hdrs=["rigid_wing.h"],
  deps=["//util:node",
        ":rigid_wing_proto",
        "//control:util",
        "@gflags//:gflags",
        ],
)
cc_binary(
  name="rigid_wing_ping",
  srcs=["rigid_wing_ping.cc"],
  deps=[":rigid_wing", "//ui:server"],
)
cc_binary(
  name="rigid_wing_main",
  srcs=["rigid_wing_main.cc"],
  deps=[":rigid_wing"],
)
