package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="monitor",
  srcs=["monitor.cc"],
  hdrs=["monitor.h"],
  deps = ["//util:node",
          "//can:can_proto",
         ],
)
cc_binary(
  name="monitor_main",
  srcs=["monitor_main.cc"],
  deps=[":monitor"],
)
