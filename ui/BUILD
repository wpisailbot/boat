package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="server",
  srcs=["server.cc"],
  hdrs=["server.h"],
  deps = [
    "//util:node",
    "//ipc:queue",
    "//util:msg_proto",
    "@protobuf//:protobuf",
    "@ws//:ws",
    "//util:proto_util",
  ]
)
cc_binary(
  name="server_main",
  srcs=["server_main.cc"],
  deps=[":server"],
)
