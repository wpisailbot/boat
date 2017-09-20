package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="elqr_lib",
  srcs=["elqr_routing.cc"],
  hdrs=["elqr_routing.h"],
  deps=["//control:util"],
)
cc_binary(
  name="elqr_bin",
  deps=[":elqr_lib"],
)
