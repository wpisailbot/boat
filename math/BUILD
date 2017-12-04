package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="polygon",
  srcs=["polygon.cc"],
  hdrs=["polygon.h"],
  deps=["@eigen//:eigen", "@glog//:glog"],
)
cc_test(
  name="polygon_test",
  srcs=["polygon_test.cc"],
  deps=[":polygon",
        "@gtest//:main"],
)
