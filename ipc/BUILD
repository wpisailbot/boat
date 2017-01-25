package(default_visibility = ['//visibility:public'])
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_library(
  name="test_queue",
  srcs=["test_queue.cc"],
  hdrs=["test_queue.h"],
  deps=["//util:clock"],
  linkopts=["-pthread"],
)
cc_library(
  name="queue",
  srcs=["queue.cc"],
  hdrs=["queue.hpp", "message_queue.hpp"],
  deps=["//util:msg_proto", "//util:clock", "@glog//:glog", "@boost//:boost", ":test_queue"],
  linkopts=["-lrt", "-pthread"]#, "-lglog"],
)
cc_proto_library(
  name="queue_test_msg",
  srcs=["queue_test_msg.proto"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
# Unfortunately, can't access system resources (ie, shm) inside cc_test sandbox
cc_test(
  name="queue_test",
  srcs=["queue_test.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "@gtest//:main",
    ":queue",
    ":queue_test_msg",
  ],
)
