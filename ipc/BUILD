cc_library(
  name="queue",
  hdrs=["queue.hpp"],
  linkopts=["-lrt", "-pthread"],
)
cc_binary(
  name="queue_test",
  srcs=["queue_test.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "@gtest//:main",
    ":queue",
  ],
)
