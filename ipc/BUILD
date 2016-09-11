cc_library(
  name="queue",
  hdrs=["queue.hpp"],
  linkopts=["-lrt", "-pthread"],
)
# Unfortunately, can't access system resources (ie, shm) inside cc_test sandbox
cc_binary(
  name="queue_test",
  srcs=["queue_test.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "@gtest//:main",
    ":queue",
  ],
)
