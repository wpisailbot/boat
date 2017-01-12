cc_library(
  name = "main",
  srcs = glob(
    ["src/*.cc"],
    exclude = ["src/gtest-all.cc"]
    ),
  hdrs = glob([
    "include/**/*.h",
    "src/*.h"
    ]),
  includes = ["gtest/include", "include"],
  linkopts = ["-pthread"],
  visibility = ["//visibility:public"],
)
