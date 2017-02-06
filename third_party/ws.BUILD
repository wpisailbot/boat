cc_library(
  name = "ws",
  srcs = glob(["src/*.cpp"]),
  hdrs = glob(["src/*.h"]),
  deps = ["@zlib//:zlib", "@uv//:uv", "@openssl//:dep_libs"],
  #linkopts = ["-lssl", "-lcrypto"],
  includes = ["src/"],
  visibility = ["//visibility:public"],
)
