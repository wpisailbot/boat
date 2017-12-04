new_http_archive(
  name = "gtest",
  url = "https://github.com/google/googletest/archive/release-1.8.0.zip",
  sha256 = "f3ed3b58511efd272eb074a3a6d6fb79d7c2e6a0e374323d1e6bcbcc1ef141bf",
  build_file = "third_party/gtest.BUILD",
  strip_prefix = "googletest-release-1.8.0/googletest",
)

git_repository(
  name = "protobuf",
  remote = "https://github.com/google/protobuf.git",
  tag = "v3.5.0",
  #commit = "e7982e409deab9cb4390dd574441604e846caf7f", # master on 20160727
)

git_repository(
  name="gflags",
  remote = "https://github.com/wpisailbot/gflags.git",
  commit = "68abb16f1917f1a59a3986062beafb8eb0d0c16c",
)

new_git_repository(
  name="glog",
  remote = "https://github.com/google/glog.git",
  commit = "b6a5e0524c28178985f0d228e9eaa43808dbec3c",
  build_file = "third_party/glog.BUILD"
)

new_git_repository(
  name="ws",
  remote = "https://github.com/uWebSockets/uWebSockets.git",
  tag = "v0.14.0",
  build_file = "third_party/ws.BUILD"
)

# For BBB
new_http_archive(
  name = 'org_linaro_components_toolchain_gcc_5_3_1_gnueabihf',
  build_file = 'compilers/linaro_linux_gcc_5.3.1_gnueabihf.BUILD',
  #url = 'https://bazel-mirror.storage.googleapis.com/releases.linaro.org/components/toolchain/binaries/latest-5/arm-linux-gnueabihf/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabihf.tar.xz',
  url = 'https://releases.linaro.org/components/toolchain/binaries/5.3-2016.05/arm-linux-gnueabihf/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabihf.tar.xz',
  strip_prefix = 'gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabihf',
)

# For Pi
#new_http_archive(
#  name = 'org_linaro_components_toolchain_gcc_5_3_1_gnueabi',
#  build_file = 'compilers/linaro_linux_gcc_5.3.1_gnueabi.BUILD',
#  url = 'https://releases.linaro.org/components/toolchain/binaries/5.3-2016.05/arm-linux-gnueabi/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi.tar.xz',
#  strip_prefix = 'gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi',
#)
# This compiler was compiled using https://github.com/wpisailbot/rpi-compiler
# Couldn't figure out how/if it was possible to use the BBB compiler for the
# RPI. Tested on the RPI Zero W.
new_http_archive(
  name = 'rpi_gcc_5_4_1_gnueabihf',
  build_file = 'compilers/manual_rpi.BUILD',
  url = 'http://users.wpi.edu/~jbkuszmaul/sailbot/arm-rpi-linux-gnueabihf.tar.xz',
  strip_prefix = 'arm-rpi-linux-gnueabihf',
)

new_http_archive(
  name = "boost",
  url = "http://downloads.sourceforge.net/project/boost/boost/1.62.0/boost_1_62_0.zip?r=&ts=1505781564&use_mirror=phoenixnap",
  build_file = "third_party/boost.BUILD",
  type = "zip",
  strip_prefix = "boost_1_62_0/",
)

new_http_archive(
  name = "eigen",
  url = "http://bitbucket.org/eigen/eigen/get/3.3.2.tar.bz2",
  build_file = "third_party/eigen.BUILD",
  strip_prefix = "eigen-eigen-da9b4e14c255/",
)

new_git_repository(
  name = 'zlib',
  remote = 'https://github.com/madler/zlib.git',
  commit = '50893291621658f355bc5b4d450a8d06a563053d',
  build_file = 'third_party/zlib.BUILD',
)

new_git_repository(
  name = 'uv',
  remote = 'https://github.com/libuv/libuv.git',
  tag = 'v1.11.0',
  build_file = 'third_party/uv.BUILD',
)

git_repository(
  name = 'openssl',
  remote = 'https://github.com/wpisailbot/bazel-openssl.git',
  commit = 'd5c1dcb247ee74000b58d54408da612c550b4978',
  init_submodules = True,
)
