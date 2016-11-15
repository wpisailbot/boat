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
  tag = "v3.0.2",
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

new_http_archive(
  name = 'org_linaro_components_toolchain_gcc_5_3_1',
  build_file = 'compilers/linaro_linux_gcc_5.3.1.BUILD',
  url = 'https://bazel-mirror.storage.googleapis.com/releases.linaro.org/components/toolchain/binaries/latest-5/arm-linux-gnueabihf/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabihf.tar.xz',
  strip_prefix = 'gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabihf',
)

new_http_archive(
  name = "boost",
  url = "https://sourceforge.net/projects/boost/files/boost/1.62.0/boost_1_62_0.zip/download",
  build_file = "third_party/boost.BUILD",
  type = "zip",
  strip_prefix = "boost_1_62_0/",
)
