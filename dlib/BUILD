package(default_visibility = ['//visibility:public'])
cc_library(
  name="dlib",
  srcs=["build/dlib/libdlib.a"],
  hdrs=glob(["dlib/**"]),
  linkopts=["-Ldlib/build/dlib -ldlib"],
)
