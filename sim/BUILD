cc_library(
  name="sim_physics",
  srcs=["sim_physics.cc"],
  hdrs=["sim_physics.h", "util.h"],
  deps=["//control:actuator_cmd_proto"],
)
cc_library(
  name="sim_inter",
  srcs=["sim_inter.cc"],
  hdrs=["sim_inter.h"],
  deps=[":sim_physics","//control:actuator_cmd_proto","//util:node"],
)
