package(default_visibility = ['//visibility:public'])
genrule(
  name="rudder_gains",
  srcs=["controls.py"],
  outs=["rudder_gains.h"],
  cmd="./$(location rudder.py) $@",
  tools=["rudder.py"],
)
