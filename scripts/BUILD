genrule(
  name="zip_outs",
  srcs=["bringup-can.sh",
        "startup.sh",
        "killall.sh",
        "waypoints.pba",
        "//util:logger_main",
        "//can:can-dump",
        "//ui:server_main",
        "//control:simple_control_main",
        "//control:ballast_control_main",
        "//control:adaptive_control_main",
        "//control:line_plan_main",
        "//control:scamp_main",
        "//control:line_tacking_main",
        "//control:waypoint_manager_main",
        "//sensor:state_estimator_main",
        "//sensor:sbus-test-run",
        "//can:can-ping",
        "//rigid_wing:rigid_wing_ping",
        "//rigid_wing:rigid_wing_main",
        "//management:monitor_main",
        "//util:log_replay_test",
        "//util:ping",
        "//util:pong"],
  outs=["output.zip"],
  cmd="zip -j $(location output.zip) $(SRCS)",
)
sh_binary(
  name="deploy",
  srcs=["deploy.sh"],
  # Strictly, this will provide the functional behavior we want if we
  # just remove the select and have data = [...] and run with
  # --auto_cpu_environment_groups=//tools:cpus. However, that produces
  # an inscrutable error message (as of bazel 0.9.0, it will just
  # say that no target exists, because auto_cpu_en... silently
  # removes targets that can't be built for a particular cpu).
  # If you want to deploy without these constraints, remove the
  # select statement and just leave the ["output.zip", "//ui:site.zip"],
  # as well as the restricted_to statement.
  # The \033[1m and \033[0m turn on/off bolding for error.
  data = select({'//tools:deploy_bbb' : ["output.zip", "//ui:site.zip"]},
                no_match_error = "\033[1mShould be run with -c opt and --cpu bbb\033[0m"),
  args=["$(location output.zip) $(location //ui:site.zip)"],
  restricted_to = ['//tools:bbb'],
)
