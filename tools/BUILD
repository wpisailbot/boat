package(default_visibility = ['//visibility:public'])

# Configuration corresponding to what should be used when deploying
# to the beaglebone. See scripts/BUILD for an example of how we
# use this.
config_setting(
  name = 'deploy_bbb',
  values = {'compilation_mode' : 'opt',
            'cpu' : 'bbb'},
)

# Create environments to correspond to each CPU that we
# can build for. The --auto_cpu_environment_group flag
# will match identically named environments/CPUs (yes,
# it's a silly system) and implicitly enforce that
# environments with, e.g., a "restricted_to = ['//tools:bbb']"
# will only build when run with "--cpu bbb". If a target
# depends on some other target that has more restrictive
# dependencies, then bazel will error (no matter what), because
# the dependency graph won't make sense. This only works on
# explicity declared dependencies or something. The behavior
# when you depend on something implicitly (e.g., as in the
# //scripts:deploy where it depends implicitly on
# //scripts:zip-outs via the data field).
environment(name = "bbb")
environment(name = "k8")
environment_group(
    name = "cpus",
    environments = [":bbb", ":k8"],
    # defaults are the default set of environments that
    # are valid for any given target.
    defaults = [":bbb", ":k8"],
)
