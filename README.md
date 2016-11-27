This repository contains the code meant to run on the boat for the WPI Sailbot
in the 2016-2017 school year.

## Structure

The exact locations of things is a bit subject to change right now, but:

- The top level is reserved for build file, READMEs, etc.
- `ipc/` contains the code for, well, IPC.
- `util/` contains what is, right now, any other infrastructure-related code

## Building and Running the Code

I have been using [Bazel](https://www.bazel.io/) for the build system. If you
install bazel on your system, you should be able to run `bazel build //...` to
build all of the code.

Unfortunately, I have not yet integrated all of the dependencies with bazel and
there are almost certainly prerequisites missing beyond that which I list here:

- A C++ compiler that bazel will recognize/locate (I assume gcc and clang both work)
- Probably some other things I'm missing

In order to compile for the BBB, use `bazel build --crosstool_top=//tools/arm_compiler:toolchain --cpu=armeabi-v7a //some/target:here`

### Tests and Examples

There are a few tests/examples that I have throughout the code:
- `//ipc:queue_test` needs system resources in order to run, and so must be run
    manually, rather than through `bazel test`.
- `//util:ping`, `//util:pong`, and `//util:logger_main` can be run
    simultaneously to test out the framework and see how it works.
    The logs can then be read back using the example `//util:log_reader_main`

## TODO

- Clean up build process so it has fewer prerequisites and is generally cleaner
  - Also, the gtest download/build should be cleaned up
- Documentaion (always)
- More unit testing
- Validate system more thoroughly
- Get started on hardware-specific libraries
- Set up cross-compilation for BBB (Mostly done)
- Separate out ping, pong, and log examples into separate location
- Set up infrastructure for log replay
  - Actually replaying the logs
- Take a closer look at Boost's implementation of queues

## Notes
- We are using proto2, not proto3, because I (James) observed some oddities
  where ParseFromArray would Clear the message and force re-allocation of
  internal sub-messages. Still not 100% sure why proto2 is working better than 3.
