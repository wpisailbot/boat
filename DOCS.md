This documentation’s purpose is mainly to record the status of the software for the person that handles the software next year -James

# Introduction

The software for the boat, for reference, is stored at
https://github.com/wpisailbot/boat This repository contains all the code that
runs onboard the beaglebone, with the intention of providing the necessary
build and deploy tools to handle the overall process. In the future, it may
contain other code (e.g., tools necessary to build/deploy for the vision
processor, or maybe even for the various microcontrollers). For now, some of
the other code can be found on the main wpisailbot repository (https://github.com/wpisailbot).

Overall, the code is organized in a manner reminiscent of ROS, for those
familiar with ROS. This means that there are various nodes running as processes
on the BBB, and each process communicates to other processes by sending and
receiving on “queues”, in a publisher-subscriber manner. Each individual
process contains the processing for some reasonably coherent individual
component (e.g., the sail controller, the CAN interface, etc.). The main
objective of separating these components into processes is primarily stylistic,
although there is some technical convenience—namely in the ability to specify
process priorities and to more easily choose which nodes to run on the fly,
although I believe that both of those things could plausibly be implemented in
useful ways using threads.

The structure of this report will be to progress incrementally through the
entire system, starting at the lowest level reasonable, covering how the code
itself is built, deployed, and run on the BBB. I then work my way up to the
actual code, covering the underlying communications and how the node
infrastructure works. I then cover some of the other low-level tools, such as
the CAN interface. I then discuss the testing infrastructure. Finally, I go into
a discussion of the actual logic controlling the boat and discuss the various
controllers and the such.

# Nomenclature

IPC: InterProcess Communication, refers to any method of communication between
separate processes running on a computer, in our case via shared memory.

BBB: BeagleBone Black, the main computer used onboard the boat

Bazel: The build system, created by Google, which we use for building the code

For listing locations, file paths are specified from the root of the boat
repository (github.com/wpisailbot/boat); When referring to build rules or
individual nodes, I use Bazel's notation. In this case, a preceding `//` refers
to the root of the git repository, with directory names specified before
slashes. A third-party library (specified in the WORKSPACE file) is referred to
by `@library_name//`. A individual target/rule is specified by a colon following
the directory path and then the name.

For instance, to refer to the `node` library, which is specified in the BUILD
file `util/BUILD`, I would refer to `//util:node`.

# Building the Code

The code is built using Bazel as the build system. Bazel is an open-source build
system that is developed by Google (they use a similar, but slightly more
customized, version called Blaze). The primary reasons to use Bazel are:
 - Builds are designed to be as reproducible as possible---in an ideal world,
   you could check out the git repository on any machine and the build would
   produce exactly the same binary regardless of external environment. Our current
   setup doesn’t quite manage this, but it still does much better than most build
   systems would.
   - These properties make it relatively easy to handle the cross-compilation for the BBB architecture
   - Currently, to get the code built on a new Ubuntu 16.04 instance, it takes minimal
 - Builds make reasonably effective use of caching and multithreading—as good or better than any other build system you’ll use.
 - It is relatively easy, once the original setup is done, to maintain and to perform simple tasks like adding new files that need to be built and the such.
 - I happen to like it

There are disadvantages, the biggest of which is a lack of support---there are
relatively few answers on StackOverflow, some features are still in beta,
experimental features may not work properly, etc. I haven’t had too many issues,
but occasionally an update will go out and there’ll be some minor change that
produces an obscure looking error.

Existing documentation can be found on Bazel’s website (bazel.build).

## How Bazel Runs

When Bazel goes to build a project, it does the following:
 1. Performs any initial setup (the first time it is run, e.g., creating build directories)
 2. Look in the WORKSPACE folder at the top of the project, attempting to execute each rule present there, which generally consists of:
    1. Downloading some sort of dependency (a specific version from a git repository, a .tar.gz file, etc.)
    2. Building the project (for dependencies that don’t use bazel, you must create a build file, which may just end up calling some other build system)
    3. Allowing the results from the build to then be accessed easily from the rest of the project
 3. Locate the BUILD file in each directory, corresponding with whatever you are trying to build
 4. Calculate the tree of dependencies required to build everything (they have some fancy optimization that calculates dependencies on the fly).
 5. For each rule that is a dependency, attempt to execute it (e.g., a C++ build, creating a zip file, running a shell script):
    1. For things like C++, attempt to locate the correct toolchain to use
 6. For C++ toolchains, there are big configuration files for specifying exactly
    what to use for a compiler, linker, etc. Currently, this uses the computer’s
    default g++ for all of this, but for the BBB, it downloads the toolchain and
    has a big configuration file to specify exactly where all the libraries, system
    headers, C++ standard library, etc. are

It is also possible to define custom rules in bazel (e.g., if you had some new programming language that you wanted to be able to build, you could specify it). I’m not terribly familiar with it, but Skylark is the right term to search for, and involves writing things in a Python-esque syntax.

I will now cover the various aspects of our build setup in more detail.

## Third-Party Libraries

There are various third-party libraries that we use. These are imported and
defined in the WORKSPACE file at the top level of the git repository. These fall
into roughly three categories:

 1. The libraries that work essentially out of the box because they already use Bazel (basically just some Google libraries).
 2. Libraries where I had to create a BUILD file just for us to be able to build the library—these generally are relatively simple.
 3. The cross-compiler toolchains for the BBB and for the raspberry pi

### Basic Syntax
Before going further, I will review the basic syntax and setup that is used to
specify the third party libraries. A typical entry in the WORKSPACE file might
look like:
```python
new_http_archive(
  name = "boost",
  url = "http://downloads.sourceforge.net/project/boost/boost/1.62.0/boost_1_62_0.zip?r=&ts=1505781564&use_mirror=phoenixnap",
  build_file = "third_party/boost.BUILD",
  type = "zip",
  strip_prefix = "boost_1_62_0/",
)
```

This entry specifies that we are trying to retrieve a library from a zip file
online (a similar syntax exists for retrieving from git repositories, which also
supplies a way of specifying exactly what commit or tag you want to use). We are
using the name `boost` to refer to the library---this means that
in the rest of the build files, we use `@boost//` to reference entries under the
`boost` library.

We are retrieving the file from the url provided (`url`), specifying that it
is a zip file and so will need to be extracted, and noting with the
`strip_prefix` that, once extracted, there will be on more layer of directories
than is really necessary and so removing the `boost_1_62_0` from everything will
make it easier to handle.

Finally, we specify a build file to use for the repository. If the library
already has some build file in it, then we do not need to specify one. However,
in this particular case it is required.

The full BUILD file in this case is:

```python
# The visibility must be set to be public; if not set,
# then no other parts of the project would be able to reference
# the library (you can also specify publicity individually by
# rule, but in most cases it is easiest just to do a blanket
# declaration).
package(default_visibility = ['//visibility:public'])

# Because boost is C++ library, we use the cc_library rule. If we had
# to, e.g., run a shell script to do some stuff, we might use a genrule or
# the such. See https://docs.bazel.build/versions/master/bazel-overview.html
# for the Bazel documentation and full lists of existing rules.
cc_library(
  # By giving this the name `boost`, we will be able to depend
  # on this library be referring to `@boost//:boost`. Note that
  # the first `boost` refers to the overall library, and the
  # second refers to this rule. For most of our libraries,
  # you only ever care about one rule under the library, making
  # the syntax mildly cumbersome, but it is of no major issue.
  name = "boost",
  # For the actual work we are doing, note that Boost is purely
  # a header library and so we just need to specify where all
#the header files are and include them overly generously.
  includes = ["./"],
  hdrs = glob(["boost/**"]),
)
```
The comments provide some background on what each line is doing.

Note that this works exactly the same as if the BUILD file were instead provided
in the top level directory of the original zip archive/git repo that we
download.

Once all of this is done, a normal build rule can depend on Boost by adding
`@boost//:boost` to its dependencies, see e.g. `ipc/BUILD`:

```python
cc_library(
  name="queue",
  srcs=["queue.cc"],
  hdrs=["queue.hpp", "message_queue.hpp"],
  deps=["//util:msg_proto", "//util:clock", "@glog//:glog", "@boost//:boost", ":test_queue"],
  linkopts=["-lrt", "-pthread"]
)
```

### Pre-packaged Libraries

The libraries that work out of the box (i.e., the original authors of the
library fully support Bazel) are:
 - [Google Protobuf](https://github.com/google/protobuf.git)
 - [gflags](https://github.com/wpisailbot/gflags.git) (albeit with a minor
   modification to make it play nice with `glog`)

#### Protobuf

The protobuf libraries work without modification. I did once encounter an error
where the protobuf libraries stopped working when Bazel was upgraded. However,
updating to a newer version of protobuf fixed the problem.

Google also provides a `cc_proto_library` Bazel rule that handles all the
building for a protobuf library. This does currently require placing a
`load("@protobuf//:protobuf.bzl", "cc_proto_library")` at the top of every
relevant BUILD file and then calling `cc_proto_library` as appropriate (see
`ipc/BUILD` for an example).

NOTE: Bazel has, since I originally set this up, added a builtin protobuf setup.
However, it is not a drop-in replacement, and I have yet to go through and start
using it.

If you need an introduction to what protobuf actually is, I suggest reviewing
their
[documentation](https://developers.google.com/protocol-buffers/docs/overview).
Essentially it provides a way to specify a message format and provides all the
utilities for serialization and the such. This is used by the interprocess
communication protocols and is discussed more later.

TODO: Make sure I actually do discuss it more later...

#### gflags

The gflags library works without modification. However, the glog library relies
on glog and expects to find it in a particular namespace (when calling it from
C++). As such, I have made a [single
modification](https://github.com/wpisailbot/gflags/commit/68abb16f1917f1a59a3986062beafb8eb0d0c16c)
to the gflags library so that we can use it more easily. There is nothing else
of particular note in the build. To depend on it, use `@gflags//:gflags`.

See the gflags [documentation](https://gflags.github.io/gflags/) for
usage---there is nothing particularly special about how we use gflags, and
examples of its usage can be seen, e.g., in `rigid_wing/rigid_wing_ping.cc`.
When you call `sailbot::util::Init`, the initialization required for gflags will
be called.

### Libraries Requiring BUiLD files

These libraries either require separate BUILD files, or are not officially
supported and I am using some separate person's BUILD file, generally with minor
modifications:

 - [glog](https://github.com/google/glog) (Since I originally set this up, their
   official repository now uses Bazel and I have not migrated to use their
   implementation yet)
 - [gtest](https://github.com/google/googletest) (same deal as glog)
 - [boost](http://www.boost.org/) (see examples above)
 - [Eigen](http://eigen.tuxfamily.org/) (Matrix library)
 - [ws](https://github.com/uWebSockets/uWebSockets.git) (WebSocket library)
 - [zlib](https://github.com/madler/zlib.git) (Random dependency for ws)
 - [uv](https://github.com/libuv/libuv.git) (Another ws dependency)
 - [OpenSSL](https://github.com/wpisailbot/bazel-openssl.git) (Another ws dependency)

#### glog

As mentioned, when I first set everything up, Bazel was less used and more in beta
than it is now. `glog` now has a pre-existing Bazel setup. It would be valuable
for someone to go in and update to this (and do the same for `gtest`). However,
for the time being, I have used some older workarounds for getting everything
where I want it.

See `third_party/glog.BUILD` for the BUILD file I use and where I got it from.
It depends on `gflags` and `glog` can be depended on by `@glog//:glog`.

`glog` provides us with the ability to easily to textual logging. There is some
[documentation](http://rpg.ifi.uzh.ch/docs/glog.html) available at a third
party, but I'm not sure about official documentation. This logging will involve
disc writes and so are non-realtime. Unless you are:
 - Only logging in debugging mode (see glog documentation)
 - Only logging warnings/errors
 - In a non-realtime portion of the code
For examples of logging warnings/errors, you can glance at
`rigid_wing/rigid_wing.cc`, which has various warnings/errors when attempting to
use sockets.

There are also lots of logging statements in the simulation code, e.g.
`sim/sim_physics.cc`, which uses verbosity levels to avoid spamming too much
logging unless the user requests it on the command line with `-v 3` (or some
other number).

#### gtest

The Google testing framework, `gtest` or `googletest`, provides a framework for
doing unit tests.

The build file is in `third_party/gtest.BUILD` and is relatively simple, as it
just needs to build and include everything. As mentioned, in the future, we
should transition to using Google's BUILD file.

Tests should use `@gtest//:main`, e.g. the `queue_test` rule in `ipc/BUILD`.
`ipc/queue_test.cc` contains the corresponding example for basic test writing.

If you are doing simulations or running nodes, there are some utilities that I
have written, e.g. `util/testing.[h,cc]`.

See the [gtest
documentation](https://github.com/google/googletest/blob/master/googletest/docs/Primer.md)
for usage.

#### Boost

[Boost](http://www.boost.org) is a set of general-purpose C++ header libraries.
The way they are built is discussed in a section above, using
`third_party/boost.BUILD`, which essentially just adds all the headers to the
include path.

Currently, Boost is only used by the IPC (see `//ipc:queue`), although if other
Boost libraries are helpful, they may certainly be used.

#### Eigen

Eigen is a matrix header library that is built in essentially the same manner as Boost.

Eigen is a matrix utility library that provides convenient ways to construct,
multiply, etc. matrices. IT also contains various mathematical and linear
algebra functions for manipulating matrices, although nothing too out of the
ordinary. It is currently just used in some experimental controls things and in
simulation.

#### zlib, uv, and ws

Nothing of particular note; each has a BUILD file in `third_party`. The
websocket library is currently only used by `//ui:server`. The UI server is
discussed in more detail later in this document.

#### OpenSSL

We do not actually _use_ any OpenSSL features, however in order to build `ws`,
we need OpenSSL. Currently, it takes a long time to build and would ideally be
removed as a dependency (or, if we stop using WebSockets, then it can go away).

The OpenSSL build is in [our own
repository](https://github.com/wpisailbot/bazel-openssl), and the BUILD file
primarily calls `make` to build everything. It does involve a bit of work to get
it to use the BBB cross-compiler (all this is done in the BUILD file on that
repository). By calling `make`, we are breaking some of the guarantees that
Bazel provides about build reproducibility.

### Toolchain libraries

In order to actually build code for the BBB, we need a cross-compiler. I also
have a cross-compiler setup for the Raspberry Pi, but much of the high-level setup is
essentially identical. However, it should be noted that there are generally
going to be lots of little configuration knobs you will need to tweak for
different target architectures. I am not an expert on these things.

## Deploying the Code

# BBB Setup

Steps:

1. Download the Debian Stretch 9.3 for IoT on the BeagleBone from the
Beagleboard [Latest Images](https://beagleboard.org/latest-images) page. The GUI
version should also work, but we have no need for a GUI.
2. Image a microSD card. On linux, you would do something like
  `xzcat bone-debian-9.3-iot-armhf-2018-01-28-4gb.img.xz | sudo dd of=/dev/mmcblk0`,
  where `/dev/mmcblk0` is whatever device corresponds with the SD card.
3. Plug the BBB into your computer, ssh in with `ssh debian@192.168.7.2`,
password `temppwd` by default.
4. While ssh'd in, do `mkdir -p ~/bin/sailbot` to create the directory where
   sailbot stuff will happen.
5. Deploy code to the beaglebone by `bazel run -c opt --cpu bbb //scripts:deploy -- 192.168.7.2`
5. To setup the web UI, you need to configure the Apache server:
   1. Change `/etc/apache2/apache2.conf`, and add:

      ```
      <Directory /home/debian/bin/sailbot/html>
        Options Indexes FollowSymLinks
        AllowOverride None
        Require all granted
      </Directory>
      ```

       Preferably after a similar entry for `/var/www`
   2. Then, change `/etc/apache2/sites-enabled/000-default.conf` by changing `/var/www` to `/home/debian/bin/sailbot/html`.
6. ***NOTE: Depending on the version of code you are using, you may need to
change `bringup-can.sh` to refer to `BB-CAN1` instead of `BB-DCAN1`***
7. Make the code run on startup by adding an entry `@reboot /home/debian/bin/sailbot/startup.sh`
   to the root crontab (accessible via `sudo crontab -e`).
7. Start the code by running a `sudo ~/bin/sailbot/startup.sh`
8. The code should now be running. Check that all the processes look like
   they're running (via the web interface or `top`. You can also check, e.g.,
   `/tmp/can-dump.ERROR` for if there are any interesting errors going on.
9. If you'd like, flash everything to the eMMC by modifying the last line in the
`/boot/uEnv.txt` (there should be a comment there), rebooting with the SD card
in, wait for all the lights to go off, remove the SD card, and boot again.

# Alternate config for configuring CAN on boot
The following steps replace the `bringup-can.sh` file:
1. Add the following to /boot/uEnv.txt (may need to change to `BB-CAN1` depending on setup)
   capemgr.enable_partno=BB-DCAN1
2. Make a new file `/etc/modules-load.d/can.conf` with contents:
   can
   can-dev
   can-raw
3. Append the following to `/etc/network/interfaces`:
   allow-hotplug can0
     iface can0 can static
       bitrate 250000

   
