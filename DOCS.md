This documentation’s purpose is mainly to record the status of the software for the person that handles the software next year -James

Builds to pdf decently with:
`pandoc -s -N -S --toc -o DOCS.pdf -t latex  -f markdown_github-hard_line_breaks DOCS.md`

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

There is a section at the end on the controller/planner and the things that we
learned at IRSC2018. All of the non-controller/planner code seemed to be largely
robust at IRSC. The only change I would suggest beyond that would be to
investigate more thoroughly the performance and usefulness of the current
websocket server, as due to lossy connections with the 900 MHz radios at ranges
>~200m, the current protocol may not make sense for telemetry and RC control.

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
 - The above qualities make it easier to build and deploy code to the BBB
   because, if we do not build everything ourselves from source, then we have to
   handle locating built libraries for the correct ARM architecture and
   whenever we set up a new BBB we'd have to reinstall the appropriate
   libraries.
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

I suspect that Bazel has changed the preferred method for specifying toolchains,
but I will briefly discuss the current setup.

Most of the code for handling the toolchains is in `tools/` and `compilers/`.
The relevant toolchains themselves are downloaded in the `WORKSPACE` file (see
`org_linaro_components_toolchain_gcc_5_3_1_gnueabihf` and
`rpi_gcc_5_4_1_gnueabihf`).

The `compilers/*.BUILD` files merely group the files in the downloaded archives
so that they have useful names for usage by the rest of Bazel.

`tools/BUILD` shouldn't be necessary for basic cross-compilation. What it does
do is provide rules to make it so that you can specify individual targets (e.g.,
the deploy) as only being buildable for certain environments, so that you don't
accidentally deploy local code to the BBB.

`tools/bazel.rc` is also mostly cosmetic and provides some default arguments for
when you call Bazel that make things generally easier.

In `tools/arm_compiler/`, we have the main work for actually configuring at
setting up the cross-compiler. The `BUILD` file does some more work to group
files together as well as specifying exactly which toolchains are present, which
configuration files they use, etc. The `CROSSTOOL` file itself is the most
opaque of the configuration files and provides all the configuration options for
the cross-compiler itself. This consists of a lot of obscure compiler options,
specifying system library locations, etc. These files are generally constructed
by copying existing `CROSSTOOL` files and modifying them till they work.
Ideally, someone would know what each and every option did and be able to notice
that we should really be doing something differently. But that would be a lot of
work.

The `tools/arm_compiler/linaro_linux_gcc/` directory contain some shell scripts
for calling the actual cross-compiler binaries, with one directory for the BBB
compilers and one for the RPi.

## Deploying the Code

In order to deploy the code, we *currently* use `scp` to copy over a zip file of
all the binaries and then extract them. We could consider, e.g., using `rsync`
or some other protocol to more efficiently copy only the necessary bytes.

This work is done in `scripts/deploy.sh`. This actual `scp` is not overly
interesting.

In order to force the user to only deploy code built (a) for the BBB processor
and (b) with optimization flags, we use the `data` and `restricted_to` portions
of the `//scripts:deploy` build target. See the `scripts/BUILD` file for
details.

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
   sailbot stuff will happen, and do a `mkdir ~/logs` to ensure that the boat
   will have a place to log data.
5. Deploy code to the beaglebone by `bazel run -c opt --cpu bbb //scripts:deploy -- 192.168.7.2`
5. To setup the web UI, you need to configure the Apache server:
   1. Change `/etc/apache2/apache2.conf`, and add:

          <Directory /home/debian/bin/sailbot/html>
            Options Indexes FollowSymLinks
            AllowOverride None
            Require all granted
          </Directory>

       Preferably after a similar entry for `/var/www`
   2. Then, change `/etc/apache2/sites-enabled/000-default.conf` by changing `/var/www` to `/home/debian/bin/sailbot/html`.
6. To overlay the device tree for CAN1 at boot, edit the /boot/uEnv.txt file line from: \
   `uboot_overlay_addr4=/lib/firmware/<file4>.dtbo` \
   to: \
   `uboot_overlay_addr4=/lib/firmware/BB-CAN1-00A0.dtbo` \
   On the next boot, `/sys/kernel/debug/pinctrl/44e10800.pinmux/pinmux-pins` should show (need to log into
   root in order to be able to find this file): \
     `pin 96 (PIN96): 481d0000.can (GPIO UNCLAIMED) function pinmux_dcan1_pins group pinmux_dcan1_pins` \
     `pin 97 (PIN97): 481d0000.can (GPIO UNCLAIMED) function pinmux_dcan1_pins group pinmux_dcan1_pins`
   If you have troubles with this and you are booting off an SD card with an
   older OS still in the builtin eMMC, try upgrading the eMMC OS as it may be
   out of date and still providing uboot (TODO(james): Check this for myself;
   Cosine reported it on an unrelated project).
7. Make the code run on startup by adding an entry `@reboot /home/debian/bin/sailbot/startup.sh`
   to the root crontab (accessible via `sudo crontab -e`).
7. Start the code by running a `sudo ~/bin/sailbot/startup.sh`
8. The code should now be running. Check that all the processes look like
   they're running (via the web interface or `top`. You can also check, e.g.,
   `/tmp/can-dump.ERROR` for if there are any interesting errors going on.
9. If you'd like, flash everything to the eMMC by modifying the last line in the
`/boot/uEnv.txt` (there should be a comment there), rebooting with the SD card
in, wait for all the lights to go off, remove the SD card, and boot again.
10. The BBB should have the IP address of `192.168.0.21` on the boat network. Assuming we are using
    Ethernet, add the following to the `/etc/network/interfaces` file:
    ```
    allow-hotplug eth0
    iface eth0 inet static
        address 192.168.0.21
        netmask 255.255.255.0
    ```

Furthermore, you may want to install the `fake-hwclock` package on debian.
This will make it so that, when you reboot the system, rather than the
system clock resetting to the same epoch time every time, it will stay
incremented. The clock will still be inaccurate, but it will be monotonic.

### Alternate config for configuring CAN on boot (Only tested for Debian 8.6)
The following steps replace the `bringup-can.sh` file:
1. Add the following to /boot/uEnv.txt (may need to change to `BB-CAN1` depending on setup)\
   `cape_enable=bone_capemgr.enable_partno=BB-DCAN1` <-Debian 8.6
2. Make a new file `/etc/modules-load.d/can.conf` with contents:\
   `can
   can-dev
   can-raw`
3. Append the following to `/etc/network/interfaces`: \
   ```
     allow-hotplug can0
     iface can0 can static
         bitrate 250000
   ```

# Software Infrastructure

This section shall cover the generally applicable software infrastructure,
namely:
- The IPC (Interprocess Communication)
- Node structure/setup
- General utilities for the above (e.g., clocks)
- Logging/Log-replay
- WebSocket UI
- Testing infrastructure

## IPC

The IPC in this system is based on the [Boost
message_queue](http://www.boost.org/doc/libs/1_66_0/doc/html/interprocess/synchronization_mechanisms.html#interprocess.synchronization_mechanisms.message_queue)
and Google Protobufs.
The Boost message_queue uses shared memory to facilitate communications. You can
see this while the processes are running by glancing at `/dev/shm/`, which is a
file system mapped to the physical RAM. Boost has written interprocess
semaphores and setup the message queue so that race conditions and the such
*shouldn't* be an issue.

I have modified the Boost `message_queue.hpp` (see `ipc/message_queue.hpp`) such
that, rather than being a queue in which each message published will only be
processed by a single subscriber, each subscriber will process each published
message exactly once.

It is important to note that, if the process is terminated while working with a
message queue, the queue may be left in an inconsistent state. This was the
original motivation to ensure that a `SIGINT` (Ctrl-C) would be handled cleanly
and shutdown the process deliberately rather than immediately terminating.

In order to handle cleanup of the message queues, I use a Boost IPC semaphore
(unmodified from Boost's version) to keep track of the number of publishers on a
queue. When the number drops to zero when the last writer finishes, I delete the
queue.

### Files

`util/msg.proto`: Contains `LogEntry` protobuf, which is a list of all the
  different queues and their message types

`util/node.*`: Contains `RegisterHandler` and `AllocateMessage` utility
  functions (discussed below)

`ipc/`: The main folder for IPC stuff

`ipc/BUILD`: Build file for IPC stuff

`ipc/message_queue.hpp`: Modified Boost library for IPC over shared memory

`ipc/queue.hpp`: Main header file for queues, including the `Queue` utility
  class and the `ProtoQueue` template class which wraps `Queue`.

`ipc/queue.cc`: Source for the `Queue` class, uses `ipc/message_queue.hpp`

`ipc/test_queue.*`: Implementation spoofing `ipc/message_queue.hpp` for tests
  run within a single process that may not want to accidentally interfere with
  other processes.

`ipc/queue_test_msg.proto`: Simple protobuf definition for testing

`ipc/queue_test.cc`: Basic unit tests for classes in `ipc/queue.*`.

### Protobuf IPC Setup

In order to actually structure the data in the raw message queue (as Boost is
just sending raw bytes back and forth), I use Google Protobufs. Protobufs allow
for a message structure specification (providing the general primitive types you
would expect such as `double`, `int`, etc. as well as nesting of messages). Out
of laziness, I will direct readers to the Protobuf website for documentation on
[Protobuf and
C++](https://developers.google.com/protocol-buffers/docs/cpptutorial).

For serializing the data, we simply use the protobuf [utility functions](https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.message_lite#MessageLite.SerializeToArray.details)
for that.

Finally, there is still a bit of extra work that we do to facilitate easier
logging and to coordinate what message types are expected on what queues.
There are two issues at play here. First, when we send a message on a given
queue, any subscribers must be able to know how to parse the received message.
Second, when logging, we must in some way indicate what message queue a given
log message was sent on, as well as any other metadata we may want to include
(namely a timestamp).

The way that we accommodate these is to have only a single message type that is
used for the actual serialization. This message type contains a time stamp and
then a bunch of optional fields, one for each message queue. When we send a
message, we only fill out one of those fields (plus the timestamp). When
receiving on a queue, we assume that the necessary fields should be filled out
and error if not. When reading a log, we find out which field is filled out and
send out on that queue. If not exactly one field (plus timestamp) is filled out,
then there is an error.

To make this a bit more concrete, we look at a portion of the `util/msg.proto`,
which contains the `LogEntry` protobuf which is this main logging structure.

```protobuf
message LogEntry {
  optional google.protobuf.Timestamp time = 1;

  // The name of all members should be the EXACT same as their corresponding
  // queue name.

  // Actually useful msgs
  optional BoatState boat_state = 500;
  optional SailCmd sail_cmd = 501;
  optional RudderCmd rudder_cmd = 502;
  optional Vector3f wind = 503;
  optional HeadingCmd heading_cmd = 504;
  optional WaypointList waypoints = 505;
  optional BallastCmd ballast_cmd = 506;
...
```

The name of each entry is the name of the queue, matching exactly.

### Using the IPC

The current main interface for the IPC is the `ProtoQueue` class, which takes as
a template parameters the protobuf type of the message. The constructor takes
the queue name and whether we are a publisher or subscriber. The send and
receive functions can then be used to *send* and *receive* messages. There are
still a couple utilities defined by the Node class that make it so that you
don't have to directly call the receive functions most of the time.

The `Queue` class is a lower-level utility class which just sends raw data back
and forth. This is basically just used by the `ProtoQueue` class and by
somethings, e.g. the logger, which don't care about the underlying structure of
the message.

In order to use the `ProtoQueue` class to send a `QueueTestMsg` on the
`test_queue` queue , you can first note the `QueueTestMsg` declaration in
`ipc/queue_test_msg.proto`:

```protobuf
syntax = "proto2";
package sailbot.msg.test;

option cc_enable_arenas = true;

// The syntax = "proto2" line is because the proto3 version of protobuf had some
// issues when I first set it up, so I force version 2
// This is related to how arenas are handled (and the option cc_enable_arenas =
// true must exist in all of the protobuf files). I discuss arenas later below.
// package x.y.z means that the protobuf will show up in namespace x::y::z when
// compiled to C++

message QueueTestMsg {
  optional float foo = 1;
}
```

Which is built in `ipc/BUILD`:
```python
load("@protobuf//:protobuf.bzl", "cc_proto_library")
cc_proto_library(
  name="queue_test_msg",
  srcs=["queue_test_msg.proto"],
  protoc="@protobuf//:protoc",
  default_runtime = "@protobuf//:protobuf",
)
```

And an entry named "test_queue" is made in `util/msg.proto`:
```protobuf
syntax = "proto2";
...
import "ipc/queue_test_msg.proto";
// Remember to add "//ipc:queue_test_msg" to the dependencies in util/BUILD
...
package sailbot.msg;
option cc_enable_arenas = true;

...
message LogEntry {
...
  optional test.QueueTestMsg test_queue = 2002;
...
}  // message LogEntry
```

We can now use the queue:

```cpp
// Protobuf files compile to a .pb.h file
// Note that the protobuf target should be added to the deps in the BUILD target
// for this C++ file.
#include "ipc/queue_test_msg.pb.h"
using namespace sailbot;

// Receiver
ProtoQueue<msg::test::QueueTestMsg> data("test_queue", /*writer=*/false);
msg::test::QueueTestMsg rcv;
data.receive(&rcv);
LOG(INFO) << "data.foo: " << data.foo();
```
And the sender:
```cpp
#include "ipc/queue_test_msg.pb.h"
using namespace sailbot;
// Sender
ProtoQueue<msg::test::QueueTestMsg> data("test_queue", /*writer=*/true);
msg::test::QueueTestMsg send;
send.set_foo(123.456);
data.send(&send);
```

### Node class IPC utilities

This section overlaps slightly with the Node section, but is relevant to mention
here.

The `Node` class includes a couple of useful utilities for working with the
queues.

The first is a way to register a function as a handler to be called whenever we
receive anything. This consists of calling `RegisterHandler`, generally in a
constructor (it may not work properly after initialization is complete).
`RegisterHandler` requires a template argument for the queue type, as well as
the queue name, but otherwise is relatively straightforward. You have to pass a
function that can take const reference to a protobuf message, which will be most
recent message passed, and which has no return value. I generally do this with a
lambda expression, e.g.:

```cpp
namespace sailbot {
class QueueExample : public Node {
 public:
  QueueExample() : Node(/*loop period=*/-1) {
    RegisterHandler<msg::test::QueueTestMsg>("test_queue",
        [](const msg::test::QueueTestMsg &msg) {
          if (msg.has_foo()) {
            LOG(INFO) << "Got a foo value of " << msg.foo();
          } else {
            // If you don't fill in a field (i.e., never set foo),
            // then protobuf may fill in some default value or not
            // fill in anything meaningful. It is generally good to
            // check whether a field was filled in and produce a warning
            // if it wasn't (unless you don't expect to get the field).
            LOG(WARNING) << "No foo provided :( default value of " << msg.foo();
          }
        });
  }
};  // class QueueExample
}  // namespace sailbot
```

The handler will be called each time that a new message is received and should
exit promptly (if you take longer to process a message than the time between
messages, then you will likely end up behind on messages---there is no other
consequence, beyond CPU/memory usage).

The second important feature is an `AllocateMessage` function which allocates
memory for the protobufs in a special, preallocated area. This means that the
protobufs do not have to dynamically allocate memory using `malloc`, which can
have unpredictable performance. This uses a protobuf feature called Arenas,
which are just blocks of preallocated memory which the Protobuf libraries can
then use. This has a few components:
- Using `proto2` instead of `proto3` is important for some reason. If I recall
  correctly, if you use `proto3`, then protobuf handles poorly the deletion and
  recreation of nested messages.
- Do not constantly be allocating new messages; when messages are deleted, that
  doesn't necessarily free up space in the Arena, because to do that, the Arena
  would need to be running its own fancy memory allocation algorithm, which
  would lead to non-deterministic performance. As such, you should only allocate
  message in the constructor/initialization of your code and then reuse things
  on each iteration rather than reallocating them.
- The `option cc_enable_arenas = true` must be present in all the `.proto` files
  (it may be possible to alter the configuration of the protobuf compiler to
  enable this by default, but I have not done it).

`AllocateMessage` just takes a template argument for the type of the message you
are constructing and returns a valid pointer.

### Testing queues

In some cases, you may not want to expose your queues outside of your process.
This is mostly useful in the case of the writing tests, where you may, instead
of running multiple processes, run each node as a separate thread. By having the
queues private to a single process, you can avoid worrying about running
multiple tests at once, or with some previous process having left the queues in
an inconsistent state.

The code in `ipc/test_queue.*` implements this. The `TestQueue` class provides
an interface identical to that of the boost message_queue, and the `Queue` class
chooses which to use based on a `testing_` flag.

These are implemented by just keeping track of queues in a large, statically
defined, `std::map`,
with each element in the map having a key of the queue name and contents
corresponding to the actual data.

See the actual code of `TestQueue` for comments on how it works.

### IPC Discussion

The elephant in the room with respect to all of this IPC is, of course, why not
simply use an existing solution such as ROS's messaging structure? Part of the
answer is because I wanted to try out setting it up myself when I first did it.
Part of it is momentum---switching to ROS would be time consuming and require a
fair bit of restructuring of a lot of the code, as well as require using CMake
and other inconvenient build systems. For ROS specifically, you can also see
some of the criticisms of ROS on the [ROS 2
Wiki](http://design.ros2.org/articles/why_ros2.html) (ROS 2 is still unstable
and would be dubious for use in a real system).

It should be noted that using protobufs is an easily defensible decision, as
protobufs are widely used, well supported, and have a good feature set.

The second possibility for a messaging system would have been to use something
like [ZeroMQ](http://zeromq.org/). I have no good reason not to have done this
(and still used something like protobufs for serialization); it would've been an
existing platform, well supported, and had more features (e.g., allow us to
communicate between machines easily).

## Clock Utilities

At its most basic level, a set of clock utilities might consist of some wrappers
for `usleep` and `gettimeofday` or the such. We involve a bit more complication
because:
- We want to use monotonic clocks
- We want real-time behaviors
- Processes must be able to shut down cleanly on `SIGINT`
- Simulations should be able to run at faster (or slower) than wall-clock time

### Files

`util/clock.h`: Clock utility header files (used primarily by `Node`)

`util/clock.cc`: Clock utility source

`util/clock_test.cc`: A single basic unit test for looping.

Note that the `util/clock.*` files are reasonably well commented, so I try to
keep to calling out the key features in this documentation.

Please do read the code comments before directly using any of these classes.

### Classes/functions

Here I just go through and provide some high-level context on all of the classes/functions present.

`monotonic_clock` provides a C++
[Clock](http://en.cppreference.com/w/cpp/concept/Clock) which has guaranteed
monotonicity---namely, that time will never go backwards, which can happen with
certain clocks that may change, e.g. a wall clock that may receive corrections
if it gets ahead of the world time. Relies on `clock_gettime` and
`clock_nanosleep` under the hood for actual timing when run normally. Has
support for spoofing the clock using a fake time, as occurs when we have to do
testing.

`ClockInstance` provides an interface for the `monotonic_clock` that clears out
some of the boilerplate and provides some of the structure required for the fake
clock.

`ClockManager` purely exists for managing the fake clock.

`Loop` is the class used by `Node` to create timed loops.

`Init()` sets up gflags, glogging, and the signal handler of Ctrl-C's

`Isshutdown()`, `CancelShutdown()`, and `RaiseShutdown()` provide utilities for
  managing the shutdown state of the system.

`SetcurrentThreadREaltimePriority()` sets the priority of the current thread.
This is primarily used for things which should be run at higher priorities
(e.g., the CAN interface and the control/sensing stacks). Currently, does two
things:
- Sets the maximum contiguous runtime of a single process to half a second to
  prevent it from blocking anything else (particularly important for a process
  with elevated privileges, to prevent it from hanging and freezing the BBB).
- Set the current thread scheduler to `SCHED_FIFO` and the appropriate priority.
  `SCHED_FIFO` is one of the two real-time schedulers (see man page for
  `sched_setscheduler`).

**Note on real-timeness**: Currently we don't do anything to lock the memory of
our processes. Even if we avoid dynamic memory allocation, this could lead to
page faults, which are bad.

### Fake Clock

I will not discuss the implementation in detail here, as it is documented in the
comments in `util/clock.h`. Basically, we maintain a condition variable to
notify threads when the time has incremented upwards, and then wait for all
the threads to go back to sleep before incrementing time again, this time to
the earliest time at which any of the sleeping threads need to wake up. This has some
tricky edge cases and also must accommodate being able to cleanly shutdown if
`RaiseShutdown()` is called.

In order to use the fake clock, you set the fake clock using
`ClockManager::SetFakeClock` and it will then run on fake time.

## Node structure

The `Node` class has already been mentioned several times, but to cover it
again: The `Node` class provides the basic class which essentially all
individual processes should inherit and use to run the main body of your
program. The `Node` class provides:
- An interface to all the clock/timing infrastructure, to allow easily running
  in a timed loop
- Utilities to ensure that the program will shutdown cleanly when asked to
- The aforementioned `RegisterHandler` function for subscribing to queues, as
  well as the `AllocateMessage` function for using preallocated storage for
  queues.

### Usage

Essentially all individual processes consist of some primary class which
inherits from `Node`, even the logger and UI server, which tend to do a lot of
relatively customized things, use `Node`.

A good, simple example of a `Node` is in the `Ping` class in `util/ping.cc`:

```cpp
#include "util/node.h" // For the class Node
#include "util/msg.pb.h" // For the PongMsg and PingMsg protobufs
#include "ipc/queue.hpp" // For the ProtoQueue sender
#include "glog/logging.h" // To do logging/printing

// All sailbot classes are in namespace sailbot
namespace sailbot {

class Ping : public Node {
 public:
  Ping()
      : Node(0.001), queue_("ping", true), msg_(AllocateMessage<msg::PingMsg>()) {
    RegisterHandler<msg::PongMsg>(
        "pong", [](const msg::PongMsg &msg) { LOG(INFO) << msg.b(); });
  }

 private:
  void Iterate() override {
    msg_->set_a(msg_->a() + 0.001);
    queue_.send(msg_);
  }

  ProtoQueue<msg::PingMsg> queue_;
  msg::PingMsg *msg_;
};

}  // namespace sailbot

int main(int argc, char *argv[]) {
  // All programs should call Init()
  sailbot::util::Init(argc, argv);
  // Create and run our Ping node, using the Run function inherited  from Node
  sailbot::Ping ping;
  ping.Run();
}
```

There are really only two things that *must* be done by the classes that you
write which inherit from `Node`:

- Call the `Node::Node` constructor with a period of the loop. If negative,
  then assumes infinite period and `Iterate()` will never be called. If zero,
  then never sleeps.
- Setup an `Iterate()` function (may be empty) to be called every timestep (as
  set by the period in the constructor).

### `RegisterHandler` Implementation

There are two portions to setting up the handlers for the queue:
1. Copy the queue name into a buffer that will persist after `RegisterHandler`
   exits
2. Launch a new thread with `RunHandlerCaller` which actually sets up the
   `ProtoQueue` to receive a message and just constantly waits on `receive`s,
   calls the callback, and shuts down if something has changed.

### Arena usage

I discuss how the Arenas work some in the IPC section. The `Node` class creates
a protobuf Arena with some semi-arbitrary settings (namely, giving it 10000
bytes of memory to work with), and then sets the `max_block_size` to zero to
prevent future blocks from being allocated (because if they are allocated, that
will cause a call to `malloc`, which is not real-time).

## Logging

We do logging so that we can completely replay and understand the state of the
system following a given run. We do not always perfectly capture every aspect of
the system, but ideally we log enough that we can rerun the code using the
logged messages and debug it that way.

As is noted in the IPC section, each log message is of the `LogEntry` proto
type, and includes a timestamp as metadata as well as a single additional field
corresponding with the actual message itself.

In order to perform the logging, we, in essence, write the raw serialized
`LogEntry` to a file, and then to read the logs, read it back and use the
protobuf reflection API to figure out which field is filled in and send the raw
message out on that buffer.

### Files

`util/logging.*`: Source and header for main logging and log reading operations

`util/logger_main.cc`: Simply contains `main()` for producing logger binary

`util/log-replay.*`: Log replay source and header information

`util/log-replay-main.cc`: Example of running log-replay, printing out and
displaying the data we most commonly care about on the boat.

`util/log_deprecation.*`: Code for handling the conversions between old and new
protobuf formats---currently, just exists for when we switched for single to
double precision floating point on GPS lat/lon.

`util/log_reader_main.cc`: Nothing useful

### Log File Format

The format for the log files is relatively basic:
1. 8 bytes at the start of the file, little-endian, the integer number of
   nanoseconds since the start of the Epoch.
2. For each message, 2 (little-endian) bytes which are the length of the
   following message, in bytes, and then the actual result of serializing
   the protobuf. There are no special terminating characters or the such.
   The length bytes do not count themselves, just the length of the message
   itself.

Note that there is no sort of error checking or ability to recover from errors
should the log file be corrupted or any individual write operation go wrong,
although the parts of a log file leading up to the corruption will be fine.

### Log File Writer Implementation

The log file writing is implemented in the `Logger` class in `util/logger.*`.
The general operation is as follows:

1. Maintain a file writer (in this case a `std::ofstream`) for the whole close,
   with a mutex to acccess it.
2. On initialization:
   1. Open the file, record the initial timestamp
   2. Set up custom handlers for each and every queue defined in the `LogEntry`
      protobuf.
3. In the handler, create a reader `Queue` (this is the lower-level
   that just sends back and forth raw bytes, without doing processing
   with protobufs). Wait on the `Queue::receive`s and each time we
   receive a new message, take out the lock on the file descriptor and write the
   length of the received message and then the message itself.
4. On every `Iterate` call (current once a second), flush the file descriptor to
   ensure that things are getting written to disk (if we not do so, we might
   more easily lose data on a hard reboot).

This is the first place that we have which makes use of a command line flags
with `gflags`. The `-logfilename` flag is defined at the top of the file with
`DEFINE_string(logfilename, "/tmp/logfilename", "The name of the file to log to")`
and used when creating the `std::ofstream` by accessing the `FLAGS_logfilename`
variable. Part of the `scripts/startup.sh` script which initializes the code
chooses an appropriate filename for the log descriptor using the current Unix
time (seconds since epoch) and passes it to the log file main.

### Log File Reader and Replay

An example of basic reading of the log file can be found in the `ReadFile`
function in `util/logger.cc`. However, this is not typically used for anything
practical. The far more common usage point is in the `LogReplay` class found in
`util/log-replay.*`.

The log replay must accomplish the following:
- Actually read the raw data from the file
- Parse it to figure out which queue to send it out on
- Send the raw data on the queue at the appropriate (simulated) time
- Account for differences between old/new log files
- Allow ignoring/renaming individual queues so that we can do things like
  logsim, where we might publish sensor data unaltered to see how the controller
  responds to real data, but we want to be able to separate the old/new control
  commands, so we might rename, e.g., the `sail_cmd` queue to `orig_sail_cmd`
  (which would have to be defined in the `LogEntry` proto itself).

Reading from the file is simply accomplished with an `std::ifstream`.

To determine which queue to send on, we must parse the message and do a bit of
work with reflection.
I have thus far only mentioned it in passing, and I believe that the only place
any reflection has been used was briefly in the `ProtoQueue` to provide some
convenience by extracting the user-relevant message from the `LogEntry` protobuf
and packaging appropriately when sending. More detail on reflection can be found
below.

To send out the message at the appropriate time, we take the timestamp as read
from parsing the particular logged message. Since we are using a fake clock in
replaying (we actually set the fake clock in `LogReplay::Init`), we don't have
to worry about the fact that the logged times are different from the current
true time. We then do a sleep until that timestamp comes to send out the
message. During initialization, we startup the `ClockManager` with the start
time listed in the first 8 bytes of the log file.

Differences between old/new logfiles are handled by introducing a method to
allow providing handlers which will preprocess any Queue before being sent out.
Currently this is used on the `boat_state` message to look at the position
fields and, if the proper position field isn't filed out but one of the old ones
is, it copies the data over. Other handlers may be added when or if other
protobuf definitions change, but this is an example of why we don't like
changing protobufs---it makes it hard to process old log files, even if all we
want to do is plot the data (because the fields that we want to plot may have
new names).

For renaming, we allow the user to pass in a map of queues to rename, with the
old names as keys and the new names as the data. If we have been asked to rename
a queue, we then retrieve the `FieldDescriptor`s for both the old name and new
name portions within the `LogEntry` proto and then use the reflection for the
`LogEntry` to copy the data between fields.

#### Diversion on Reflection

Reflection is the ability to understand (and possibly alter) the nature of a
data structure at runtime. In our case, we use the [Protobuf
Reflection](https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.message#Reflection)
to identify which message field is filled in and send things out on that queue.

In our case, we retrieve a list of all the fields in the protobuf, ensure that
it is only of length two (i.e., has a time portion and a data portion), and then
go through both fields and retrieve the necessary data (in our case, the time
data and the name of the non-time field).

```cpp
LogEntry entry;
...
std::vector<const google::protobuf::FieldDescriptor*> fields;
entry.GetReflection()->ListFields(entry, &fields);
CHECK_LE(fields.size(), 2) << "We only support log entries with a timestamp + ONE message";
for (const auto field : fields) {
  if (field->lowercase_name() == "time") {
    ...
  } else {
    queue_name = field->lowercase_name();
  }
}
```

Broadly speaking (and more documentation can be found on the Protobuf website or
by looking at examples in our code), the reflection relies on a few concepts:
- The `FieldDescriptor`, which is the thing that we use for accessing and
  getting information about a field.
- The `Reflection` class from `Message::GetReflection`, which allows us to
  access the actual data of a protobuf (e.g., getting a list of the fields, or
  getting the data in a field using `GetReflection()->GetMessage(Message,
  FieldDescriptor)`).
- The `Descriptor` class, which allows us to get data about the type of message
  (rather than the individual instance of the message), e.g. it allows us to
  access individual fields by name by retrieving the `FieldDescriptor` with a
  call to `Message::GetDescriptor()->FindFieldByName(name)`.

By accessing these, we can do things like find out fields that are filled in,
find out what type a field is, locate fields given a string as a name, etc, even
if we don't know all that information at compile-time.

## WebSocket Server

In order to communicate with off-boat processes (namely, the UI and the
remote joystick controller), we need something that functions over the
WiFi/Ethernet network, rather than the CAN bus or the on-board RAM of
the BBB.

The code for this server is in `ui/server.*`, the HTML/Javascript
that interfaces with it is in the various `ui/*.js`, and the
python Gamepad controller using this is in our
[GamePad library](https://github.com/wpisailbot/F310_Gamepad_Parser/blob/master/parser_main.py).

The method presented here was created in a somewhat ad-hoc manner to be easy
to use when creating a Javascript/HTML UI. It is _not_ intended to be a remotely
real-time thing nor is it particularly efficient (it uses JSON encoding, which
is optimized more for ease of human use/debugging than performance).

Now, to actually describe what we use:
We use a third-party WebSocket library,
[uWebSockets](https://github.com/uNetworking/uWebSockets). WebSockets, in
general, are a protocol that functions over regular sockets and happens
to have support in JavaScript. I don't really know what the details are the
protocol are, but it does appear to do things like support encryption and
what-not. Over this WebSocket connection, we transmit JSON back and forth. Each
message is a JSON object where queue names are keys and the contents of the
messages are the message itself (such that it can be parsed by the protobuf
libraries). When sending requests to the server running on the Beaglebone,
if the value of a field is null, then you are requesting that the server
send you data on the message. The server does not proactively send messages,
it only responds to a client message being sent.

On-board the Beaglebone, the uWebSocket library is used to run a server which
serves requests on some specified port (we've been using 13000, but that is
arbitrary). This allows us to spawn a thread for each message we receive.
We then parse the JSON, using the example client message
`{"foo":null,"bar.baz":null,"foobar":{"foo":1,"bar":2}}`, which is attempting to
retrieve the `foo` message, the `baz` field of the `bar` message, and send
a new message on the `foobar` queue with the `foo` and `bar` fields set
to 1 and 2 respectively:
1. Manually identifying the comma-separated fields of the message, in
   the example breaking it into `"foo":null`, `"bar.baz":null`, and
   `"foobar":{"foo":1,"bar":2}`.
2. Identify which queue this corresponds to and what the value is:
   1. If value is null, finish parsing the name down as many fields
      deep as it goes, and retrieve the most recent value from that queue.
      This applies to the `foo` and `bar` requests.
   2. If not null, the name should be just the name of a queue (you can't
      send just the sub-field of a message-it is somewhat
      meaningless to do so). Parse this using the protobuf
      `JsonStringToMessage` function. In this example, we have the
      `foobar` queue to send the `{"foo":1,"bar":2}` message on.
3. Construct a reply that consists, at a minimum, of a `time` message
   which just is a double of the current time (based on the monotonic clock,
   may not be system time). This is sent even if the client wasn't requesting
   the values of any messages. We also, of course, include the messages
   the client requested.
   For this example, let us say that the current value of the `foo` queue
   is consists of `{"a":1,"b":0}` and the current value of `bar.baz` is
   `0.5`, and the current reply is `123.456`, so the reply will be:
   `{"time":123.456,"foo":{"a":1,"b":0},"bar.baz":0.5}`.

### Discussion

This setup has a variety of issues:

- It is not highly performant. If you look at the CPU usage on the BBB,
  the server consistently has the highest CPU usage. The fact that it has higher
  than average CPU usage is not particularly surprising, but there are almost
  certainly performance issues. We have not done a thorough analysis to know
  which parts to focus on optimizing, but something almost certainly can be.
- The uWebSocket library depends on OpenSSL. This is, in itself, entirely
  reasonable. However, we do not need OpenSSL, and OpenSSL takes a long time
  to build, thus bloating our code-base and build process.
- The uWebSocket library has no mechanism by which to shut-down. As such,
  when attempting to cleanly shut-down when, say, running a series of
  test suites, it has some probability of seg-faulting. This only
  seems to occur if there are open connections at the time of the shut-down.

The primary convenience of the setup is that it makes it quite easy to, for instance,
write a quick python program to control the robot.

## Testing Infrastructure

There are two broad classes of testing which we attempt to perform using the
build system itself:
- Unit testing, to test individual functions
- Simulation tests, where we spin up as much of the stack as we can easily
  test/simulate. This is more for debugging/testing than performing automated
  tests.

For all the testing, we use the googletest libraries, which provides some useful
utilities for the unit testing (for simulations, it is less valuable, but we
still use it).

For unit testing, see e.g. `math/polygon_test.cc` for a relatively simple
example. The build target for this is a `cc_test` target specified in
`math/BUILD` and can be run by doing `bazel test //math:polygon_test`.

We add a bit more complication to support the simulation testing so that
we can minimize code duplication. The main shared code here is in
`util/testing.*` where we create a wrapper class for testing which:
- Sets up the fake clock
- Turns on in-process queues to avoid interacting with outside world
- Handles shutdown state

This is then used in `control/control_test.cc`, which is the main simulation
test. This test works by using the googletest Test Fixture system (the thing
which allows creating a class that is created/removed on each test to share code
and setup between tests). The test constructs and starts nodes for each of the
system components, including the simulator which spoofs the sensors and system
dynamics. By also starting up the UI server, we can then debug the system by
looking at the UI on `localhost` or the such.

### Running simulations

In order to run the simulations, you follow these steps:
1. _optional_ In order to view the simulation live, you need to setup a local webserver
   to view things. Follow step 6 of the 
   [BBB Setup](https://github.com/wpisailbot/boat/blob/master/DOCS.md#bbb-setup)
   but on your personal laptop and point `/home/debian/bin/sailbot/html` at the
   `ui/` folder in this repository (e.g., `/home/james/sailbot/ui`).
2. To run a simulation of a particular scenario, do e.g.
   `bazel build -c opt //control:control_test && ./bazel-bin/control/control_test --gtest_filter=*Navigation*`.
   Note that once it starts running, you can use the UI webpage in chrome to set new waypoints
   or the such. If you edit the `control/control_test.cc` you can change how long the
   test runs for, what its default waypoints are, etc. Look for the
   `TEST_F(SimpleControlTest, NavigationChallenge) {` for the sample Navigation challenge points
   (this is what the `gtest_filter` argument is doing; making you just run the tests with
   `Navigation` in the name).
3. Running the previous command creates a CSV file with data from the simulation run at 
   `sim/python/basic_sim_data.csv`; you can then `cd sim/python` and run `./plot_basic_sim.py`
   to view a plot of the output CSV data.

# Boat-Specific Infrastructure

Much of the previously discussed software infrastructure is relevant to many
types of robots. We now discuss some of the software we have for interacting
with boat-specific utilities:

- NMEA 2000, the CAN protocol used (this section is the longest)
- The rigid wing, which essentially just requires opening a socket
- Controlling the code using a joystick or other external method
- UART-based RC control using a Hobby-style RC controller
- Interacting with the custom motor controllers, referred to in 2016-2017 as
  SCAMPs

## NMEA 2000

NMEA 2000 is a CAN protocol which is used in marine applications. It is based
heavily on the J1939 standard for automotive applications.
We use NMEA 2000 and CAN because:
- The Airmar 220WX uses NMEA 2000
- CAN itself is an appropriate choice for on-boat communications (more than
  enough bandwidth for our purposes, robust, widely used)
- NMEA 2000 is not complicated enough to make it obnoxious to use

The code for interacting with CAN on the robot can be found in the `can/`
directory.

### Basic CAN Information

The CAN bus, as you likely know, is a two-wire differential bus which tends to
be reasonably robust and is commonly used in robotic and automotive
applications. Many processors (in our case, both the BBB and the Jetson TX2)
on-chip CAN interfaces which just require adding a transceiver chip to connect
to the actual CAN bus.

The CAN protocol itself consists of frames sent over the bus. Each frame
consists of a CAN ID and up to 8 bytes of data, as well as a few other sundry
bits and protocol details (e.g., error-checking bits, bit-stuffing). There
exists a basic and extended frame format, where the main distinction is the
length of the ID (11 or 29 bits). We use the extended format for NMEA 2000. If
multiple devices attempt to transmit at the same time on the bus, then priority
is decided by whoever first transmits a recessive bit when the other transmits a
dominant bit. This means that IDs with smaller first bits will be higher
priority. This is taken advantage of in the NMEA 2000 message structure.

On the beaglebone, we take advantage of the SocketCAN libraries to provide
software access to the CAN bus, whereby we access the bus in a similar manner to
that of a regular network socket. The CAN device itself is enabled by performing
the steps in the BBB Setup section of this document (namely, adding it to the
device tree, then enabling appropriate kernel modules, and then bringing up the
network interface with the appropriate baud rate).

To actually access the CAN bus in code, we treat it very similarly to a normal
socket, albeit with a few slightly different structures, e.g. to allow us to
pass CAN IDs back and forth, or to specify whether we want to use standard or
extended frames.

The socket setup is not particularly novel, and can be found in the `CanNode` object
constructor in `can/can.cc`, where more work goes into setting the generic
socket options to create a 1-second `read` timeout than the actual CAN setup (we
set a timeout so that if we ask the code to shutdown, we can guarantee that the
`read` calls will always complete within 1 second and so make shutdown time at
most 1 second).

When sending/receiving messages, we use the `read`/`write` system calls, sending
and receiving `can_frame` structs. The `can_frame` struct (from the SocketCAN
library) consists of:
```c
struct can_frame {
  canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  __u8    __pad;   /* padding */
  __u8    __res0;  /* reserved / padding */
  __u8    __res1;  /* reserved / padding */
  __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};
```

The `can_id` field is the 29 bits of the CAN ID itself; the other bits are EFF
(whether or not we use the Extended Frame Format, RTR (Remote Transmission
Request, which we never use), and ERR (presumably something to do with errors,
not sure though). The 29 CAN ID bits are the lowest order bits, and the EFF bit
is the highest order bit (on the BBB, the EFF bit is accessible by doing
`0x80000000 & can_id`).
The `can_dlc` is the number of data bytes, and `data` is the data.

### NMEA 2000 Details

NMEA 2000 specifies a few important details (namely, 250000 bits / second on the
CAN bus and Extended Frame Format), and the rest of the protocol consists of
specifying the exact format of the CAN ID field and the data formats. It also
consists of a library of PGNs and what the format of each is. It also specifies
some interactions, e.g. how to assign device IDs when multiple devices are on
the bus. We ignore these issues.

All of the information presented here is based a large part on the work
of the [CAN Boat](https://github.com/canboat/canboat) project,
which attempts to reverse engineer the NMEA 2000 standard; in particular, while
information on the construction of the CAN ID for NMEA 2000 can be found on the
Internet with relative ease, information on the actual message formats and
construction is harder to come across (as it is part of the proprietary
standard) and so we must rely on our own experience and the CAN Boat project's
reverse engineering.

PGNs are a key component of the standard. The PGN is an identifier specifying
the data format of a message, and is sent as part of the CAN ID. For instance,
the "Rate of Turn" message has a PGN of 127251, which says that the data shall
be 5 bytes long, with 1 byte that is a counter (to distinguish between duplicate
messages) and 4 bytes that will be the rotation rate, where 4-byte rotation
rates are a specific type in the NMEA 2000 standard and so has a particular
resolution and data format. We use the definitions from the CAN Boat project
and use a subset of their code in `can/canboat-pgn.h`, which contains a list of
PGN definitions.

But first, the details of how
the CAN ID is packed: The NMEA 2000 standard specifies that the CAN ID will be
packed as follows, starting from the highest order (first transmitted, highest
magnitude when setting the 32-bit int in C++) bits:

| Bits           | Field   |
| -------------- | ------- |
| 29,28,27       | Priority (lower # = higher priority) |
| 26             | Reserved |
| 25             | Highest bit of PGN |
| 24-17 (1 byte) | Higher order byte of PGN |
| 16-9  (1 byte) | If previous byte is <240, destination address, otherwise lower order byte of PGN |
| 8-1 (1 byte) | Source Address |

See, e.g., `can/can_test.cc` for an example of converting from the
PGN + Address(es) + Priority to CAN ID and back.

In our setup, we have only worked with things without destination addresses and
largely ignored issues of priority and source addresses. As such, there may
be errors in the code or misinformation in this document regarding the exact
handling of those issues.

For message which consist of multiple packets, the CAN ID does not change; the
information for such a message is encoded in the data fields. When receiving a
multi-packet message, the only way to know that it is multi-packet is if the PGN
specification says so. If it is multi-packet, then the protocol is as follows:
- The first byte of every packet is an index; the highest 3 bits (mask `0xE0`)
  of the byte identify which message it is a part of, the lower 5 (mask `0x1F`)
  identify the index within the message. This means a multi-packet message can
  consist of at most 32 packets, corresponding to (7 * 32 - 1 = 223) bytes.
- The second byte of the first packet in each message will be the length in
  bytes of
  the message overall. I do not know if this length includes the length and
  index
  bytes themselves.

We do not currently support sending multi-packet messages; as I mentioned,
we also do not pay attention to the content of the length bytes, and we
do not support repeated fields within messages.

Finally, when it comes to the actual content of the data fields, things
become a bit more tricky, and at this point reading some of the code
in `can/can.cc` may become necessary for proper understanding:
1. It appears that if all the bits within a field are set to 1 then
   that field should be interpreted as unset and ignored. I am not
   sure if this is part of the NMEA 2000 standard or simply a convention.
2. Bytes are stored in little-endian format, which is convenient for the
   BBB, because, for multi-byte byte-aligned fields, we can simply use
   a `memcpy` to copy fields over.
3. Fields which are not byte aligned (i.e., they start or end in the middle
   of a byte somewhere), the exact ordering issues are dealt with in
   `CanNode::ExtractNumberField`. We currently do not support sending
   non-byte aligned fields.

The exact lengths of each field are defined in the PGN specification,
for which we use the code from CAN Boat as mentioned before in
`can/canboat-pgn.h`. The code should be somewhat self-explanatory,
but here is a commented example:

```cpp
// Each entry in the array is of type Pgn, for which the commented
// definition is available in can/canboat-pgn.h
// First, the name of the message (can be anything), then the PGN
// number itself, then an irrelevant boolean, then the length in bytes,
// then whether there are any repeating fields in the message.
{ "Wind Data", 130306, true, 6, 0,
    // Each field is of type Field and consists of a name,
    // a length in bits (BYTES(N) = 8 * N), a "resolution",
    // whether the field is signed/unsigned, the units of the field
    // and a longer description.
    // The SID is a common field which is used as an index to identify
    // out-of-order or skipped messages.
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Angle", BYTES(2), RES_RADIANS, false, "rad", "" }
  , { "Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { 0 }
  }
}
```

These definitions include one relevant issue not mentioned: The resolution
of each field. For all fields, the resolution is either a positive
number, in which case you simply multiply the integer value of the field
by the resolution to get the number in the correct units. For
special types of field, the resolution will be a negative integer
which has some special meaning depending on the field. e.g., dates
have a special format. In the wind example, the reference
field refers to whether the wind is being measured relative to the boat,
absolutely, whether it uses true or magnetic north, etc. Special
fields require different logic to handle them in the parsing code.

### Queue Interface

When we are parsing the message, we simultaneously fill in the
corresponding protobuf message, using definitions from `can/can.proto`.
For each PGN that we want set out over the logs, a queue is defined in
the `LogEntry` protobuf with a name defined as `can12345` where `12345`
is the PGN (this specific format is so that the names can easily be
parsed; an astute observer will note that this shouldn't strictly
be necessary, but it is how the code is written); equally if not
more importantly, the id number within the protobuf is the PGN. Each
queue is of type `CANMaster` and, similar to how the `LogEntry` proto
itself is used, the `CANMaster` proto contains a single message
for each relevant PGN and at any given time only one should be filled in.
Within the `CANMaster` proto, the name of each message is irrelevant,
but the numbers must correspond with the PGNs. The `CANMaster` protobuf
also includes an `outgoing` field. This is used to signal whether
a given message is something that came in over the CAN bus and
is being sent out by the CAN node or whether the message is being sent out
by one of our processes and the CAN node should then send it out.

For each message, the definition corresponds closely with the definition
of the PGN in `can/canboat-pgn.h`, e.g. for the wind one:

```cpp
message WindData {
  optional uint32 SID = 1;
  optional float wind_speed = 2; // m/s
  optional float wind_angle = 3; // radians
  enum WIND_REFERENCE {
    TRUE_NORTH_REF = 0;
    MAGNETIC_NORTH_REF = 1;
    APPARENT = 2;
    TRUE_BOAT_REF = 3;
    TRUE_WATER_REF = 4;
  }
  optional WIND_REFERENCE reference = 4;
}
```

Note how the numbers for each field correspond to their order in
the actual NMEA2000 messages. If you do not want to have a particular
field of a message parsed, you can simply skip
that number when creating the protobuf definition (e.g., the `SystemTime`
definition does this). For non-numeric fields which have a resolution
of `RES_LOOKUP`, we use enumerations where the numbers within the enumeration
correspond with the actual numbers sent on the bus for that message.

### Implementation

The bulk of our implementation occurs in `can/can.*`, with some other
files in the `can/` directory being relevant.

## Rigid Wing

The code that runs on the rigid wing is available at
https://github.com/wpisailbot/rigid_wing and should probably be rewritten.
The code for talking to the rigid wing is available in `rigid_wing/`.

For talking between the rigid wing and the boat, we use regular sockets, with
the specific definition that the messages _to_ the boat and _from_ the wing
shall consist of 13 bytes in the format:
`[123 456 789]`, where the spaces and brackets are somewhat irrelevant but
should be there, and the three three-digit numbers must always be three
digits long, shall be parsed as base-10, and correspond with:
1. The current angle of attack of the wing, in degrees
2. The current servo position (range 0-100)
3. The current battery voltage (range 0-999)

For messages _from_ the boat and _to_ the wing, we use: `[0 123 45 678]`,
where we have:
1. The wing state to set (see `rigid_wing/rigid_wing.proto`)
2. The current heel angle (degrees)
3. The maximum desired heel angle, for auto-trimming (degrees, 0-90)
4. The desired servo position (0-100)

The main issues with this setup are that:
1. It uses a custom protocol; we would prefer to avoid having too many
   custom protocols to maintain on the boat.
2. The connections can be flaky and high-latency at times. This may
   be a TCP/UDP problem, or an isue with the WiFi chip being used on
   the rigid wing, or a code issue with how the WiFi chip is used
   by the teensy.

### Teensy Code

On the teensy side, all the code is in a single file (see
https://github.com/wpisailbot/rigid_wing, rigid_teensy.ino),
and essentially consists of initializing the teensy and then running
in loops trying to make everything run.

Surprisingly, the teensy is actually able to reconnect to the boat
after losing a connection or never gaining one in the first place.

### Boat Code

The on-boat code is relatively limited, with most of the complexity
in the actual initialization of the socket (which is mostly boilerplate). The
only subtlety is doing the same as we do on any other I/O related processes:
setting up the socket so that it is only blocking for some period of time;
that way, the `read`s are always guaranteed to return in finite time and so we
can shutdown the code cleanly by simply checking if we need to shutdown
whenever a read times out.

## External Joystick Control

In order to provide a way to remote-control the boat, we use a Logitech
joystick hooked up to a laptop, which then uses the WebSocket interface
to send commands to the boat.

The WebSocket UI has already been described, and on the laptop side
all that this control really requires is that we have some way
of reading the joystick inputs and then sending a json message over
the websocket.
Thus far, we have used the code in
https://github.com/wpisailbot/F310_Gamepad_Parser which is forked from
https://github.com/JohnLZeller/F310_Gamepad_Parser. There is nothing
particularly novel here; the joysticks are mapped to the sail/rudder/ballast
and buttons are used for changing modes and setting the rigid wing and the such.

There is marginally more complexity on the boat side, but that is purely
to allow us to easily switch between different modes. In order to make it so
that we can switch between who gets to control different actuators, we
define a `control_mode` queue which consists of a `ControlMode` message
defined in `control/actuator_cmd.proto`:

```protobuf
message ControlMode {
  enum MODE {
    // Direct control from Hobby RC receiever, ignoring safety limits
    MANUAL_RC = 0;
    // Allow autonomous controller to run, by listening to *_cmd
    AUTO = 1;
    // Run based on manual control over the network, listening to *_manual_cmd queues
    MANUAL_WIFI = 2;
    // Disable the actuator
    DISABLE = 3;
    // Run from a Hobby RC receiver, but with safety limits
    FILTERED_RC = 4;
  }
  enum TACKER {
    // No tacker: just try to drive straight to waypoints
    NONE = 0;
    // Line Tacker: See control/line_tacking.*
    LINE = 1;
    // Heuristic based tacker, also defined in control/line_tacking.*
    REWARD = 2;
    // No tacker, do not send out heading commands (allows for RC of heading)
    DISABLED = 3;
  }
  optional MODE winch_mode = 1;
  optional TACKER tacker = 2;
  optional MODE rudder_mode = 3;
  optional MODE rigid_mode = 4;
  optional MODE ballast_mode = 5;
}
```

The tacker mode and actuator modes are each handled slightly differently, mostly
because when originally done, all the tackers were defined in a single source file
whereas the actuators where being controlled from a variety of sources.

The tacker mode is handled in the tacking code by any tackers that are running
at the time. The tacker code itself takes on the obligation of not sending
out heading commands; if anyone does not fulfill this obligation, then multiple
commands may be sent out over the `heading_cmd` queue, confusing the controller.

For the actuator modes, the switching is handled by the destination code, i.e.
whatever code actually processes the commands. For everything but the
rigid wing, this is `control/scamp.*` and for the rigid wing it is
`rigid_wing/rigid_wing.*`. For these, separate queues are defined where
relevant (there is a separated RC, manual, and autonomous queue for
each). This has the advantage that there is only one process that
must handle the switching, but requires defining more queues.

Once we do all this, the joystick command is handled no differently
then the regular autonomous command, except that it kicks in for a different
mode.

## RC Control

For RC control, we are referring to the use of a traditional RC
transmitter/receiver pair which will typically have longer range than WiFi. This
is no longer particularly relevant now that we have the longer range Ethernet
bridges for communications, but it was the simplest solution originally.

On the boat, this consisted of a RC receiver hooked such that it provided an
SBUS command to the boat (for our particular hookup, this involved a separate
board that parsed the PWM signals and output SBUS, but some RC receivers will
output SBUS by default). SBUS is simply a UART protocol that just required
inverting the signal (switching HIGH/LOW) to be readable by the Beaglebone
itself. The code to parse the SBUS signal is defined in `sensor/read-sbus*`,
and outputs an array of channel values.

Once this happens, the `control/scamp.*` code scales the values into
an appropriate range for the motor controllers and passes the commands through
(if we are in one of the RC modes).

Using this code requires that you ensure that a UART port is open on the
Beaglebone (the code specifically expects UART4, but that is readily changed).

I am not sure if the UART ports on the current versions of the beaglebone
get enabled by default on boot or if you will need to make an appropriate
entry in the `/boot/uEnv.txt` file, but no more setup than that should
be necessary, as the baud rate and the such are all set by the C++ code.

## Motor Controllers

The code that runs on-board the newest (Spring 2018) motor controllers is available
at https://github.com/wpisailbot/stm32_motor_controller

The code that ran on the SCAMPs in 2017 is at
https://github.com/wpisailbot/SCAMP

On the boat side of things, interfacing with both of these controllers is done
via custom-defined NMEA2000 messages for which we use PGNs of the form `0xFF--`.
This choice of PGN is purely arbitrary in that it did not seem to overlap with
any existing PGNs. To facilitate ease of use, we define the messages
as consisting of byte-aligned fields and keep individual messages to at most
8 bytes so that they can fit inside a single packet.

There is not much complexity on the boat side when it comes to actually handling
these interactions, as the actual parsing is handled by the CAN code and the
higher level logic mostly consists of the code in `control/scamp.cc` deciding
which numbers to pass through to the actual CAN messages based on the mode
code described above. As such, I will just go through the three
messages that we have defined at this writing (May 7, 2018) for interacting
with the motor controllers:

```cpp
// As defined in can/canboat-pgn.h:

// Analog Read is a holdover from when we had an analog pot
// on the SCAMP in 2017 to measure winch angle. It is meant
// to be the feedback from the winch, and should provide a
// 1-byte message representing the winch position in some way.
// The current field should just provide the current running
// through the motor controller, in whatever units are desirable
{ "Analog Read", 0xFF01, true, 1, 0,
  { { "Val", BYTES(1), 4, false, 0, "" }
  , { "Current", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

// The name of this is also a holdover.
// This message provides voltage commands for each of the
// three actuators, using values from 0-180 (fitting with typical
// arduino Servo commands). 90 should signify zero volts, with 0 and
// 180 being the two extremes.
,
{ "PWM Write", 0xFF02, true, 3, 0,
  { { "Winch", BYTES(1), 1, false, 0, "" } // Range 0-180
  , { "Rudder", BYTES(1), 1, false, 0, "" } // Range 0-180
  , { "Ballast", BYTES(1), 1, false, 0, "" } // Range 0-180
  , { 0 }
  }
}
,

// Provide the current heel angle and ballast arm angle as
// measured by the ballast sensors.
{ "Ballast State", 0xFF05, true, 4, 0,
  { { "Heel", BYTES(2), RES_RADIANS, true, "radians", "Heel angle of boat"}
  , { "Ballast Arm", BYTES(2), RES_RADIANS, true, "radians", "Angle of ballast arm" }
  , { 0 }
  }
}
```

## Status Monitor and On-Boat Display

As of 2018, we have an [On-Boat
Display](https://github.com/wpisailbot/OnBoatDisplay) which can display certain
information. The interactions with this are largely similar to how we interact
with the motor controllers (albeit with slightly different information being
sent back and forth).

In order to provide some useful information to show on the display, we need to
keep track of some general status information to use.

# Controls Software

Simulation, controllers, "state estimation"/zeroing/etc

## Controls Notes
- Airmar heading data is unreliable--should use combination of gps velocity +
  heading (e.g., weighted average that weights gps more as speed increases)
  I currently hypothesize that this has to do with excessive heel angles--when
  the Airmar is known to stop providing _any_ attitude data past +/-50 degrees.
- For turning on the previous boat, it turned out to be critical to add some
  code that lets out the sail when turning downwind and sheets in when turning
  upwind.
- Movable ballast control seems to be cleanest if you have two PID loops: One
  (lower-level) that takes a goal ballast position and drives the arm to that
  angle; and a second (higher-level) that takes a goal heading and chooses a
  ballast angle based on the heading error.

Overall, the entire control system could be much more model-based.

## State Estimation

Currently, we have very little actual work done on the state estimation. It
essentially consists of taking the raw Airmar data and transforming some of the
angles and the such to be in a convenient coordinate system (e.g., the provided
winds are provided in compass headings and refer to the direction the wind is
_coming_ from; I've preferred to provide a velocity vector for the wind,
oriented in the direction the wind is blowing _to_, with an (East, North)
coordinate system instead of a (North, West) coordinate system).

Thus far, this system has only caused issues when Airmar data has been
inaccurate. However, there is some potential gain to be had:
- For movable ballast related things in particular, a more model-based
  estimation system (presumably a Kalman Filter or EKF) would allow for better
  dynamic estimation, e.g., of the current heel angle, and would allow us to be
  more tolerant of noise in the Airmar (and the real world).

### Airmar Heading Inaccuracy

In the endurance race at Sailbot 2018, I started to observer some serious
issues with the heading from the Airmar, which was tending to be >30 degrees
off of our true heading and so causing us to go substantially of course. When
running courses near the shore, this caused us to veer off into the shore
without being able to correct for it.

The obvious solution is to simply use GPS velocity whenever possible; this would
also increase our tolerance to genuine issues such as leeway.
Also, a more model based estimation approach might allow a reduced reliance on
the Airmar.

I am not entirely sure of the source of the error in the Airmar, but I do
believe that it has always been there to some extent; previously I had just discounted it by
assuming that it was due to leeway issues or if I was checking it shortly after
starting the Airmar, that it was due to compass calibrations not having kicked
in.

Based on a cursory look at the data, the error has the following traits:
- Pretty consistently results in heading being off from GPS velocity by 30-50
  deg
- Sometimes, though, it is fine.
- Which direction it is off by seems to correlate with the current tack of the
  boat; it is not always off in the same direction.
- There is not an immediately obvious correlation with, e.g., extreme heel and
  the issue (it occurs even at relatively normal heel angles).
- Under dynamic situations (i.e., while turning), the Airmar tends to behave
  poorly. Sometimes it lags, sometimes it over-responds, sometimes it jumps
  around (and not necessarily to the right place).

There are a few potential sources of this error; they could well be compounding
on each other in some way:
- Magnetic Declination; magnetic North != true North, but at most can account
  for 15 deg of the issue and doesn't explain why we don't see it 100% of the
  time or why we sometimes have seen the issue occur in different directions.
- Poor Airmar mounting; if the Airmar is not mounted straight, it could cause
  some issue, but it'd be the same type of issue as magnetic vs. true North
- Missing calibration--perhaps there was a calibration procedure that we failed
  to follow
- Bad Airmar filtering algorithms--the Airmar is a black box of filtering and
  may do an atrocious job of filtering the data, to the point where it may
  be less useful than just using a cheap, unfiltered sensor. It may, e.g., do a
  bad job of accounting for rocking of the boat when calculating heading, or
  poorly handle even slight heel angles.
- Actual leeway--the boat doesn't go straight; there is some leeway. However,
  this leeway should not be anywhere near even 30, let alone 50, degrees.
- Hardware damage to the Airmar--I don't believe this is the case, but if some
  water damage managed to get something off without breaking it in the Airmar,
  then that may be a source of some issues.

## Controller

In Fall of 2017, I worked on a more model-based adaptive controller for the sail
and rudder. Unfortunately, because we never had any chance to do any testing on
the real boat and I wanted to be conservative in taking risks at the
competition, the controller has not been tested outside of a single run in
October or November of 2017 and simulation. I believe it should work (it doesn't
yet have ballast, but that'd actually be a relatively easy addition), and it
should account for things like weather-helm better than the hand-selected
algorithms in simple.cc. The implementation can be found in
`control/adaptive.*`.

Currently, the basic method of operation (see my RBE594 final paper for some
more discussion) is to numerically solve (in a very coarse fashion) for the
optimal sail/rudder positions to minimize a cost function where we are trying to
maximize forwards force and achieve some overall yaw torque/acceleration.
To add ballast control, I would probably add ballast arm angle as an
optimization variable, and work to figure out some semi-analytical solution so
that you don't have to numerically solve a multi-DOF system in the controller.

The adaptiveness is also somewhat naively written; I would encourage
investigating alternative methods for providing adaptive behavior. On-the-water
testing may be necessary.

## Planner

For RBE594, I also worked on a new planner for generating goal headings. This
actually did get used, because the existing planner was awful.

This is in `control/line_plan.*`

The main work on the planner is to try to tune it to improve stability.
Currently there are a couple of stability issues:
- When near a buoy (so it only has to plan a short path), the gradient descent
  tends to produce significantly different solutions iteration to iteration. I'm
  not entirely sure why, but assume it has to do with the overall cost values
  being small relative to the cost gradient, and so the variations in how well
  the gradient descent happened to work are greater than the variations in the
  actual cost, causing different solutions to be better each time.
- When the goal point is in an obstacle, similar issues occur, where the
  gradient descent becomes unstable due to the high cost gradients.
- Sometimes, possible due to noisy wind readings or brief gusts, the generated
  path will change for a single iteration. I don't know if this is an issue
  with the planner itself or with the filtering on the wind data (or the
  filtering on any other data), but it should be looked at).


Note that currently, in order to create obstacles for the shore, you should make
them massive (and they have to be convex). If you don't make them deep enough,
then sometimes the planner ends up finding a path that involves going around the
obstacle you created for the shore, or it discovers that it can produce a
reasonably low cost path by cutting through the shore in some manner.

There're a bunch of tuning parameters for the planner in `control/line_plan.h`.
Be sure you know what they mean before changing them too much.

As a note on gradient descent-based optimization algorithms, you do have to
ensure that any cost functions you add are smooth and won't result in
discontinuities or even unusually steep gradients.

One sest of parameters that may be interesting to change are kMaxTotalpts` and
`kMaxLegs`; the first is the maximum number of turning points that we will
attempt to use in our optimization, while the maximum legs are the maximum
number of buoys we will look forwards (which must be at least one). Increasing
these will very rapidly increase the run-time of the overall algorithm (either
exponential or combinatorial; too lazy to think about which it is), so when you
make changes, be sure that you try them out on a BBB to see what sort of
performance implications they have.
