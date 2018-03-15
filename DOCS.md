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
  # the header files are and include them overly generously.
  includes = ["./"],
  hdrs = glob(["boost/**"]),
)
```
The comments provide some background on what each line is doing.

### Google Libraries

The libraries that work out of the box

