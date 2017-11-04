[![Build Status](https://travis-ci.org/wpisailbot/boat.svg?branch=master)](https://travis-ci.org/wpisailbot/boat)

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

In order to compile for the BBB, use `bazel build --cpu=bbb //some/target:here`.

To deploy code, do `bazel run --cpu=bbb -c opt //scripts:deploy -- 192.192.X.X`
replacing the IP address as appropriate. The `-c opt` builds the code
with sundry optimization options enabled. If you want the code to be restarted
after being deployed (by default the code on the boat will keep running until
manually killed or the BBB is rebooted), than add any extra argument after the
IP address, e.g. `bazel run --cpu=bbb -c opt //scripts:deploy -- 192.168.0.21 r`.

## BBB Setup

1) Create BBB with debian jessie running on it
2) `apt-get install unzip g++-4.7 libatomic-ops-dev`
3) Create symlink between sailbot html files and /var/www/sailbot/

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
- Take a closer look at Boost's implementation of queues

## Notes
- We are using proto2, not proto3, because I (James) observed some oddities
  where ParseFromArray would Clear the message and force re-allocation of
  internal sub-messages. Still not 100% sure why proto2 is working better than 3.

## Interfacing With the BeagleBone

(Note: I haven't double checked this section--it may have typos. Let me
 know if there are any issues)

In order to send/receive queue information from the beaglebone, use a
websocket connection and send JSON information over it. This is
what is used in the main web UI.

Let us say that you wanted to create a program on the beaglebone
which received a message, multiplied a number by some other number,
and returned the message.

First, we must specify the messages. For this, we specify the message
types. For convenience, we will do this in `util/msg.proto`, but in
general the messages probably belong elsewhere.

To do this, add:

```protobuf
  message MultiplyRequest {
    optional float a = 1;
    optional float b = 2;
  }

  // Note: We could get away with just using a straight float
  // rather than defining a whole separate message for the response,
  // but this way it is more extensible if you want to add information
  // to the response later.
  message MultiplyResponse {
    optional float ab = 1;
  }
```

We then must add the messages somewhere in the LogEntry proto so
that the queue system is aware of the message. To do this,
add some lines near the end of the LogEntry proto:

```protobuf
  optional MultiplyRequest mult_req = 4000;
  optional MultiplyResponse mult_resp = 4001;
```

We then define our actual C++ program, which I will not explain
for the time being, but would look something like:
```cpp
#include "util/node.h"
#include "util/msg.pb.h"

namespace sailbot {

class Multiplier : public Node {
 public:
  Multiplier()
      : Node(-1), queue_("mult_resp", true), msg_(AllocateMessage<msg::MultiplyResponse>()) {
    RegisterHandler<msg::MultiplyRequest>(
        "mult_req", [](const msg::MultiplyRequest &msg) {
            msg_->set_ab(msg.a() * msg.b());
            queue_.send(msg_);
        });
  }

 protected:
  virtual void Iterate() {}

 private:
  ProtoQueue<msg::MultiplyResponse> queue_;
  msg::MultiplyResponse *msg_;
};

}  // sailbot

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::Multiplier node;
  node.Run();
}
```

And in order to build it, we must add the following to the BUILD file in whatever directory
it is in:
```python
cc_binary(
  name="mult",
  srcs=["mult.cc"],
  deps = [
    "//util:node",
    "//util:msg_proto",
  ],
)
```

And if we would like it to start up with all the rest of the code on the beaglebone,
`//path/to:mult` must get added to the `zip_outs` `srcs` list in `scripts/BUILD`
and the binary must be given a line in `scripts/deploy.sh`.

Now that we have done this, let us say we want to write a python program on some
network machine to request multiplied numbers from the beaglebone.

The format of the messages is that the client will send a message as a single JSON
object. Each key corresponds to a queue name (in this case, `mult_req`). For queues
where you wish to *send* messages, the client should fill in the value with the
JSON corresponding to the protobuf message structure. For queues for which the
client is requesting the latest value on the queue, the client should fill in
`null` on the message value. The response from the server will be in the same format,
with the exceptions that it will not be requesting anything (i.e., no `null`s) and
it will add an additional `time` queue to all replies.

Furthermore, the client may request the values of subfields of messages (e.g.,
`mult_resp.ab`) and the server will respond appropriately. You may not send
just the subfields of messages.

In this case, a single request from the client would look like:

```json
{"mult_req":{"a":2,"b":4},"mult_res.ab":null}
```

And the response from the server would be:

```json
{"time":123123123.123123123,"mult_res.ab":8}
```

The python to do this would be:

```python
from websocket import create_connection
ws = create_connection("ws://192.168.0.21:13000")
ws.send('{"mult_req":{"a":2,"b":4},"mult_resp":null}')
print ws.recv()
```

Note: The server may respond before the node itself is able to send off
its results, so in reality you would need to wait a moment before
requesting the results from the server.
