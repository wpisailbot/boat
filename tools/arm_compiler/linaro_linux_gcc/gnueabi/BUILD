package(default_visibility = ["//tools/arm_compiler:__pkg__"])

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
    visibility = ["//tools/arm_compiler:__pkg__"],
)

filegroup(
    name = "gcc",
    srcs = [
        "arm-linux-gnueabi-gcc",
        "@rpi_gcc_5_4_1_gnueabihf//:gcc",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "arm-linux-gnueabi-ar",
        "@rpi_gcc_5_4_1_gnueabihf//:ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "arm-linux-gnueabi-ld",
        "@rpi_gcc_5_4_1_gnueabihf//:ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "arm-linux-gnueabi-nm",
        "@rpi_gcc_5_4_1_gnueabihf//:nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "arm-linux-gnueabi-objcopy",
        "@rpi_gcc_5_4_1_gnueabihf//:objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "arm-linux-gnueabi-objdump",
        "@rpi_gcc_5_4_1_gnueabihf//:objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "arm-linux-gnueabi-strip",
        "@rpi_gcc_5_4_1_gnueabihf//:strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "arm-linux-gnueabi-as",
        "@rpi_gcc_5_4_1_gnueabihf//:as",
    ],
)

filegroup(
    name = "tool-wrappers",
    srcs = [
        ":ar",
        ":as",
        ":gcc",
        ":ld",
        ":nm",
        ":objcopy",
        ":objdump",
        ":strip",
    ],
)
