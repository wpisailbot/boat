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
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:gcc",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "arm-linux-gnueabi-ar",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "arm-linux-gnueabi-ld",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "arm-linux-gnueabi-nm",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "arm-linux-gnueabi-objcopy",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "arm-linux-gnueabi-objdump",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "arm-linux-gnueabi-strip",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "arm-linux-gnueabi-as",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:as",
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
