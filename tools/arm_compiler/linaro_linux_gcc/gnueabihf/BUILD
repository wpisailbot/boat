package(default_visibility = ["//tools/arm_compiler:__pkg__"])

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
    visibility = ["//tools/arm_compiler:__pkg__"],
)

filegroup(
    name = "gcc",
    srcs = [
        "arm-linux-gnueabihf-gcc",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:gcc",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "arm-linux-gnueabihf-ar",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "arm-linux-gnueabihf-ld",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "arm-linux-gnueabihf-nm",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "arm-linux-gnueabihf-objcopy",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "arm-linux-gnueabihf-objdump",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "arm-linux-gnueabihf-strip",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "arm-linux-gnueabihf-as",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:as",
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
