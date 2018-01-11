# This is the entry point for --crosstool_top.  Toolchains are found
# by lopping off the name of --crosstool_top and searching for
# 'cc-compiler-${CPU}' in this BUILD file, where CPU is the target CPU
# specified in --cpu.
#
# This file group should include
#   * all cc_toolchain targets supported
#   * all file groups that said cc_toolchain might refer to,
# including the default_grte_top setting in the CROSSTOOL
# protobuf.
#filegroup(
#    name = "toolchain_fg",
#    srcs = [
#        ":cc-compiler-bbb",
#        ":cc-compiler-rpi",
#        ":cc-compiler-k8",
#        ":linaro_linux_all_gnueabihf_files",
#        ":linaro_linux_all_gnueabi_files",
#        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:compiler_components",
#        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabi//:compiler_components",
#    ],
#)

#filegroup(
#    name = "srcs",
#    srcs = glob(["**"]) + [
#        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:srcs",
#    ],
#    visibility = ["//:__pkg__"],
#)

cc_toolchain_suite(
    name = "toolchain",
    # target_cpu | compiler
    toolchains = {
        "bbb|gcc": "cc-compiler-bbb",
        "rpi|gcc": "cc-compiler-rpi",
        "k8|compiler": "cc-compiler-k8",
    },
)

filegroup(
    name = "linaro_linux_all_gnueabihf_files",
    srcs = [
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:tool-wrappers",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_linker_gnueabihf_files",
    srcs = [
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:ar",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:gcc",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:ld",
        "@org_linaro_components_toolchain_gcc_5_3_1_gnueabihf//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_compiler_gnueabihf_files",
    srcs = [
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:as",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:gcc",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:ld",
    ],
)

filegroup(
    name = "linaro_linux_all_gnueabi_files",
    srcs = [
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:tool-wrappers",
        "@rpi_gcc_5_4_1_gnueabihf//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_linker_gnueabi_files",
    srcs = [
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:ar",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:gcc",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:ld",
        "@rpi_gcc_5_4_1_gnueabihf//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_compiler_gnueabi_files",
    srcs = [
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:as",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:gcc",
        "//tools/arm_compiler/linaro_linux_gcc/gnueabi:ld",
    ],
)

filegroup(
    name = "empty",
    srcs = [],
)

cc_toolchain(
    name = "cc-compiler-bbb",
    all_files = ":linaro_linux_all_gnueabihf_files",
    compiler_files = ":linaro_linux_compiler_gnueabihf_files",
    cpu = "bbb",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":linaro_linux_linker_gnueabihf_files",
    objcopy_files = "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:objcopy",
    static_runtime_libs = [":empty"],
    strip_files = "//tools/arm_compiler/linaro_linux_gcc/gnueabihf:strip",
    supports_param_files = 1,
    visibility = ["//visibility:public"],
)

cc_toolchain(
    name = "cc-compiler-rpi",
    all_files = ":linaro_linux_all_gnueabi_files",
    compiler_files = ":linaro_linux_compiler_gnueabi_files",
    cpu = "rpi",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":linaro_linux_linker_gnueabi_files",
    objcopy_files = "//tools/arm_compiler/linaro_linux_gcc/gnueabi:objcopy",
    static_runtime_libs = [":empty"],
    strip_files = "//tools/arm_compiler/linaro_linux_gcc/gnueabi:strip",
    supports_param_files = 1,
    visibility = ["//visibility:public"],
)

cc_toolchain(
    name = "cc-compiler-k8",
    all_files = ":empty",
    compiler_files = ":empty",
    cpu = "k8",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":empty",
    objcopy_files = ":empty",
    static_runtime_libs = [":empty"],
    strip_files = ":empty",
    supports_param_files = 1,
    visibility = ["//visibility:public"],
)
