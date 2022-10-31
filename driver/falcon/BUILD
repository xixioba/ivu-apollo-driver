load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# falcon driver
cc_binary(
    name = "libinnovusion_falcon.so",
    srcs = [
        "driver_falcon.cc",
        "sdk/src/sdk_common/converter/cframe_converter.cpp",
    ] + select({
        "@platforms//cpu:x86_64": [
            "sdk/lib/linux-x86/libinnolidarsdk.so",
            "sdk/lib/linux-x86/libinnolidarsdkclient.so",
            "sdk/lib/linux-x86/libinnolidarsdkcommon.so",
            "sdk/lib/linux-x86/libinnolidarutils.so",
        ],
        "@platforms//cpu:aarch64": [
            "sdk/lib/linux-arm/libinnolidarsdk.so",
            "sdk/lib/linux-arm/libinnolidarsdkclient.so",
            "sdk/lib/linux-arm/libinnolidarsdkcommon.so",
            "sdk/lib/linux-arm/libinnolidarutils.so",
        ],
    }),
    linkopts = [
        "-fvisibility=hidden",
        # "-Wl,--exclude-libs,ALL",
        # "-Wl,-Bsymbolic",
        "-Wl,-rpath,$$ORIGIN",  # for release run
    ] + select({
        "@platforms//cpu:x86_64": [
            "-Wl,-rpath,/apollo/modules/drivers/lidar/innovusion/driver/falcon/sdk/lib/linux-x86",  # for build run
        ],
        "@platforms//cpu:aarch64": [
            "-Wl,-rpath,/apollo/modules/drivers/lidar/innovusion/driver/falcon/sdk/lib/linux-arm",  # for build run
        ],
    }),
    linkshared = True,
    linkstatic = False,
    deps = [
        ":lib_falcon",
    ],
)

cc_library(
    name = "lib_falcon",
    hdrs = [
        "driver_falcon.h",
        "//modules/drivers/lidar/innovusion/driver:driver_factory.h",
        "//modules/drivers/lidar/innovusion/driver:httplib.h",
    ],
    includes = [
        "sdk",
        "sdk/apps",
        "sdk/src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/innovusion/proto:innovusion_config_cc_proto",
    ],
)

# install falcon libs
filegroup(
    name = "library_falcon_x86",
    srcs = glob([
        "sdk/lib/linux-x86/*.so*",
    ]),
)

filegroup(
    name = "library_falcon_arm",
    srcs = glob([
        "sdk/lib/linux-arm/*.so*",
    ]),
)

install(
    name = "install",
    data = select({
        "@platforms//cpu:x86_64": [
            ":library_falcon_x86",
        ],
        "@platforms//cpu:aarch64": [
            ":library_falcon_arm",
        ],
    }),
    data_dest = "lib/modules/drivers/lidar/innovusion/driver",
    data_strip_prefix = select({
        "@platforms//cpu:x86_64": [
            "sdk/lib/linux-x86",
        ],
        "@platforms//cpu:aarch64": [
            "sdk/lib/linux-arm",
        ],
    }),
)

cpplint()
