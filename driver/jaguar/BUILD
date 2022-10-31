load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

#jaguar driver
cc_binary(
    name = "libinnovusion_jaguar.so",
    srcs = [
        "driver_jaguar.cc",
    ] + select({
        "@platforms//cpu:x86_64": [
            "sdk/lib/linux-x86/static/libinnolidar.a",
            # "sdk/lib/linux-x86/libinnolidar.so",
        ],
        "@platforms//cpu:aarch64": [
            "sdk/lib/linux-arm/static/libinnolidar.a",
            # "sdk/lib/linux-arm/libinnolidar.so",
        ],
    }),
    linkopts = [
        "-fvisibility=hidden",
        "-Wl,--exclude-libs,ALL",
        "-Wl,-Bsymbolic",
        # "-Wl,-rpath,/apollo/modules/drivers/lidar/innovusion/driver/jaguar/sdk/lib",  # for build
        # "-Wl,-rpath,$$ORIGIN",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":lib_jaguar",
    ],
)

cc_library(
    name = "lib_jaguar",
    hdrs = [
        "driver_jaguar.h",
        "//modules/drivers/lidar/innovusion/driver:driver_factory.h",
        "//modules/drivers/lidar/innovusion/driver:httplib.h",
    ],
    includes = [
        "sdk",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/innovusion/proto:innovusion_config_cc_proto",
    ],
)

#install jaguar libs(static lib no need anymore)
filegroup(
    name = "library_jaguar_x86",
    srcs = glob([
        "sdk/lib/linux-x86/*.so*",
    ]),
)

filegroup(
    name = "library_jaguar_arm",
    srcs = glob([
        "sdk/lib/linux-arm/*.so*",
    ]),
)

install(
    name = "install",
    data = select({
        "@platforms//cpu:x86_64": [
            ":library_jaguar_x86",
        ],
        "@platforms//cpu:aarch64": [
            ":library_jaguar_arm",
        ],
    }),
    data_dest = "lib/modules/drivers/lidar/innovusion/driver/jaguar",
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
