load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

#falcon driver
cc_binary(
    name = "libinnovusion_falcon.so",
    srcs = [
        "driver_falcon.cc",
        "sdk/src/sdk_common/converter/cframe_converter.cpp",
        "sdk/lib/libinnolidarsdk.so",
        "sdk/lib/libinnolidarsdkclient.so",
        "sdk/lib/libinnolidarsdkcommon.so",
        "sdk/lib/libinnolidarutils.so",
    ],
    linkopts = [
        "-fvisibility=hidden",
        # "-Wl,--exclude-libs,ALL",
        # "-Wl,-Bsymbolic",
        "-Wl,-rpath,$$ORIGIN",  # for release run
        "-Wl,-rpath,/apollo/modules/drivers/lidar/innovusion/driver/falcon/sdk/lib",  # for build run
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":lib_falcon",
    ],
)

exports_files(["sdk/apps/pcs/legacy_cframe.h"])

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

#install falcon libs
filegroup(
    name = "library_falcon",
    srcs = glob([
        "sdk/lib/*.so*",
    ]),
)

install(
    name = "install",
    data = [
        ":library_falcon",
    ],
    data_dest = "lib/modules/drivers/lidar/innovusion/driver",
    data_strip_prefix = ["sdk/lib"],
)

cpplint()
