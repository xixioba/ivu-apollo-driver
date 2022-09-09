load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

#jaguar driver
cc_binary(
    name = "libinnovusion_jaguar.so",
    srcs = [
        "driver_jaguar.cc",
        # "sdk/lib/libinnolidar.so",
        "sdk/lib/static/libinnolidar.a",
    ],
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
    name = "library_jaguar",
    srcs = glob([
        "sdk/lib/*.so*",
    ]),
)

install(
    name = "install",
    data = [
        ":library_jaguar",
    ],
    data_dest = "lib/modules/drivers/lidar/innovusion/driver/jaguar",
    data_strip_prefix = ["sdk/lib"],
)

cpplint()
