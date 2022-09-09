load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_import", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

#innovusion driver all in one
cc_binary(
    name = "adapter_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        "lib_falcon",
        "lib_jaguar",
        ":adapter_component",
    ],
)

cc_import(
    name = "lib_falcon",
    shared_library = "//modules/drivers/lidar/innovusion/driver/falcon:libinnovusion_falcon.so",
)

cc_import(
    name = "lib_jaguar",
    shared_library = "//modules/drivers/lidar/innovusion/driver/jaguar:libinnovusion_jaguar.so",
)

cc_library(
    name = "adapter_component",
    srcs = [
        "adapter_component.cc",
    ],
    hdrs = [
        "adapter_component.h",
        "driver_factory.h",
        "httplib.h",
        "//modules/drivers/lidar/innovusion/driver/falcon:driver_falcon.h",
        "//modules/drivers/lidar/innovusion/driver/jaguar:driver_jaguar.h",
    ],
    includes = [
        "./falcon/sdk",
        "./falcon/sdk/apps",
        "./falcon/sdk/src",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/innovusion/proto:innovusion_cc_proto",
        "//modules/drivers/lidar/innovusion/proto:innovusion_imu_cc_proto",
        "//modules/drivers/lidar/innovusion/proto:innovusion_config_cc_proto",
        "@com_github_nlohmann_json//:json",
    ],
)

#uint test
cc_test(
    name = "adapter_component_test",
    size = "small",
    srcs = [
        "adapter_component_test.cc",
        "//modules/drivers/lidar/innovusion/driver/falcon:libinnovusion_falcon.so",
        "//modules/drivers/lidar/innovusion/driver/jaguar:libinnovusion_jaguar.so",
    ],
    deps = [
        ":adapter_component",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

#install rule
install(
    name = "install",
    targets = [
        ":adapter_component.so",
    ],
    deps = [
        "//modules/drivers/lidar/innovusion/driver/falcon:install",
    ],
)

cpplint()
