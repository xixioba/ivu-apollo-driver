load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

#innovusion_config_proto
py_proto_library(
    name = "innovusion_config_py_proto",
    deps = [
        ":innovusion_config_proto",
    ],
)

cc_proto_library(
    name = "innovusion_config_cc_proto",
    deps = [
        ":innovusion_config_proto",
    ],
)

proto_library(
    name = "innovusion_config_proto",
    srcs = [
        "innovusion_config.proto",
    ],
)

#innovusion_imu_proto
py_proto_library(
    name = "innovusion_imu_py_pb2",
    deps = [
        ":innovusion_imu_proto",
        "//modules/common/proto:header_py_pb2",
    ],
)

cc_proto_library(
    name = "innovusion_imu_cc_proto",
    deps = [
        ":innovusion_imu_proto",
    ],
)

proto_library(
    name = "innovusion_imu_proto",
    srcs = ["innovusion_imu.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

#innovusion_proto
py_proto_library(
    name = "innovusion_py_pb2",
    deps = [
        ":innovusion_proto",
        "//modules/common/proto:header_py_pb2",
    ],
)

cc_proto_library(
    name = "innovusion_cc_proto",
    deps = [
        ":innovusion_proto",
    ],
)

proto_library(
    name = "innovusion_proto",
    srcs = ["innovusion.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)
