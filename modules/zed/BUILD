load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

# PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

# filegroup(
#     name = "camera_detection_single_stage_files",
#     srcs = glob([
#         "conf/**",
#         "dag/**",
#         "data/**",
#     ]),
# )

apollo_component(
    name = "libzed_component.so",
    srcs = [
        "zed_component.cc",
        "zed_device.cc",
    ],
    hdrs = [
        "zed_component.h",
        "zed_device.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/zed/proto:zed_component_cc_proto",
        "@zed",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@opencv//:imgproc"
    ],
)

apollo_package()

cpplint()
