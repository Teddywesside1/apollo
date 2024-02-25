load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "zed",
    # srcs = glob(["lib/libsl_zeD_static.a"]),
    srcs = glob(["lib/*"]),
    hdrs = glob(["include/**/*.hpp","include/**/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"], 
    linkstatic = 1,
)