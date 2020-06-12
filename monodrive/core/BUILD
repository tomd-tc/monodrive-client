package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monodrive_client",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["src/*.h", "src/*.hpp"]),
    includes = ["/usr/include/boost", "/usr/lib/x86_64-linux-gnu", "src"],
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu",
        "-L/usr/lib/x86_64-linux-gnu/",
        "-lboost_system",
    ],
)
