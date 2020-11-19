package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monodrive",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["src/*.h", "src/*.hpp"]),
    #includes = ["/usr/lib/x86_64-linux-gnu", "src"],
    includes = ["src"],
    linkopts = [
        #"-L/usr/lib/x86_64-linux-gnu",
        #"-L/opt/apollo/sysroot/lib",
        #"-lboost_system",
    ],
    deps = [
	"@boost",
	"@opencv//:highgui",
	"@opencv//:imgcodecs",

    ]
)
