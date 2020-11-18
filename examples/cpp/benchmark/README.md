# Benchmark for FPS

Example for benchmarking frame rate of simulator


### Setup and build

Follow instruction in the root directory of this repository.


**Note** that on Windows, these examples will be built to the `Debug` or `Release` folder
```
./build/examples/cpp/benchmark/Release/benchmark.exe
```


## Run locally

Run benchmark with full sensor suite
```
./build/examples/cpp/benchmark/benchmark --iterations 16 --sensors examples/cpp/benchmark/workload_full.json
```

Other options
```
./build/examples/cpp/benchmark/benchmark --help
```


## Build docker image

Build image
```
docker build -t monodrive-fps:0.0  .
```


## Run with docker image
Run with the same program using your built image
```
docker run -it --network host monodrive-fps:0.0 ./benchmark --iterations 16 --workload full
```
