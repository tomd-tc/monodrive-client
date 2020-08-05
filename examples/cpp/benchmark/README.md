# Benchmark for FPS
Example for benchmarking frame rate of simulator


## Requirements
CMake, Git, Boost, GCC
```
apt-get update && \
    apt-get install -y \
    git \
    build-essential \
    gcc \
    cmake \
    libboost-all-dev
```


## Build
```
mdkir build
cd build
cmake ..
make -j 8
```


## Run locally
Run with full sensor suite
```
cd ..
./build/benchmark --iterations 16 --workload full
```

Other options
```
./build/benchmark --help
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
