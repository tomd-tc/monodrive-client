# docker image for FPS benchmark build
FROM ubuntu:18.04 as build

# setup
ENV BASE /home/benchmark
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR ${BASE}

# setup pre-reqs
RUN apt-get update && \
    apt-get install -y \
    git \
    build-essential \
    gcc \
    cmake \
    libboost-all-dev

# build
COPY benchmark.cpp ${BASE}/benchmark.cpp
COPY CMakeLists.txt ${BASE}/CMakeLists.txt
WORKDIR ${BASE}/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j 4

# runtime image for FPS benchmark
FROM ubuntu:18.04

# setup
ENV BASE /home/benchmark
ARG DEBIAN_FRONTEND=noninteractive

# setup pre-reqs
RUN apt-get update && \
    apt-get install -y \
    libboost-all-dev \    
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# copy over built libraries
COPY --from=build /home/benchmark/build/benchmark ${BASE}/benchmark
COPY --from=build \
    /home/benchmark/build/monodrive/src/monodrive-external-build/libmonodrive.so \
    ${BASE}/libmonodrive.so

# link
RUN ldconfig

# add configs
COPY config ${BASE}/config

# runtime
WORKDIR ${BASE}
ENV LD_LIBRARY_PATH ${BASE}:${LD_LIBRARY_PATH}
CMD ./benchmark --help
