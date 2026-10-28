FROM ubuntu:24.04 AS build
ARG PX4_VERSION=v1.17.0
ARG BUILD_JOBS=4
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git build-essential cmake ninja-build python3-dev python3-venv \
    libxml2-utils libssl-dev libeigen3-dev pkg-config file zip unzip \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --branch ${PX4_VERSION} --depth 1 --recurse-submodules \
    --shallow-submodules https://github.com/PX4/PX4-Autopilot.git /src
RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH
WORKDIR /src
RUN pip install --no-cache-dir -r Tools/setup/requirements.txt
# Build only. The executable and ROMFS are copied into the runtime image.
RUN make px4_sitl_default -j${BUILD_JOBS}
COPY docker/px4/rc.mavlink /src/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 libgomp1 bc \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/build/px4_sitl_default/bin/ /opt/px4/bin/
COPY --from=build /src/build/px4_sitl_default/etc/ /opt/px4/etc/
COPY --chmod=755 docker/px4/entrypoint.sh /usr/local/bin/start-sitl
ENV PATH=/opt/px4/bin:$PATH
WORKDIR /state
ENTRYPOINT ["start-sitl"]
