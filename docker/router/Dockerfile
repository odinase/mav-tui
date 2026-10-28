FROM ubuntu:24.04 AS build
ARG ROUTER_VERSION=v4
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git g++ meson ninja-build pkg-config \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --branch ${ROUTER_VERSION} --depth 1 --recurse-submodules \
    --shallow-submodules https://github.com/mavlink-router/mavlink-router.git /src
WORKDIR /src
COPY docker/router/tcp-send.patch /tmp/tcp-send.patch
RUN git apply --check /tmp/tcp-send.patch && git apply /tmp/tcp-send.patch \
    && meson setup build --prefix=/usr/local --buildtype=release -Dsystemdsystemunitdir=/tmp/systemd \
    && ninja -C build && DESTDIR=/install ninja -C build install

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /install/usr/local/bin/mavlink-routerd /usr/local/bin/mavlink-routerd
COPY docker/router/main.conf /etc/mavlink-router/main.conf
EXPOSE 5760/tcp 14560/udp 14561/udp
ENTRYPOINT ["mavlink-routerd", "-c", "/etc/mavlink-router/main.conf"]
