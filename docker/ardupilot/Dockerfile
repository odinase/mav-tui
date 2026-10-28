FROM ubuntu:24.04 AS build
ARG ARDUPILOT_VERSION=Copter-4.7.2
ARG BUILD_JOBS=4
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git build-essential python3-dev python3-venv pkg-config \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH
RUN pip install --no-cache-dir empy==3.3.4 pexpect future lxml pymavlink packaging setuptools
RUN git clone --branch ${ARDUPILOT_VERSION} --depth 1 --recurse-submodules \
    --shallow-submodules https://github.com/ArduPilot/ardupilot.git /src
WORKDIR /src
RUN ./waf configure --board sitl && ./waf copter -j${BUILD_JOBS}

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 libgomp1 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/build/sitl/bin/arducopter /usr/local/bin/arducopter
COPY --from=build /src/Tools/autotest/default_params/copter.parm /opt/sitl/copter.parm
COPY docker/ardupilot/defaults.parm /opt/sitl/defaults.parm
COPY --chmod=755 docker/ardupilot/entrypoint.sh /usr/local/bin/start-sitl
WORKDIR /state
ENTRYPOINT ["start-sitl"]
