ARG BASE_IMAGE=ubuntu:latest
FROM ${BASE_IMAGE} AS downloader

ARG WEBOTS_VERSION=R2023b
ARG WEBOTS_PACKAGE_PREFIX=
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes --no-install-recommends \
    bzip2 \
    ca-certificates \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && wget "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots-${WEBOTS_VERSION}-x86-64${WEBOTS_PACKAGE_PREFIX}.tar.bz2" \
    && tar xjf webots-*.tar.bz2 \
    && rm webots-*.tar.bz2

FROM ${BASE_IMAGE}

ARG EREBUS_VERSION=v26.0.0
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes --no-install-recommends \
    ca-certificates \
    git \
    locales \
    python3 \
    python3-pip \
    python3-venv \
    wget \
    xvfb \
    && rm -rf /var/lib/apt/lists/* \
    && wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh \
    && chmod +x linux_runtime_dependencies.sh \
    && ./linux_runtime_dependencies.sh \
    && rm ./linux_runtime_dependencies.sh \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/local
COPY --from=downloader /webots /usr/local/webots/

ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH="/usr/local/webots:/opt/erebus/.venv/bin:${PATH}"
ENV USER=root

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN git clone --depth 1 --branch "${EREBUS_VERSION}" \
    https://github.com/robocup-junior/erebus.git /opt/erebus

RUN python3 -m venv /opt/erebus/.venv \
    && /opt/erebus/.venv/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/erebus/.venv/bin/pip install --no-cache-dir \
    numpy \
    overrides \
    requests \
    termcolor \
    pillow \
    opencv-python

# MainSupervisor writes generated target textures under /opt/erebus/game/protos.
# Keep Erebus writable when the container runs with host UID/GID.
RUN mkdir -p /opt/erebus/game/protos/textures/targets \
    && chmod -R a+rwX /opt/erebus

WORKDIR /workspace

EXPOSE 1234

CMD ["/usr/local/webots/webots", "/opt/erebus/game/worlds/world1.wbt"]
