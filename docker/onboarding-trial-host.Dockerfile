# Disposable Ubuntu Docker host for maintainer onboarding measurements.
#
# This image contains observer prerequisites only.  Project images, datasets,
# and outputs belong to a fresh bind-mounted /var/lib/docker and /trial for
# each trial.  Run it only with the isolation procedure documented in
# docs/onboarding-trial-execution.md; never mount the host Docker socket.
ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ARG UBUNTU_VERSION
ARG OBSERVER_RECIPE_SHA256=unknown

LABEL io.github.rsasaki0109.lidarslam.observer-contract="1" \
      io.github.rsasaki0109.lidarslam.observer-ubuntu="${UBUNTU_VERSION}" \
      io.github.rsasaki0109.lidarslam.observer-recipe-sha256="${OBSERVER_RECIPE_SHA256}"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ca-certificates \
    docker.io \
    iproute2 \
  && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["dockerd"]
CMD ["--host=unix:///var/run/docker.sock", "--data-root=/var/lib/docker", "--storage-driver=overlay2"]
