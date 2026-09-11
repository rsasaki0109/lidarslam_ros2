# Additive opt-in Phase 3d r3 candidate.  The build context must contain a
# host-verified `prefetch/` directory.  This recipe has no remote archive
# instruction, package
# manager, or network operation; the planner invokes Docker with --network
# none --pull=false.  The active r2 recipe and receipts are untouched.
FROM ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f AS build

ARG JOBS=2
ARG BOOST_ARCHIVE_SHA256=6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e
ARG GLIM_ARCHIVE_SHA256=d8176e85199a2297269d34fcfb57e4cb2c2d53e593439f665f614cae57972c25
ARG GTSAM_ARCHIVE_SHA256=50bd99ddbb363f03f145d814995df234c83ad38f867080fba5b60f6c151b348a
ARG GTSAM_POINTS_ARCHIVE_SHA256=dc977a8b2a6aeda5d48107920e2cdd7e0e45e1478c3590e45f053335eb975479
ARG GLIM_COMMIT=faa264a1bce1bda406f73457e35511f56cdc2eaa
ARG GTSAM_COMMIT=2f3e56c0ddbd3a1aa54ed043643b553d26a069f6
ARG GTSAM_POINTS_COMMIT=9d32e7dbecf6015560d84b4901d6b0a6f483ec46

ENV CMAKE_BUILD_PARALLEL_LEVEL=2 \
    MAKEFLAGS=-j2 \
    BOOST_PREFIX=/opt/install/boost

# The prefetch verifier is copied into the image so the same allowlist and
# byte checks run immediately before any build command.  The current manifest
# is NOT_READY because the base-image apt/deb closure is not sealed; this
# deliberate failure is the promotion gate, not a hidden network fallback.
COPY phase3d/r3/offline_dependency_manifest.json /opt/offline_dependency_manifest.json
COPY phase3d/r3/dependency_prefetch.py /opt/dependency_prefetch.py
COPY prefetch/ /opt/prefetch/
RUN set -eux; \
    test "${JOBS}" = 2; \
    python3 /opt/dependency_prefetch.py verify \
      --root /opt/prefetch --manifest /opt/offline_dependency_manifest.json; \
    find /opt/prefetch -maxdepth 1 -type f -name '*.deb' -print0 | sort -z | \
      xargs -0 -r dpkg --unpack; \
    dpkg --configure -a; \
    command -v cmake; command -v c++; command -v make; command -v python3; \
    test -f /usr/include/eigen3/Eigen/Core; \
    test -f /usr/include/spdlog/spdlog.h

COPY phase1/boost_tree_hash.py /opt/boost_tree_hash.py
RUN set -eux; \
    printf '%s  %s\n' \
      "${BOOST_ARCHIVE_SHA256}" /opt/prefetch/boost_1_83_0.tar.bz2 | sha256sum -c -; \
    mkdir -p /opt/boost-src; \
    tar --extract --no-same-owner --no-same-permissions --no-same-devices \
      --strip-components=1 -f /opt/prefetch/boost_1_83_0.tar.bz2 -C /opt/boost-src; \
    python3 /opt/boost_tree_hash.py /opt/boost-src \
      --expected feb033ff2277bff1befa57ca65ede54a2c7a1fd4f6498d238daae8470dd2fbb0 \
      --output /opt/boost-tree.json; \
    printf '%s  %s\n' \
      c9bff75738922193e67fa726fa225535870d2aa1059f91452c411736284ad566 \
      /opt/boost-src/LICENSE_1_0.txt | sha256sum -c -; \
    cd /opt/boost-src; \
    ./bootstrap.sh --prefix="${BOOST_PREFIX}" \
      --with-libraries=serialization,graph,filesystem,thread,program_options,date_time,timer,chrono,regex,system; \
    ./b2 -j"${JOBS}" --build-dir=/opt/build/boost \
      --prefix="${BOOST_PREFIX}" --layout=system \
      --with-serialization --with-graph --with-filesystem --with-thread \
      --with-program_options --with-date_time --with-timer --with-chrono \
      --with-regex --with-system variant=release cxxstd=17 threading=multi \
      link=shared runtime-link=shared cxxflags=-O3 install

RUN set -eux; \
    printf '%s  %s\n' "${GLIM_ARCHIVE_SHA256}" \
      /opt/prefetch/glim-faa264a1bce1bda406f73457e35511f56cdc2eaa.tar.gz | sha256sum -c -; \
    printf '%s  %s\n' "${GTSAM_ARCHIVE_SHA256}" \
      /opt/prefetch/gtsam-2f3e56c0ddbd3a1aa54ed043643b553d26a069f6.tar.gz | sha256sum -c -; \
    printf '%s  %s\n' "${GTSAM_POINTS_ARCHIVE_SHA256}" \
      /opt/prefetch/gtsam_points-9d32e7dbecf6015560d84b4901d6b0a6f483ec46.tar.gz | sha256sum -c -; \
    mkdir -p /opt/glim-src /opt/gtsam-src /opt/gtsam-points-src; \
    tar --extract --no-same-owner --no-same-permissions --no-same-devices \
      --strip-components=1 -f \
      /opt/prefetch/glim-faa264a1bce1bda406f73457e35511f56cdc2eaa.tar.gz \
      -C /opt/glim-src; \
    tar --extract --no-same-owner --no-same-permissions --no-same-devices \
      --strip-components=1 -f \
      /opt/prefetch/gtsam-2f3e56c0ddbd3a1aa54ed043643b553d26a069f6.tar.gz \
      -C /opt/gtsam-src; \
    tar --extract --no-same-owner --no-same-permissions --no-same-devices \
      --strip-components=1 -f \
      /opt/prefetch/gtsam_points-9d32e7dbecf6015560d84b4901d6b0a6f483ec46.tar.gz \
      -C /opt/gtsam-points-src; \
    test "${GLIM_COMMIT}" = faa264a1bce1bda406f73457e35511f56cdc2eaa; \
    test "${GTSAM_COMMIT}" = 2f3e56c0ddbd3a1aa54ed043643b553d26a069f6; \
    test "${GTSAM_POINTS_COMMIT}" = 9d32e7dbecf6015560d84b4901d6b0a6f483ec46; \
    printf '%s  %s\n' \
      e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4 /opt/glim-src/LICENSE \
      38af8103282a75115a94ab29bea125d1c36fe0af04b9ac5f664144d0b399b775 /opt/gtsam-src/LICENSE \
      e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4 /opt/gtsam-points-src/LICENSE \
      > /opt/source-licenses.sha256; \
    sha256sum -c /opt/source-licenses.sha256

RUN set -eux; \
    cmake -S /opt/gtsam-src -B /opt/build/gtsam -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/gtsam \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX}" -DBOOST_ROOT="${BOOST_PREFIX}" \
      -DBOOST_INCLUDEDIR="${BOOST_PREFIX}/include" -DBOOST_LIBRARYDIR="${BOOST_PREFIX}/lib" \
      -DBoost_NO_SYSTEM_PATHS=ON -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON -DBUILD_SHARED_LIBS=ON \
      -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_USE_SYSTEM_EIGEN=ON; \
    cmake --build /opt/build/gtsam --parallel 2; \
    cmake --install /opt/build/gtsam; \
    cmake -S /opt/gtsam-points-src -B /opt/build/gtsam_points -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/gtsam_points \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX};/opt/install/gtsam" \
      -DBOOST_ROOT="${BOOST_PREFIX}" -DBOOST_INCLUDEDIR="${BOOST_PREFIX}/include" \
      -DBOOST_LIBRARYDIR="${BOOST_PREFIX}/lib" -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON \
      -DBUILD_TESTS=OFF -DBUILD_TESTS_PCL=OFF -DBUILD_DEMO=OFF \
      -DBUILD_EXAMPLE=OFF -DBUILD_TOOLS=OFF -DBUILD_WITH_TBB=OFF \
      -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_MARCH_NATIVE=OFF; \
    cmake --build /opt/build/gtsam_points --parallel 2; \
    cmake --install /opt/build/gtsam_points; \
    cmake -S /opt/glim-src -B /opt/build/glim -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/glim \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX};/opt/install/gtsam;/opt/install/gtsam_points" \
      -DBOOST_ROOT="${BOOST_PREFIX}" -DBOOST_INCLUDEDIR="${BOOST_PREFIX}/include" \
      -DBOOST_LIBRARYDIR="${BOOST_PREFIX}/lib" -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON \
      -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF \
      -DBUILD_WITH_MARCH_NATIVE=OFF; \
    cmake --build /opt/build/glim --parallel 2; \
    cmake --install /opt/build/glim

COPY CMakeLists.txt /opt/clean-room/CMakeLists.txt
COPY include /opt/clean-room/include
COPY src /opt/clean-room/src
COPY phase3a /opt/clean-room/phase3a
COPY phase3b /opt/clean-room/phase3b
COPY phase3d /opt/clean-room/phase3d
RUN set -eux; \
    cmake -S /opt/clean-room/phase3d -B /opt/build/phase3d -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DGLIM_INSTALL_PREFIX=/opt/install/glim \
      -DGTSAM_INSTALL_PREFIX=/opt/install/gtsam \
      -DGTSAM_POINTS_INSTALL_PREFIX=/opt/install/gtsam_points \
      -DGLIM_JSON_INCLUDE_DIR=/opt/glim-src/thirdparty/json/include \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX};/opt/install/glim;/opt/install/gtsam_points;/opt/install/gtsam"; \
    cmake --build /opt/build/phase3d --parallel 2; \
    cmake --install /opt/build/phase3d --prefix /opt/install/phase3d

FROM ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f AS runtime
COPY phase3d/r3/offline_dependency_manifest.json /opt/offline_dependency_manifest.json
COPY phase3d/r3/dependency_prefetch.py /opt/dependency_prefetch.py
COPY prefetch/ /opt/prefetch/
RUN set -eux; \
    python3 /opt/dependency_prefetch.py verify \
      --root /opt/prefetch --manifest /opt/offline_dependency_manifest.json; \
    find /opt/prefetch -maxdepth 1 -type f -name '*.deb' -print0 | sort -z | \
      xargs -0 -r dpkg --unpack; \
    dpkg --configure -a
COPY --from=build /opt/install /opt/install
COPY phase3d/r3/entrypoint.sh /opt/phase3d-r3/entrypoint.sh
COPY phase3d/r3/collect_phase3d.py /opt/phase3d-r3/collect_phase3d.py
RUN chmod 0555 /opt/phase3d-r3/entrypoint.sh /opt/phase3d-r3/collect_phase3d.py

ENV PATH=/opt/install/phase3d/lib/glim_clean_room_phase3d:$PATH \
    LD_LIBRARY_PATH=/opt/install/boost/lib:/opt/install/glim/lib:/opt/install/gtsam_points/lib:/opt/install/gtsam/lib:/opt/install/phase3d/lib

LABEL org.opencontainers.image.title="GLIM clean-room Phase 3d r3 opt-in candidate" \
      org.opencontainers.image.version="phase3d-r3-candidate-v1" \
      org.opencontainers.image.benchmark_eligible="false" \
      org.opencontainers.image.network_policy="offline-prefetch-only"
