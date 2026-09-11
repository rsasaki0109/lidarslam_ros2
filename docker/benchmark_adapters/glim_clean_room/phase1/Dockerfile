# Additive GLIM clean-room Phase 1 container candidate r2 only.
# Build with the candidate root as context and this file as -f phase1/Dockerfile.
# The OS image is digest-pinned.  Boost is built from one exact official release
# archive because the pinned ROS base does not contain the required Boost closure.
FROM ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f AS build

ARG JOBS=2
ARG BOOST_VERSION=1.83.0
ARG BOOST_ARCHIVE_SHA256=6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e
ARG BOOST_ARCHIVE_BYTES=122892751
ARG BOOST_SOURCE_TREE_SHA256=feb033ff2277bff1befa57ca65ede54a2c7a1fd4f6498d238daae8470dd2fbb0
ARG BOOST_LICENSE_SHA256=c9bff75738922193e67fa726fa225535870d2aa1059f91452c411736284ad566
ARG GLIM_COMMIT=faa264a1bce1bda406f73457e35511f56cdc2eaa
ARG GTSAM_COMMIT=2f3e56c0ddbd3a1aa54ed043643b553d26a069f6
ARG GTSAM_POINTS_COMMIT=9d32e7dbecf6015560d84b4901d6b0a6f483ec46
ARG GLIM_ARCHIVE_SHA256=d8176e85199a2297269d34fcfb57e4cb2c2d53e593439f665f614cae57972c25
ARG GTSAM_ARCHIVE_SHA256=50bd99ddbb363f03f145d814995df234c83ad38f867080fba5b60f6c151b348a
ARG GTSAM_POINTS_ARCHIVE_SHA256=dc977a8b2a6aeda5d48107920e2cdd7e0e45e1478c3590e45f053335eb975479

ENV DEBIAN_FRONTEND=noninteractive \
    CMAKE_BUILD_PARALLEL_LEVEL=2 \
    MAKEFLAGS=-j2 \
    BOOST_PREFIX=/opt/install/boost

# The base preflight intentionally checks only dependencies that remain external
# after the Boost source stage.  CMake package/compile/link probes are required;
# a header or command existing alone is not sufficient.
RUN set -eux; \
    test "${JOBS}" = 2; \
    command -v cmake; command -v c++; command -v make; command -v python3; \
    test -f /usr/include/eigen3/Eigen/Core; \
    test -f /usr/include/spdlog/spdlog.h; \
    mkdir -p /tmp/base-preflight; \
    printf '%s\n' \
      'cmake_minimum_required(VERSION 3.16)' \
      'project(base_dependency_probe LANGUAGES CXX)' \
      'find_package(Eigen3 REQUIRED)' \
      'find_package(spdlog REQUIRED)' \
      'find_package(OpenMP REQUIRED)' \
      'add_executable(base_dependency_probe probe.cpp)' \
      'target_compile_features(base_dependency_probe PRIVATE cxx_std_17)' \
      'target_link_libraries(base_dependency_probe PRIVATE Eigen3::Eigen spdlog::spdlog OpenMP::OpenMP_CXX)' \
      > /tmp/base-preflight/CMakeLists.txt; \
    printf '%s\n' \
      '#include <Eigen/Core>' \
      '#include <omp.h>' \
      '#include <spdlog/spdlog.h>' \
      'int main() { Eigen::Vector3d value(1.0, 2.0, 3.0); spdlog::info("{}", value.x()); return omp_get_max_threads() > 0 ? 0 : 1; }' \
      > /tmp/base-preflight/probe.cpp; \
    cmake -S /tmp/base-preflight -B /tmp/base-preflight-build -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release; \
    cmake --build /tmp/base-preflight-build --parallel 2

FROM build AS boost-build

COPY phase1/boost_tree_hash.py /opt/boost_tree_hash.py
ADD https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2 /tmp/boost_1_83_0.tar.bz2

RUN set -eux; \
    test "${BOOST_VERSION}" = 1.83.0; \
    test "$(stat -c '%s' /tmp/boost_1_83_0.tar.bz2)" = "${BOOST_ARCHIVE_BYTES}"; \
    echo "${BOOST_ARCHIVE_SHA256}  /tmp/boost_1_83_0.tar.bz2" | sha256sum -c -; \
    mkdir -p /opt/boost-src; \
    tar --extract --no-same-owner --no-same-permissions \
      --strip-components=1 -f /tmp/boost_1_83_0.tar.bz2 -C /opt/boost-src; \
    python3 /opt/boost_tree_hash.py /opt/boost-src \
      --expected "${BOOST_SOURCE_TREE_SHA256}" --output /opt/boost-tree.json; \
    test -f /opt/boost-src/LICENSE_1_0.txt; \
    printf '%s  %s\n' "${BOOST_LICENSE_SHA256}" /opt/boost-src/LICENSE_1_0.txt \
      > /opt/boost-source-license.sha256; \
    sha256sum -c /opt/boost-source-license.sha256; \
    cd /opt/boost-src; \
    ./bootstrap.sh --prefix="${BOOST_PREFIX}" \
      --with-libraries=serialization,graph,filesystem,thread,program_options,date_time,timer,chrono,regex,system; \
    ./b2 -j"${JOBS}" --build-dir=/opt/build/boost \
      --prefix="${BOOST_PREFIX}" --layout=system \
      --with-serialization --with-graph --with-filesystem --with-thread \
      --with-program_options --with-date_time --with-timer --with-chrono \
      --with-regex --with-system \
      variant=release cxxstd=17 threading=multi link=shared runtime-link=shared \
      cxxflags=-O3 install; \
    test -f "${BOOST_PREFIX}/include/boost/version.hpp"; \
    for library in serialization graph filesystem thread program_options date_time timer chrono regex system; do \
      test -f "${BOOST_PREFIX}/lib/libboost_${library}.so"; \
    done; \
    find "${BOOST_PREFIX}" -type f -print0 | sort -z | xargs -0 sha256sum \
      > /opt/boost-installed-files.sha256; \
    test -s /opt/boost-installed-files.sha256

ADD https://github.com/koide3/glim/archive/faa264a1bce1bda406f73457e35511f56cdc2eaa.tar.gz /tmp/glim.tar.gz
ADD https://github.com/borglab/gtsam/archive/2f3e56c0ddbd3a1aa54ed043643b553d26a069f6.tar.gz /tmp/gtsam.tar.gz
ADD https://github.com/koide3/gtsam_points/archive/9d32e7dbecf6015560d84b4901d6b0a6f483ec46.tar.gz /tmp/gtsam_points.tar.gz

RUN set -eux; \
    test "${GLIM_COMMIT}" = faa264a1bce1bda406f73457e35511f56cdc2eaa; \
    test "${GTSAM_COMMIT}" = 2f3e56c0ddbd3a1aa54ed043643b553d26a069f6; \
    test "${GTSAM_POINTS_COMMIT}" = 9d32e7dbecf6015560d84b4901d6b0a6f483ec46; \
    echo "${GLIM_ARCHIVE_SHA256}  /tmp/glim.tar.gz" | sha256sum -c -; \
    echo "${GTSAM_ARCHIVE_SHA256}  /tmp/gtsam.tar.gz" | sha256sum -c -; \
    echo "${GTSAM_POINTS_ARCHIVE_SHA256}  /tmp/gtsam_points.tar.gz" | sha256sum -c -; \
    mkdir -p /opt/glim-src /opt/gtsam-src /opt/gtsam-points-src; \
    tar --extract --no-same-owner --no-same-permissions --strip-components=1 -f /tmp/glim.tar.gz -C /opt/glim-src; \
    tar --extract --no-same-owner --no-same-permissions --strip-components=1 -f /tmp/gtsam.tar.gz -C /opt/gtsam-src; \
    tar --extract --no-same-owner --no-same-permissions --strip-components=1 -f /tmp/gtsam_points.tar.gz -C /opt/gtsam-points-src; \
    test "$(git -C /opt/glim-src rev-parse HEAD 2>/dev/null || true)" = ""; \
    test -f /opt/glim-src/LICENSE; test -f /opt/gtsam-src/LICENSE; test -f /opt/gtsam-points-src/LICENSE; \
    printf '%s  %s\n' \
      e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4 /opt/glim-src/LICENSE \
      38af8103282a75115a94ab29bea125d1c36fe0af04b9ac5f664144d0b399b775 /opt/gtsam-src/LICENSE \
      ec9746ecc24d4b36c4c8412ced73a9c7351238396d1fd17b70b3ba1e6a2e5bd0 /opt/gtsam-src/LICENSE.BSD \
      27e1fb1934109aec52982bea8cda801e3791c683690c2eb0c089c496ba394ac6 /opt/gtsam-src/gtsam/3rdparty/CCOLAMD/Doc/License.txt \
      9c98af085ddc729768ca22879825cc95c875ac05e2b6dc268211bdff254011b7 /opt/gtsam-src/gtsam/3rdparty/ceres/jet.h \
      > /opt/phase1-source-licenses.sha256; \
    printf '%s  %s\n' \
      c83230b770f17ef1386ea1fd3681271dd98aa93646bdbfb5bff3a1b7050fff9d /opt/gtsam-src/gtsam/3rdparty/Eigen/COPYING.README \
      64ab9475d7b289ad76e935adff51b31e4ac160df7dfad24480dbaa452e39bbe79 /opt/gtsam-src/gtsam/3rdparty/metis/LICENSE.txt \
      328d5075d23d78d8d70a32bb8fa6d04a5d871236614c226f510a121a14398382 /opt/gtsam-src/gtsam/3rdparty/Spectra/GenEigsSolver.h \
      e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4 /opt/gtsam-points-src/LICENSE \
      >> /opt/phase1-source-licenses.sha256; \
    sha256sum -c /opt/phase1-source-licenses.sha256

RUN set -eux; \
    cmake -S /opt/gtsam-src -B /opt/build/gtsam -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/gtsam \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX}" \
      -DBOOST_ROOT="${BOOST_PREFIX}" -DBOOST_INCLUDEDIR="${BOOST_PREFIX}/include" \
      -DBOOST_LIBRARYDIR="${BOOST_PREFIX}/lib" -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON \
      -DBUILD_SHARED_LIBS=ON -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
      -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
      -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON; \
    cmake --build /opt/build/gtsam --parallel 2; \
    cmake --install /opt/build/gtsam

RUN set -eux; \
    cmake -S /opt/gtsam-points-src -B /opt/build/gtsam_points -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/gtsam_points \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX};/opt/install/gtsam" \
      -DBOOST_ROOT="${BOOST_PREFIX}" -DBOOST_INCLUDEDIR="${BOOST_PREFIX}/include" \
      -DBOOST_LIBRARYDIR="${BOOST_PREFIX}/lib" -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON \
      -DBUILD_TESTS=OFF -DBUILD_TESTS_PCL=OFF -DBUILD_DEMO=OFF -DBUILD_EXAMPLE=OFF \
      -DBUILD_TOOLS=OFF -DBUILD_WITH_TBB=OFF -DBUILD_WITH_OPENMP=ON \
      -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_MARCH_NATIVE=OFF; \
    cmake --build /opt/build/gtsam_points --parallel 2; \
    cmake --install /opt/build/gtsam_points

RUN set -eux; \
    cmake -S /opt/glim-src -B /opt/build/glim -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/glim \
      -DCMAKE_PREFIX_PATH="${BOOST_PREFIX};/opt/install/gtsam;/opt/install/gtsam_points" \
      -DBOOST_ROOT="${BOOST_PREFIX}" -DBOOST_INCLUDEDIR="${BOOST_PREFIX}/include" \
      -DBOOST_LIBRARYDIR="${BOOST_PREFIX}/lib" -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON \
      -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF \
      -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_MARCH_NATIVE=OFF; \
    cmake --build /opt/build/glim --parallel 2; \
    cmake --install /opt/build/glim

FROM ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f AS smoke
ENV CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 BOOST_PREFIX=/opt/install/boost
COPY --from=boost-build /opt/install /opt/install
COPY --from=boost-build /opt/phase1-source-licenses.sha256 /opt/phase1-source-licenses.sha256
COPY --from=boost-build /opt/boost-source-license.sha256 /opt/boost-source-license.sha256
COPY --from=boost-build /opt/boost-tree.json /opt/boost-tree.json
COPY --from=boost-build /opt/boost-installed-files.sha256 /opt/boost-installed-files.sha256
COPY . /opt/candidate
RUN set -eux; \
    cmake -S /opt/candidate -B /opt/build/adapter -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="/opt/install/boost;/opt/install/glim;/opt/install/gtsam_points;/opt/install/gtsam" \
      -DBOOST_ROOT=/opt/install/boost -DBOOST_INCLUDEDIR=/opt/install/boost/include \
      -DBOOST_LIBRARYDIR=/opt/install/boost/lib -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON -DBoost_USE_STATIC_LIBS=OFF \
      -DBoost_USE_STATIC_RUNTIME=OFF -DBoost_USE_MULTITHREADED=ON \
      -DGLIM_BOOST_PREFIX=/opt/install/boost \
      -DGLIM_CORE_CMAKE_PREFIX=/opt/install/glim \
      -DGLIM_CORE_COMMIT=faa264a1bce1bda406f73457e35511f56cdc2eaa; \
    cmake --build /opt/build/adapter --parallel 2; \
    LD_LIBRARY_PATH=/opt/install/boost/lib:/opt/install/glim/lib:/opt/install/gtsam_points/lib:/opt/install/gtsam/lib \
      ctest --test-dir /opt/build/adapter --output-on-failure; \
    LD_LIBRARY_PATH=/opt/install/boost/lib:/opt/install/glim/lib:/opt/install/gtsam_points/lib:/opt/install/gtsam/lib \
      ctest --test-dir /opt/build/adapter -R glim_core_public_api_link_smoke --output-on-failure; \
    find /opt/install -type f -print0 | sort -z | xargs -0 sha256sum > /opt/phase1-installed-files.sha256; \
    test -s /opt/phase1-installed-files.sha256

LABEL org.opencontainers.image.title="GLIM clean-room core public API candidate" \
      org.opencontainers.image.version="phase1-v2" \
      org.opencontainers.image.benchmark_eligible="false"
