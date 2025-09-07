set shell := ["bash", "-cu"]

APT_FLAGS := "--quiet=2 --yes"
JAVACPP_VERSION := "1.5.10"
LINUX_RESOURCES_DIR := "src/main/resources/yo-variables/linux-x86_64"

CMAKE_CXX_RELEASE_FLAGS := "-O3"
CMAKE_CXX_DEPLOY_FLAGS := "-Ofast -march=native"

NTHREADS := "10"

# Default help task
default:
    just --summary

# Setup dependencies and symlinks
setup:
    @echo "NOTE: This task installs dependencies via apt, so be sure to run with sudo, e.g. 'sudo just setup'."
    apt-get {{APT_FLAGS}} update
    apt-get {{APT_FLAGS}} install build-essential clang gdb cmake catch2
    apt-get {{APT_FLAGS}} install libeigen3-dev
    ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
    ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported

# Show installed dependencies
remove:
    @echo "The installed packages via apt are:"
    @echo "  - build-essential"
    @echo "  - clang"
    @echo "  - gdb"
    @echo "  - cmake"
    @echo "  - catch2"
    @echo "  - libeigen3-dev"
    @echo "Use your own judgement in 'sudo apt remove'-ing these packages."

# Install Pinocchio, Crocoddyl, and Wrapper
install:
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS={{CMAKE_CXX_RELEASE_FLAGS}} \
        -DCMAKE_INSTALL_PREFIX="install/yo-variables"
    cmake --build build --target install

# Clear all builds and installs
clear:
    rm -rf build install/yo-variables
