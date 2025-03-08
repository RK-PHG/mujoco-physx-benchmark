 #!/usr/bin/env bash
 set -e

 echo "SimBenchmark Build"
 echo ""
 echo "==========================================================================="
 echo "Install dependencies..."
 echo ""

 # install apt packages
 echo "Install apt packages..."

 sudo add-apt-repository ppa:ubuntu-toolchain-r/test
 sudo apt update

 # sudo add-apt-repository ppa:coinor/coinor
 # sudo apt update

 sudo apt install -y -qq g++-7
 sudo apt install -y -qq \
 libeigen3-dev \
 libboost-all-dev \
 libglew-dev \
 libglm-dev \
 libsdl2-dev \
 libassimp-dev \
 libsoil-dev \
 libsdl2-ttf-dev \
 liburdfdom-dev \
 libgtest-dev \
 google-perftools \
 libgoogle-perftools-dev \
 libpng-dev \
 libccd-dev \
 libfcl-dev \
 libncurses5-dev \
 libncursesw5-dev \
 doxygen \
 coinor-libipopt-dev \
 libnlopt-dev \
 libode-dev \
 libbullet-dev \
 libflann-dev \
 libtinyxml2-dev