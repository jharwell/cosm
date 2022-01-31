#!/bin/bash
#
# Bootstraps the project, including:
#
# - Install .deb dependencies from repositories (ubuntu assumed)
# - Install python package dependencies
# - Build and install ARGoS from source

usage() {
    cat << EOF >&2
Usage: $0 --prefix [/usr/local|$HOME/<dir>] [--rroot <dir>] [--cores <n_cores>] [--nosyspkgs ] [--opt] [-h|--help]

--prefix <dir>: The directory to install ARGoS and other project dependencies
                to. To install ARGoS systemwide (and therefore not have to set
                LD_LIBRARY_PATH), pass '/usr/local' (no quotes); this will
                require sudo access. If you are going to be modifying ARGoS at
                all, then you should not use '/usr/local'. Default=$HOME/.local.

--rroot <dir>: The root directory for all repos for the project. All github
               repos will be cloned/built in here. Default=$HOME/research.

--cores: The # cores to use when compiling. Default=$(nproc).

--nosyspkgs: If passed, then do not install system packages (requires sudo
             access). Default=YES (install system packages).

--eepuck3D: If passed, install a custom version of the epuck for 3D
            simulations.

--opt: Perform an optimized build of COSM. Default=NO.

-h|--help: Show this message.
EOF
    exit 1
}

# Make sure script was not run as root or with sudo
if [ $(id -u) = 0 ]; then
    echo "This script cannot be run as root."
    exit 1
fi

# For better debugging when the script doesn't work
# set -x

repo_root=$HOME/research
install_sys_pkgs="YES"
prefix=$HOME/.local
n_cores=$(nproc)
build_type="DEV"
eepuck3D="NO"
options=$(getopt -o h --long help,prefix:,rroot:,cores:,nosyspkgs,opt  -n "BOOTSTRAP" -- "$@")
if [ $? != 0 ]; then usage; exit 1; fi

eval set -- "$options"
while true; do
    case "$1" in
        -h|--help) usage;;
        --prefix) prefix=$2; shift;;
        --rroot) repo_root=$2; shift;;
        --cores) n_cores=$2; shift;;
        --eepuck3D) eepuck3D="YES";;
        --nosyspkgs) install_sys_pkgs="NO";;
        --opt) build_type="OPT";;
        --) break;;
        *) break;;
    esac
    shift;
done

################################################################################
# Functions
################################################################################
function bootstrap_argos() {

    if [ -d argos3 ]; then rm -rf argos3; fi
    git clone https://github.com/swarm-robotics/argos3.git
    cd argos3
    mkdir -p build && cd build

    git checkout devel
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
          -DCMAKE_C_COMPILER=gcc-9\
          -DCMAKE_CXX_COMPILER=g++-9\
          -DARGOS_BUILD_FOR=simulator\
          -DARGOS_BUILD_NATIVE=ON\
          -DARGOS_THREADSAFE_LOG=ON\
          -DARGOS_DYNAMIC_LIBRARY_LOADING=ON\
          -DARGOS_USE_DOUBLE=ON\
          -DARGOS_DOCUMENTATION=ON\
          -DARGOS_WITH_LUA=OFF\
          -DARGOS_INSTALL_LDSOCONF=$argos_sys_install \
          -DCMAKE_INSTALL_PREFIX=$prefix \
	      ../src
    make -j $n_cores
    make doc

    argos_sys_install=$([ "/usr/local" = "$prefix" ] && echo "YES" || echo "NO")

    if [ "YES" = "$argos_sys_install" ]; then
        sudo make install;
    else
        make install;
    fi;

    cd ../../

    # Install extended E-puck ARGoS model
    if [ "YES" = "$eepuck3D" ]; then
        bootstrap_eepuck3D
    fi

}

function bootstrap_eepuck3D() {

    if [ -d argos3-eepuck3D ]; then rm -rf argos3-eepuck3D; fi
    git clone https://github.com/swarm-robotics/argos3-eepuck3D.git
    cd argos3-eepuck3D
    mkdir -p build && cd build
    git checkout devel

    # This code expects ARGoS to be installed system wide, so we have to
    # tell it that where the necessary ARGoS cmake, pkgconfig, files are.
    cmake -DCMAKE_BUILD_TYPE=Release\
          -DCMAKE_CXX_COMPILER=g++-9\
          -DCMAKE_LIBRARY_PATH=$prefix/lib\
          -DCMAKE_PREFIX_PATH=$prefix/include\
          -DCMAKE_MODULE_PATH=$prefix/share\
          -DCMAKE_INSTALL_PREFIX=$prefix\
          ../src

    make -j $n_cores

    if [ "YES" = "$argos_sys_install" ]; then
        sudo make install;
    else
        make install;
    fi;

    cd ../../
}

function bootstrap_rcppsw() {
    wget\
        https://raw.githubusercontent.com/swarm-robotics/rcppsw/devel/scripts/bootstrap.sh\
        -O bootstrap-rcppsw.sh

    chmod +x bootstrap-rcppsw.sh
    rcppsw_syspkgs=$([ "YES" = "$install_sys_pkgs" ] && echo "" || echo "--nosyspkgs")
    rcppsw_opt=$([ "OPT" = "$build_type" ] && echo "--opt" || echo "")

    ./bootstrap-rcppsw.sh \
        --rroot $repo_root \
        --cores $n_cores \
        $rcppsw_syspkgs \
        $rcppsw_opt \
        --prefix $prefix \
        --arch x86_64

    ./bootstrap-rcppsw.sh \
        --rroot $repo_root \
        --cores $n_cores \
        $rcppsw_syspkgs \
        $rcppsw_opt \
        --prefix $prefix \
        --arch armhf
}

function bootstrap_cosm() {
    if [ -d cosm ]; then rm -rf cosm; fi
    git clone https://github.com/swarm-robotics/cosm.git
    cd cosm
    git checkout feature/164/base-ros-controller
    git submodule update --init --recursive --remote

    if [ "OPT" = "$build_type" ]; then
        er="NONE"
    else
        er="ALL"
    fi

    npm install
    mkdir -p build && cd build
    arch=$1
    
    if [ "$arch" = "x86_64" ]; then
        cmake\
            -DCMAKE_C_COMPILER=gcc-9 \
            -DCMAKE_CXX_COMPILER=g++-9 \
            -DCMAKE_INSTALL_PREFIX=$prefix \
            -DCOSM_BUILD_FOR=ARGOS_FOOTBOT \
            ..
    elif [ "$arch" = "armhf" ]; then
        source /opt/ros/noetic/setup.bash 
        cmake\
            -DCMAKE_TOOLCHAIN_FILE=../libra/cmake/arm-linux-gnueabihf-toolchain.cmake \
            -DCMAKE_INSTALL_PREFIX=$prefix \
            -DCOSM_BUILD_FOR=ROS_TURTLEBOT3 \
            ..
    fi
    make -j $n_cores
    make install

    cd ../..
}
################################################################################
# Bootstrap main
################################################################################

function bootstrap_main() {
    echo -e "********************************************************************************"
    echo -e "COSM BOOTSTRAP START:"
    echo -e "PREFIX=$prefix"
    echo -e "REPO_ROOT=$repo_root"
    echo -e "N_CORES=$n_cores"
    echo -e "SYSPKGS=$install_sys_pkgs"
    echo -e "BUILD_TYPE=$build_type"
    echo -e "********************************************************************************"

    mkdir -p $repo_root && cd $repo_root

    # First, bootstrap RCPPSW
    bootstrap_rcppsw

    # Install system packages
    if [ "YES" = "$install_sys_pkgs" ]; then
        cosm_pkgs=(qtbase5-dev
                   libfreeimageplus-dev
                   freeglut3-dev
                   libeigen3-dev
                   libudev-dev
                   ros-noetic-desktop-full # Not present on rasberry pi
                   ros-noetic-ros-base
                   liblua5.3-dev
                  )

        # Install packages (must be loop to ignore ones that don't exist)
        for pkg in "${cosm_pkgs[@]}"
        do
            sudo apt-get -my install $pkg
        done
    fi

    # Exit when any command after this fails. Can't be before the
    # package installs, because it is not an error if some of the
    # packages are not found (I just put a list of possible packages
    # that might exist on debian systems to satisfy project
    # requirements).
    set -e

    # Install ARGoS and eepuck (maybe)
    set -x
    # bootstrap_argos

    # Bootstrap COSM
    bootstrap_cosm x86_64
    bootstrap_cosm armhf

    # If we installed ARGoS as root, all project repos are also owned by
    # root, so we need to fix that.
    if [ "$YES" = "$argos_sys_install" ]; then
        sudo chown $SUDO_USER:$SUDO_USER -R $repo_root
    fi;

    # Made it!
    echo -e "********************************************************************************"
    echo -e "COSM BOOTSTRAP SUCCESS!"
    echo -e "********************************************************************************"
}

bootstrap_main