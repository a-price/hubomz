# This script will
#   -patch the huboplus OpenRAVE model
#   -create a build diretory
#   -run cmake to create a makefile
#   -compile

# create the patch if it doesn't already exit
if [ ! -d "huboplus" ]; then
    cd ..
    # clone openHubo repo if it doesn't exist
    if [ ! -d "openHubo" ]; then
        echo "Cloning openHubo from https://github.com/daslrobotics/openHubo.git"
        git clone https://github.com/daslrobotics/openHubo.git
    fi

    # patch the huboplus OpenRAVE model
    cd openHubo
    patch -p0 < ../hubomz/huboplus.patch
    cd ../hubomz
    ln -s ../openHubo/robots/huboplus
fi

# create build directory if not there and cmake
if [ ! -d "build" ]; then
    echo "Creating build directory"
    mkdir build
fi

cd build
echo "Generating compilation files"
cmake .. -DCMAKE_BUILD_TYPE=Release
echo "Building zmp-daemon executable"
make zmp-daemon
make zmpgui
