cd ..
# clone openHubo repo if it doesn't exist
if [ ! -d "openHubo" ]; then
    git clone https://github.com/daslrobotics/openHubo.git
fi

# clone patch the huboplus OpenRAVE model
cd openHubo
patch -p0 < ../hubomz/huboplus.patch
cd ../hubomz
ln -s ../openHubo/robots/huboplus
