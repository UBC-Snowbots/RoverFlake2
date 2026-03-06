cd ${RANDOM_INSTALL_FILE_DIR}

# clone the sweep-sdk repository
git clone https://github.com/scanse/sweep-sdk

# enter the libsweep directory
cd sweep-sdk/libsweep

# create and enter a build directory
mkdir -p build
cd build

# build and install the libsweep library
echo CONFIGUIRING SWEEPSDK
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
echo BUILDNG SWEEP
cmake --build .
sudo cmake --build . --target install
sudo ldconfig

cd ${ROVERFLAKE_ROOT}