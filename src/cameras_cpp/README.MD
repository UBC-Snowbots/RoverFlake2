
### Usage:

#### Run encoder on jetson:

```
ros2 launch cameras_cpp jetson_decompressor.launch.py
```

#### Run decoder on cbs:

```
ros2 launch cameras_cpp jetson_compressor.launch.py
```

The cameras are named in src\cameras_cpp\config\cameras.json

They publish to /{camera_name}/image_raw_decoded

Please do not access the image_raw or the image_raw/ffmpeg directly!!!

### Dependencies on jetson

I built ffmpeg from src on the jetson. Follow these steps to reproduce.

Step 1:

```
git clone https://github.com/FFmpeg/FFmpeg.git
```

Step 2:

```
git clone https://github.com/jocover/jetson-ffmpeg.git
```

Step 3
```
cd jetson-ffmpeg
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Step 4
```
cd ffmpeg
wget https://github.com/jocover/jetson-ffmpeg/raw/master/ffmpeg_nvmpi.patch
git apply ffmpeg_nvmpi.patch
./configure --enable-nvmpi
make
```