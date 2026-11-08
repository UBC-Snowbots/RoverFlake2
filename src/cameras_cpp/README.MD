
### Usage:

#### GStreamer compressor (no ffmpeg build needed)

Alternative to the ffmpeg/nvmpi pipeline below. Captures each camera in
`config/cameras.json`, hardware-encodes with `nvv4l2h265enc` (ships with
JetPack) and streams RTP/H.265 over UDP straight to the base station,
bypassing ROS/DDS for the video path.

Run on the jetson:

```
ros2 launch cameras_cpp jetson_gst_compressor.launch.py host:=192.168.1.5
```

Camera *i* in cameras.json streams to port `5600 + i` (mapping is printed at
launch). To publish the streams back into ROS on the base station (same
`/{camera_name}/image_raw_decoded` topics as the ffmpeg pipeline), run:

```
ros2 launch cameras_cpp gst_decompressor.launch.py
```

This uses [gscam2](https://github.com/clydemcqueen/gscam2)
(`sudo apt install ros-$ROS_DISTRO-gscam2`).

Or just view a stream directly without ROS:

```
gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96" ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false
```

Width/height/framerate/bitrate are launch arguments, see the launch file docstring.

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

#### Get id on jetson:

ls /dev/v4l2/by-id/

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