#!/usr/bin/env python3
"""Add a new camera to cameras_cpp/config: registers it in cameras.json and
creates a params yaml file with its /dev/v4l/by-id device id.

Usage:
    python3 add_camera.py <name> <id>

<id> can be given as:
    - a full path, e.g. /dev/v4l/by-id/usb-046d_081d_374D8990-video-index0
    - just the by-id fragment, e.g. usb-046d_081d_374D8990-video-index0
The device id can be found by running: ls -l /dev/v4l/by-id/
"""
import argparse
import json
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent.parent / "src" / "cameras_cpp" / "config"
CAMERAS_JSON = CONFIG_DIR / "cameras.json"

YAML_TEMPLATE = '''/**:
  ros__parameters:
    video_device: "{video_device}"
    #image_width: 640
    #image_height: 480
    #framerate: 30.0
    #pixel_format: "yuyv"
    #image_raw:
      #ffmpeg:
        #encoder: "h264_nvmpi"
        #bit_rate: 2000000
        #gop_size: 20 # Set the Group of Pictures (GOP) size for the encoder. A smaller GOP size can improve error resilience but may increase file size, while a larger GOP size can reduce file size but may be less resilient to errors.
        #qmax: 20 # Set the maximum quantization parameter (Q) for the encoder. Lower values result in higher quality but larger file sizes, while higher values result in lower quality but smaller file sizes.
        #encoder_av_options: 'num_capture_buffers:4;profile:main;preset:ultrafast'
'''


def resolve_video_device(camera_id: str) -> str:
    if camera_id.startswith("/dev/"):
        return camera_id
    return f"/dev/v4l/by-id/{camera_id}"


def load_cameras_json() -> dict:
    if CAMERAS_JSON.exists():
        with open(CAMERAS_JSON, "r") as f:
            return json.load(f)
    return {"cameras": []}


def add_camera(name: str, camera_id: str) -> None:
    yaml_filename = f"{name}.yaml"
    yaml_path = CONFIG_DIR / yaml_filename

    data = load_cameras_json()
    cameras = data.setdefault("cameras", [])
    existing = next((c for c in cameras if c.get("name") == name), None)
    if existing is not None:
        existing["params_file"] = yaml_filename
    else:
        cameras.append({"name": name, "params_file": yaml_filename})

    video_device = resolve_video_device(camera_id)

    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(yaml_path, "w") as f:
        f.write(YAML_TEMPLATE.format(video_device=video_device))

    with open(CAMERAS_JSON, "w") as f:
        json.dump(data, f, indent=4)
        f.write("\n")

    print(f"Added camera '{name}' -> {video_device}")
    print(f"  Updated {CAMERAS_JSON}")
    print(f"  Wrote   {yaml_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="Camera name, e.g. realsense415")
    parser.add_argument(
        "id",
        help="by-id device fragment or full path, e.g. usb-046d_081d_374D8990-video-index0",
    )
    args = parser.parse_args()

    add_camera(args.name, args.id)


if __name__ == "__main__":
    main()
