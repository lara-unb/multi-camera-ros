#!/usr/bin/env python3
import os
import time
import glob
import sys
import subprocess

def get_device_type(tokens):
    return 'kinect' if len(tokens) >= 3 and tokens[-3] == 'Xbox' else 'usb_cam'

# ── Part 1: Kinect detection (unchanged) ──────────────────────────────────────
output = os.popen("lsusb | grep 'Camera\\|Webcam\\|Xbox'").read()
devices = output.splitlines()

num_kinect = sum(1 for d in devices if 'Xbox' in d)

# ── Part 2: USB camera detection via by-path symlinks ───────────────────────
v4l_paths = sorted(glob.glob("/dev/v4l/by-path/*-video-index0"))
num_usb_cam = len(v4l_paths)

if num_usb_cam < 1:
    print("Error: no USB cameras found! Please plug in at least one.")
    sys.exit(1)

# ── Part 3: Launch everything ───────────────────────────────────────────────
cam_id = 1

# 1) start roscore
print("[INFO] Launching roscore...")
os.system("gnome-terminal --tab --title='roscore' -- roscore")
time.sleep(2)

# 2) Kinect nodes
for kinect_id in range(1, num_kinect + 1):
    cmd = (
        f"roslaunch launch/launch_camera.launch "
        f"cam_id:={cam_id} cam_type:=kinect device_id:={kinect_id}"
    )
    print(f"[INFO] {cmd}")
    os.system(f"gnome-terminal --tab --title='kinect_{kinect_id}' -- {cmd}")
    cam_id += 1
    time.sleep(1)

# 3) USB cams
for usb_symlink in v4l_paths:
    # resolve symlink → actual /dev/videoX
    video_dev = os.path.realpath(usb_symlink)
    print(f"[INFO] Launching cam_id={cam_id} on {video_dev}")
    # attempt pre‑init, but timeout if it hangs
    try:
        print(f"[INFO] Pre‑initializing {video_dev} (5s timeout)...")
        subprocess.run(
            ["v4l2-ctl", "-d", video_dev, "--stream-mmap", "--stream-count=1"],
            timeout=5,
            check=False,
        )
    except subprocess.TimeoutExpired:
        print(f"[WARN] Pre‑init hung on {video_dev}, skipping pre‑init.")

    time.sleep(1)

    # kill any leftover processes on the real device node
    print(f"[INFO] Releasing {video_dev} if in use...")
    subprocess.run(["sudo", "fuser", "-k", video_dev],
                   stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)

    # now launch
    cmd = (
        f"roslaunch launch/launch_camera.launch "
        f"cam_id:={cam_id} cam_type:=usb_cam device_path:={video_dev}"
    )
    print(f"[INFO] {cmd}")
    os.system(f"gnome-terminal --tab --title='usb_cam_{cam_id}' -- {cmd}")

    # give ample time for driver to settle
    time.sleep(3)
    cam_id += 1

print(f"[INFO] Launched {num_kinect + num_usb_cam} camera nodes.")
sys.exit(0)
