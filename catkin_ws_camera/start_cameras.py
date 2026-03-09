#!/usr/bin/env python3
import os
import time
import glob
import sys
import subprocess
import shlex
import shutil
from typing import Optional

def is_kinect_camera_line(line: str) -> bool:
    """Detect Kinect v1 camera interface only (not motor/audio)."""
    s = line.lower()
    is_kinect = "xbox nui" in s or "kinect" in s or "primesense" in s
    is_camera = "camera" in s
    return is_kinect and is_camera


def launch_process(title: str, command: str) -> Optional[subprocess.Popen]:
    """Launch command in GUI terminal if available; otherwise run in background."""
    gnome_terminal = shutil.which("gnome-terminal")
    has_display = bool(os.environ.get("DISPLAY"))

    if gnome_terminal and has_display:
        term_cmd = [
            gnome_terminal,
            "--tab",
            f"--title={title}",
            "--",
            "bash",
            "-lc",
            command,
        ]
        subprocess.Popen(term_cmd)
        return None

    print(f"[INFO] Running in background (no GUI terminal): {command}")
    return subprocess.Popen(shlex.split(command))


# ── Part 1: Kinect detection ──────────────────────────────────────────────────
lsusb_output = subprocess.run(
    ["lsusb"], check=False, text=True, capture_output=True
).stdout
lsusb_lines = lsusb_output.splitlines()
kinect_lines = [line for line in lsusb_lines if is_kinect_camera_line(line)]
num_kinect = len(kinect_lines)

# ── Part 2: USB camera detection via by-path symlinks ───────────────────────
v4l_paths = sorted(glob.glob("/dev/v4l/by-path/*-video-index0"))
num_usb_cam = len(v4l_paths)

if num_kinect == 0 and num_usb_cam == 0:
    print("Error: no Kinect or USB cameras found. Check lsusb and /dev/v4l/by-path.")
    sys.exit(1)

print(f"[INFO] Detected {num_kinect} Kinect device(s) and {num_usb_cam} USB camera(s).")
for idx, line in enumerate(kinect_lines, start=1):
    print(f"[INFO] Kinect #{idx}: {line}")

# ── Part 3: Launch everything ───────────────────────────────────────────────
cam_id = 1
launched_bg = []

# 1) start roscore
print("[INFO] Launching roscore...")
roscore_proc = launch_process("roscore", "roscore")
if roscore_proc is not None:
    launched_bg.append(("roscore", roscore_proc))
time.sleep(2)

# 2) Kinect nodes
for kinect_id in range(1, num_kinect + 1):
    # freenect expects '#1', '#2', etc. (not plain integers)
    freenect_device_id = f"#{kinect_id}"
    cmd = (
        f"roslaunch launch/launch_camera.launch "
        f"cam_id:={cam_id} cam_type:=kinect device_id:={freenect_device_id}"
    )
    print(f"[INFO] {cmd}")
    proc = launch_process(f"kinect_{kinect_id}", cmd)
    if proc is not None:
        launched_bg.append((f"kinect_{kinect_id}", proc))
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
    fuser_bin = shutil.which("fuser")
    if fuser_bin:
        subprocess.run([fuser_bin, "-k", video_dev],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL,
                       check=False)

    # now launch
    cmd = (
        f"roslaunch launch/launch_camera.launch "
        f"cam_id:={cam_id} cam_type:=usb_cam device_path:={video_dev}"
    )
    print(f"[INFO] {cmd}")
    proc = launch_process(f"usb_cam_{cam_id}", cmd)
    if proc is not None:
        launched_bg.append((f"usb_cam_{cam_id}", proc))

    # give ample time for driver to settle
    time.sleep(3)
    cam_id += 1

print(f"[INFO] Launched {num_kinect + num_usb_cam} camera nodes.")
if launched_bg:
    print("[INFO] Background process list:")
    for name, proc in launched_bg:
        print(f"[INFO]   {name}: pid={proc.pid}")
sys.exit(0)
