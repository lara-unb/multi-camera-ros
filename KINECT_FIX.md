# Kinect Detection Issue - SOLVED

## Problem Identified
The diagnostic revealed: **`LIBUSB_ERROR_BUSY`**

This means the Kinect USB interfaces are already claimed by kernel drivers on your **host Ubuntu system**. Even though Docker runs in privileged mode with USB access, libfreenect cannot claim interfaces that the host kernel has already bound.

## Solution: Unbind Host Kernel Drivers

You have **two options**:

---

### Option 1: Manual Unbind (Quick Fix)

Run this **on your HOST machine** (outside Docker) every time before using Kinect:

```bash
cd /home/tambs/tcc/multi-camera-ros
chmod +x unbind_kinect_host.sh
./unbind_kinect_host.sh
```

This script will unbind kernel drivers from all Kinect USB interfaces.

**Then** run your Docker container and start cameras:
```bash
docker compose exec multi-camera-ros bash
cd ~/catkin_ws_camera
python3 start_cameras.py
```

**Downside**: You need to run the unbind script every time you reconnect the Kinect or reboot.

---

### Option 2: Permanent Fix with udev Rules (Recommended)

Install a udev rule **on your HOST machine** to automatically prevent kernel drivers from claiming Kinect:

```bash
cd /home/tambs/tcc/multi-camera-ros
sudo cp 51-kinect.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then **unplug and replug your Kinect** (or reboot).

After this, Kinect will always be available for libfreenect without manual unbinding.

**Test it works:**
```bash
# Inside Docker:
docker compose exec multi-camera-ros bash
freenect-glview
# You should see the camera stream (press Ctrl+C to exit)
```

---

## Why This Happens

1. Ubuntu's kernel automatically loads drivers for USB devices (like `uvcvideo` for cameras, `snd-usb-audio` for audio)
2. These drivers bind to Kinect's USB interfaces immediately when plugged in
3. Once bound, libusb/libfreenect cannot claim the interfaces (LIBUSB_ERROR_BUSY)
4. The solution is to either:
   - Manually unbind before each use (Option 1)
   - Use udev rules to prevent auto-binding (Option 2)

---

## Verification

After applying either solution, verify it works:

```bash
# Inside Docker container:
freenect-glview
```

If you see:
- ❌ `LIBUSB_ERROR_BUSY` → Drivers still bound, try Option 1 or check udev rules
- ✅ Camera stream → Success! Now `start_cameras.py` will work

---

## Next Steps

Once the Kinect is accessible:

```bash
# Inside Docker:
cd ~/catkin_ws_camera
python3 start_cameras.py
```

The script will now successfully launch the Kinect camera node.
