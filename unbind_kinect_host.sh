#!/bin/bash
# Run this script on the HOST machine (outside Docker) to unbind kernel drivers from Kinect
# This allows libfreenect to claim the device

set -e

echo "========================================="
echo "KINECT USB INTERFACE UNBIND SCRIPT"
echo "Run this on the HOST (not in Docker)"
echo "========================================="
echo

if [ -f /.dockerenv ]; then
    echo "ERROR: This script must run on the HOST, not inside Docker!"
    echo "Exit the container and run this script on your main Ubuntu system."
    exit 1
fi

echo "Checking for Kinect devices..."
lsusb | grep -i "Xbox NUI\|Kinect"

echo
echo "Finding Kinect USB device paths..."

# Find all Kinect-related devices
for device in $(lsusb | grep -i "Xbox NUI\|Kinect" | awk '{print $2":"$4}' | sed 's/:$//'); do
    BUS=$(echo $device | cut -d: -f1)
    DEV=$(echo $device | cut -d: -f2)
    
    echo
    echo "Processing Bus $BUS Device $DEV..."
    
    # Find the USB device path in sysfs
    USB_PATH="/sys/bus/usb/devices/${BUS}-*"
    
    # Try to find the actual device directory
    for dir in /sys/bus/usb/devices/*; do
        if [ -f "$dir/busnum" ] && [ -f "$dir/devnum" ]; then
            bus_num=$(cat "$dir/busnum" | xargs printf "%03d")
            dev_num=$(cat "$dir/devnum" | xargs printf "%03d")
            
            if [ "$bus_num" = "$BUS" ] && [ "$dev_num" = "$DEV" ]; then
                echo "  Found device at: $dir"
                
                # Unbind all interfaces
                for interface in "$dir"/*:*; do
                    if [ -d "$interface" ]; then
                        interface_name=$(basename "$interface")
                        driver_path="$interface/driver"
                        
                        if [ -L "$driver_path" ]; then
                            driver=$(basename $(readlink "$driver_path"))
                            echo "    Interface $interface_name: bound to driver '$driver'"
                            echo "    Unbinding..."
                            echo "$interface_name" | sudo tee "$driver_path/unbind" > /dev/null 2>&1 || true
                            echo "    ✓ Unbound"
                        else
                            echo "    Interface $interface_name: not bound to any driver (OK)"
                        fi
                    fi
                done
            fi
        fi
    done
done

echo
echo "========================================="
echo "Unbind complete!"
echo "Now you can run start_cameras.py inside Docker"
echo "========================================="
echo
echo "NOTE: If you reconnect the Kinect or reboot,"
echo "you'll need to run this script again."
echo
