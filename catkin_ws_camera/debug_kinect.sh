#!/bin/bash
# Kinect debugging information gatherer

echo "========================================="
echo "KINECT DIAGNOSTIC INFORMATION"
echo "========================================="
echo

echo "1. USB DEVICE DETECTION"
echo "------------------------"
echo "All Kinect-related USB devices:"
lsusb | grep -i "xbox\|kinect\|microsoft"
echo

echo "2. USB DEVICE DETAILS"
echo "------------------------"
for line in $(lsusb | grep -i "xbox\|kinect" | awk '{print $2":"$4}' | tr -d ':'); do
    bus=$(echo $line | cut -d: -f1)
    dev=$(echo $line | cut -d: -f2)
    echo "Bus $bus Device $dev details:"
    lsusb -s $bus:$dev -v 2>/dev/null | grep -E "iSerial|iProduct|iManufacturer|bInterfaceClass|bInterfaceProtocol" | head -20
    echo
done

echo "3. USB DEVICE NODE ACCESS"
echo "------------------------"
for line in $(lsusb | grep -i "xbox\|kinect" | awk '{print $2":"$4}' | tr -d ':'); do
    bus=$(echo $line | cut -d: -f1 | sed 's/^0*//')
    dev=$(echo $line | cut -d: -f2 | sed 's/^0*//')
    usbnode=$(printf "/dev/bus/usb/%03d/%03d" $bus $dev)
    echo "Checking: $usbnode"
    if [ -e "$usbnode" ]; then
        ls -la "$usbnode"
        echo "Readable: $([ -r "$usbnode" ] && echo YES || echo NO)"
        echo "Writable: $([ -w "$usbnode" ] && echo YES || echo NO)"
    else
        echo "  ERROR: Device node does not exist!"
    fi
    echo
done

echo "4. KERNEL MODULES"
echo "------------------------"
echo "Loaded modules that might interfere:"
lsmod 2>/dev/null | grep -E "gspca|kinect|uvc|video" || echo "lsmod not available (inside container)"
echo

echo "5. LIBFREENECT TEST"
echo "------------------------"
if command -v freenect-glview &> /dev/null; then
    echo "Testing with freenect-glview (will timeout in 3 seconds)..."
    timeout 3 freenect-glview 2>&1 | head -20 || echo "Test completed/timed out"
else
    echo "freenect-glview not found"
fi
echo

echo "6. ROS ENVIRONMENT"
echo "------------------------"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
echo "Freenect launch file:"
rospack find freenect_launch 2>/dev/null && echo "  Found" || echo "  Not found"
echo

echo "7. PERMISSIONS & CONTAINER INFO"
echo "------------------------"
echo "Current user: $(whoami)"
echo "User groups: $(groups)"
echo "Running in container: $([ -f /.dockerenv ] && echo YES || echo NO)"
echo "Privileged mode indicators:"
cat /proc/1/status 2>/dev/null | grep CapEff || echo "Cannot read /proc/1/status"
echo

echo "========================================="
echo "DIAGNOSTIC COMPLETE"
echo "========================================="
