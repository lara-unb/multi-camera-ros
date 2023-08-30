import os
import time

def get_device_type(tokens):
    if tokens[-3] == 'Xbox':
        return 'kinect'
    return 'usb_cam' 

error_flag = False

# Run shell commnad and get output 
output = os.popen("lsusb | grep 'Camera\|Webcam'").read()
devices = output.splitlines()

# Separate the devices by USB Bus
all_usb_bus = {}
num_kinect = 0
num_usb_cam = 0
for device in devices:
    tokens = device.split(' ')
    bus_id = tokens[1] #Checks Bus ID
    
    if bus_id in all_usb_bus: 
        error_flag = True
        print("Error: Two Devices can't be connect in the same USB Bus")
        break
    else:
        device_type = get_device_type(tokens)
        if device_type == 'kinect':
            num_kinect = num_kinect + 1
        else:
            num_usb_cam = num_usb_cam + 1
        all_usb_bus[bus_id] = device_type #Association between device type and bus for debugging puposes

# Launching Camera Nodes based on camera type 
if not error_flag:
    cam_id = 1
    os.system("gnome-terminal --tab --title='rosocore' -- roscore")
    time.sleep(1)
    for kinect_id in range(1, num_kinect + 1):
        os.system(f"gnome-terminal --tab -- roslaunch launch/launch_camera.launch cam_id:={cam_id} cam_type:=kinect device_id:={kinect_id}")
        time.sleep(1)
        cam_id = cam_id + 1
    for usb_cam_id in range(1, num_usb_cam + 1):
        os.system(f"gnome-terminal --tab -- roslaunch launch/launch_camera.launch cam_id:={cam_id} cam_type:=usb_cam device_id:={usb_cam_id}")
        time.sleep(1)
        cam_id = cam_id + 1

    

