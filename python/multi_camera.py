########################################################################
#
# Copyright (c) 2020, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    Multi cameras sample showing how to open multiple ZED in one program
"""

import pyzed.sl as sl 
import cv2  
import numpy as np  
import threading  
import time 
import signal  

# Global variables for storing camera instances, images, timestamps, and thread control
zed_list = []
left_list = []
depth_list = []
timestamp_list = []
thread_list = []
stop_signal = False
ready_signal = [False, False]  # List to check if cameras are ready

# Function to handle interrupt signal (e.g., Ctrl+C)
def signal_handler(signal, frame):
    global stop_signal
    stop_signal = True  # Set stop signal to True to stop all threads
    time.sleep(0.5)
    exit()

# Function to grab frames from a specific camera
def grab_run(index):
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global ready_signal

    runtime = sl.RuntimeParameters()  # Runtime parameters for the ZED camera
    while not stop_signal:  # Loop until stop_signal is set to True
        err = zed_list[index].grab(runtime)  # Grab a frame
        if err == sl.ERROR_CODE.SUCCESS:  # If successful
            zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)  # Retrieve left image
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)  # Retrieve depth map
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns  # Get timestamp
            ready_signal[index] = True  # Signal that this camera has grabbed a frame
        time.sleep(0.001)  # Sleep for 1ms

    zed_list[index].close()  # Close the camera when done

# Main function to initialize and run the cameras
def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global thread_list
    global ready_signal

    signal.signal(signal.SIGINT, signal_handler)  # Setup signal handler for Ctrl+C

    print("Running...")
    init = sl.InitParameters()  # Initialize parameters for the ZED camera
    init.camera_resolution = sl.RESOLUTION.HD720  # Set camera resolution
    init.camera_fps = 30  # Set camera FPS to 30 to avoid USB3 bandwidth issues

    # List and open cameras
    name_list = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()  # Get list of connected ZED cameras
    index = 0
    for cam in cameras:
        init.set_from_serial_number(cam.serial_number)  # Set camera by serial number
        name_list.append("ZED {}".format(cam.serial_number))  # Add camera name to list
        print("Opening {}".format(name_list[index]))
        zed_list.append(sl.Camera())  # Create a new camera instance
        left_list.append(sl.Mat())  # Create a Mat for the left image
        depth_list.append(sl.Mat())  # Create a Mat for the depth map
        timestamp_list.append(0)  # Initialize timestamp list
        last_ts_list.append(0)  # Initialize last timestamp list
        status = zed_list[index].open(init)  # Open the camera
        if status != sl.ERROR_CODE.SUCCESS:  # If there's an error
            print(repr(status))  # Print the error
            zed_list[index].close()  # Close the camera
        index = index + 1  # Increment index

    # Start camera threads
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():  # Check if the camera is opened
            thread_list.append(threading.Thread(target=grab_run, args=(index,)))  # Create a thread for each camera
            thread_list[index].start()  # Start the thread

    # Ensure all cameras are ready before starting display loop
    while not all(ready_signal):  # Wait until all cameras are ready
        time.sleep(0.1)  # Sleep for 100ms

    # Display camera images
    key = ''
    while key != 113:  # Loop until 'q' key is pressed (ASCII 113)
        max_ts = max(timestamp_list)  # Get the latest timestamp
        for index in range(0, len(zed_list)):
            if zed_list[index].is_opened():
                if (timestamp_list[index] > last_ts_list[index]):  # If new frame is available
                    # Check synchronization window
                    if abs(timestamp_list[index] - max_ts) < 1000000:  # 1ms tolerance for synchronization
                        cv2.imshow(name_list[index], left_list[index].get_data())  # Display the left image
                        x = round(depth_list[index].get_width() / 2)
                        y = round(depth_list[index].get_height() / 2)
                        err, depth_value = depth_list[index].get_value(x, y)  # Get depth value at the center
                        if np.isfinite(depth_value):  # Check if the depth value is valid
                            print("{} depth at center: {}MM".format(name_list[index], round(depth_value)))
                        last_ts_list[index] = timestamp_list[index]  # Update last timestamp
        key = cv2.waitKey(10)  # Wait for 10ms
    cv2.destroyAllWindows()  # Destroy all OpenCV windows

    # Stop the threads
    stop_signal = True  # Set stop signal to True to stop all threads
    for index in range(0, len(thread_list)):
        thread_list[index].join()  # Wait for all threads to finish

    print("\nFINISH")

if __name__ == "__main__":
    main()  # Run the main function
