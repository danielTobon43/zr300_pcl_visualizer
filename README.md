# zr300_pcl_visualizer
A simple C++ program to visualize the 3D pointcloud from a ZR300 Intel Realsense camera using PCL 1.9.0, CMake and IntelRealsense SDK Legacy (ZR300). This program detected if 1 or 2 Intelrealsense cameras are connected to a USB3.0 port and display a viewer. 

pcl - librealsense wrapper: https://joshmcculloch.nz/#!realsense

<img src="./zr300.png" align="center" height="400" width="500"><br>
<img src="./example.png" align="center" height="400" width="500"><br>

## How to compile IntelRealsense Legacy Library
Follow the instructions on: https://github.com/IntelRealSense/librealsense/tree/legacy or,

* Ubuntu 16.04.6 with kernel (4.15.0-69-generic)

      Dependencies:
      - sudo apt-get update
      - sudo apt-get install libssl-dev
      - sudo apt-get install libusb-1.0-0-dev pkg-config
      
       Relevant dependencies:
       - PCL 1.9.0 (You don't have to install it)
       - CMake >= 3.5     
      
      Library Build Process:
      
      - mkdir build && cd build
      - cmake ../src/
      - make (You don't have to install it)
      
      Video4Linux backend preparation:
      
      - sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
      - sudo udevadm control --reload-rules && udevadm trigger
      
      At the root library folder, run:
      - ./scripts/patch-realsense-ubuntu-xenial.sh
      
## How to compile ZR300 pcl visualizer
   * Download or clone this repo: https://github.com/danielTobon43/zr300_pcl_visualizer.git
   * Create a "build" folder
   * Modify the *CMakeList.txt* file with your own path:
      
            e.g: 
            
            PCL BUILD DIR:          /opt/pcl-1.9.0/build 
            REALSENSE LIB DIR:      /opt/librealsense-legacy
            
            and save it
            
   * Run cmake,
   
            - cmake ../src/
            - make
    
## Test  
  
     
      - Connect camera ZR300 to USB 3.0 port
      - cd build/
      - ./pcl-realsense
      
## Output example 

For 1 camera:

      There are 1 connected RealSense devices.
      -> Using device 0, an Intel RealSense ZR300
          Serial number: 4071801127
          Firmware version: 2.0.71.28

      -> -------------------------------
      -> Setting visualizer
      -> Setting loop for streaming
      ********************************
      Iteration No.[0] 
      ********************************

      ********************************
      Iteration No.[1] 
      ********************************

      ********************************
      Iteration No.[2] 
      ********************************
      
For 2 cameras:

      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      ***      MAIN PROGRAM     ***
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      -> Reading camera ...There are 2 connected RealSense devices.
      -> Using device 0, an Intel RealSense ZR300
          Serial number: 4071801092
          Firmware version: 2.0.71.28

      -> Using device 1, an Intel RealSense ZR300
          Serial number: 4071801127
          Firmware version: 2.0.71.28

      -> -------------------------------
      -> Setting visualizer
      -> Setting viewport
      -> Removing all pointclouds
      -> Setting loop for streaming
      ********************************
      Iteration No.[0] 
      ********************************

       ---------------------------------
       *******     Camera 1     ******* 
       ---------------------------------
      -> Points rgb: 	 		484
      -> Points xyzi:  		      484
      -> Normal points: 		484
      -> Keypoints:			383
      -> Descriptors:			383

       ---------------------------------
       *******     Camera 2     ******* 
       ---------------------------------
      -> Points rgb: 	 		1018
      -> Points xyzi:  		      1018
      -> Normal points: 		1018
      -> Keypoints:			797
      -> Descriptors:			797
      
            
For none cameras:

      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      ***      MAIN PROGRAM     ***
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      > -> Loading
      No devices detected. Is it plugged in?
      Min: 1 camera, Max: 2 cameras
      > Visualizer closed.





    
