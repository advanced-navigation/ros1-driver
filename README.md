# Advanced Navigation ROS1 Driver

## Introduction

This is an example using the Advanced Navigation Spatial SDK to create a ROS2 driver that reads and decodes the Advanced Navigation Packet Protocol (ANPP) (in this case packet #20 and packet #28) and publishes the information as ROS topics / messages. 

This example also includes the encoding of ANPP packet #55 and pushes the information to the Spatial INS.

It is designed to work with all Advanced Navigation INS devices using ANPP.

The code has been written to be easy to understand and for ease of extensibility with other ANPP packets.

This example has been developed and tested using **Ubuntu Linux v16.04 LTS** and **ROS1 Lunar**. Installation instructions for ROS2 can be found here: http://wiki.ros.org/lunar/Installation/Ubuntu

If you require any assistance using this code, please email support@advancednavigation.com



## ROS1 Getting Started Guide

The following guides are useful in getting started with ROS2 if you are not familiar:

- Setting up a ROS1 Workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Basic Tutorial on importing an Example ROS1 code and compiling and running: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29


## Build Instruction

- Open a New Terminal, and navigate to `workspace-folder-name\src\`
- Get the Advanced Navigation ROS1 Driver   
  ```
  git clone https://github.com/advanced-navigation/ros1-driver.git
  ```

- In the root of your workspace, `workspace-folder-name`, source and build the package:
    - Build your new package:
    ```
    catkin_make
    ```
    - Source the ROS2 Environment to the current folder:
    ```
    source devel/setup.bash
    ```

## Device Configuration

To use this example code, your Advanced Navigation device should be configured to output ANPP packets #20 and #28.

If you are not sure how to configure your Advanced Navigation Device please refer to the Reference Manual on the relevant product page (https://www.advancednavigation.com/products/all). 



## Run Instructions

Open a new terminal or new tab, navigate to `workspace-folder-name`, and source the setup files:
```
source devel/setup.bash
```

- Run the Driver in the following manners:
  1. Baud Rate and Comm Port as arguments:
     ```
     usage: rosrun package_name package_name [baud_rate] [comm_port]
        package_name     Name of the ROS package
        baud_rate        The Baud rate configured on the device. Default 115200
        comm_port        The COM port of the connected device. Default /dev/ttyUSB0
     ```
     ***e.g. rosrun ros1-driver ros1_driver 115200 /dev/ttyUSB0***
  2. Baud Rate, Comm Port and NTRIP as arguments:
     ```
     rosrun package_name package_name -B [baud_rate] -D [comm_port] -s [server_url] -m [mountpoint]  -u [username] -p [password]
       package_name     Name of the ROS package
       -B baud_rate     Baud rate configured on the device. Default 115200
       -D comm_port     COM port of the connected device. Default /dev/ttyUSB
       -s server_url    URL of the NTRIP server
       -m mountpoint    Name of the mountpoint
       -u username      Username of your NTRIP account
       -p password      Password of your NTRIP account 
     ```
     ***e.g. rosrun ros1-driver ros1_driver -B 115200 -D /dev/ttyUSB0 -s alldayrtk.com -m MOUNTPOINT_20  -u yourUsername -p yourPassword***


## Published Topics
Use RQT Monitor to view published topics. Here you will find details on how to use RQT: https://index.ros.org/doc/ros2/Tutorials/RQt-Overview-Usage/
- Run RQT Monitor by entering the following:
  ```
  rqt
  ```
- To view the published messages in RQT Monitor, click **Plugins-Topics** and click **Topic Monitor**
