# monoDrive C++ Client

## Windows Prerequisites

- Windows 10
- Visual Studio 2019 Community Edition
- [VSCode](https://code.visualstudio.com/)
- [Boost 1.65.1](https://sourceforge.net/projects/boost/files/boost-binaries/1.65.1/boost_1_65_1-msvc-14.1-64.exe/download)
    *Note: Use the default install location*
 

## Ubuntu 18.04 Prerequisites
- Ubuntu 18.04
- [VSCode](https://code.visualstudio.com/)
- Install packages: 
    ```bash 
    sudo apt-get install update && sudo apt-get install libboost-dev build-essentials libeigen3-dev
    ```
## VSCode Setup and Build Instructions
1. Open VSCode.
2. Add the following VSCode extensions:
    - CMake
    - CMake Tools
    - C/C++
3. Select `File -> Open Folder` and navigate to this folder to build the cpp-examples or simulator-cpp-client to build just the client library.
4. Click the CMake extension
    1. Click the Configure All Projects.
    2. If prompted to Scan for Kits select Yes.
    
    *Windows* Choose: `Visual Studio Community 2019 Release - amd64`.
    
    *Linux* Choose: `Choose the compiler of your choice, tested with g++ 7.5.0`.
    
5. Build the client by clicking the `Build All Projects` icon.

# monoDrive ROS Client

## Ubuntu 18.04 Prerequisites
- [monoDrive c++ client](##ubuntu-18.04-prerequisites)
- [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) *Note: Tested with melodic*
- ROS Bridge: 
```bash 
sudo apt-get install ros-melodic-rosbridge-suite
````


### monoDrive ROS Packages and example build

1. Execute the following to build the ROS packages: 
```bash
cd ros-examples
catkin_make
source devel/setup.bash
```
2. Add the setup file to your .bashrc to add the packages to your ros path on terminal load:
```bash
echo "source <path/to/devel/setup.bash>" >> ~/.bashrc
```

3. TODO: Add launch file, for now go to the next step

4. To launch the monoDrive examples create 3 tabs and run each command in a separate terminal:
    1. Launch rosbridge, you can leave this running: 
    ```bash
    roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
    ```
    2. Start the vehicle control node which will subscribe to the state sensor topic and publish vehicle controls (the simulator does not need to be running)
    ```bash
    rosrun vehicle_control node
    ```
    *Note: The vehicle_control example only requires the monodrive_msgs package and provides an example of how to connect your code to monoDrive through ROS messages.*

    3. Make sure the monoDrive simulator is running since the next command will connect to and start the simulator scenario running.
    ```bash
    rosrun simulator_control node
    ```

