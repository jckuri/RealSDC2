# Programming a Real Self-Driving Car

Self-Driving Car Engineer Nanodegree Program<br/>
https://classroom.udacity.com/nanodegrees/nd013

# Installation

Go to the Udacity workspace of the project [Programming a Real Self-Driving Car](https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/1dc566d7-03d4-40da-af2c-b8ec85f2e4dd/lessons/e43b2e6d-6def-4d3a-b332-7a58b847bfa4/concepts/51c2ea21-5317-4bbd-ab82-047e5fd6849b).

Click on the `+` sign in the left panel where the directory structure is.
Click on "Upload File". Upload the file [RealSDC2-main.zip](https://github.com/jckuri/RealSDC2/archive/refs/heads/main.zip) in the directory `/home/workspace`.

Unzip the file `RealSDC2-main.zip`.
Go to the directory `RealSDC2-main`.
How? Use these commands in the terminal:

```
unzip RealSDC2-main.zip
cd RealSDC2-main
```

# Usage

If the GPU is not enabled, go to the GPU panel at the bottom left corner and
click on `ENABLE`.

Click on `GO TO DESKTOP`, the blue button at the bottom right corner.
Go to the Desktop that was just opened.

Open a Terminator and type:

```
roscore
```

Open another Terminator and type:

```
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

Once you launch the process, many messages should appear. 
You should not see any error. Otherwise, you did something wrong.
At the end, you should see the INFO log message:

```
[INFO] [ ]: TRAFFIC LIGHT CLASSIFIER WAS LOADED SUCCESSFULLY.
```

Click on the icon `Capstone Simulator` located in the Desktop.
Set the screen resolution to `640 x 480`.
Set the Graphics Quality to `Fastest`.
Click on `OK`.

Select the `Highway track`.
Activate the Camera.
Deactivate Manual.
Zoom out the scene by rolling down your mouse wheel.
Enjoy the automatic ride.

# Demo in the Highway

If you follow the instructions correctly, you should be able to see a
simulation like this:

**Programming a Real Self-Driving Car (Highway)<br/>
https://youtu.be/qocT3uHOLaA**
<img src='imgs/demo.png'/>

# Demo in the Test Lot

Terminate the process `roslaunch launch/styx.launch` by pressing `Ctrl+C`.

Go to the root directory of the project.
Execute the script `CHANGE_TO_TEST_LOT.sh`.

```
cd ..
sh CHANGE_TO_TEST_LOT.sh
```

This script changes the waypoints of the file `ros/src/waypoint_loader/launch/waypoint_loader.launch` to:

```
<?xml version="1.0"?>
<launch>
    <node pkg="waypoint_loader" type="waypoint_loader.py" name="waypoint_loader">
        <param name="path" value="$(find styx)../../../data/churchlot_with_cars.csv" />
        <param name="velocity" value="40" />
    </node>
</launch>
```

Go to the directory `ros`.
Execute the process `roslaunch launch/styx.launch` again.

```
cd ros
roslaunch launch/styx.launch
```

Once you launch the process, many messages should appear. 
You should not see any error. Otherwise, you did something wrong.
At the end, you should see the INFO log message:

```
[INFO] [ ]: TRAFFIC LIGHT CLASSIFIER WAS LOADED SUCCESSFULLY.
```

Click on the icon `Capstone Simulator` located in the Desktop.
Set the screen resolution to `640 x 480`.
Set the Graphics Quality to `Fastest`.
Click on `OK`.

Select the `Test Lot track`.
Deactivate Manual.
Zoom out the scene by rolling down your mouse wheel.
Enjoy the automatic ride.

If you follow the instructions correctly, you should be able to see a
simulation like this:

**Programming a Real Self-Driving Car (Test Lot)<br/>
https://youtu.be/gE28pyRXaHA**
<img src='imgs/demo2.png'/>

If you want to execute the simulation in the `Highway track` again, you need to follow
a similar procedure. But this time, you should execute the script 
`CHANGE_TO_TEST_LOT.sh`.

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
