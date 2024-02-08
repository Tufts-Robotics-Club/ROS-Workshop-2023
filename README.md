# Tufts Robotics 2023 ROS Workshop Starter Code

Example ROS node for basic movement control in TurtleSim :)

## How to Run
Assuming you've followed the setup instructions sent out prior to the meeting (if not, see [setup instructions](#setup-instructions) below), you should be able to run this code as follows:

### MacOS
In a terminal window, run

```sh
# set up environment
conda activate robostackenv
cd ~/catkin_ws
source devel/setup.sh

# run the ros master (server over which the nodes communicate)
roscore
```

This may take a moment, but should give command line output similar to
```bsh
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt

started roslaunch server http://Emmas-MacBook-Pro.local:64025/
ros_comm version 1.15.14


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.14

NODES

auto-starting new master
process[master]: started with pid [68833]
ROS_MASTER_URI=http://Emmas-MacBook-Pro.local:11311/

setting /run_id to d212418c-b9fe-11ed-a83f-fed2a189e5b7
process[rosout-1]: started with pid [68834]
started core service [/rosout]
```

In a separate command line window (while ros master continues running in the old one), run
```sh
# set up environment
conda activate robostackenv
cd ~/catkin_ws
source devel/setup.sh

# run the turtlesim
rosrun turtlesim turtlesim_node
```
This should open a pop-up window with an unmoving turtle in the middle of it.

In a 3rd command line window, run
```sh
# set up environment
conda activate robostackenv
cd ~/catkin_ws
source devel/setup.sh

# run the starter code
rosrun turtle_demo turtle_controller.py
```

### Windows

In an anaconda prompt window, run

```sh
# set up environment
conda activate robostackenv
cd ~\catkin_ws
source devel\setup.bat

# run the ros master (server over which the nodes communicate)
roscore
```

This may take a moment, but should give command line output similar to
```
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
started roslaunch server http://127.0.0.1:54062/
ros_comm version 1.15.15


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.15

NODES

auto-starting new master
process[master]: started with pid [68833]
ROS_MASTER_URI=http://http://127.0.0.1:11311/

setting /run_id to d212418c-b9fe-11ed-a83f-fed2a189e5b7
process[rosout-1]: started with pid [68834]
started core service [/rosout]
```

In a separate anaconda prompt window (while ros master continues running in the old one), run
```sh
# set up environment
conda activate robostackenv
cd ~\catkin_ws
source devel\setup.bat

# run the turtlesim
rosrun turtlesim turtlesim_node
```
This should open a pop-up window with an unmoving turtle in the middle of it.

In a 3rd anaconda prompt window, run
```sh
# set up environment
conda activate robostackenv
cd ~\catkin_ws
source deve\setup.bat

# run the starter code
rosrun turtle_demo turtle_controller.py
```

## Functionality

This starter code should send the turtle moving in a random direction, bouncing off walls as needed. Feel free to add whatever functionality you want! If you find yourself out of ideas, here are some potential ones:

- Draw a square

- Draw a spiral

- Move to particular coordinates (randomly-generated, or given as command line input)

The full list of message topics (as well as their types) that the turtlesim nodes publishes and is subscribed to can be found [here](http://wiki.ros.org/turtlesim). Happy coding!

## Setup Instructions

For this workshop, we will be running ROS in a conda environment using robostackenv. [Here](https://robostack.github.io/GettingStarted.html) are instructions on how to install conda, create a conda environment, and install robostackenv into it. You should follow the instructions under "Installation mamba", "Installation ros", and "Installation tools for local development". We will be using ROS1 for this workshop, so you can ignore anything that says it's for ROS2 only.

Once you've got everything installed, you can set up your ROS environment as follows:

### MacOS

In a terminal window, enter the following commands:

```sh
# activating the environment you just created
conda activate ros_env

# creating a file structure for your ROS stuff
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
If those commands complete without throwing errors, your environment should be set up correctly! If you now run `ls`, you should see three directory names; `build`, `devel`, and `src`.

The entire contents of this github repo should be in the `src` directory, inside a directory called `turtle_demo`. You can accomplish this by running
```sh
cd src
git clone https://github.com/Tufts-Robotics-Club/ROS-Workshop-2023.git
mv ROS-Workshop-2023 turtle_demo
```
Once you have the code in the right place, run

```sh
cd ~/catkin_ws
source devel/setup.sh
catkin_make
```

If those commands complete without error, you should be good to go :)

### Windows
First, type "Anaconda" into the start menu. You should see two shell options: one called Anaconda Prompt and another called Anaconda Powershell Prompt. Make sure to pick Anaconda Prompt; the Powershell version, for whatever reason, doesn't work for this particular application.

In this shell, run

```sh
# activating the environment
conda activate ros_env

# creating a file structure for your ROS stuff
mkdir -p ~\catkin_ws\src
cd ~\catkin_ws\
catkin_make
```

If those commands complete without throwing errors, your environment should be set up! If you now run `dir`, you should see three directory names; `build`, `devel`, and `src`.

The entire contents of this Github repo should be put in the `src` directory, inside a directory called `turtle_demo`. If you already have git installed on your computer, you can accomplish this simply by running
```sh
cd src
git clone https://github.com/Tufts-Robotics-Club/ROS-Workshop-2023.git
move ROS-Workshop-2023 turtle_demo
```

If you do not have git installed, scroll to the top of this page and click the green "code" button, and then "Download ZIP" in the dropdown menu. Move the resulting ZIP file into the `~\catkin_ws\src` directory on your computer, unzip it, and rename the resulting directory to `turtle_demo`.

Once you have the code in the right place, run

```sh
cd ~\catkin_ws
call devel\setup.bat
catkin_make
```

If these commands run without issue, you should be good to go!
