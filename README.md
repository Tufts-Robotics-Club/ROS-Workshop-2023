# Tufts Robotics 2023 ROS Workshop Starter Code

Example ROS node for basic movement control in TurtleSim :)

## How to Run
Assuming you've followed the setup instructions sent out prior to the meeting (if not, see [setup instructions](#setup-instructions) below), you should be able to run this code as follows:

First, you should know which shell you are using (bash, zsh, etc). If you don't, run
```sh
# set up environment
conda activate robostackenv
cd ~/catkin_ws
source devel/setup.sh

# run the ros master (server over which the nodes communicate)
roscore
```

This may take a moment, but should give command line output similar to
```bash
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt

started roslaunch server http://Emmas-MacBook-Pro.local:64025/
ros_comm version 1.15.14


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.14
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

This should start the turtle moving in a random direction, bouncing off walls as needed. Feel free to add whatever functionality you want! If you find yourself out of ideas, here are some potential ones:

- Draw a square

- Draw a spiral

- Move to particular coordinates (randomly-generated, or given as command line input)

- Add another turtle & make them bounce off each other

- Add like 10 turtles and make them ALL bounce off each other like a really sadistic aquarium

## Setup Instructions

For this workshop, we will be running ROS in a conda environment using robostackenv. If you don't have anaconda installed on your computer, you should first do that [here](https://docs.conda.io/en/latest/miniconda.html).

Once you have conda installed, run the following commands in the command line (probably Terminal on MacOS, Powershell on Windows, etc):

```sh
# installing all the dependencies into a conda environment (this one will take a while; when it prompts you for input, just put the letter y and hit enter)
conda create -n robostackenv ros-noetic-desktop python=3.9 -c robostack-staging -c conda-forge --no-channel-priority --override-channels

# activating the envornment you just created
conda activate robostackenv

# creating a file structure for your ROS stuff
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
If those commands complete without throwing errors, your environment should be set up! If you now run, 
```sh
ls
```
you should see three directory names; `build`, `devel`, and `src`. The entire contents of this github repo should be in the `src` directory, inside a directory called `turtle_demo`. If you alredy have git installed on your computer, you can accomplish this simply by running
```sh
cd src
git clone https://github.com/Tufts-Robotics-Club/ROS-Workshop-2023.git
mv ROS-Workshop-2023 turtle_demo
```

If you do not have git installed, scroll to the top of this page and click the green "code" button, and then "Download ZIP" in the dropdown menu. Move the resulting ZIP file into the `~/catkin_ws/src` directory on your computer, unzip it, and rename the resulting directory to `turtle_demo`.

Once you have the code in the right place, run

```sh
cd ~/catkin_ws
source devel/setup.sh
catkin_make
```

If those commands complete without error, you should be good to go :)