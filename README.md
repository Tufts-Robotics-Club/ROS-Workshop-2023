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

- Add another turtle & make them bounce off each other

- Add like 10 turtles and make them ALL bounce off each other like a really sadistic aquarium

## Setup Instructions

For this workshop, we will be running ROS in a conda environment using robostackenv. If you don't have anaconda installed on your computer, you should first do that [here](https://docs.conda.io/en/latest/miniconda.html). The default settings suggested by the installer should be fine.

### MacOS

Once you've installed anaconda, open a terminal window and enter the following commands:

```sh
# installing all the dependencies into a conda environment (this one will take a while; when it prompts you for input, just hit enter)
conda create -n robostackenv ros-noetic-desktop python=3.9 -c robostack-staging -c conda-forge --no-channel-priority --override-channels

# activating the environment you just created
conda activate robostackenv

# creating a file structure for your ROS stuff
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
If those commands complete without throwing errors, your environment should be set up! If you now run `ls`, you should see three directory names; `build`, `devel`, and `src`.

The entire contents of this github repo should be in the `src` directory, inside a directory called `turtle_demo`. If you alredy have git installed on your computer, you can accomplish this simply by running
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

### Windows
**Disclaimer:** For whatever reason, the setup process on Windows is significantly more annoying than the MacOS one. If you have a friend with a Mac who is also coming to the workshop I would highly recommend just sharing with them!

Type "Anaconda" into the start menu. You should see two shell options: one called Anaconda Prompt and antoher called Anaconda Powershell Prompt. Make sure to pick Anaconda Prompt; the powershell version, for whatever reason, doesn't work for this particular application.

Once you've found the correct shell, run the following commands:
```sh
# installing all the dependencies into a conda environment (this one will take a while; when it prompts you for input, just hit enter)
conda create -n robostackenv ros-noetic-desktop python=3.9 -c robostack-staging -c conda-forge --no-channel-priority --override-channels

# activating the environment you just created
conda activate robostackenv
```

Now here comes the annoying part: installing Visual Studio. Note that this is NOT the same as Visual Studio Code, which is a text editor you may have installed for CS11 or 15; Visual Studio is a full IDE (integrated development environment) with C/C++ compiler support that is, for some reason, necessary for the software we're using to run ROS. You can install Visual Studio 2019 (make sure it's the 2019 version!) with C/C++ support [here](https://learn.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=msvc-160); the community version is free.

Once VSCode is installed, return to your command window. Run
```sh
conda install vs2019_win-64
```

to allow your ROS environment to find those C/C++ dependencies. Then, close this window and open a new one (still with the Anaconda Prompt application). In this window, run

```sh
# activating the environment
conda activate robostackenv

# creating a file structure for your ROS stuff
mkdir -p ~\catkin_ws\src
cd ~\catkin_ws\
catkin_make
```

If those commands complete without throwing errors, your environment should be set up! If you now run `dir`, you should see three directory names; `build`, `devel`, and `src`.

The entire contents of this github repo should be put in the `src` directory, inside a directory called `turtle_demo`. If you alredy have git installed on your computer, you can accomplish this simply by running
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
