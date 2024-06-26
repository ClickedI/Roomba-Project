# **Effective Robotic Cleaning:** Wall-Following with JPS Pathfinding

## Objectives

Our main goal is to optimize the coverage area of traditional vacuuming robots. We do so by combining a wall-following algorithm with Jump Point Search that allows for total room coverage.

![JPS+ in action](https://media.springernature.com/lw685/springer-static/image/chp%3A10.1007%2F978-981-99-0479-2_267/MediaObjects/539027_1_En_267_Fig10_HTML.png)

## Getting Started
### Hardware
To properly run our program you will need the following:
- Create3 educational robot - designed to replicate the iRobot roomba
- Raspberry Pi 4 - a tiny computer equipped with [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
- RPLiDAR - LiDAR (light detection and ranging) scanner for mapping
- Ethernet/USB to USB-C [adapter](https://www.amazon.com/Ethernet-Adapter-uni-Gigabit-Compatible/dp/B0871ZL9TG/ref=asc_df_B0871ZL9TG/?tag=hyprod-20&linkCode=df0&hvadid=693310954762&hvpos=&hvnetw=g&hvrand=16954073619488731607&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9001843&hvtargid=pla-1123808946192&mcid=ab8721b29d5033a99494d584b3d2629a&gad_source=1&gclid=CjwKCAjw9IayBhBJEiwAVuc3fv7x_C3fqsNCKoSJi1NQ9luoM_0g8-K-M9aF9xyh8DQZ1dmS7RkLcRoCUT0QAvD_BwE&th=1)
- External computer used to SSH into the Raspberry Pi

### Setup & Software
Begin by plugging a mouse and keyboard into your Raspberry Pi 4 and connect it to a monitor using a micro-usb to HDMI cable and connect to the internet. Open a terminal and run:
```
ifconfig
```
Under **wlan** look for the ip address labeled inet and record it somewhere. This ip will vary depending on what network you are on so it is important to keep track of it as you need it to SSH into the Raspberry Pi.
![ip address location](https://www.mbtechworks.com/wp-uploads/ifconfig-1.jpg)
>[!WARNING]
>Make sure to enable SSH in the Interfaces tab under Raspberry Pi Configuration in Preferences more detailed description [here](https://www.onlogic.com/blog/how-to-ssh-into-raspberry-pi/).

***Install Ros2***  
Step 1 is installing the Robot Operating System 2 Humble software whose documentation and installation instructions can be found [here](https://docs.ros.org/en/humble/Installation.html).  
![Ros2 Humble](https://docs.ros.org/en/humble/_static/humble-small.png)  

***Install SLAM Toolbox***  
We suggest watching [this](https://www.youtube.com/watch?v=ZaiA3hWaRzE&t=504s) great video from Articulated Robotics before doing the next 2 steps (replacing foxy with humble of course).  
Install Slam toolbox using the command:  
```
sudo apt install ros-humble-slam-toolbox
``` 
The documentation and instructions on using SLAM Toolbox in tandem with the LiDAR scanner can be found [here](https://github.com/iRobotEducation/create3_examples/tree/humble/create3_lidar_slam).  

***Install Navigation 2***  
Install the NAV2 packages using:
```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
Then install the tool twist-mux
```
sudo apt install ros-humble-twist-mux
```
The Twist-mux tool multiplexes multiple twist topics into one singular output.
>[!IMPORTANT]
>These steps should also be followed on your external computer as you will need to run Rviz (robot visualiser) on your computer as well.
  
Create a new directory:
```
mkdir create3_ws
```
Enter the directory by running:
```
cd create3_ws
```
and run:
```
git clone https://github.com/ClickedI/Roomba-Project.git
```
The last step is to build and source:
```
colcon build --symlink-install
source ~/create3_ws/install/setup.bash
```

### Connecting Everything
-Begin by mounting the LiDAR atop the Create 3 robot.  
-In the rear of the robot you can remove the back panel revealing the cargo bay, on the bottom of this panel you will mount the Raspberry Pi.  
-Once everything is securely mounted plug the lidar into one of the Pi's USB ports.  
-Then plug the adapter into the robot's USB-C port (located towards the top of the cargo bay) followed by connecting the Pi to the adapter using a *small* ethernet cable.  
-Finish by reinserting the cargo bay panel and everything is hooked up!  

### Running the code
Power on the Create3 Robot by connecting it to its charging dock and follow [these steps](https://edu.irobot.com/create3-setup) to connect it to your network. Once completed on your external computer SSH into the Raspberry Pi using the command:
```
ssh [username]@[IP Address of Rasp Pi]
```
>[!NOTE]
>If using Widnows make sure you are using an administrative Powershell whose documentation can be found [here](https://learn.microsoft.com/en-us/powershell/scripting/install/installing-powershell-on-windows?view=powershell-7.4).  

Navigate to the directory you saved and unzipped create3.py and run it using the command:  
```
ros2 create3.py
```
This command starts Rviz allowing us to visualize the heat map generated by the LiDAR and for our code to run. Open another tab in your terminal, navigate to the directory you saved the git clone to and run:
```
python3 Lidar.py
```

Lidar.py is a program that executes a basic wall follow using a PID controller.

To view JPS and A* examples simply run the JPS and A* python files using the commands
```
python3 JPS.py
python3 astar.py
```
>[!CAUTION]
>Complete implimentation of the JPS+ and A* algorithms using the robot was nearly impossible given the resources available but with more finetuning could vastly improve upon cleaning area.
