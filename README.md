# ai-touch-interface

This is the GitHubt repository for CRAI Project 7, developing an AI touch interface to control multi-robots system. The readme here consists of instructions on how to prepare the environment for the project and run the app and simulation. 

I do think there are other things you need to know, like the high-level task scenarios in the simulation and task expectation. I will send you separate emails on those topics. Thank you.

## 1. Description


| Folder Names             |   Contents                            |
| ------------------------ | ------------------------------------- |
| ai-touch-interface       |   Interface                           |
| collector                |   An app to collect gesture data      |
| recognizer               |   APIs to train the recognizer        |
| server                   |   Some ROS server nodes serves as a bridge between the simulator and the interface; an offline server that is independent to ROS; also some ROS related contents I used   |
| pictures                 |   Some screenshots used to illustrate the readme file        |

## 2. AI Touch Interface

### Preparation

Operating System: macOS

Python version: 3.7.4

Packages: [kivy](https://kivy.org/#home), [numpy](https://numpy.org/)

```
# Installation
$ pip3 install kivy
$ pip3 install numpy
```

### Run the app on computers

Before running the program, make sure that you: 

1. Run a server first. The server can be found in the "server" folder. The best one to begin is the "offline_server.py". You can run it anywhere even without ROS. Before running the offline server, make sure to adjust its IP address and the port number. 

2. Adjust the IP address and the port number in the "main.py" in "ai-touch-interface/dev/" to fit the server. 

Then, type the following commands to the terminal:

```
$ cd ../ai-touch-interface/dev
$ python3 main.py
```

The interface should pop out. The interface should also print something to show that the connection is established. 

### Run the app on iPad

You can start at any folder you like. Many of the following steps come from https://github.com/kivy/kivy-ios. I will adjust them to avoid potential problems later when we publish the works. 

Before we start, we strongly advise to use a Python virtual environment to install Python packages.

```
$ python3 -m venv venv
```

Then, install [kivy-ios](https://github.com/kivy/kivy-ios) which serves to convert the project into an iOS app: 

```
$ pip3 install kivy-ios
```

Additionally you would need few system dependencies and configuration. Xcode 10 or above, with an iOS SDK and command line tools installed:

```
$ xcode-select --install
```

Using brew, you can install the following dependencies:

```
$ brew install autoconf automake libtool pkg-config
$ brew link libtool
```

Now, we can start to build the recipe. It will take a long time (around 30 mins) to complete. Start the compilation by: 

```
$ toolchain build python3 numpy kivy
$ toolchain pip3 install pure-predict
```

Then, build the XCode project: 

```
toolchain create proj_name path_to_proj_main.py
```

For example, it looks like the following on my laptop: 

```
toolchain create AI-Interface ~/Desktop/work/CRAI/dev
```

Then, open the newly created project file which is proj_name + "-ios", and open the .xcodeproj file. Before you can actually build and run the XCode project, you need to do two things: 

1. Add the team name under the Signing & Capability section. A screenshot may give you some hints: 

![picture1](https://github.com/liu00222/ai-touch-interface/blob/master/pictures/picture1.png)

2. Make sure that the iOS deployment target under the Build Setting section is iOS 9.0 or above. You can also refer to the screenshot below to get some ideas:

![picture1](https://github.com/liu00222/ai-touch-interface/blob/master/pictures/picture2.png)

Now, you can build the project and run it. The XCode built-in iOS devide simulators are relatively slow, so I would recommend you to use your own iPad. Mine is iPad Air 3rd generation, so I designed the UI based on its sreen size. If you are running on other machines, it may be somewhat creepy. I will adjust this later. 

If you do not know how to run it on your own iPad, you can check [Apple's official document](https://developer.apple.com/documentation/xcode/running_your_app_in_the_simulator_or_on_a_device) or contact me. 

Note that now you may use the offline server to connect to the interface. Later you may go to ROS and you can open the server node there to receive messages from the interface. After you have some knowledge in ROS and Gazebo (the simulator), you will know how to start the server there. 

## 3. ROS and the Simulator

### Preparation before starting everything

Operating System: Ubuntu 16.04[https://releases.ubuntu.com/16.04/?_ga=2.73618645.615752798.1598417073-1774150147.1598417073]

ROS version: Kinetic Kame

Gazebo version: Gazebo7

Note that these three versions are strictly fit. The next generation of this bundle is Ubuntu 18.04 + ROS melodic + Gazebo9. Make sure that the versions of these three match each other, otherwise you will suffer from the same pain as I had 2 weeks ago, keeping re-installing everything and begging them to work. 

The detailed documentation about how to install ROS can be found on their [offcial website](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). You don't need to install Gazebo on purpose. It will come with the installation of ROS. Once you install ROS successfully, try this to make sure that Gazebo is there and its version is compatible with your ROS version: 

```
$ gazebo --version
```

The requirements above are just the necessary environment for my **current works in ROS**. If you feel like a newer version would be better for you to complete the work, feel free to move towards that. 

### Introduction to ROS

I highly recommend you to go through the [Beginner's level of ROS's official tutorial](http://wiki.ros.org/ROS/Tutorials) first. Don't overlook it from its tag, "begginer". The key things to use ROS to control robots in the simulator are inside of the beginner's level of ROS. The worst thing of ROS is that there is no enough high-quality outside tutorial of it (at least I did not find any), so it seems that the only choice to get into it may be reading its docs. 

### Preparation before running ROS and the Simulator

The world settings and the robots models are from outside sources. I currently use the [RotorS Simulator](https://github.com/ethz-asl/rotors_simulator), which is a work in Gazebo. You can follow their instruction to install the simulator. RotorS was developed by researchers in ETH and they have several publication around it, so we will need to cite their works when we step into the paper-wirting part. 

If you fail to install RotorS, don't feel sad as I also failed several weeks ago. You can go to [BeboS](https://github.com/gsilano/BebopS). BebopS is a helper package to help you install RotorS simulator. As long as you follow BebopS's instruction of installation and succeed, you will suprisingly find that the packges of RotorS simulator are also on your computer. Now you can launch RotorS simulator by the commands introduced in the GitHub page of RotorS simulator. 

### Run the Demo that I showed during our first meeting

You will notice that "rotors_gazebo" is a package of [RotorS Simulator](https://github.com/ethz-asl/rotors_simulator). I made some changes to several files in the package in order to make it fit our project. The changed files are in "server/rotors_gazebo". You can copy the files on our repository to replace the files in their package. The changed (or newly added) files are "CMakeLists.txt", 7 C++ sources in "src", and 2 launch files in "launch". 

For the server node, you can create a new package with any name in your catkin workspace. Then create a folder called "scripts" in that new package. Then you just copy and paste the "touch_interface_server.py" and "touch_interface_server_swarm.py" into the "scripts" folder. I use Python to write the server, because using Python means you don't need to run "catkin build" again to build the project, which will break down your simulation. If you feel using C++ is better, please feel free to move to that. Just remember to let me know before it. 

To run my current simple tiny simulation, open two terminals. After building the catkin project, in the first terminal, type and run the command: 

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

The Gazebo window will pop out with one drone in it. Then, in the second terminal, type and run the command: 

```
$ python src/your_package_name/scripts/touch_interface_server.py
```

For example, I put the server into the package "beginner_tutorials". So the command would be: 

```
$ python src/beginner_tutorials/scripts/touch_interface_server.py
```

As long as the server starts, you can send commands from your hand gestures to the simulation. 

To run the simulation with multiple robots, you can just do the following. In the first terminal: 

```
$ roslaunch rotors_gazebo crai_firefly_swarm.launch mav_name:=firefly world_name:=basic
```

Then, in the second terminal, run the corresponding server: 

```
$ python src/your_package_name/scripts/touch_interface_server_swarm.py
```

After the simulation and the server start successfully, you can open the interface again to play with the demo, as the video I showed you in our first meeting on Friday. 
