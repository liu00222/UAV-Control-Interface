# ai-touch-interface

This is the GitHubt repository for CRAI Project 7, developing an AI touch interface to control multi-robots system. 

## 1. Description


| Folder Names             |   Contents                            |
| ------------------------ | ------------------------------------- |
| ai-touch-interface       |   Interface                           |
| collector                |   An app to collect gesture data      |
| recognizer               |   APIs to train the recognizer        |
| server                   |   Some ROS server nodes serves as a bridge between the simulator and the interface; an offline server that is independent to ROS; also some ROS related contents I used   |

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

1. Add the team name under the Signing & Capability section. 

2. Make sure that the iOS deployment target under the Build Setting section is iOS 9.0 or above. 

Now, you can build the project and run it. The XCode built-in iOS devide simulators are relatively slow, so I would recommend you to use your own iPad. Mine is iPad Air 3rd generation, so I designed the UI based on its sreen size. If you are running on other machines, it may be somewhat creepy. I will adjust this later. 

If you do not know how to run it on your own iPad, you can check [Apple's official document](https://developer.apple.com/documentation/xcode/running_your_app_in_the_simulator_or_on_a_device) or contact me. 

## 3. ROS and Simulator

                   
