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

Python version: 3.7.4

Packages: [kivy](https://kivy.org/#home), [numpy](https://numpy.org/)

```
# Installation
pip3 install kivy
pip3 install numpy
```

### Run the program on computers

Before running the program: 

1. Run a server first. The server can be found in the "server" folder. The best one to begin is the "offline_server.py". You can run it anywhere even without ROS. Before running the offline server, make sure to adjust its IP address and the port number. 

2. Adjust the IP address and the port number in the "main.py" in "ai-touch-interface/dev/" to fit the server. 

## 3. ROS and Simulator

                   
