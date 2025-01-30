# CMPUT 412 - Duckiebot Autonomous Control

This repository contains our lab exercises code developed for CMPUT 412, where we implemented autonomous control for our Duckiebot. The project includes step-by-step instructions for utilizing the Duckiebot safely and effectively. Each exercise is organized into separate folders for clarity and ease of use.

## Authors
This project was developed by **Tina Nguyen** and **Haran Song**, who share equal ownership of the code and have collaborated to build this implementation.

## Website
For additional resources and project updates, please visit our website:
[Tina's Website](https://tina-nguyen3004.github.io/duckieLabs/)

---

## Connecting to the Duckiebot
### Identifying the Duckiebot
The Duckiebot available on the network is identified by the name: **`csc22919`**.

### Powering On the Duckiebot
To safely power on the Duckiebot, follow these steps:
1. Press the battery button once.
2. Wait for the Duckiebot's LEDs, the on-board computer LED, the fan, and other components to turn on fully.

### Checking if the Duckiebot is On
To verify that the Duckiebot is powered on and ready, run the following command:
```
dts fleet discover
```
Once the status indicates "ready" the Duckiebot is prepared for maneuvering.


### Viewing the Image Stream on Your Laptop
To see the Duckiebot's camera feed on your laptop, use the following command:
```
dts start_gui_tools csc22919
```
Then, run:
```
rqt_image_view
```
This will allow you to visualize the live image stream from the Duckiebot's camera.


### Accessing the Duckiebot Dashboard
Once the Duckiebot is powered on, you can access its dashboard through a web browser using the following URL:
```
http://csc22919.local/
```

### Connecting via SSH
To establish an SSH connection with the Duckiebot, use the following command:
```
ssh duckie@csc22919.local
```
List topics
You can see a list of published topics with the command:

rostopic list

Show topics frequency
You can use rostopic hz to see the statistics about the publishing frequency:

rostopic hz /csc22919/camera_node/image/compressed

Show topics data
You can view the messages in real time with the command rostopic echo:

rostopic echo /csc22919/camera_node/image/compressed

Calibrate the wheels
`rosparam set csc22919/kinematics_node/trim -0.01325`


### Run a basic Docker program: 
You can go inside the foler hello_world folder and run 
- On my computer:
`dts devel build -f` to build the image
`dts devel run` 
- On my duckiebot:
`ping csc22919.local` to ensure that we have connection 
`dts devel build -f --arch arm32v7 -H csc22919.local`
`docker -H csc22919.local run -it --rm --net=host duckietown/hello_world:v3-arm32v7`



### Shutting Down the Duckiebot
To safely turn off the Duckiebot, run the following command:
```
dts duckiebot shutdown csc22919
```

---

## Repository Structure
Each exercise is organized into a dedicated folder to facilitate navigation and execution of specific tasks. The structure ensures modularity and ease of development.

---

## Acknowledgments
We would like to thank the **CMPUT 412** course instructors and teaching staff for their guidance and support in developing this project.