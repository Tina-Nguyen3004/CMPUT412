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