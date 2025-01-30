# CMPUT 412 - Duckiebot Autonomous Control

This repository contains the lab exercises and implementations for **CMPUT 412**, where we developed autonomous control solutions for the Duckiebot. The project focuses on structured exercises that enhance the safe and effective utilization of the Duckiebot for autonomous navigation and control.

## **Authors**
This project was collaboratively developed by **Tina Nguyen** and **Haran Song**, who share equal ownership and contributions to the implementation.

## **Project Website**
For additional resources, documentation, and project updates, visit:
**[Tina's Duckiebot Labs](https://tina-nguyen3004.github.io/duckieLabs/)**

---

## **Connecting to the Duckiebot**

### **Identifying the Duckiebot**
The Duckiebot on the network is identified as:
**`csc22919`**

### **Powering On the Duckiebot**
To safely power on the Duckiebot:
1. Press the battery button once.
2. Wait for the Duckiebot’s LEDs, onboard computer LED, fan, and other components to fully initialize.

### **Checking the Duckiebot’s Status**
To confirm the Duckiebot is powered on and ready, run:
```bash
dts fleet discover
```
When the status shows "ready," the Duckiebot is fully operational.

### **Accessing the Duckiebot’s Camera Stream on Your Laptop**
To visualize the live image stream from the Duckiebot's camera:
```bash
dts start_gui_tools csc22919
rqt_image_view
```

### **Accessing the Duckiebot Dashboard**
Once powered on, access the Duckiebot’s dashboard via a web browser at:
```
http://csc22919.local/
```

### **Connecting to the Duckiebot via SSH**
To establish an SSH connection:
```bash
ssh duckie@csc22919.local
```

### **Interacting with ROS Topics**
To view a list of published topics:
```bash
rostopic list
```
To check the publishing frequency of a topic:
```bash
rostopic hz /csc22919/camera_node/image/compressed
```
To display real-time messages from a topic:
```bash
rostopic echo /csc22919/camera_node/image/compressed
```

### **Calibrating the Duckiebot’s Wheels**
To adjust wheel calibration:
```bash
rosparam set csc22919/kinematics_node/trim -0.01325
```

---

## **Running a Docker Program**
We have included a Docker program `hello_world` in this repository. To use it, follow these steps:

1. Navigate to the `hello_world` folder within this repository.

### **On a Local Machine**
2. Build the Docker image:
   ```bash
   dts devel build -f
   ```
3. Run the container:
   ```bash
   dts devel run
   ```

### **On the Duckiebot**
2. Ensure the Duckiebot is accessible:
   ```bash
   ping csc22919.local
   ```
3. Build the image for the Duckiebot’s architecture:
   ```bash
   dts devel build -f --arch arm32v7 -H csc22919.local
   ```
4. Run the container on the Duckiebot:
   ```bash
   docker -H csc22919.local run -it --rm --net=host duckietown/hello_world:v3-arm32v7
   ```

---

## **Shutting Down the Duckiebot**
To safely power off the Duckiebot:
```bash
dts duckiebot shutdown csc22919
```

---

## **Repository Structure**
This repository is structured to ensure clarity and modularity. Each exercise is contained within a dedicated folder for easy navigation and execution of specific tasks.

```
CMPUT412/
│── exercise1/
│   ├── hello_world/
│   ├── color_detector/
│   ├── ...
│── ...
│── LICENSE
│── README.md
```

---

## **Acknowledgments**
We extend our gratitude to the **CMPUT 412** course instructors and teaching staff for their guidance and support throughout this project.
