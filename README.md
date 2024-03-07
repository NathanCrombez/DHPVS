# DHPVS: Dual-Hemispherical Photometric Visual Servoing

**DHPVS** is an extension of photometric visual servoing to 360-degree optical rigs
composed of two wide-angle lenses oriented in opposite directions.
The photometric visual feature coupled to dual-hemispherical
acquisitions that contain the whole surrounding scene provide
useful complementary information, showing large convergence domains,
straighter camera trajectories, high accuracy and a high degree of 
robustness to various perturbations.

### Video

[![Alt text](https://img.youtube.com/vi/hoYNN9570LE/0.jpg)](https://www.youtube.com/watch?v=hoYNN9570LE)


```
### Build
```bash
sudo apt install ros-noetic-visp ros-noetic-visp-bridge
cd ~/catkin_ws/src/
git clone https://github.com/NathanCrombez/DHPVS
cd ..
catkin_make
```

### Usage
Your Dual-Hemispherical camera needs to be fully calibrated.
Intrinsic parameters and extrinsic (pose wrt your robot's flange) have to be known for both hemispherical cameras.
* Remap the expected topics in the launch file for: 
  * Your right-camera's images topic
  * Your left-camera's images topic
  * Your right-camera's info topic ¹
  * Your left-camera's info topic ¹
  * Your robot arm topic to set flange velocities
  * Your robot arm topic to get flange pose
* Edit the two static_transform_publishers:
  * flange2rightcamera: robot's flange to right-camera transformation
  * flange2leftcamera: robot's flange to left-camera transformation 

¹ Note that we use the Unified Central projection Model (`João P. Barreto,
A unifying geometric representation for central projection systems,
Computer Vision and Image Understanding, Volume 103, Issue 3`). 
Parameter ξ is expected to be the fifth element of the array D in the sensor_msgs/CameraInfo messages.

Then,
```bash
roslaunch DHPVS DHPVS.launch
```

### Paper

For further details, please see our paper:
```
@article{DHPVS,
  author       = "Nathan Crombez, Jocelyn Buisson, Antoine N. André, Guillaume Caron", 
  title        = "DHPVS: Dual-Hemispherical Photometric Visual Servoing",
  journal      = "IEEE Robotics and Autonomous Letters (RA-L)",
  year         = "2024"
}