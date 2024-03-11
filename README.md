# DHPVS: Dual-Hemispherical Photometric Visual Servoing
[Nathan Crombez](http://nathancrombez.free.fr/), [Jocelyn Buisson](https://www.ciad-lab.fr/jocelyn_buisson/), [Antoine N. André](https://antoineandre.github.io/), [Guillaume Caron](https://home.mis.u-picardie.fr/~g-caron), IEEE Robotics and Autonomous Letters (RA-L), 2024

[[Paper]](http://nathancrombez.free.fr/) [[Video]](https://www.youtube.com/watch?v=hoYNN9570LE)

### 1. Description
**DHPVS** is an extension of photometric visual servoing to 360-degree optical rigs
composed of two wide-angle lenses oriented in opposite directions.
The photometric visual feature coupled to dual-hemispherical
acquisitions that contain the whole surrounding scene provide
useful complementary information, showing large convergence domains,
straighter camera trajectories, high accuracy and a high degree of 
robustness to various perturbations.

![alt text](https://raw.githubusercontent.com/NathanCrombez/DHPVS/main/img/DHPVS.gif)

### 3. Build
```bash
sudo apt install ros-noetic-visp ros-noetic-visp-bridge
cd ~/catkin_ws/src/
git clone https://github.com/NathanCrombez/DHPVS
cd ..
catkin_make
```

### 4. Instructions
Your Dual-Hemispherical camera needs to be fully calibrated.
Intrinsic and extrinsic parameters have to be known for both hemispherical cameras.
* Remap the expected topics in the launch file for: 
  * Your right-camera's images topic
  * Your left-camera's images topic
  * Your right-camera's info topic ¹
  * Your left-camera's info topic ¹
  * Your robot arm topic to set flange velocities
  * Your robot arm topic to get flange pose
* Edit the two static_transform_publishers in the launch file:
  * flange2rightcamera: robot's flange to right-camera transformation
  * flange2leftcamera: robot's flange to left-camera transformation 

¹ Note that we use the Unified Central projection Model (`João P. Barreto,
A unifying geometric representation for central projection systems,
Computer Vision and Image Understanding, Volume 103, Issue 3`). 
Parameter ξ is expected to be the fifth element of the array D in the sensor_msgs/CameraInfo messages.

### 5. Usage
When all is set, simply launch: 
```bash
roslaunch DHPVS DHPVS.launch
```

### 6. Citation

For further details, please see our paper and if you find our work usefull, please cite:
```
@article{DHPVS,
  author={Crombez, Nathan and Buisson, Jocelyn and André, Antoine N. and Caron, Guillaume},
  journal={IEEE Robotics and Automation Letters}, 
  title={Dual-Hemispherical Photometric Visual Servoing}, 
  year={2024},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2024.3375114}
}
```

### 7. Acknowledgement
This work was supported by AIST, ITH department international collaboration project [DVS-straight](https://unit.aist.go.jp/jrl-22022/en/projects/project-dvsstraight.html) (R3-I-02, 2021-2025).