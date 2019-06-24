# VINS-Mono
## A Robust and Versatile Monocular Visual-Inertial State Estimator

## VINS-Mono单目代码详情
### 1.概论
**VINS-Mono源于香港科技大学沈邵劼课题组的开源杰作。相关论文：**

  • 	VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen arXiv:1708.03852
  
  •	  Autonomous Aerial Navigation Using Monocular Visual-Inertial Fusion, Yi Lin, Fei Gao, Tong Qin, Wenliang Gao, Tianbo Liu, William Wu, Zhenfei Yang, Shaojie Shen, J Field Robotics. 2017;00:1–29. https://doi.org/10.1002/rob.21732
  
  
开源代码地址：
Linux端：https://github.com/HKUST-Aerial-Robotics/VINS-Mono

Mobile phone端：https://github.com/HKUST-Aerial-Robotics/VINS-Mobile

Linux端程序基于ROS系统，通过发布和订阅消息的方式实现进程间的通信。通过rosbag或者用户自己定义的节点发布IMU和CAMERA信息
官方建议CAMERA帧率超过20Hz，IMU超过100Hz。

建议大家先熟悉一下ROS系统，读官方的教程就可以，基本上读完“核心ROS教程部分”就可以对ROS有基本的了解，
可以解决大部分问题，内容不多，很快就可以读完，链接如下：wiki.ros.org/cn/ROS/Tutorials/

### 2.代码详解
**代码结构图**



