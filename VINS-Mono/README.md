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

概论里提到基于ROS系统，通过发布和订阅消息的方式实现进程间的通信，通过rosbag或者用户自己定义的节点发布IMU和CAMERA信息。
VINS_MONO 进程分为三个node: 

+ feature_tracker_node: 前端，处理图像特征
+ vins_estimator:      主要进程，负责实现vio
+ pose_graph：		回环检测

三个进程在代码框图中所占的部分如下：

![代码结构](https://github.com/leekaka/github_pics/blob/master/VINS_MONO/%E4%BB%A3%E7%A0%81%E6%A1%86%E6%9E%B6.png?raw=true)

#### 2.1 Feature_tracker_node
该node除了main的主线程之外，只有图像处理一个子线程img_callback(),在回调函数之前,会读取参数,相机模型,鱼眼Mask等等;

图像处理中有个重要的函数，readimag(img,time),进入读取函数后,首先就会对图片进行预处理,转成cv能处理的类型,之后就是调用cv函数对图片进行特征提取,主要用到了两个函数:

+ calcOpticalFlowPyrLK()
+ goodFeaturesToTrack()

一个是光流，一个是特征提取，在第一张图时，需要直接进入特征提取，之后就可以使用光流进行特征跟踪了，当光流跟踪的特征个数不够多时，需要再次进入特征提取器进行补充，这里有很多细节的地方需要注意。



比如：



#### 2.2 Vins_estimator_node

#### 2.3 Pose_graph节点
位姿优化里面除了主线程之外，新起了三个线程：

#### 2.4 Ceres优化库详解

### 3. VINS移植及重要函数推导





