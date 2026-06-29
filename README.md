编译会遇到的问题


一、没有mid360的驱动，需要我们自己安装
https://gitcode.com/gh_mirrors/li/livox_ros_driver2?utm_source=csdn_github_accelerator&isLogin=1&from_link=169f004da7ba7757dc3b16e41aa88eee
这是国内gitcode的livox_ros_driver2分支
https://gitcode.com/gh_mirrors/li/Livox-SDK2?utm_source=csdn_github_accelerator&isLogin=1&from_link=e83667b0c2e34ce12924eb6fcbf2a5fc
这是国内gitcode的livox_sdk2分支
根据readme来进行安装
可以自己建一个工作空间来存放雷达驱动


二、缺少quadrotor_msgs

<img width="476" height="361" alt="image" src="https://github.com/user-attachments/assets/9207ab64-005c-45cd-83ac-b8adc04095ee" />

可以从fast_drone_250获取，https://github.com/ZJU-FAST-Lab/Fast-Drone-250，src/utils/quadrotor_msgs


三、
<img width="1012" height="215" alt="image" src="https://github.com/user-attachments/assets/a33e781a-06b7-47b0-8761-e800ab8485a6" />
这个问题显然是没有找到livox_ros_driver2，我们这里需要source一下我们之前安装的livox_ros_driver2的这个工作空间
source ~/livox_ws/devel/setup.bash


四、
<img width="953" height="218" alt="image" src="https://github.com/user-attachments/assets/1f024064-820b-437c-a6fb-ee5d262a4184" />
由于我使用的是livox_ros_driver2，他源码中driver和driver2都有 我就直接给driver删掉，如下图
<img width="362" height="124" alt="image" src="https://github.com/user-attachments/assets/6f96f08b-bc0a-43d2-bf78-d70890bbad06" />


五、
<img width="578" height="83" alt="image" src="https://github.com/user-attachments/assets/dd6d9d1a-2e82-44f3-a38c-d498133939b4" />
这个是没有定位到包
修改fastlio/CMakelist下的内容，找到find_package，在末尾加入genmsg
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  message_generation
  eigen_conversions
  genmsg
)
同时在结尾加入add_dependencies(fastlio_mapping fast_lio_generate_messages_cpp)


六、
<img width="578" height="64" alt="image" src="https://github.com/user-attachments/assets/bfffa286-6f49-4d69-baca-ae8dae0ba030" />
这个是在代码中的包用了driver，应该用driver2，把代码改一下，可以参考https://blog.csdn.net/qq_16775293/article/details/132408005


