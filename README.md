# tf-pose-estimation
## Overview
tf-pose-estimationで人間の関節推定を行うパッケージ

## Install
```
$ cd $(ros-workspace)/src/tf-pose-estimation
$ pip install -r tf-pose-estimation/requirements.txt
$ cd ../../
$ catkin_make
```
ただし、GPUを有効する場合は`tensorflow-gpu `を導入する必要あり。  
更に、関節の3D情報を取得する場合は`freenect`とPointCloudライブラリ(PCL)が必須である。

PCLの導入は多分これ↓
```
$ sudo apt-get update
$ sudo apt-get install libpcl-all
```

## Usage
```
roslaunch freenect_launch freenect.launch
roslaunch tfpose_ros kinect.launch  
```

## Node
**`name` tfpose\_ros\_kinect**  
3D情報を送信するノード

### Subscribe Topic

* **`/camera/depth_registered/points`** freenectからの3D情報（ sensor_msgs::PointCloud2 ）

* **`/pose_estimator/pose`** tfpose\_estimationからの平面関節座標情報（ tfpose_ros::Persons ）

### Publish Topic

* **`/tf_pose/kinect_image`** freenectの point cloud から変換した color image ( sensor_msgs::Image )

* **`/pose_estimator/pose_3d`** tfpose\_estimation と freenectのpoint cloud を合わせた立体関節座標情報（ tfpose_ros::Persons ）

## Node
**`name` Visualization**  
関節情報を表示するノード。

## Node
**`name` TfPoseEstimatorROS**  
tfposeとros通信をやり取りするメインノード