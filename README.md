```
sudo apt install ros-$ROS_DISTRO-image-tools -y
sudo apt install ros-$ROS_DISTRO-composition -y
```

* check whether image_tools Components exist.

```
$ ros2 component types
...
image_tools
  image_tools::Cam2Image
  image_tools::ShowImage
```

* composition usage1 - ros2cli

```
# terminal 1
ros2 run rclcpp_components component_container
ros2 component list
/ComponentManager

# terminal 2
ros2 component load /ComponentManager image_tools image_tools::Cam2Image

ros2 component unload /ComponentManager 1
```

* composition usage1 - launch file

```
ros2 launch cpp_camera_composition eloquent_img_cvt_composition.launch.py 
```

# Composition vs Node standalone

```
ros2 launch cpp_camera_composition eloquent_camshow_composition.launch.py 
```

```
ros2 launch cpp_camera_composition eloquent_camshow_node.launch.py 
```

### Result

* composition case

CPU Usage (using `top`)
```
composition : CPU: ~40% / Mem: 0.7%

standalone nodes : CPU: 24.4 / Mem: 0.6
```