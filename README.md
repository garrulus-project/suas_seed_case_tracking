# Seed Case Tracking with Pylon Basler Ace Camera

This ROS2 package contains Basler Ace camera wrapper (SDK and ROS2 wrapper) and seed case tracking interface.

## SDK documentation
* Camera type: [aca720-520uc](https://docs.baslerweb.com/aca720-520uc)
* [pylon C++](https://docs.baslerweb.com/pylonapi/cpp/index)
* Download links
  * https://downloads-bsl.baslerweb.com/software/pylon-8.1.0_linux-x86_64_setup.tar.gz
  * https://downloads-ctf.baslerweb.com/dg51pdwahxgw/3qrLyDYYJDuZFVzGxrhuLq/4e645b6dfd2ac27d0627221d124bfe16/pylon-supplementary-package-for-blaze-1.7.2.f00d43816_x86_64_setup.tar.gz

## Dependencies

** Only `pylon_ros2_camera_interface` is required**

* Clone `pylon-ros-camera` to your ros2_ws/src
  ```
  https://github.com/basler/pylon-ros-camera/tree/jazzy
  ```
* Install dependencies
  ```
  # install pylon ros deps
  sudo apt install ros-jazzy-diagnostic-updater
  sudo apt install ros-jazzy-pcl-ros

  rosdep install --from-paths src --ignore-src -r -y
  ```

## Build

Make sure you have [pylon-ros-camera](https://github.com/basler/pylon-ros-camera/tree/jazzy) cloned in the workspace

```
colcon build --symlink-install --packages-select suas_seed_case_tracking \
        pylon_ros2_camera_interfaces \
        --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

Note: `pylon_ros2_camera_interface` is only required for the ROS service `.srv` definition

## Run

* Launch the node
  ```
  ros2 launch suas_seed_case_tracking seed_case_tracking.launch.py 
  ```

* Trigger camera capture (when sotware triggered is used)
  ```
  ros2 topic pub /basler_camera/basler_wrapper/event_in std_msgs/String "{data: 'e_start'}"
  ```

## Run full pipeline in simulation

* Enable manual trigger via browser 10.10.10.10
* Run mavlink router
  ```
  # run mavlink router 
  mavlink-routerd -e 192.168.1.196:51552 -e 10.10.10.10 -t 0  /dev/ttyACM0:11520
  ```
* Watch trigger counts in terminal
  ```
  # watch trigger
  watch -n 1 'curl -s 10.10.10.10/stats | grep -Eo "\"sprouter_(arm|trigger)_commands_sent\":[0-9]+"'
  ```

## Topics and Services

### Subscribed Topics
* `/basler_camera/basler_wrapper/event_in` (`std_msgs/msg/String`): Software event trigger interface (e.g., send `"{data: 'e_start'}"` to trigger). If even trigger is enabled, the camera thread will not run until it receives `e_start`
* `/mavros/global_position/global` (`sensor_msgs/msg/NavSatFix`)
* `/mavros/imu/data` (`sensor_msgs/msg/Imu`)
* `/mavros/sprouter/trigger-signal` (Custom): Hardware trigger signals from the drone sprouter mechanism.
* `/mavros/sprouter/trigger-timestamp` (Custom): Precise timestamps corresponding to sprouter triggers.
* `/cougar_camera/camera_driver_node/image_raw` (`sensor_msgs/msg/Image`): Wide angle camera

### Published Topics
* `/image_raw` (`sensor_msgs/msg/Image`): Main high-speed image stream produced by the Basler camera.


## Issues and Notes
* FPS are much lower in ROS (issue)[https://github.com/basler/pylon-ros-camera/issues/200], ros spi will impact the rate. It gives 90-100fps for rgb8 and 175 for mono8
* Basler wrapper: ~290-300fps (rgb8), faster if no visualization on rviz and 328 (mono8)
* If `trigger_mode` is set to `on` for software trigger, frame drops to 133fps
* When `TriggerSource` is set to `Line1` cam.TriggerSource.GetValue() witll return `30 (camera register space)`. Use `camera_.TriggerSource.ToString().c_str()` instead
* 

## ToDo
* [x] Trigger: e_start -> setup camera params -> execute capture
* Handle usb unplugged
* [x] Setup servcies for required camera params
* ERROR: camera has been initialized (somewhere in the ros2 pylon wrapper)
* [x] Listen to the following topics:
  ```
  /mavros/global_position/global  sensor_msgs/msg/NavSatFix
  /mavros/imu/data   sensor_msgs/msg/Imu
  /mavros/sprouter/trigger-signal   custom
  /mavros/sprouter/trigger-timestamp  custom

  /cougar_camera/camera_driver_node/image_raw    sensor_msgs/msg/Image
  
  ```
* [] Listen to and save cougar images with the same timestamp as basler (need details).
* [] Add trigger timestamp to the file (need details)
* [] test camera calib file (if it exists)
