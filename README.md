# EE346_FinalLab_JefferySherlock

## Description

This project is the final lab in course EE346 ***Mobile Robot Navigation and Control***, based on robot **turtlebot burger**  and computer operating system **ubuntu18.04**.


This project implemented camera calibration, aruco tag detection, automatic navigation, lane following and other functions.

The details are contained in the file ***Final_Report.pdf***. 
        The following tutorial will guide you on how to use these programs.

The main programs are in directory **turtlebot3_autorace2020/turtlebot3_autorace_traffic_light**

## Camera calibration

Calibrating the camera is very important for autonomous driving. The following describes how to simply calibrate the camera step by step.

1. Launch **roscore** on PC

   ```
   roscore
   ```

2. Trigger the camera on turtlebot

   ```
   roslauch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
   ```

3. Execute **rqt_image_view** on PC

   ```
   rqt_image_view
   ```

4. Select **/camera/image/compressed** (or **/camera/image/**) topic on the check box

5. Run a intrinsic camera calibration launch file on PC

   ```
   export AUTO_IN_CALIB=calibration
   export GAZEBO_MODE=false
   roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
   ```

6. Use the checkerboard to calibrate the camera, and click **CALIBRATE**. After calibration, click **Save** to save the intrinsic calibration data.
   
7. **calibrationdata.tar.gz** folder will be created at **/tmp** folder. Extract **calibrationdata.tar.gz** folder, and open **ost.yaml**. Copy and paste the data from **ost.yaml** to **camerav2_320x240_30fps.yaml**.

   

## Aruco code detection

1. Launch **roscore** on PC

   ```
   roscore
   ```

2. Trigger the camera on turtlebot

   ```
   roslauch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
   ```

3.  Run a intrinsic camera calibration launch file on PC

      ```
      export AUTO_IN_CALIB=action
      export GAZEBO_MODE=false
      roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
      ```

4. Run a python file to detect the Aruco code

   ```python
   # if the file can not be executed, please change the permission by
   # chmod u+x speaker.py
   rosrun turtlebot3_autorace_traffic_light_detect speaker.py
   ```



## Navigation



Before starting the program, you need to put turtlebot on P1.



1. Launch **roscore** on PC

   ```
   roscore
   ```

2. If the **bringup** is not running on turtlebot, launch the **bringup** on turtlebot

3. Launch the navigation

   ```
   export TURTLEBOT3_MODEL=burger
   roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
   ```

4. Run a python file to publish goals to turtlebot

   ```
   rosurn turtlebot3_navigation pub_navga_dst.py
   ```




## Lab7 

1. Launch **roscore** on PC

   ```
   roscore
   ```

2. Trigger the camera on turtlebot

   ```
   roslauch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
   ```

3. Run a intrinsic camera calibration launch file on PC

   ```
   export AUTO_IN_CALIB=action
   export GAZEBO_MODE=false
   roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
   ```

4. Launch **Bringup** on turtlebot

5. Run the program to finish "lane following"

   ```
   rosrun turtlebot3_autorace_traffic_light_detect follow_lane_final3.py
   ```

6. Run the program to detect aruco tag

   ```
   rosrun turtlebot3_autorace_traffic_light_detect speaker.py
   ```




## Demo Video

https://www.bilibili.com/video/BV1At4y1p7kJ?share_source=copy_web
