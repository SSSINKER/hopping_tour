# hopping_tour
 
Source code for Mission 2 of KABOAT 2021 </br>
Enables autonomous maneuvering toward & between specified coordinates using GPS and IMU

Installation & Execution Guide
--
```1. ~$ cd catkin_ws/src```</br>
```2. ~/catkin_ws/src$ git clone https://github.com/doyle34/hopping_tour.git```</br>
```3. ~/catkin_Ws/src$ cd ..```</br>
```4. ~/catkin_ws$ catkin_make```</br>
</br>
To execute, use launch file or run every scripts independently
* Run preceeding topics independently (slow but stable way)</br>
```~$ rosrun nmea_navsat_driver nmea_serial_driver```</br>
```~$ rosrun ebimu_odometry imu_odom_pub_v1.py```</br>
```~$ rosrun rosserial_python serial_node.py```</br>
```~$ rosrun hopping_tour hopping_tour_IMU.py```</br>
* run a launch file (fast but unstable way)</br>
```~$ roslaunch hopping_tour kaboat_rc.launch```</br>


Package Guide
--
_Recommended: use hopping_tour_IMU.py_ </br>

* ```hopping_tour_IMU.py```: Subscribe to ```\imu_data``` and ```\fix``` topic. Calculate current heading angle by IMU
* ```hopping_tour_GPS.py```: Subscribe to ```\fix``` topic only. Calculate current heading angle by GPS coordinate difference between current and previous timestep.
