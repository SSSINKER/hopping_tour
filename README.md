# hopping_tour
 
Source code for Mission 2nd of KABOART 2021 </br>
Enables autonomous maneuvering toward & between specified coordinates using GPS and IMU

Installation Guide
--
```1. ~$ cd catkin_ws/src```</br>
```2. ~/catkin_ws/src$ git clone https://github.com/doyle34/hopping_tour.git```</br>
```3. ~/catkin_Ws/src$ cd ..```</br>
```4. ~/catkin_ws$ catkin_make```</br>
```5. ~$ rosrun hopping_tour {script_name}```</br>

Package Guide
--
_Recommended: use hopping_tour_IMU.py_ </br>

* ```hopping_tour_IMU.py```: Subscribe to ```\imu_data``` and ```\fix``` topic. Calculate current heading angle by IMU
* ```hopping_tour_GPS.py```: Subscribe to ```\fix``` topic only. Calculate current heading angle by GPS coordinate difference between current and previous timestep.
