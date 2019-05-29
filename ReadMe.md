# Human Motion Imitation

## Dependencies
Install the periferals from [here](https://www.reddit.com/r/ROS/comments/6qejy0/openni_kinect_installation_on_kinetic_indigo/)

Download this [openni_tracker](https://github.com/AshayAswale/openni_tracker.git) instead of the ros-drivers one, 
and catkin_make the workspace

If no device found or something like that while launching openni_launch, try 
```bash
roslaunch openni_launch openni.launch device_id:=#2
```
But this will impact other packages. So uninstall the install.sh using 
```bash
cd ~/kinect/SensorKinect/Platform/Linux/Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh -u # Untill you see, no file exists
sudo ./install.sh
```

## Running the code
Run the following command to launch everything. 
```bash
roslaunch human_motion_imitation imitation_setup.launch
```

## Calibration
#### Tracker
Calibrate the tracker by standing in the _surrender pose_. On completion of the calibration message will read
`Calibration complete, start tracking user`,

#### Razer Controller
To get in calibration pose, Stand in _surrender pose_ then rotate the hands about the biceps axis by 90 degree, 
then about elbow such that the arms are pointing away from the body. 

To calibrate long press the [Button 5](https://dl.razerzone.com/master-guides/Hydra/HydraOMG-ENG.pdf) on both the controllers
till you see the `Calibrating!!!` message on console.

