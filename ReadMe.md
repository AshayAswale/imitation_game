# Human Motion Imitation

## Dependencies

### Kinect
Install the periferals from [here](https://www.reddit.com/r/ROS/comments/6qejy0/openni_kinect_installation_on_kinetic_indigo/)

NITE is now proprietary, but there are still links to older versions around the internet. I found mine [here](https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23.git). For Ubuntu install x64 (and x86 if your Ubunutu is 32bits)

Download this [openni_tracker](https://github.com/AshayAswale/openni_tracker.git) instead of the ros-drivers one, 
and catkin_make the workspace

If no device found or multiple devices are found while launching openni_launch, try 
```bash
cd ~/kinect/SensorKinect/Platform/Linux/Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh -u # Untill you see, no file exists
sudo ./install.sh
```
If the problem persists, try launching with argument
```bash
roslaunch openni_launch openni.launch device_id:=#2
```

### Eigen 3.3.7
Please download the Eigen 3.3.7 from [here](http://bitbucket.org/eigen/eigen/get/3.3.7.zip)

Rename and copy the subdirectory `Eigen` from the downloaded file to the location as `/usr/include/eigen3/Eigen3.3.7`

## Running the code
Run the following command to launch everything. 
```bash
roslaunch imitation_game imitation_setup.launch
```

## Calibration
#### Tracker
Calibrate the tracker by standing in the _surrender pose_. On completion of the calibration message will read
`Calibration complete, start tracking user`,


#### Razer Controller(Optional)
If you want to use Razer Hydra along with the kinect, First download the archeived `frame_publisher` from commit [f37c46d](https://github.com/AshayAswale/imitation_game/tree/f37c46d6b7fb32aa26c90e0f3bc3b6a83d7d30be/src)

Then uncomment the `hydra.launch` launch file and `frame_publisher` from the `imitation_setup.launch` launch file. 

To get in calibration pose, Stand in _surrender pose_ then rotate the hands about the biceps axis by 90 degree, 
then about elbow such that the arms are pointing away from the body. 

To calibrate long press the [Button 5](https://dl.razerzone.com/master-guides/Hydra/HydraOMG-ENG.pdf) on both the controllers
till you see the `Calibrating!!!` message on console.

