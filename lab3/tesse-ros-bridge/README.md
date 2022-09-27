# tesse-ros-bridge

Provides a ROS interface to the TESSE Unity environment.

## Commands
Use the following commands when having the Unity window in focus:

- Shift+T: disable keyboard input
- w,a,s,d: control agent using forces
- x: stops motion of agent.
- r: respawns agent.
- 'left ctrl + left shift + g': enter spawn point capture mode for next respawn: press 'r' until you get to a nice location, then enter capture mode so that Unity restarts where you left. Note that in this mode, it'll capture a new point every second while moving around. You can stop capturing using 'left ctrl + left shift + g'.
- ESC: to quit game.


### Setup for ROS
To use this interface in ROS, you will need to import the package in the ROS folder into your catkin workspace.

```bash
# Clone repo
cd ~/catkin_ws/src/
git clone git@github.mit.edu:rss/tesse-ros-bridge.git

# Initialize submodules
cd tesse-ros-bridge/
git submodule init
git submodule update --init --recursive

# Install tesse python package
cd tesse-interface
pip install -r requirements.txt
python setup.py develop --user  # If using a virtualenv, ommit `--user`

# Some more dependencies; you should already have them
pip install empy opencv-python PyYAML scipy

# Install dependencies from rosinstall file using wstool
# If this fails, install wstool: sudo apt-get install python-wstool
cd ../..
wstool init
wstool merge tesse-ros-bridge/ROS/install/tesse_ros_bridge.rosinstall
wstool update

# Compile code
cd ../
catkin_make

# Refresh environment
source ~/catkin_ws/devel/setup.bash
```


## Usage

### Basic ROS Bridge

To run the ROS node:
```bash
roslaunch tesse_ros_bridge tesse_bridge.launch
```

### Trajectory Recording 

To record a trajectory:
```bash
roslaunch tesse_ros_bridge record_traj.launch
```
Make sure to change the name of the trajectory in the launch file, and to use the ROS teleop window that is spun up instead of the simulator window for controlling the agent.

### Trajectory Playback and Rosbag Recording

To replay a saved trajectory:
```bash
roslaunch tesse_ros_bridge play_traj.launch
```
Make sure to change the name of the trajectory in the launch file. This is the best way to record repeatable rosbags using different camera or object settings.

For recording rosbags, first launch the simulator executable. In another terminal, launch `roscore`. Then, use the rosbag recording script:
```bash
roscd tesse_ros_bridge && cd scripts
./rosbag_record.bash
```
Then in a separate terminal launch the play script as above. Launching the rosbag record script before the play script is important for getting all the required static transforms.

After recording a rosbag, make sure to restamp the rosbag using the provided script:
```bash
roscd tesse_ros_bridge && cd scripts
./restamp_rosbag.py -i input_rosbag_name.bag -o output_rosbag_name.bag
```
This will set the expected frame rate and imu rate for the bag.

### Visualization

You can use rviz for general visualization, we provide a configuration file:
```bash
cd ~/catkin_ws/src/tesse-ros-bridge/ROS/
rviz -d rviz/tesse_rss.rviz
```

You can also plot IMU and ground-truth odometry from the simulator using `rqt_multiplot` and the config file we provide:
```bash
cd ~/catkin_ws/src/tesse-ros-bridge/ROS/
rqt_multiplot --multiplot-config:=rqt_multiplot/tesse.xml
```

### Additional Scripts

In the [scripts](/ROS/scripts/) directory there are a few extraneous scripts.

- Use [convert_static_tf_csv.py](/ROS/scripts/convert_static_tf_csv.py) to change the reference frame of the stock static TF csv given by the TESSE simulator executable package for each scene. Those csv files are exported from Unity in the Unity reference frame, which is not the same as the ROS right-handed world frame used by the ROS node. Use the `-i` and `-o` at commandline to specify the input and output files respectively. Converted csv files can be used to publish static TFs straight to ROS in the usual way.

- Use [restamp_rosbag.py](/ROS/scripts/restamp_rosbag.py) to fix freshly recorded rosbags, as sepcified in the previous section.

- Use [rosbag_record.bash](/ROS/scripts/rosbag_record.bash) to record relevant topics to a bag.

### FAQ

 Seeing missing `Lidar` global name error? make sure you did not miss git submodule 
