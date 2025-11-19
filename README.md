# CPS Final Project

## Install F1tenth simulator

1. First, create a virtual environment, as this code relies on older versions of common packages (e.g. Numpy)
2. `python3 -m venv ~/.venvs/f1_gym_env`
3. `echo "alias f1_env='source ~/.venvs/f1_gym_env/bin/activate'" >> ~/.bashrc`
4. `source ~/.bashrc`

```
python3 -m venv ~/.venvs/f1_gym_env  
echo "alias f1_env='source ~/.venvs/f1_gym_env/bin/activate'" >> ~/.bashrc
source ~/.bashrc
```

You should now have a virtual environment that can be activated with the command `f1_env`. Invoke that command now, and look for the parenthetical at the command line that indicates it is active.

With the *virtual environment active*, 
1. `git clone https://github.com/f1tenth/f1tenth_gym ~/Downloads/f1tenth_gym/`
2. `cd ~/Downloads/f1tenth_gym && git checkout dev-dynamics`
3. `pip3 install -e`
4. `pip3 install transforms3d`

It is recommended to run each of these lines one-by-one.
```
git clone https://github.com/f1tenth/f1tenth_gym ~/Downloads/f1tenth_gym/
cd ~/Downloads/f1tenth_gym && git checkout dev-dynamics
pip3 install -e
pip3 install transforms3d
```


Step 2 ensures we are using a version that is compatible with Humble. Step 4 ensures that this library gets installed (it does not, natively, but it is necessary later).

## Installing F1tenth ROS bridge
1. `cd ~ && mkdir -p f1sim_ws/src`
2. `sudo apt-get update`
3. `sudo apt-get install python3-rosdep`
4. `cd ~/f1sim_ws/src`
5. `git clone https://github.com/burg54/f1tenth_gym_ros.git`
6. `source /opt/ros/humble/setup.bash`
7. `cd ..`
8. `sudo rosdep update`
9. `rosdep update`
10. `rosdep install -i --from-path src --rosdistro humble -y`

All in one place (again, however, recommended to implement line-by-line), this is:
```
cd ~ && mkdir -p f1sim_ws/src
sudo apt-get update
sudo apt-get install python3-rosdep
cd ~/f1sim_ws/src
git clone https://github.com/burg54/f1tenth_gym_ros.git
source /opt/ros/humble/setup.bash
cd ..
sudo rosdep update
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

## Building the Workspace
1. `cd ~/f1sim_ws`
2. `colcon build`
3. `source install/local_setup.bash`


## Running and testing the simulator
If everything has been built correctly and sourced (and with the venv activated), run

`ros2 launch f1tenth_gym_ros gym_bridge_launch.launch`

It may take a moment for the sensors and vehicle model to fire up properly in `RViz`. For example, `LaserScan` may have a warning and the car is a blank white. Wait several moments, and the car should populate with a primitive rendering, and the Lidar scans should show up as a red-to-green colormap along the track walls.

Then, in a new tab, run
```
source install/local_setup.bash
ros2 run f1tenth_gym_ros follow_gap
```

The car should begin moving forward and then ride off into the distance. You are now ready to begin designing your own follow-the-gap algorithms.

`control+c` out of the `follow_gap` node should gracefully kill the algorithm (although the car will keep moving). If you want to stop the car from moving after killing the follow-the-gap algorithm, you can run the `stop_car` node.

Then, x-ing out of RVIZ and then `control+c` from the launch terminal should gracefully kill the entire simulation. 





