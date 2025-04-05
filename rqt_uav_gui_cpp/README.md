# The updated GUI

## HOW TO INSTALL:

It is considered that you have already installed the [previous version of the GUI](./../GUI).

If not, follow the installing steps from there.

## HOW TO RUN:

1. Build your catkin workspace

```shell
cd <your_workspace>
catkin_make
```

2. In the same terminal type:

```shell
roslaunch px4 mavros_posix_sitl.launch
```

OR

```shell
roslaunch px4 px4_agriculture.launch
```

3. In a new terminal type:

```shell
rqt
```

4. In RQT open:

> *Plugins → Swarm GUI → GUI 2.0*

5. In a new terminal type:

```shell
rosrun px4_sim nik_teleop
```

The motors of the UAV in Gazebo will automatically turn on. Now you can control the UAV.

