# mobile_manipulation_qp
A robotics project combining the UR5 manipulator, Husky mobile base, and a gripper to perform mobile manipulation using QP-based motion planning and onboard sensing.


# installation
In the `mobile_manipulation_qp` folder, run the following commands to clone the external dependencies
```bash
vcs import < mobile_manipulation.repos
```


At the root of your ROS workspace, run
```bash
sudo apt update && rosdep install --from-paths src -y --ignore-src
```

Build the packages using
```bash
colcon build --merge-install --symlink-install
```

Source your workspace after building
```bash
source install/setup.bash
```

