This repo has launch files to start gazebo simulation with moveit. It also has an [example code](https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html) to move the robot end-effector to a set goal point.

## Prerequisites
Install franka_ros2 - humble.

## Using this package

After successful installation clone this repo and place it in `src` folder.

```
git clone https://github.com/harshaguda/franka_gazebo_moveit.git
```

Due to some hardcoding of getting `franka_gazebo_controllers.yaml` (TODO: Fix this), the file in `src/franka_gazebo_moveit/config/franka_gazebo_controllers.yaml` needs to be replaced with `src/franka_gazebo_moveit/config/franka_gazebo_controllers.yaml` else the fr3_controllers will not be properly loaded, even though planning happens, the robot will not move.


Build the project,
```
colcon build --packages-select franka_gazebo_moveit franka_gazebo_bringup
```
In one terminal run the following launch file,
```
ros2 launch franka_gazebo_moveit gazebo_franka_moveit.launch.py load_gripper:=true franka_hand:='franka_hand' use_fake_hardware:=true
```

In another,

```
ros2 launch franka_gazebo_moveit hello_moveit.launch.py 
```

This moves the robot to a set goal pose.

Future improvements
- Get the goal from a topic.

## Trouble shooting
Replace 
```xml
<xacro:if value="${gazebo and hand}">
  <xacro:configure_joint joint_name="${arm_id}_finger_joint1" initial_position="0.0" />
</xacro:if>
```
with 

```xml
<xacro:if value="${(gazebo or use_fake_hardware) and hand}">
  <xacro:configure_joint joint_name="${arm_id}_finger_joint1" initial_position="0.0" />
</xacro:if>
```

If you get base link error change this line as following or similar
```shell
Error: Virtual joint does not attach to a link on the robot (link 'base' is not known)
```

```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base"/>
```
to
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="${prefix}link0"/>
```