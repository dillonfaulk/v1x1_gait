--- V1X1_Kinetics Pipeline ---

1.) run command in terminal: ros2 launch urdf_tutorial display.launch.py model:=/home/pimaster/ros2_ws/src/my_robot_description/URDF/URDF/full_quadruped_robot_GZ.urdf.xacro

(this will launch the RSP, JSP_GUI, and RViz2 with the URDF. close the JSP_GUI as it interferes with the gait_engine joint_states.)

2.) run command in another terminal: ros2 run v1x1_gait gait_engine

(this will run the v1x1 Gait Engine Node that generates the entire gait for all 4 legs.)

3.) run command in another terminal: ros2 run v1x1_gait gait_commander

(this will run the v1x1 Gait Commander Node that handles inverted and rotated gait trajectory management.)


Gait Commander Terminal Commands:
    Forward/Reverse Gait:
        ros2 service call /set_direction std_srvs/srv/SetBool "{data: true}"        = inverted gait (backwards)
                            or
        ros2 service call /set_direction std_srvs/srv/SetBool "{data: false}"       =   normal gait (forward)


    Rotated Trajectories:
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.0174533}"    = 1 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.0872665}"    = 5 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.174533}"     = 10 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.261799}"     = 15 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.349066}"     = 20 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.436332}"     = 25 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.523599}"     = 30 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.610865}"     = 35 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.698132}"     = 40 DEG
        ros2 topic pub /gait_rotation_angle std_msgs/msg/Float64 "{data: 0.785398}"     = 45 DEG

4.) Run command in another terminal (after GaitEngine is activated):

ros2 run v1x1_gait stance_commander

(This will run the v1x1 Stance Commander Node that automatically returns the robot to idle position when the gait engine stops.)

**Stance Commander Parameters:**
- `idle_timeout`: Time in seconds before engaging idle stance after gait stops (default: 0.5)

  ros2 run v1x1_gait stance_commander --ros-args -p idle_timeout:=1.0


**Behavior:**
- Monitors gait engine activity on `/joint_states`
- When gait engine stops (timeout exceeded), automatically publishes idle stance (all joints at 0.0)
- Disengages when gait engine becomes active again
- Prevents robot drift when gait is not running
