#!/usr/bin/env python3


# --- V1X1 & TestRobot Gait Engine --- #

""" 
Description:
TODO

"""

import os
import tempfile
import time
import math
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ikpy.chain import Chain
from std_msgs.msg import Float64



def extract_leg_urdf(original_urdf_path, hip_joint_name):
    tree = ET.parse(original_urdf_path)
    robot = tree.getroot()
    joints = {j.get("name"): j for j in robot.findall("joint")}
    links = {l.get("name"): l for l in robot.findall("link")}
    hip = joints[hip_joint_name]
    hip_parent = hip.find("parent").get("link")
    hip_child = hip.find("child").get("link")
    subtree = set([hip_parent, hip_child])
    stack = [hip_child]
    while stack:
        cur = stack.pop()
        for j in robot.findall("joint"):
            p = j.find("parent").get("link")
            c = j.find("child").get("link")
            if p == cur and c not in subtree:
                subtree.add(c)
                stack.append(c)
    new_robot = ET.Element("robot", {"name": robot.get("name", "robot")})
    for ln in subtree:
        new_robot.append(links[ln])
    for jn, j in joints.items():
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        if p in subtree and c in subtree:
            new_robot.append(j)
    fd, path = tempfile.mkstemp(suffix=".urdf")
    os.close(fd)
    ET.ElementTree(new_robot).write(path, encoding="utf-8", xml_declaration=True)
    return path

class LegGait(Node):
    def __init__(self, name, original_urdf, hip_joint_name, ee_link_name, joint_names, marker_ns, marker_color, side, phase_offset=0, gait_params=None):
        super().__init__(name)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.marker_pub = self.create_publisher(Marker, f"/{marker_ns}", 10)

        # Parameters
        self.declare_parameter("gait_direction", "forward")
        self.add_on_set_parameters_callback(self.param_callback)

        # NEW: rotation angle state + subscriber
        self.rotation_angle = 0.0
        self.rotation_sub = self.create_subscription(
            Float64,
            '/gait_rotation_angle',
            self.rotation_callback,
            10
        )

        self.side = side
        self.joint_names = joint_names
        self.marker_color = marker_color

        # Build per-leg kinematic chain
        temp_urdf = extract_leg_urdf(original_urdf, hip_joint_name)
        self.chain = Chain.from_urdf_file(temp_urdf)
        os.remove(temp_urdf)
        self.chain_link_names = [getattr(l, "name", "") for l in self.chain.links]

        zero = [0.0] * len(self.chain.links)
        self.neutral_z = float(self.chain.forward_kinematics(zero)[2, 3])

        # Gait parameters
        if gait_params is None:
            gait_params = {
                "front": 0.0,
                "back": 0.3,
                "step_height": 0.1,
                "x_lat": 0.35,
                "x_swing": 0.025,
                "stance_ratio": 0.7,
                "points": 30
            }
        self.front = gait_params["front"]
        self.back = gait_params["back"]
        self.stride = self.front + self.back
        self.step_height = gait_params["step_height"]
        self.x_lat = gait_params["x_lat"]
        self.x_swing = gait_params["x_swing"]
        self.stance_ratio = gait_params["stance_ratio"]
        self.points = gait_params["points"]

        # Build trajectory
        lateral = -self.x_lat if side == "L" else self.x_lat
        self.traj = []
        for i in range(self.points):
            phase = i / float(self.points)
            if phase < self.stance_ratio:
                frac = phase / self.stance_ratio
                y = self.front - frac * self.stride
                x = lateral
                z = self.neutral_z
            else:
                frac = (phase - self.stance_ratio) / (1.0 - self.stance_ratio)
                y = -self.back + frac * self.stride
                z = self.neutral_z + math.sin(math.pi * frac) * self.step_height
                if side == "R":
                    x = lateral - math.sin(math.pi * frac) * self.x_swing
                else:
                    x = lateral + math.sin(math.pi * frac) * self.x_swing
            self.traj.append(np.array([x, y, z]))

        self.idx = phase_offset % self.points
        self.ee_path = []

        # Map joints
        self.mapped_indices = []
        for jn in self.joint_names:
            matched = False
            for i, lname in enumerate(self.chain_link_names):
                if lname == jn or lname.lower().replace(" ", "") == jn.lower().replace(" ", ""):
                    self.mapped_indices.append(i)
                    matched = True
                    break
            if not matched:
                raise RuntimeError("Joint name " + jn + " not found in per-leg chain; available: " + str(self.chain_link_names))

        self.joint_signs = [1.0] * len(self.mapped_indices)
        self.timer = self.create_timer(0.01, self.step)
    # NEW: subscriber callback
    def rotation_callback(self, msg):
        self.rotation_angle = msg.data

    def step(self):
        target = self.traj[self.idx]

        # NEW: apply Z-axis rotation
        theta = self.rotation_angle
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])
        rotated_target = rotation_matrix @ target

        # IK on rotated target
        angles = self.chain.inverse_kinematics(
            target_position=rotated_target,
            target_orientation=[[0.0,0.0,0.0],
                                [0.0,0.0,0.0],
                                [0.0,0.0,0.0]],
            orientation_mode="all"
        )

        # Publish joint states
        published = []
        for idx, sign in zip(self.mapped_indices, self.joint_signs):
            if idx < len(angles):
                published.append(float(sign * angles[idx]))
            else:
                published.append(0.0)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = published
        self.joint_pub.publish(msg)

        # Visualization marker
        fk = self.chain.forward_kinematics(angles)
        p = Point(x=float(fk[0,3]), y=float(fk[1,3]), z=float(fk[2,3]))
        self.ee_path.append(p)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ee_paths_" + ("L" if self.side == "L" else "R")
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.marker_color
        marker.points = self.ee_path
        self.marker_pub.publish(marker)

        # Advance trajectory index
        direction = self.get_parameter("gait_direction").get_parameter_value().string_value
        if direction == "forward":
            self.idx = (self.idx + 1) % len(self.traj)
        elif direction == "reverse":
            self.idx = (self.idx - 1) % len(self.traj)

    def param_callback(self, params):
        for param in params:
            if param.name == "gait_direction":
                self.get_logger().info(f"{self.get_name()} switched to {param.value}")
        return SetParametersResult(successful=True)




def main():
    rclpy.init(args=None)
    #urdf_path = "/home/n3z-laptop/ros2_ws/src/v1x1_description/urdf/full_quadruped_robot.urdf.xacro"
    urdf_path = "/home/n3z-laptop/ros2_ws/src/v1x1_description/urdf/v1x1.urdf.xacro"
    rear_gait = {
        "front": -0.04,         # was 0.0
        "back": 0.3,            # was 0.28
        "step_height": 0.04,
        "x_lat": 0.35,          # was 0.35
        "x_swing": 0.02,
        "stance_ratio": 0.5,
        "points": 16
    }
    front_gait = {
        "front": 0.3,           # was 0.3
        "back": -0.04,
        "step_height": 0.04,
        "x_lat": 0.35,          # was 0.35
        "x_swing": 0.02,
        "stance_ratio": 0.5,
        "points": 16
    }
    #TR_left = LegGait("rear_left_leg", urdf_path, "leg1_joint1", "leg1_ee_link", ["leg1_joint1","leg1_joint2","leg1_joint3","leg1_ee_joint"], "ee_path_left", (1.0,1.0,0.0,1.0), "L", phase_offset=0, gait_params=rear_gait)
    #TR_right = LegGait("rear_right_leg", urdf_path, "leg2_joint1", "leg2_ee_link", ["leg2_joint1","leg2_joint2","leg2_joint3","leg2_ee_joint"], "ee_path_right", (0.0,0.0,1.0,1.0), "R", phase_offset=45, gait_params=rear_gait)
    #TR_FWDleft = LegGait("front_left_leg", urdf_path, "leg3_joint1", "leg3_ee_link", ["leg3_joint1","leg3_joint2","leg3_joint3","leg3_ee_joint"], "ee_path_fwd_left", (1.0,1.0,0.0,1.0), "L", phase_offset=45, gait_params=front_gait)
    #TR_FWDright = LegGait("front_right_leg", urdf_path, "leg4_joint1", "leg4_ee_link", ["leg4_joint1","leg4_joint2","leg4_joint3","leg4_ee_joint"], "ee_path_fwd_right", (0.0,0.0,1.0,1.0), "R", phase_offset=0, gait_params=front_gait)
    V1X1_left = LegGait("rear_left_leg", urdf_path, "leg1_joint1", "leg1_ee_link", ["leg1_joint1","leg1_joint2","leg1_joint3","leg1_joint4","leg1_ee_joint"], "ee_path_left", (1.0,1.0,0.0,1.0), "L", phase_offset=0, gait_params=rear_gait)
    V1X1_right = LegGait("rear_right_leg", urdf_path, "leg2_joint1", "leg2_ee_link", ["leg2_joint1","leg2_joint2","leg2_joint3","leg2_joint4","leg2_ee_joint"], "ee_path_right", (0.0,0.0,1.0,1.0), "R", phase_offset=90, gait_params=rear_gait)
    V1X1_FWDleft = LegGait("front_left_leg", urdf_path, "leg3_joint1", "leg3_ee_link", ["leg3_joint1","leg3_joint2","leg3_joint3","leg3_joint4","leg3_ee_joint"], "ee_path_fwd_left", (1.0,1.0,0.0,1.0), "L", phase_offset=135, gait_params=front_gait)
    V1X1_FWDright = LegGait("front_right_leg", urdf_path, "leg4_joint1", "leg4_ee_link", ["leg4_joint1","leg4_joint2","leg4_joint3","leg4_joint4","leg4_ee_joint"], "ee_path_fwd_right", (0.0,0.0,1.0,1.0), "R", phase_offset=45, gait_params=front_gait)   
    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        #executor.add_node(TR_left)
        #executor.add_node(TR_right)
        #executor.add_node(TR_FWDleft)
        #executor.add_node(TR_FWDright)
        executor.add_node(V1X1_left)
        executor.add_node(V1X1_right)
        executor.add_node(V1X1_FWDleft)
        executor.add_node(V1X1_FWDright)
        executor.spin()
    except KeyboardInterrupt:
        pass
    #TR_left.destroy_node()
    #TR_right.destroy_node()
    #TR_FWDleft.destroy_node()
    #TR_FWDright.destroy_node()
    V1X1_left.destroy_node()
    V1X1_right.destroy_node()
    V1X1_FWDleft.destroy_node()
    V1X1_FWDright.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()