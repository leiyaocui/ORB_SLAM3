from tqdm import tqdm
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import numpy as np
import threading
from sensor_msgs.msg import JointState

from os import path
from threading import Thread
from time import sleep

rclpy.init()

# Create node for this example
node = Node("husky_commander")

pos_node = rclpy.create_node('position_velocity_publisher')
pub = pos_node.create_publisher(JointState, 'joint_commands', 10)

gripper_pose = JointState()

gripper_pose.name = [
    "base_y_base_x",
    "base_theta_base_y",
    "base_link_base_theta",
]

traj = [
    [ 0.0, 0.0, 0.0 ],
    [ -1.5, 0.0, 0.0 ],
    [ -2.0, -0.5, np.pi / 2 ],
    [ -2.0, -4.0, np.pi / 2 ],
    [ -2.0, -4.0, np.pi ],
    [ 1.0, -4.0, np.pi ],
    [ 1.0, -4.0, np.pi / 2 ],
    [ 0.0, -3.5, np.pi / 2 ],
    [ -0.5, -2.5, np.pi ],
    [ -0.5, -0.5, np.pi * 3 / 2 ],
    [ -2.0, -0.5, np.pi * 3 / 2 ],
    [ -2.5, -0.5, np.pi ],
    [ 1.0, -0.5, np.pi / 2  ],
    [ 0.0, 0.0, 0.0 ],
    [ -1.0, 0.0, 0.0 ],
]

def execute_q_traj(tran, interp=0, t_sleep=0.75):
    tran = np.asarray(tran)
    if interp > 0:
        # Interpolate `tran` (N x q_len) to smooth it
        q_interp = []
        for i in range(len(tran) - 1):
            q_interp.append(tran[i])
            for j in range(1, interp):
                q_interp.append(tran[i] * (1.0 - float(j) / interp) + tran[i+1] * float(j) / interp)
        q_interp.append(tran[-1])
        tran = q_interp 

    for q in tqdm(tran):
        gripper_pose.position = np.array(q, dtype=float).tolist()
        pub.publish(gripper_pose)
        sleep(t_sleep)

thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()
rate = node.create_rate(2)

while not rclpy.ok():
    rate.sleep()

current_state = traj[0]

for t, q in enumerate(traj[1:]):
    print(q)
    execute_q_traj([ current_state, q ], interp=100, t_sleep=0.2)
    current_state = q

rclpy.shutdown()
exit(0)
