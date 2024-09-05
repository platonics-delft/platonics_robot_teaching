import numpy as np
from geometry_msgs.msg import PoseStamped

def square_exp(pose_i : PoseStamped, pose_j : PoseStamped, length_scale: float) -> float:
    dist_norm_squared =(pose_i.pose.position.x - pose_j.pose.position.x) ** 2 + (pose_i.pose.position.y - pose_j.pose.position.y) ** 2 + (pose_i.pose.position.z - pose_j.pose.position.z) ** 2
    sq_exp = np.exp(-dist_norm_squared / length_scale ** 2)
    return sq_exp