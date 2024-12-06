#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_inverse
from scipy.spatial.transform import Rotation as R

# Store previous data for computing derivatives
previous_pose = None
previous_time = None
previous_velocity = None

def normalize_quaternion(quaternion):
    """
    Normalize a quaternion to ensure it is a unit quaternion.
    """
    norm = np.linalg.norm(quaternion)
    if norm == 0:
        rospy.logwarn("Received zero quaternion. Returning identity quaternion.")
        return [0, 0, 0, 1]  # Identity quaternion
    return [q / norm for q in quaternion]

def compute_angular_velocity(q1, q2, dt):
    """
    Compute angular velocity from two quaternions.
    q1: Previous quaternion
    q2: Current quaternion
    dt: Time difference
    """
    
    # This might need to switch q2 and q1
    q_delta = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative quaternion

    angular_velocity = [
        2 * q_delta[0] / dt,
        2 * q_delta[1] / dt,
        2 * q_delta[2] / dt,
    ]

    # Converting from global frame to local frame
    Rot = R.from_quat(q2).as_matrix()

    local_angular_velocity = Rot.T @ np.array(angular_velocity).T

    return local_angular_velocity

def compute_linear_acceleration(p1, p2, v_prev, dt, q2):
    """
    Compute linear acceleration from positions and previous velocity.
    p1: Previous position
    p2: Current position
    v_prev: Previous velocity
    dt: Time difference
    q2: Current quaternion
    """

    #rospy.loginfo("position1: %s", str(p1))
    #rospy.loginfo("prev_vel: %s", str(v_prev))
    velocity = [(p2[i] - p1[i]) / dt for i in range(3)]  # Current velocity
    acceleration = [(velocity[i] - v_prev[i]) / dt for i in range(3)] if v_prev is not None else [0.0, 0.0, 0.0]
    acceleration = np.array(acceleration)
    acceleration[2] -= 9.81

    # Converting from global frame to local frame_id
    Rot = R.from_quat(q2).as_matrix()

    local_velocity = (Rot.T @ np.array([velocity]).T).T.squeeze()
    local_acceleration = (Rot.T @ np.array([acceleration]).T).T.squeeze()

    rospy.loginfo("local_velocity: %s", str(local_velocity))
    rospy.loginfo("local_acceleration: %s", str(local_acceleration))

    return local_velocity, local_acceleration

def pose_to_imu_callback(pose_msg):
    global previous_pose, previous_time, previous_velocity

    # Get current time
    current_time = pose_msg.header.stamp.to_nsec()*1e-9

    # Initialize for the first message
    if previous_pose is None:
        previous_pose = pose_msg
        previous_time = current_time
        previous_velocity = [0.0, 0.0, 0.0]
        return

    if current_time == previous_time:
        return

    if pose_msg.pose.position.x == 0 \
    and pose_msg.pose.position.y == 0 \
    and pose_msg.pose.position.z == 0 \
    and pose_msg.pose.orientation.x == 0 \
    and pose_msg.pose.orientation.y == 0 \
    and pose_msg.pose.orientation.z == 0 \
    and pose_msg.pose.orientation.w == 0:
        print("Pose unable to be found by Mocap")
        return 

    # Time difference
    dt = current_time - previous_time
    if dt <= 0:
        return

    dt = max(dt, 5e-3)

    #rospy.loginfo("current_time: %s", str(current_time))
    rospy.loginfo("dt: %s", str(dt))

    # Extract positions and orientations
    p1 = [previous_pose.pose.position.x, previous_pose.pose.position.y, previous_pose.pose.position.z]
    p2 = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]

    q1 = [
        previous_pose.pose.orientation.x,
        previous_pose.pose.orientation.y,
        previous_pose.pose.orientation.z,
        previous_pose.pose.orientation.w,
    ]
    q2 = [
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w,
    ]

    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)

    # Compute angular velocity
    angular_velocity = compute_angular_velocity(q1, q2, dt)

    # Compute linear acceleration
    velocity, linear_acceleration = compute_linear_acceleration(p1, p2, previous_velocity, dt, q2)

    # Create Imu message
    imu_msg = Imu()
    imu_msg.header = pose_msg.header
    imu_msg.header.frame_id = 'imu_camera'
    imu_msg.orientation.x = 0
    imu_msg.orientation.y = 0
    imu_msg.orientation.z = 0
    imu_msg.orientation.w = 1
    imu_msg.angular_velocity.x = angular_velocity[0]
    imu_msg.angular_velocity.y = angular_velocity[1]
    imu_msg.angular_velocity.z = angular_velocity[2]
    imu_msg.linear_acceleration.x = linear_acceleration[0]
    imu_msg.linear_acceleration.y = linear_acceleration[1]
    imu_msg.linear_acceleration.z = linear_acceleration[2]

    # Publish the Imu message
    imu_publisher.publish(imu_msg)

    # Update previous values
    previous_pose = pose_msg
    previous_time = current_time
    previous_velocity = velocity

if __name__ == "__main__":
    rospy.init_node("pose_to_imu_republisher")

    # Subscriber to PoseStamped topic
    pose_subscriber = rospy.Subscriber("/mocap/posestamped", PoseStamped, pose_to_imu_callback)

    # Publisher to Imu topic
    imu_publisher = rospy.Publisher("/imu", Imu, queue_size=10)

    rospy.spin()
