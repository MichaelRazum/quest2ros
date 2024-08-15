#!/usr/bin/env python
import rospy
import sys
import redis
import json
from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist


class ROS2Quest:
    def __init__(self):
        # Initialize Redis connection
        self.redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

        # Subscribers to quest topics
        self.ovr2ros_right_hand_pose_sub = rospy.Subscriber(
            "/q2r_right_hand_pose", PoseStamped, self.ovr2ros_right_hand_pose_callback)
        self.ovr2ros_right_hand_twist_sub = rospy.Subscriber(
            "/q2r_right_hand_twist", Twist, self.ovr2ros_right_hand_twist_callback)
        self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber(
            "/q2r_right_hand_inputs", OVR2ROSInputs, self.ovr2ros_right_hand_inputs_callback)
        self.ovr2ros_left_hand_pose_sub = rospy.Subscriber(
            "/q2r_left_hand_pose", PoseStamped, self.ovr2ros_left_hand_pose_callback)
        self.ovr2ros_left_hand_twist_sub = rospy.Subscriber(
            "/q2r_left_hand_twist", Twist, self.ovr2ros_left_hand_twist_callback)
        self.ovr2ros_left_hand_inputs_sub = rospy.Subscriber(
            "/q2r_left_hand_inputs", OVR2ROSInputs, self.ovr2ros_left_hand_inputs_callback)

        # Publishers to quest topics
        self.ros2ovr_right_hand_haptic_feedback_pub = rospy.Publisher(
            "/q2r_right_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
        self.ros2ovr_left_hand_haptic_feedback_pub = rospy.Publisher(
            "/q2r_left_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
        self.ros2ovr_dice_twist_pub = rospy.Publisher(
            "/dice_twist", Twist, queue_size=1)
        self.ros2ovr_q2r_twist_pub = rospy.Publisher(
            "/q2r_twist", Twist, queue_size=1)

    def publish_to_redis(self, key: str, data: dict):
        """Convert data to JSON and publish to Redis."""
        json_data = json.dumps(data)
        self.redis_client.set(key, json_data)

    def ovr2ros_right_hand_pose_callback(self, data: PoseStamped):
        self.publish_to_redis('right_hand_pose', self.pose_stamped_to_dict(data))

    def ovr2ros_right_hand_twist_callback(self, data: Twist):
        self.publish_to_redis('right_hand_twist', self.twist_to_dict(data))

    def ovr2ros_right_hand_inputs_callback(self, data: OVR2ROSInputs):
        self.publish_to_redis('right_hand_inputs', self.ovr2ros_inputs_to_dict(data))

        # If the lower button is pressed, send the twist back to the quest to move the q2r; 0 otherwise
        q2r_twist = Twist()
        if data.button_lower:
            q2r_twist = self.twist_to_dict(self.right_hand_twist)
        self.ros2ovr_q2r_twist_pub.publish(q2r_twist)

        # Send the triggers as frequency and amplitude of vibration back to the quest
        right_hand_haptic_feedback = OVR2ROSHapticFeedback()
        right_hand_haptic_feedback.frequency = data.press_index
        right_hand_haptic_feedback.amplitude = data.press_middle
        self.ros2ovr_right_hand_haptic_feedback_pub.publish(right_hand_haptic_feedback)

    def ovr2ros_left_hand_pose_callback(self, data: PoseStamped):
        self.publish_to_redis('left_hand_pose', self.pose_stamped_to_dict(data))

    def ovr2ros_left_hand_twist_callback(self, data: Twist):
        self.publish_to_redis('left_hand_twist', self.twist_to_dict(data))

    def ovr2ros_left_hand_inputs_callback(self, data: OVR2ROSInputs):
        self.publish_to_redis('left_hand_inputs', self.ovr2ros_inputs_to_dict(data))

        # If the lower button is pressed, send the twist back to the quest to move the dice; 0 otherwise
        dice_twist = Twist()
        if data.button_lower:
            dice_twist = self.twist_to_dict(self.left_hand_twist)
        self.ros2ovr_dice_twist_pub.publish(dice_twist)

        # Send the triggers as frequency and amplitude of vibration back to the quest
        left_hand_haptic_feedback = OVR2ROSHapticFeedback()
        left_hand_haptic_feedback.frequency = data.press_index
        left_hand_haptic_feedback.amplitude = data.press_middle
        self.ros2ovr_left_hand_haptic_feedback_pub.publish(left_hand_haptic_feedback)

    @staticmethod
    def pose_stamped_to_dict(pose_stamped: PoseStamped) -> dict:
        return {
            "seq": pose_stamped.header.seq,
            "pose": {
                "position": {
                    "x": pose_stamped.pose.position.x,
                    "y": pose_stamped.pose.position.y,
                    "z": pose_stamped.pose.position.z,
                },
                "orientation": {
                    "x": pose_stamped.pose.orientation.x,
                    "y": pose_stamped.pose.orientation.y,
                    "z": pose_stamped.pose.orientation.z,
                    "w": pose_stamped.pose.orientation.w,
                },
            },
        }

    @staticmethod
    def twist_to_dict(twist: Twist) -> dict:
        return {
            "linear": {
                "x": twist.linear.x,
                "y": twist.linear.y,
                "z": twist.linear.z,
            },
            "angular": {
                "x": twist.angular.x,
                "y": twist.angular.y,
                "z": twist.angular.z,
            },
        }

    @staticmethod
    def ovr2ros_inputs_to_dict(inputs: OVR2ROSInputs) -> dict:
        return {
            "button_lower": inputs.button_lower,
            "press_index": inputs.press_index,
            "press_middle": inputs.press_middle,
        }


def main(args):
    rospy.init_node('quest2rosdemo', anonymous=True)
    r2q = ROS2Quest()

    r = rospy.Rate(1000)  # 1000hz
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
