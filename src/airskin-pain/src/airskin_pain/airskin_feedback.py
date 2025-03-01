#!/usr/bin/env python3
import rospy
from airskin_pain.utils import AirskinFeedback
import argparse

def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
    )

    arg_parser.add_argument(
        "--setup",
        "-s",
        dest="setup",
        required=False,
        default="sim",
        help="Setup"
    )


    args = arg_parser.parse_args()
    return args.setup


if __name__ == "__main__":
    setup = prepare_parser()
    behaviour = "stop"

    if setup == "real":
        from airskin.msg import AirskinStatus
    else:
        from bullet_ros_ur.msg import AirskinStatus

    rospy.init_node("airskin_feedback_node")
    airskin_detector = AirskinFeedback(setup, 30)
    airskin_detector.sub = rospy.Subscriber("/airskin_status", AirskinStatus, airskin_detector, queue_size=1)

    rospy.spin()
