"""
:Author: Lukas Rustler
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "pyKUKA"))
from pykuka import pyKUKA
from utils import Pose


def move(client):
    # Get current pose
    cur_pose = client.end_effector.get_position()

    # Copy, the pose and go 25 in X
    new_pose = Pose(cur_pose.pos, cur_pose.ori)
    new_pose.pos[0] -= 0.25

    # move
    client.move_cartesian(new_pose, wait=True)


    # get current pose
    pose = client.end_effector.get_position()
    # go back
    pose.pos[0] += 0.25
    # move; do not wait for completion; and move a bit faster
    client.move_cartesian(pose, wait=False, velocity=2)
    # wait manually
    while not client.motion_done():
        client.update_simulation(0.01)

    """
    Wait could be also achieved with:
    client.wait_motion_done()
    """

    client.logger.info("Test finished!")


if __name__ == "__main__":
    # load the robot with correct world/config
    client = pyKUKA(config="default.yaml")

    # move the robot
    move(client)

    # just wait until the gui is closed
    while client.is_alive():
        client.update_simulation()
