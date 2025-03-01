"""
:Author: Lukas Rustler
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "pyKUKA"))
from pykuka import pyKUKA
import numpy as np


def move(client):

    # move second joint
    client.move_position("iiwa_joint_2", -np.pi/3)

    # move the same joint back
    client.move_position("iiwa_joint_2", 0, wait=False, velocity=5)
    # wait manually
    while not client.motion_done():
        client.update_simulation()

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
