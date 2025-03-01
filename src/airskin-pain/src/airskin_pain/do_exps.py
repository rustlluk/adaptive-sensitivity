#!/usr/bin/env python3

from subprocess import call, Popen
import time
import argparse
import os
import datetime
import signal


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
        help="Which setup to use: sim or real"
    )

    arg_parser.add_argument(
        "--save_bag",
        "-b",
        dest="save_bag",
        action="store_true",
        required=False,
        default=False,
        help="Whether to save the bag file"
    )

    arg_parser.add_argument(
        "--offscreen",
        "-o",
        dest="offscreen",
        action="store_true",
        required=False,
        default=False,
        help="Whether to run without RVIZ"
    )

    arg_parser.add_argument(
        "--iterations",
        "-i",
        dest="iterations",
        required=False,
        default=10,
        help="number of iteration per config file"
    )

    arg_parser.add_argument(
        "--mode",
        "-m",
        dest="skin_mode",
        required=False,
        default="normal",
        help="which skin mode to use"
    )

    arg_parser.add_argument(
        "--exp_name",
        "-e",
        dest="exp_name",
        required=False,
        default=None,
        help="Name of the experiment"
    )

    arg_parser.add_argument(
        "--velocities",
        "-v",
        dest="velocities",
        required=False,
        default="[0.6]",
        help="Velocities to test"
    )

    arg_parser.add_argument(
        "--robot",
        "-r",
        dest="robot",
        required=False,
        default="ur ",
        help="Which robot to use: ur or kuka"
    )
    args = arg_parser.parse_args()
    return (args.setup, "--save_bag" if args.save_bag else "", args.exp_name, args.offscreen, int(args.iterations),
            args.skin_mode, eval(args.velocities), args.robot)


def create_ts():
    start_time_ = datetime.datetime.now()
    return str(start_time_).replace(".", "-").replace(" ", "-").replace(":", "-"), start_time_


if __name__ == "__main__":
    setup, save_bag,  exp_name, offscreen, iterations, skin_mode, velocities, robot = prepare_parser()

    for vel in velocities:
        start_ts, _ = create_ts()
        if exp_name is None:
            exp_name = start_ts
        log_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "exps", exp_name, f"{start_ts}_{vel}.csv")
        if not os.path.exists(os.path.dirname(log_path)):
            os.makedirs(os.path.dirname(log_path))

        with open(log_path, "w") as f:
            f.write("timestamp;mode;setup;iteration;velocity;robot;bag_name\n")

        for i in range(iterations):
            iteration_start_ts, start_time = create_ts()
            if setup == "sim":
                cmd = (f"roslaunch airskin_pain main.launch sim:={'true' if setup == 'sim' else 'false'} "
                       f"rviz:={'true' if not offscreen else 'false'} robot:={robot}")
                launch = Popen(cmd, shell=True)
                time.sleep(5)
            else:
                print(f"Starting iteration {i} of {skin_mode} in {setup} setup.")
                input("Ready?")

            if skin_mode in ["norm", "mass"]:
                cmd = f"rosrun airskin_pain eff_mass.py --mode {skin_mode} --robot {robot}"
                mass = Popen(cmd, shell=True)

            if robot == "ur":
                cmd = f"rosrun airskin_pain airskin_feedback.py --setup {setup}"
                airskin = Popen(cmd, shell=True)
                time.sleep(2)
            elif robot == "kuka":
                cmd = f"rosrun airskin_pain kuka_feedback.py --setup {setup}"
                airskin = Popen(cmd, shell=True)
                time.sleep(2)

            cmd = f"rosrun airskin_pain main_velocity.py --setup {setup} {save_bag} --bag_name {iteration_start_ts} --velocity {vel} --robot {robot}"
            mainp = call(cmd, shell=True)

            with open(log_path, "a") as f:
                f.write(f"{iteration_start_ts};{skin_mode};{setup};{i};{vel};{robot};{iteration_start_ts}.bag\n")
            if setup == "sim":
                if skin_mode in ["norm", "mass"]:
                    mass.send_signal(signal.SIGKILL)
                airskin.send_signal(signal.SIGKILL)
                launch.send_signal(signal.SIGKILL)
                call("pkill -f ros", shell=True)
                call("pkill -f airskin_feedback.py", shell=True)
                call("pkill -f kuka_feedback.py", shell=True)
                call("pkill -f eff_mass.py", shell=True)
                call("pkill -f main_velocity.py", shell=True)
                time.sleep(10)
            elif setup == "real":
                call("pkill -f airskin_feedback.py", shell=True)
                call("pkill -f eff_mass.py", shell=True)
                call("pkill -f main_velocity.py", shell=True)

            print(f"Finished iteration {i} in {setup} setup in {datetime.datetime.now()-start_time}s.")
