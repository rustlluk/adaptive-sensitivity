#!/usr/bin/env python3

import argparse
from subprocess import call, PIPE, run
import create_xauth
import datetime
try:
    from zoneinfo import ZoneInfo
except ImportError:
    from backports.zoneinfo import ZoneInfo
import re


def parse():
    parser = argparse.ArgumentParser(description='Script to deploy Docker')

    parser.add_argument(
        "--build",
        "-b",
        dest="build",
        action="store_true",
        required=False,
        default=False,
        help="whether to build the image; default False"
    )

    parser.add_argument(
        "--nvidia",
        "-nv",
        dest="nvidia",
        action="store_true",
        required=False,
        default=False,
        help="whether to use nvidia; default False"
    )

    parser.add_argument(
        "--existing",
        "-e",
        dest="existing",
        action="store_true",
        required=False,
        default=False,
        help="whether to use existing image; default False"
    )

    parser.add_argument(
        "--terminal",
        "-t",
        dest="terminal",
        action="store_true",
        required=False,
        default=False,
        help="whether to open new terminal; default False"
    )

    parser.add_argument(
        "--pull",
        "-pu",
        dest="pull",
        action="store_true",
        required=False,
        default=False,
        help="whether to pull the image from our Humanoids registry"
    )

    parser.add_argument(
        "--path",
        "-p",
        dest="path",
        required=False,
        default="",
        help="which path to link from host computer; default ''"
    )

    parser.add_argument(
        "--container",
        "-c",
        dest="container",
        required=False,
        default="my_new_docker",
        help="name of the container; default 'my_new_docker'"
    )

    parser.add_argument(
        "--python-ver",
        "-pv",
        dest="python_ver",
        required=False,
        default="3.11",
        help="python version to be used; default 3.11"
    )

    parser.add_argument(
        "--pycharm-ver",
        "-pcv",
        dest="pycharm_ver",
        required=False,
        default="2026.2.0.1",
        help="pycharm version to be used; default 2026.2.0.1"
    )

    args = parser.parse_args()
    return args.build, args.nvidia, args.existing, args.path, args.container, args.python_ver, args.pycharm_ver, \
           args.pull, args.terminal


def main():
    build, nvidia, existing, path, container, python_ver, pycharm_ver, pull, terminal = parse()
    image = container+"_image"

    if terminal:
        cmd = "docker exec -it "+container+" /bin/bash"
        call(cmd, shell=True)
        return 0

    # If we want to build the image
    if build:
        # Stop and remove existing container
        cmd = "docker stop " + container + " && docker rm "+container
        call(cmd, shell=True)

        # build with correct arguments and plain prorgess
        print("Building")
        cmd = "docker build -t "+image+" --build-arg UID=$(id -u) --build-arg GID=$(id -g)" \
              " --build-arg PYTHON_VER="+python_ver+" --build-arg PYCHARM_VER="+pycharm_ver + \
              " --progress=plain ."
        print(cmd)
        call(cmd, shell=True)

        process = run(
            ["docker", "inspect", "-f", "{{.Metadata.LastTagTime}}", image],
            capture_output=True,
            text=True
        )

        if process.returncode != 0:
            print(f"Error: Could not find image '{image}'.")
            return 0

        created_str = process.stdout.strip()
        parts = created_str.split()

        if len(parts) >= 3:
            date_part = parts[0]
            time_part = parts[1].split('.')[0]
            offset_part = parts[2]

            clean_time_str = f"{date_part} {time_part} {offset_part}"

            # Parse the time (which is in +0200), then immediately convert it to UTC!
            image_time = datetime.datetime.strptime(
                clean_time_str, "%Y-%m-%d %H:%M:%S %z"
            ).astimezone(datetime.timezone.utc)
        else:
            print(f"Error parsing Docker date string: '{created_str}'")
            return 0

        # Get current time in UTC
        now_utc = datetime.datetime.now(datetime.timezone.utc)

        # Optional: Print them out to prove they now match perfectly
        # print(f"Image time (UTC): {image_time}")
        # print(f"Now (UTC):        {now_utc}")

        seconds_since_creation = (now_utc - image_time).total_seconds()

        if seconds_since_creation > 60:
            inp = input(
                f"'{image}' was built {int(seconds_since_creation)} seconds ago. It was either built before or failed building. "
                f"Do you still want to try to run it? (y/n): ")
            if inp.lower() not in ["y", "yes"]:
                return 0

    # create correct xauth file for usage over SSH
    print("Creating xauth")
    create_xauth.main()

    if pull:
        print("Pulling image from humanoids")
        cmd = "docker pull 192.168.210.103:5050/"+image+" && docker tag 192.168.210.103:5050/"+image+" "+image
        print(cmd)
        call(cmd, shell=True)

    if existing:
        # just run existing docker and return
        print("Starting previous container")
        cmd = "docker start "+container+" && docker attach "+container
        call(cmd, shell=True)
        return 0

    # remove the old container
    print("Removing old container")
    cmd = "docker stop " + container + " && docker rm " + container
    call(cmd, shell=True, stderr=PIPE, stdout=PIPE)

    # command to run a new container with all necessary arguments
    cmd = '''docker run -it -u $(id -u):$(id -g) -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -e "XAUTHORITY=/tmp/.docker.xauth" \
             -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /dev:/dev \
             -v /etc/hosts:/etc/hosts --network host --privileged --name '''+container+''' \
             -v '''+path+':/home/docker/adaptive_skin_ws '+image

    # add nvidia runtime if needed
    if nvidia:
        cmd = cmd.replace(image, '--runtime=nvidia '+image)

    # start it
    print("Starting the container")
    call(cmd, shell=True)

    return 0


if __name__ == "__main__":
    main()
