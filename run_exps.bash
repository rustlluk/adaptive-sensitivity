#!/bin/bash
rosrun airskin_pain do_exps.py -s sim -i 100 -b -m normal_kuka -e test_local_kuka -v [0.2,0.4,0.6] -r kuka
rosrun airskin_pain do_exps.py -s sim -i 100 -b -m mass_kuka -e test_local_kuka -v [0.2,0.4,0.6] -r kuka
rosrun airskin_pain do_exps.py -s sim -i 100 -b -m norm_kuka -e test_local_kuka -v [0.2,0.4,0.6] -r kuka
rosrun airskin_pain do_exps.py -s sim -i 100 -b -m normal_ur -e test_local_kuka -v [0.2,0.4,0.6] -r ur
rosrun airskin_pain do_exps.py -s sim -i 100 -b -m mass_ur -e test_local_kuka -v [0.2,0.4,0.6] -r ur
rosrun airskin_pain do_exps.py -s sim -i 100 -b -m norm_ur -e test_local_kuka -v [0.2,0.4,0.6] -r ur
