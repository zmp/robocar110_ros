#!/bin/bash

source /usr/lib/systemd/user/rc110-prepare.sh

roslaunch --wait rc110_launch robot.launch
