#!/bin/bash
ssh drone@ert.local "source /home/drone/drone_ws/devel/setup.bash; rosnode kill -a"
