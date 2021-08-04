#!/bin/bash
# copy the drone_gnc package
rsync -avuzh $(rospack find drone_gnc) drone@ert.local:~/drone_ws/src
# copy the environment loader
rsync -avuzh $(rospack find drone_utils)/raspberrypi_interface/raspberrypi_files/remote_env_loader.sh  drone@ert.local:~/drone_ws/devel