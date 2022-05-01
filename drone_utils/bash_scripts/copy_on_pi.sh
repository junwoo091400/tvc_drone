#!/bin/bash
# copy all the necessary files to the raspberry pi
rsync -avuzh \
    "$(rospack find drone_fsm)" \
    "$(rospack find drone_navigation)" \
    "$(rospack find drone_optimal_control)" \
    "$(rospack find drone_utils)" \
    "$(rospack find mavros_interface)" \
    "$(rospack find optitrack_ekf)" \
    "$(rospack find rocket_utils)" \
    drone@ert.local:~/drone_ws/src