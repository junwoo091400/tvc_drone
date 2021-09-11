#!/bin/bash
rsync -avuzh drone@ert.local:~/drone_ws/log.bag $(rospack find drone_utils)/log/$(date "+%m-%d_%H-%M-%S").bag