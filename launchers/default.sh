#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec echo "Setting up display..."
dt-exec Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset 
export DISPLAY=:1
dt-exec echo "Done."

dt-exec echo "[Launcher]: Running dt_simulator-ros-wrapper..."
dt-exec roslaunch dt_simulator-ros-wrapper run_dt_simulator_ros_wrapper_node.launch
dt-exec echo "[Launcher]: ... Done."

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
