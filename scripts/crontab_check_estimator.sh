#!/usr/bin/env bash

# Add these lines with "crontab -e"
# * * * * * /home/project/estimator-ros/scripts/check_estimator.sh
# @reboot /home/project/estimator-ros/scripts/check_estimator.sh

LOGFILE="/home/project/estimator.log"
PATTERN="ros2 launch main main_launch.py"

if ! pgrep -f "$PATTERN" >/dev/null; then
  echo "$(date +'%F %T') — estimator not running, starting..." >> "$LOGFILE"
  cd /home/project/estimator-ros || exit 1
  . ./source.sh
  nohup ros2 launch main main_launch.py >> "$LOGFILE" 2>&1 &
else
  echo "$(date +'%F %T') — estimator already running." >> "$LOGFILE"
fi
