#!/bin/bash
# Starts the peltier cooling system: enables 1-wire and runs both ROS2 nodes.

# Check if 1-wire is already enabled (w1_gpio module loaded)
if lsmod | grep -q 'w1_gpio'; then
  echo "1-wire module already loaded."
else
  # Enable 1-wire on GPIO 4 if not already loaded
  echo "Enabling 1-wire on GPIO 4..."
  sudo dtoverlay w1-gpio gpiopin=4
fi

source "$(dirname "$0")/install/setup.bash"

ros2 run peltier_pint temperature_publisher &
ros2 run peltier_pint relay_control_subscriber

