#!/bin/bash

<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"
COMMENT

# USB low latency setup:
Startups/usb_latency_setup.sh
echo '====== USB Low Latency Setup Finished ======'
echo ''

# create shared memory
python3 -m Startups.memory_manager
echo '====== Shared Memory Created ======'
echo ''

# delete all the existing screens if any
sudo pkill screen

# create a background screen named 'bruce'
screen -d -m -S bruce

# create a window named 'dxl'
screen -S bruce -X screen -t dxl
screen -S bruce -p dxl -X stuff 'python3 -m Startups.run_dxl^M'
sleep 1.5s
echo '====== Dynamixel Motor Online ======'
echo ''

# create a window named 'bear'
screen -S bruce -X screen -t bear
screen -S bruce -p bear -X stuff 'python3 -m Startups.run_bear^M'
sleep 1.5s
echo '====== BEAR Motor Online ======'
echo ''

# create a window named 'estimation'
screen -S bruce -X screen -t estimation
read -p $'\e[31mATTENTION: Press ENTER to launch BRUCE!\e[0m'
screen -S bruce -p estimation -X stuff 'python3 -m Play.initialize^M'
sleep 0.5s
screen -S bruce -p estimation -X stuff "s^M"
sleep 3s
read -p $'\e[31mATTENTION: Place BRUCE on the ground! Press Enter to start estimation!\e[0m'
screen -S bruce -p estimation -X stuff 'python3 -m Startups.run_estimation^M'
sleep 1.0s
screen -S bruce -p estimation -X stuff "KF^M"
sleep 0.5s
echo '====== State Estimation Online ======'
echo ''

# create a window named 'low_level'
screen -S bruce -X screen -t low_level
screen -S bruce -p low_level -X stuff 'python3 -m Play.Walking.low_level^M'
sleep 2.0s
read -p $'\e[31mATTENTION: Press ENTER to start low-level control!\e[0m'
sleep 0.5s
screen -S bruce -p low_level -X stuff "y^M"
sleep 0.5s
echo '====== Low-Level Controller Online ======'
echo ''

# create a window named 'high_level'
read -p $'\e[31mATTENTION: Press ENTER to start high-level control!\e[0m'
screen -S bruce -X screen -t high_level
screen -S bruce -p high_level -X stuff 'python3 -m Play.Walking.high_level^M'
sleep 0.5s
screen -S bruce -p high_level -X stuff "y^M"
sleep 0.5s
echo '====== High-Level Controller Online ======'
echo ''

# create a window named 'top_level'
read -p $'\e[31mATTENTION: Press ENTER to start top-level control!\e[0m'
screen -S bruce -X screen -t top_level
screen -S bruce -p top_level -X stuff 'python3 -m Play.Walking.top_level^M'

# record data
#screen -S bruce -X screen -t data
#screen -S bruce -p data -X stuff 'python3 -m Util.data_record^M'

sleep 0.5s
echo '====== Top-Level Controller Online ======'
echo ''
input='n'
while [ $input != 'y' ]
do
  echo -ne 'Entering Cockpit .\033[0K\r'
  sleep 0.5s
  echo -ne 'Entering Cockpit ..\033[0K\r'
  sleep 0.5s
  echo -ne 'Entering Cockpit ...\033[0K\r'
  sleep 0.5s
  screen -r -p top_level
  sleep 0.2s
  read -p 'Exit? (y/n)' input
done

# terminate the threads
#screen -S bruce -p data -X stuff "^C"
screen -S bruce -p bear -X stuff "^C"
screen -S bruce -p dxl -X stuff "^C"
screen -S bruce -p low_level -X stuff "^C"
screen -S bruce -p high_level -X stuff "^C"
screen -S bruce -p top_level -X stuff "^C"
screen -S bruce -p estimation -X stuff "^C"
sleep 1s  # wait for the threads to terminate!

# delete the screen
sudo pkill screen
