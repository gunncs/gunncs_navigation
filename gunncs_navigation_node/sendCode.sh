#!/bin/bash

ADDRESS=192.168.2.100
USER=gunncs

ssh $USER@$ADDRESS 'cd /home/gunncs/ros_workspace/gunncs_navigation ; git checkout . '
ssh $USER@$ADDRESS 'rm -rfv /home/gunncs/receive/gunncs_navigation'
scp -r ~/ros_workspace/gunncs_navigation $USER@$ADDRESS:/home/gunncs/receive/gunncs_navigation
ssh $USER@$ADDRESS 'cd /home/gunncs/ros_workspace/gunncs_navigation ; git pull /home/gunncs/receive/gunncs_navigation'



