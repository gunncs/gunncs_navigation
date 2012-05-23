#!/bin/bash

urxvt -e sh -c 'htop'&
urxvt -e sh -c 'watch sensors'&
urxvt -e sh -c 'roscore'&
#urxvt -cd ~/ros_workspace/gunncs_navigation -e sh -c 'htop'
