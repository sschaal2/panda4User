#!/bin/tcsh
sudo setcap cap_net_admin,cap_net_raw+eip x86_64/xrprobot
sudo chmod 777 /dev/ttyUSB1

