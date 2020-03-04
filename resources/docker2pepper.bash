#!/bin/bash
# docker2pepper.bash

rm -rf /home/nao/toolkit_ws/
tar -xzf toolkit_ws.tar.gz
rm -f toolkit_ws.tar.gz
cd toolkit_ws; catkin_make; cd ..
tar -czf toolkit_ws.tar.gz toolkit_ws/
scp /home/nao/toolkit_ws.tar.gz nao@192.168.1.139:/home/nao/
rm -f toolkit_ws.tar.gz

