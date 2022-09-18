# ROS interface package for Robotous RFT64-6A01
## Dependencies
Pysoem [pysoem](https://github.com/bnjmnp/pysoem.git)
`python -m pip install pysoem`
or 
`pip install pysoem`

## Give python permission to root
To get access to ethercat, python3 script should have access to the root
`cd /usr/bin`
`ls python3.6`
If python is in the folder, give permission through,
`sudo setcap cap_net_raw+ep /usr/bin/python3.6`

## Run
`roscd rft64_6a01_ros_interface/script`
`python3 interface.py <adapter>`
* you can find out <adapter> name by
`ifconfig`
(for example "enp~")
