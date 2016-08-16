#! /bin/bash

if [ "${ROS_MASTER_URI}" == "" ]; then
    echo "You need to source the main ROS setup.bash file first... exiting"
    return
fi

# Try to get wifi address first
my_ip="$(ifconfig wlan0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"

if  [ "${my_ip}" == "" ]; then
    my_ip="$(ifconfig eth0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"
fi

#echo ${my_ip}

export ROS_IP=${my_ip}

if [ "${ROSBOTS_MASTER}" == "1" ]
then
    rcrunning="$(ps ax | grep -i roscore | grep -i python)"
    if [ "${rcrunning}" == "" ]
    then
        echo '!! Need run roscore - "roscore > ~/roscore.out 2>&1 &"'
        echo '...Eventually need to create a systemd file for this so roscore runs on bootup.'
        #nohup roscore > ~/roscore.out 2>&1 &
    else
        echo '\n\nROSCORE already running...'
    fi
    echo "For all slaves, \"export ROS_MASTER_URI=http://${ROS_IP}:11311\"\n"
else    
    echo 'As ROS slave node, you will also need to "export ROS_MASTER_URI=http://<master_ip>:11311"'
fi
