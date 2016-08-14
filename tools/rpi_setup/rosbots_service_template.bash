#! /bin/sh

### BEGIN INIT INFO
# Provides:		rosbots
# Required-Start:	$all
# Required-Stop:	$remote_fs $syslog
# Default-Start:	2 3 4 5
# Default-Stop:		0 1 6
# Short-Description:	ROSCORE and other ros launch scripts
### END INIT INFO

# Try to get wifi address first
my_ip="$(ifconfig wlan0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"

if  [ "${my_ip}" == "" ]; then
    my_ip="$(ifconfig eth0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"
fi

export ROSBOTS_HOME=_TEMPLATE_HOME
export ROSBOTS_WS_PATH=_TEMPLATE_WS_PATH
. ${ROSBOTS_WS_PATH}/build/opt/ros/kinetic/setup.sh

export ROS_IP=${my_ip}

# Add some more paths to make ros work
export PATH=${PATH}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games:/sbin:/usr/sbin:/bin:/usr/bin

start_ros () { 
    echo "ROS IP - ${ROS_IP}" > ${ROSBOTS_HOME}/roscore.log
    echo "Starting roscore..." >> ${ROSBOTS_HOME}/roscore.log
    nohup roscore >> ${ROSBOTS_HOME}/roscore.log 2>&1 & 

    #mv /home/pi/driver_rpi.log /home/pi/driver_rpi.log.old
    #nohup rosrun rosbots_driver driver_rpi.py >> /home/pi/driver_rpi.log 2>> /home/pi/driver_rpi.log &
}

stop_ros () {
    #source /opt/ros/setup.sh

    #killall nodes
    for i in $( rosnode list ); do
    rosnode kill $i;
    done

    #stop roscore
    killall roscore
}

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    start_ros
    ;;
  stop)
    stop_ros
    ;;
  *)
    echo "Usage: /etc/init.d/rosbots {start|stop}"
    exit 1
    ;;
esac

exit 0
