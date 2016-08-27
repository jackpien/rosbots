import os
import datetime as dt
import random
import time

from fabric.api import *
import fabric.contrib.files as fabfiles
from fabric.utils import fastprint
#env.hosts = ["localhost"]
env.user = 'pi'    
env.shell = '/bin/bash -l -c'

    
def _get_input(msg):
    val = raw_input(msg + "\n")
    return val

def _fp(msg):
    fastprint(msg + "\n")

def _pp(msg):
    """ 
    Print then pause
    """
    _fp(msg)
    programPause = _get_input("Press the <ENTER> key to continue...")


WS_DIR = "/ros_catkin_ws"
INSTALL_DIR = WS_DIR + "/build/opt/ros/kinetic"

def helloworld():
    run("ls -la")

    with cd("~"):
        home_path = run("pwd")
        ws_dir = home_path + WS_DIR

        put("./rosbots_service_template.bash", "~/rosbots_template")
        run("cat rosbots_template | sed 's/_TEMPLATE_HOME/" + home_path.replace("/", "\/") + "/' | sed 's/_TEMPLATE_WS_PATH/" + ws_dir.replace("/", "\/") + "/' > rosbots")

def setup_wifi_on_pi():
    run("echo 'Starting...'")

    ssid_name = _get_input("What is the SSID?")
    _fp(ssid_name)
    wpa_pwd = _get_input("What is the WPA pwd?")
    _fp(wpa_pwd)
    name = _get_input("What do you want to name this network?")
    _fp(name)

    supplicant_fn = "/etc/wpa_supplicant/wpa_supplicant.conf"
    if sudo("grep 'ssid=\"" + ssid_name + "\"' " + supplicant_fn, \
           warn_only=True).succeeded:
        _fp("This SSID is already set up")
    else:
        _fp("Adding the network you specified into " + supplicant_fn)
        network_config = "\n\n" + \
                         "network={\n" + \
                         "    ssid=\"" + ssid_name + "\"\n" + \
                         "    psk=\"" + wpa_pwd + "\"\n" + \
                         "    id_str=\"" + name + "\"\n" + \
                         "}\n"
        sudo("cp " + supplicant_fn + " " + supplicant_fn + ".old")
        sudo("echo '" + network_config + "' >> " + supplicant_fn)
        

def setup_ros_robot_packages():
    _setup_ros_other_packages("geometry_msgs")

def setup_ros_rosbots_packages():
    run("echo 'Starting...'")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"
    rosbots_path = git_path + "/rosbots"
    ws_dir = home_path + WS_DIR
    install_dir = home_path + INSTALL_DIR
    if not fabfiles.exists(git_path):
        _fp("Did not find rosbots repo, cloning...")
        run("mkdir " + git_path)
        with cd(git_path):
            run("git clone https://github.com/jackpien/rosbots.git")
            
        _fp("Creating symbolic link to main ros workspace")
        with cd(ws_dir + "/src"):
            if fabfiles.exists("rosbots_driver"):
                run("rm rosbots_driver")
            run("ln -s " + rosbots_path + "/ros_ws/src/rosbots_driver")
    else:
        _fp("Found rosbots repo, just fetching top and rebasing")
        with cd(rosbots_path):
            run("git fetch origin")
            run("git rebase origin/master")

    with cd(ws_dir):
        run("./src/catkin/bin/catkin_make_isolated --pkg rosbots_driver --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")

    # Installing RPIO DMA PWM library
    with cd(git_path):
        if not fabfiles.exists("RPIO"):
            _pp("Did not find RPIO library so downloading and setting up")
            run("git clone https://github.com/metachris/RPIO.git --branch v2 --single-branch")
            with cd("RPIO"):
                run("python setup.py build")
                _pp("Did build complete for RPIO?")
                run("export PYTHONPATH=" + home_path + "/lib/python; python setup.py -v install --home " + home_path)

                _pp("Did RPIO install correctly into " + home_path + "?")

    # Rerun the init script
    sudo("systemctl stop rosbots")
    sudo("systemctl start rosbots")
    

def _setup_ros_other_packages(rospkg):
    run("echo 'Starting...'")

    _pp("After you successfully install ros_com stuff, install some others")

    home_path = run("pwd")
    ws_dir = home_path + WS_DIR
    if not fabfiles.exists(ws_dir):
        _fp("ROS Workspace not found - run the main set up first")
        return

    with cd(ws_dir):
        ts = str(time.time()).split(".")[0]
        fn = "kinetic-custom_" + str(ts) + "_ros.rosinstall"
        run("rosinstall_generator " + rospkg + " --rosdistro kinetic --deps --wet-only --tar > " + fn)

        run("cat " + fn)

        _pp("Did rosinstall generator create the install file correctly? If so, we're going to merge and update the workspace.")
        
        run("wstool merge -t src " + fn)

        _pp("Did the wstool merge correctly?  If so, we are going to update on the install file for the workspace.")
        
        run("wstool update -t src")

        
        _pp("Did the wstool update correctly?  If so, we are going to update dependencies.")

        run("rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie")

        _pp("Did the dependencies update ok?  If so, let's compile the new packages.")

        run("./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space " + home_path + INSTALL_DIR + " -j2")


        

def setup_ros_for_pi():
    global WS_DIR
    global INSTALL_DIR

    run("echo 'Roughly following http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi'")

    _fp("Set up / compile ROS on Rasbian Jessie Lite 2016-05-27")
    _pp("* If you need to do raspi-config stuff, CTRL-C out and do that before running this script")

    # Setup ROS Repositories
    if not fabfiles.exists("/etc/apt/sources.list.d/ros-latest.list"):
        sudo("sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'")
        sudo("apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116")
        sudo("apt-get update")
        sudo("apt-get -y upgrade")
    else:
        _fp("ros-lastest.list already exists... skipping set up")
    

    # Install Bootstrap Dependencies
    sudo("apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake")
    
    # Initializing rosdep
    if not fabfiles.exists("/etc/ros/rosdep/sources.list.d/20-default.list"):
        sudo("rosdep init")
        run("rosdep update")

    home_path = run("pwd")
    ws_dir = home_path + WS_DIR

    # Create catkin workspace
    if not fabfiles.exists(ws_dir):
        run("mkdir -p " + ws_dir)
        
    # Compile
    with cd(ws_dir):
        if not fabfiles.exists("kinetic-ros_comm-wet.rosinstall"):
            run("rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall")
        
        if not fabfiles.exists("src"):
            _fp("The following wstool downloads the source code needed")
            _pp("If wstool init fails or is interrupted, you can resume the download by running:\n wstool update -j 2 -t src\n BTW, the -j 2 option downloads 2 packages in parallel")
        
            run("wstool init -j 2 src kinetic-ros_comm-wet.rosinstall")
        else:
            _pp("Looks like you had already tried 'wstool init...', so continuing with 'wstool update...'")
            run("wstool update -j 2 -t src")

        rval = _get_input("Did wstool download everything ok?\n(NO to quit & resolve, ENTER to continue)")
        if rval == "NO":
            return

        # Resolve dependencies
        run("rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:jessie")

        install_dir = home_path + INSTALL_DIR 
        
        _fp("All dependencies have been resolved, going to start compiling and install into: " + install_dir)
        
        if not fabfiles.exists(install_dir):
            run("mkdir -p " + install_dir)

        rval = _get_input("Continue with compile or skip? SKIP to skip compile, ENTER to continue...")
        if rval != "SKIP":
            run("./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")


            _rval = _get_input("Did the compile succeed?\n(NO to quit and fix, ENTER to continue)")
            if rval == "NO":
                return

        
        src_cmd = "source " + install_dir + "/setup.bash"
        if run("grep '" + src_cmd + "' ~/.bashrc", warn_only=True).succeeded:
            _fp("Sourcing of ROS env setup is already in your bashrc")
        else:
            _pp("Going to add ROS source setup into your bashrc")
            run("echo '" + src_cmd + "\n' >> ~/.bashrc")
            run("echo 'export ROSBOTS_MASTER=1\n' >> ~/.bashrc")

            # Add some custom python library paths
            run("echo 'export PYTHONPATH=/home/pi/lib/python:${PYTHONPATH}\n' >> ~/.bashrc") 

            # Add other setups for rosbots
            put("./sourceme_rosbots.bash", "~/")
            run("echo 'source ~/sourceme_rosbots.bash' >> ~/.bashrc")

    _pp("All ROS components should be compiled and installed. Going to set up init.d to run ROSBots as a service.")

    # Copy over the rosbots init script - which is kicked off by the init.d
    # service framework
    put("./rosbots_startup.sh", "~/rosbots_startup.sh")
    run("chmod +x ~/rosbots_startup.sh")

    # Set up and install the init.d service which will fork and call
    # the rosbots startup script above
    put("./rosbots_service_template.bash", "~/rosbots_template")
    run("cat rosbots_template | sed 's/_TEMPLATE_HOME/" + home_path.replace("/", "\/") + "/' | sed 's/_TEMPLATE_WS_PATH/" + ws_dir.replace("/", "\/") + "/' > rosbots")
    run("rm rosbots_template")

    sudo("mv rosbots /etc/init.d/")
    sudo("chown root:root /etc/init.d/rosbots")
    sudo("chmod 755 /etc/init.d/rosbots")
    sudo("update-rc.d rosbots defaults")
    sudo("systemctl daemon-reload")
    sudo("systemctl stop rosbots")
    sudo("systemctl start rosbots")
    
    _fp("Done...")
    
