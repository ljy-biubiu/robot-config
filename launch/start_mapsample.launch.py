#from launch import LaunchDescription
#3i#mport launch_ros.actions
#im#port ament_index_python
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions

import subprocess

def runcmd(command):
    ret = subprocess.run(command,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",timeout=1)
    if ret.returncode == 0:
        print("success:",ret)
    else:
        print("error:",ret)

    
 runcmd(["touch","123.txtx"])
