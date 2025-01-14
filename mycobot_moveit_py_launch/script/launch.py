#!/usr/bin/env python3
import subprocess
import os
from multiprocessing import Pool, Process
import time

# 获取环境变量的值
user_name = os.getenv('USER_NAME', 'u20')  # 默认值为'u20'，以防环境变量未设置

def run_1():
    os.system(f"bash /home/{user_name}/catkin_ws/src/mycobot_sim_3.0/mycobot_moveit_py_launch/sh/demo_gazebo.sh")
    
def run_2():
    os.system(f"bash /home/{user_name}/catkin_ws/src/mycobot_sim_3.0/mycobot_moveit_py_launch/sh/mycobot_moveit_py_image_sub.sh")
    
def run_3():
    os.system(f"bash /home/{user_name}/catkin_ws/src/mycobot_sim_3.0/mycobot_moveit_py_launch/sh/mycobot_moveit_py_move.sh")
    
def run_4():
    os.system(f"bash /home/{user_name}/catkin_ws/src/mycobot_sim_3.0/mycobot_moveit_py_launch/sh/mycobot_moveit_py_search.sh")
    
if __name__ == '__main__':
    p1 = Process(target=run_1)
    p2 = Process(target=run_2)   
    p3 = Process(target=run_3)    
    p4 = Process(target=run_4)    
    
    p1.start()
    time.sleep(5)
    p2.start()
    time.sleep(2)
    p3.start()
    time.sleep(2)
    p4.start()