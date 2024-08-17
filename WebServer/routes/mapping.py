#!/usr/bin/env python3
from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE
import os
import signal
import psutil
from ros.topics import set_joystick_velocity, get_location_mapping_msg, get_map_msg
from ros.service import set_initial_pose
from ros.action import send_goal, cancel_goal, get_navigation_feedback
from mongodb.db import listMaps, listGoal, saveGoal, getGoal, saveMap
from models.model import MapName, Goal, Pose, Velocity
import threading

router = APIRouter()
process = None
def start_ros2_launch():
    global process
    if not process:
        process = Popen(['ros2', 'launch', 'autoserve_mapping', 'slam.launch.py'], preexec_fn=os.setsid)

def stop_ros2_launch():
    global process
    if process:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait()
        process = None

@router.get("/mapping/start")
async def start_navigation():
    threading.Thread(target=start_ros2_launch).start()
    return {'status': 'Started'}

@router.get("/mapping/stop")
async def stop_navigation():
    threading.Thread(target=stop_ros2_launch).start()
    return {'status': 'Stopped'}

@router.post("/joystick/control")
async def joy_control(velocity: Velocity):
    set_joystick_velocity(velocity)
    return {'set'}

@router.get("/mapping/current/location")
async def get_current_location():
    location_msg = get_location_mapping_msg()
    if location_msg:
        return location_msg
    return None

@router.get("/mapping/current/map")
async def get_map():
    map_msg = get_map_msg()
    if map_msg:
        return map_msg
    return None

@router.post("/mapping/save_map")
async def joy_control(map_name: MapName):
    Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', f'./maps/{map_name.name}'], preexec_fn=os.setsid)
    saveMap(map_name.name)
    return {'saved'}
