from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE
import os
import signal
import psutil

router = APIRouter()
process = None
robot_status = "stopped"

@router.get("/robot/start")
async def start_robot():
    global process, robot_status
    if not process:
        process = Popen(['ros2', 'launch', 'autoserve_gazebo', 'gazebo.launch.py'], preexec_fn=os.setsid)
    robot_status = "started"
    return {'status': robot_status}

@router.get("/robot/stop")
async def stop_robot():
    global process, robot_status
    if process:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait()
        process = None
    robot_status = "stopped"
    return {'status': robot_status}

@router.get("/robot/status")
async def status_robot():
    return {'status': robot_status}