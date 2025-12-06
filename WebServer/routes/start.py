from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE, DEVNULL
import os
import signal
import psutil

from models.model import Emergency, LedStatus
from ros.topics import set_emergency_status, set_led_status

router = APIRouter()
process = None
robot_status = "stopped"

@router.get("/robot/start")
async def start_robot():
    global process, robot_status
    if not process:
        process = Popen(
            ['ros2', 'launch', 'autonav_bringup', 'autonav_bringup.launch.py'], 
            preexec_fn=os.setsid, stdout=DEVNULL)
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

@router.post("/emergency")
async def emergency(status: Emergency):
    print(f"Emergency status set to: {status.status}")
    set_emergency_status(status.status)
    return {'status': status}

@router.post("/led_status")
async def led_status(status: LedStatus):
    print(f"LED status set to: {status.status}")
    set_led_status(status.status)
    return {'status': status}