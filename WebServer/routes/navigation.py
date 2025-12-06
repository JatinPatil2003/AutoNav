#!/usr/bin/env python3
from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE, DEVNULL
import os
import signal
import psutil
import asyncio
from ros.topics import get_map_msg, get_location_msg, set_led_status, rotate_n_times, get_cov_threshold
from ros.service import set_initial_pose, loadMapService
from ros.action import send_goal, cancel_goal, get_navigation_feedback
from mongodb.db import listMaps, listGoal, saveGoal, getGoal, loadMap
from models.model import MapName, Goal, Pose, PointInfo

router = APIRouter()
process = None

@router.post("/navigation/start")
async def start_navigation(map_name: MapName):
    global process
    if not process:
        process = Popen(
            ['ros2', 'launch', 'autonav_navigation', 'navigation.launch.py', f'map_name:={map_name.name}.yaml'], 
            preexec_fn=os.setsid) #, stdout=DEVNULL)
    return {'Started'}

@router.post("/navigation/start_linux")
async def start_navigation():
    global process
    if not process:
        process = Popen(
            ['ros2', 'launch', 'autonav_navigation', 'navigation.launch.py'], 
            preexec_fn=os.setsid) #, stdout=DEVNULL)
    return {'Started'}

@router.get("/navigation/stop")
async def stop_naviagtion():
    global process
    if process:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait()
        process = None
    process = None
    return {'Stopped'}

@router.get("/navigation/list/maps")
async def list_maps():  
    return listMaps()

@router.post("/navigation/use_map")
async def save_map_data(name: MapName):
    global map_name
    map_name = name.name
    loadMap(map_name)
    return {"message": "Map data saved successfully"}

@router.get("/navigation/list/pose/{name}")
async def list_poses(name: str):
    return listGoal(name)

@router.post("/navigation/points")
async def get_navigation_points(map_name: MapName):
    point = listGoal(map_name.name)
    localization = []
    charging = []
    standby = []
    goals = []
    for p in point:
        if "localization" in p.lower():
            localization.append(p)
        elif "charger" in p.lower():
            charging.append(p)
        elif "standby" in p.lower():
            standby.append(p)
        else:
            goals.append(p)
    return {
        "localization": localization,
        "charging": charging,
        "standby": standby,
        "goals": goals
    }

@router.post("/navigation/localize")
async def start_localization(point: PointInfo):
    point_pose = getGoal(point.map_name, point.name)
    pose = Pose(x=point_pose['x'], y=point_pose['y'], theta=point_pose['theta'])
    set_initial_pose(pose)
    print(f"Rotating for localization at point: {point.name}")
    await asyncio.to_thread(rotate_n_times, 2)
    localize_success = get_cov_threshold()
    if localize_success:
        return {'success': True, 'message': f'Localization to {point.name} Successful'}
    return {'success': False, 'message': f'Localization Failed at {point.name}, Please Retry...'}

@router.get("/navigation/current/map")
async def get_current_map():
    map_msg = get_map_msg()
    if map_msg:
        return map_msg
    return None

@router.get("/navigation/current/location")
async def get_current_location():
    location_msg = get_location_msg()
    if location_msg:
        return location_msg
    return None

@router.post("/navigation/new/pose")
async def save_map_data(goal: Goal):
    print(goal)
    saveGoal(goal)
    return {"message": "Map data saved successfully"}

@router.post("/navigation/pose")
async def save_map_data(goal: Goal):
    print(goal)
    return getGoal(goal.map_name, goal.name)

@router.post("/navigation/goal/start")
async def save_map_data(pose: Pose):
    print(pose)
    send_goal(pose)
    return {'Navigation Started'}

@router.get("/navigation/goal/cancel")
async def stop_navigation():
    cancel_goal()
    return {'Navigation Stopped'}

@router.get("/navigation/goal/feedback")
async def feedback_navigation():
    feedback = get_navigation_feedback()
    return {feedback}

@router.post("/navigation/initial_pose")
async def initial_pose(pose: Pose):
    set_initial_pose(pose)
    return {'Initial Pose Set'}

@router.get("/navigation/list/maps_linux")
async def list_maps():
    """
    Return maps with names + thumbnail URLs
    """
    maps = listMaps()  # assume this returns a list of map names
    map_data = []
    for m in maps:
        loadMap(m)
        map_data.append({
            "id": m,
            "name": m,
            "thumbnailUrl": f"/maps/{m}.png"  # store thumbs in /static/maps/
        })
    print(map_data)
    return map_data

@router.post("/navigation/get_map")
def get_map(map_name: MapName):
    loadMap(map_name.name)
    return {
        "id": map_name.name,
        "name": map_name.name,
        "thumbnailUrl": f"/maps/{map_name.name}.png" 
    }


@router.post("/navigation/use_map_linux")
async def use_map(name: MapName):
    """
    Select and load a map
    """
    global map_name
    map_name = name.name
    loadMap(map_name)  # your existing DB/ROS logic
    loadMapService(map_name)
    return {"message": f"Map {map_name} loaded successfully"}
