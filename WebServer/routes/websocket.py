from fastapi import WebSocket, WebSocketDisconnect
from fastapi import APIRouter
import asyncio
import json

router = APIRouter()

autonav_connections = []
joystick_connections = []

async def broadcast_message(message: dict):
    """Send message to all active websocket clients"""
    for conn in autonav_connections:
        try:
            await conn.send_json(message)
        except Exception:
            pass

@router.websocket("/ws/autonav")
async def websocket_route(websocket: WebSocket):
    await websocket.accept()
    autonav_connections.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
    except WebSocketDisconnect:
        autonav_connections.remove(websocket)

@router.websocket("/ws/joystick")
async def websocket_route(websocket: WebSocket):
    from ros.topics import set_joystick_velocity
    from models.model import Velocity

    await websocket.accept()
    joystick_connections.append(websocket)
    try:
        while True:
            raw_data = await websocket.receive_text()
            try:
                data = json.loads(raw_data)
                if data["type"] == "joystick":
                    velocity = Velocity(**data["data"])
                    set_joystick_velocity(velocity)
            except Exception as e:
                print(f"Error processing message: {e}")
    except WebSocketDisconnect:
        joystick_connections.remove(websocket)
