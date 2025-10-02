# settings.py
from fastapi import APIRouter
from pydantic import BaseModel
import subprocess
import asyncio
import os

from ros.topics import check_topic_data

router = APIRouter()

# -------------------- Request Models --------------------
class WifiRequest(BaseModel):
    ssid: str
    password: str

class VpnRequest(BaseModel):
    enabled: bool

class CredentialsRequest(BaseModel):
    username: str
    password: str

# -------------------- Endpoints --------------------

@router.get("/wifi/list")
async def list_wifi():
    try:
        # List networks
        networks_output = subprocess.check_output(["nmcli", "-t", "-f", "SSID", "dev", "wifi"], text=True)
        networks = list(dict.fromkeys(
            [n.strip() for n in networks_output.split("\n") if n.strip()]
        ))
        return {"success": True, "networks": networks}
    except Exception as e:
        return {"success": False, "error": str(e)}
    
@router.get("/wifi/current")
async def list_wifi():
    try:
        current_output = subprocess.check_output(["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"], text=True)
        current = None
        for line in current_output.split("\n"):
            if line.startswith("yes:"):
                current = line.split(":")[1]

        return {"success": True, "current": current}
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.post("/wifi/connect")
async def connect_wifi(request: WifiRequest):
    """
    Connect to a WiFi network using nmcli.
    """
    try:
        if request.password and request.password.strip():
            # Connect with new password
            cmd = ["nmcli", "dev", "wifi", "connect", request.ssid, "password", request.password]
        else:
            # Connect using saved credentials
            cmd = ["nmcli", "dev", "wifi", "connect", request.ssid]
            
        subprocess.check_output(cmd, text=True)
        return {"success": True, "message": f"Connected to {request.ssid}"}
    except subprocess.CalledProcessError as e:
        return {"success": False, "error": e.output}

@router.post("/reboot")
async def reboot_robot():
    """
    Reboot the Linux system after 5 seconds.
    """
    try:
        # Inform the client immediately
        message = "Robot will reboot in 5 seconds..."
        print(message)

        # Delay reboot asynchronously
        asyncio.create_task(delayed_reboot())

        return {"success": True, "message": message}
    except Exception as e:
        return {"success": False, "error": str(e)}

async def delayed_reboot():
    await asyncio.sleep(5)
    print("Rebooting system now...")
    subprocess.Popen(["sudo", "reboot"])

@router.post("/sync_datetime")
async def sync_datetime():
    """
    Sync date & time with system NTP.
    """
    try:
        subprocess.check_output(["timedatectl", "set-ntp", "true"])
        return {"success": True, "message": "Date & time synced via NTP"}
    except Exception as e:
        return {"success": False, "error": str(e)}

# @router.get("/sensors")
# async def sensor_health():
#     """
#     Return status of sensors.
#     Modify as per your sensor hardware access.
#     """
#     # Example: pretend we can check LiDAR and IMU via system commands or files
#     sensors_status = {}
#     try:
#         # Check LiDAR (stub: modify according to your actual device)
#         sensors_status["LiDAR"] = "ok" if os.path.exists("/dev/ttyUSB0") else "fail"

#         # Check IMU (stub: modify according to your actual device)
#         sensors_status["IMU"] = "ok" if os.path.exists("/dev/i2c-1") else "fail"

#         return {"success": True, "sensors": sensors_status}
#     except Exception as e:
#         return {"success": False, "error": str(e)}

@router.get("/sensor/{sensor_name}")
async def sensor_health_single(sensor_name: str):
    """
    Check the frequency of a ROS2 topic for 5 seconds.
    Return "ok" if hz >= 10, else "fail".
    """
    topic_map = {
        "LiDAR": "/scan",
        "IMU": "/bno055/imu_raw",
        "Motor": "/motor/feedback"
    }

    topic = topic_map.get(sensor_name)
    if topic is None:
        return {"sensor": sensor_name, "status": "unknown"}

    try:
        # Start subprocess
        process = subprocess.Popen(
            ["ros2", "topic", "hz", topic, "-w", "5"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        try:
            output, _ = process.communicate(timeout=6)  # wait up to 6s
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()

        # Parse output: look for "average: XX Hz"
        average_hz = 0.0
        for line in output.splitlines():
            if "average" in line:
                parts = line.split()
                try:
                    average_hz = float(parts[-2])
                except:
                    average_hz = 0.0
                break

        status = "ok" if average_hz >= 10 else "fail"
        print(f"Sensor: {sensor_name}, Average Hz: {average_hz}, Status: {status}")
        return {"sensor": sensor_name, "status": status, "hz": average_hz}

    except Exception as e:
        return {"sensor": sensor_name, "status": "fail", "error": str(e)}

@router.post("/vpn")
async def toggle_vpn(request: VpnRequest):
    """
    Enable/Disable VPN.
    This needs actual VPN config, here just store state.
    """
    # TODO: integrate with real VPN service
    return {"success": True, "enabled": request.enabled}

@router.post("/credentials")
async def update_credentials(request: CredentialsRequest):
    """
    Store credentials (or integrate with real auth system).
    """
    # TODO: store securely, this is just a stub
    return {"success": True, "message": f"Credentials updated for {request.username}"}

@router.post("/update")
async def update_firmware():
    """
    Trigger firmware/software update.
    """
    # TODO: implement actual update
    return {"success": True, "message": "Firmware update started"}

@router.post("/reset_db")
async def reset_database():
    """
    Reset robot database.
    """
    # TODO: implement actual reset
    return {"success": True, "message": "Database reset successful"}
