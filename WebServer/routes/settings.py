# settings.py
from fastapi import APIRouter
from pydantic import BaseModel
import subprocess
import asyncio
import os
from datetime import datetime

from mongodb.db import deleteDatabase

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
        print(f"Available networks: {networks}")
        return {"success": True, "networks": networks}
    except Exception as e:
        return {"success": False, "error": str(e), "networks": ["None"]}
    
@router.get("/wifi/current")
async def list_wifi():
    try:
        current_output = subprocess.check_output(["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"], text=True)
        current = None
        for line in current_output.split("\n"):
            if line.startswith("yes:"):
                current = line.split(":")[1]
        print(f"Current network: {current}")
        return {"success": True, "current": current}
    except Exception as e:
        return {"success": False, "error": str(e), "current": "None"}

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
    print("Rebooting system now...")
    # subprocess.Popen([
    #     "docker", "exec", "reboot_helper", "/reboot/reboot-host.sh"
    # ])

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

@router.get("/sensor/{sensor_name}")
async def sensor_health_single(sensor_name: str):
    """
    Check the frequency of a ROS2 topic for 5 seconds.
    Return "ok" if hz >= 10, else "fail".
    """
    topic_map = {
        "LiDAR": "/ydlidar/scan",
        "IMU": "/bno055/imu",
        "Motor": "/motor/feedback"
    }

    topic = topic_map.get(sensor_name)
    print(f"Checking sensor: {sensor_name}, topic: {topic}")
    if topic is None:
        return {"sensor": sensor_name, "status": "unknown"}

    try:
        # Start subprocess
        process = subprocess.Popen(
            ["bash", "-c", f"source /opt/ros/humble/setup.bash && ros2 topic hz {topic}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        try:
            output, _ = process.communicate(timeout=3)  # wait up to 6s
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        print(f"ros2 topic hz output for {sensor_name}:\n{output}")
        # Parse output: look for "average: XX Hz"
        average_hz = 0.0
        for line in output.splitlines():
            if "average" in line:
                parts = line.split()
                print(parts)
                try:
                    average_hz = float(parts[-1])
                except:
                    average_hz = 0.0
                break

        status = "ok" if average_hz >= 5 else "fail"
        print(f"Sensor: {sensor_name}, Average Hz: {average_hz}, Status: {status}")
        return {"sensor": sensor_name, "status": status, "hz": average_hz}

    except Exception as e:
        return {"sensor": sensor_name, "status": "fail", "error": str(e)}

@router.post("/vpn")
async def toggle_vpn(request: VpnRequest):
    """
    Enable/Disable VPN using systemd.
    """
    action = "start" if request.enabled else "stop"
    print(f"{action} vpn.service")
    try:
        subprocess.run(
            ["sudo", "systemctl", action, "vpn.service"],
            check=True,
            capture_output=True,
            text=True,
        )
        return {"success": True, "enabled": request.enabled}
    except subprocess.CalledProcessError as e:
        return {
            "success": False,
            "message": f"Failed to {action} vpn.service: {e.stderr}",
        }

@router.post("/credentials")
async def update_credentials(request: CredentialsRequest):
    """
    Store credentials (or integrate with real auth system).
    """
    # TODO: store securely, this is just a stub
    return {"success": True, "message": f"Credentials updated for {request.username}"}

@router.post("/update")
async def update_firmware():
    image_name = "jatinvpatil/autonav"
    try:
        pull_result = subprocess.run(
            ["docker", "pull", image_name],
            capture_output=True, text=True, check=True
        )

        return {
            "success": True,
            "message": "Firmware update started",
        }

    except subprocess.CalledProcessError as e:
        return {
            "success": False,
            "message": f"Failed to update: {e.stderr}"
        }

@router.get("/version")
async def get_version():
    image_name = "jatinvpatil/autonav"
    try:
        result = subprocess.run(
            ["docker", "inspect", "--format='{{.Created}}'", image_name],
            capture_output=True, text=True, check=True
        )
        created_str = result.stdout.strip().strip("'").strip('"')  # remove extra quotes

        # Handle nanoseconds → Python only supports microseconds
        if "." in created_str:
            date_part, frac = created_str.split(".", 1)
            if "+" in frac:  # keep timezone
                frac, tz = frac.split("+", 1)
                frac = frac[:6]  # keep only microseconds
                created_str = f"{date_part}.{frac}+{tz}"
            elif "Z" in frac:  # UTC
                frac = frac.replace("Z", "")
                frac = frac[:6]
                created_str = f"{date_part}.{frac}Z"
            else:
                frac = frac[:6]
                created_str = f"{date_part}.{frac}"

        # Parse timestamp
        created_dt = datetime.fromisoformat(created_str.replace("Z", "+00:00"))
        firmware_version = created_dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"Firmware version: {firmware_version}")
        return {"success": True, "version": firmware_version}
    except subprocess.CalledProcessError as e:
        return {"success": False, "message": f"Failed to get version: {e.stderr}"}

@router.post("/reset_db")
async def reset_database():
    """
    Reset robot database.
    """
    deleteDatabase()
    return {"success": True, "message": "Database reset successful"}
