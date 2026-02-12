import json
import cv2
import av
import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
    RTCConfiguration,
    RTCIceServer
)

router = APIRouter()

pcs = set()


class CameraTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()

        self.cap = cv2.VideoCapture("/dev/robotcam")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # if not self.cap.isOpened():
        #     raise RuntimeError("Camera not available")

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()

        if not ret or frame is None:
            await asyncio.sleep(0.02)
            return await self.recv()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        if self.cap.isOpened():
            self.cap.release()
        super().stop()



@router.websocket("/ws/webrtc")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Viewer connected")

    websocket_open = True

    async def safe_send(msg: str):
        nonlocal websocket_open
        if not websocket_open:
            return
        try:
            await websocket.send_text(msg)
        except Exception:
            websocket_open = False

    config = RTCConfiguration(
        iceServers=[
            RTCIceServer(urls=[
                "stun:stun.l.google.com:19302",
                "stun:stun1.l.google.com:19302",
            ])
        ]
    )

    pc = RTCPeerConnection(configuration=config)
    pcs.add(pc)

    track = CameraTrack()
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("PC state:", pc.connectionState)
        if pc.connectionState in ["failed", "closed", "disconnected"]:
            await pc.close()

    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        if candidate:
            await safe_send(json.dumps({
                "candidate": candidate.to_sdp()
            }))

    try:
        # create offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        await safe_send(json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }))

        while websocket_open:
            try:
                message = await websocket.receive_text()
            except WebSocketDisconnect:
                break

            data = json.loads(message)

            if "sdp" in data:
                answer = RTCSessionDescription(
                    sdp=data["sdp"],
                    type=data["type"]
                )
                await pc.setRemoteDescription(answer)

    finally:
        print("Viewer disconnected")

        websocket_open = False

        try:
            track.stop()
        except:
            pass

        try:
            await pc.close()
        except:
            pass

        pcs.discard(pc)
