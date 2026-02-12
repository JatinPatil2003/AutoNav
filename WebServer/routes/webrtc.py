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

from aiortc.sdp import candidate_from_sdp

router = APIRouter()
pcs = set()


class CameraTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()

        self.cap = cv2.VideoCapture("/dev/robotcam")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()
        if not ret:
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

    pc = RTCPeerConnection(
        RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
            ]
        )
    )

    pcs.add(pc)

    track = CameraTrack()
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    async def on_state():
        print("PC state:", pc.connectionState)

    @pc.on("iceconnectionstatechange")
    async def ice_state():
        print("ICE state:", pc.iceConnectionState)

    # ---------- SEND ICE TO FLUTTER ----------
    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        if candidate:
            await websocket.send_text(json.dumps({
                "candidate": candidate.to_sdp(),
                "sdpMid": candidate.sdpMid,
                "sdpMLineIndex": candidate.sdpMLineIndex,
            }))

    # ---------- SEND OFFER ----------
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    await websocket.send_text(json.dumps({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    }))

    try:
        while True:
            message = await websocket.receive_text()
            data = json.loads(message)

            # ---------- ANSWER ----------
            if "sdp" in data:
                answer = RTCSessionDescription(
                    sdp=data["sdp"],
                    type=data["type"]
                )
                await pc.setRemoteDescription(answer)

            # ---------- ICE FROM FLUTTER ----------
            if "candidate" in data:
                candidate = candidate_from_sdp(data["candidate"])
                candidate.sdpMid = data["sdpMid"]
                candidate.sdpMLineIndex = data["sdpMLineIndex"]
                await pc.addIceCandidate(candidate)

    except WebSocketDisconnect:
        pass

    print("Viewer disconnected")

    track.stop()
    await pc.close()
    pcs.discard(pc)
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

from aiortc.sdp import candidate_from_sdp

router = APIRouter()
pcs = set()


class CameraTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()

        self.cap = cv2.VideoCapture("/dev/robotcam")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()

        if not ret:
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

    pc = RTCPeerConnection(
        RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
            ]
        )
    )

    pcs.add(pc)

    track = CameraTrack()
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    async def on_state():
        print("PC state:", pc.connectionState)

    @pc.on("iceconnectionstatechange")
    async def ice_state():
        print("ICE state:", pc.iceConnectionState)

    # ---------- SEND ICE TO FLUTTER ----------
    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        if candidate:
            await websocket.send_text(json.dumps({
                "candidate": candidate.to_sdp(),
                "sdpMid": candidate.sdpMid,
                "sdpMLineIndex": candidate.sdpMLineIndex,
            }))

    # ---------- SEND OFFER ----------
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    await websocket.send_text(json.dumps({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    }))

    try:
        while True:
            message = await websocket.receive_text()
            data = json.loads(message)

            # ---------- ANSWER ----------
            if "sdp" in data:
                answer = RTCSessionDescription(
                    sdp=data["sdp"],
                    type=data["type"]
                )
                await pc.setRemoteDescription(answer)

            # ---------- ICE FROM FLUTTER ----------
            if "candidate" in data:
                candidate = candidate_from_sdp(data["candidate"])
                candidate.sdpMid = data["sdpMid"]
                candidate.sdpMLineIndex = data["sdpMLineIndex"]
                await pc.addIceCandidate(candidate)

    except WebSocketDisconnect:
        pass

    print("Viewer disconnected")

    track.stop()
    await pc.close()
    pcs.discard(pc)
