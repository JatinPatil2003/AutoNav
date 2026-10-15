import { RTCIceCandidate, RTCPeerConnection } from "werift";
import { WsRouter } from "../../core/wsRouter";
import { logger } from "../../common/utils/logger.util";
import { IVideoSource } from "./videoSource.interface";

/**
 * Registers /ws/webrtc — same signaling shape as the old WebServer's
 * aiortc flow (offer sent first, then SDP answer + ICE candidates
 * exchanged over this same socket), reimplemented on werift. See
 * ffmpegVideoSource.ts for the one piece (the actual camera track) that
 * isn't finished yet.
 */
export function registerVideoWebSocket(wsRouter: WsRouter, videoSource: IVideoSource): void {
  wsRouter.register("/ws/webrtc", (socket) => {
    logger.info("WebRTC viewer connected");

    const pc = new RTCPeerConnection({
      iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
    });

    try {
      pc.addTrack(videoSource.getTrack());
    } catch (err) {
      logger.error({ err }, "Video source is not ready — closing WebRTC viewer connection");
      socket.close(1011, "video source not implemented yet");
      return;
    }

    pc.onicecandidate = (event) => {
      const candidate = event.candidate;
      if (candidate) {
        socket.send(
          JSON.stringify({ candidate: candidate.candidate, sdpMid: candidate.sdpMid, sdpMLineIndex: candidate.sdpMLineIndex }),
        );
      }
    };

    void (async () => {
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
      socket.send(JSON.stringify({ sdp: pc.localDescription?.sdp, type: pc.localDescription?.type }));
    })();

    socket.on("message", (raw) => {
      void (async () => {
        try {
          const data = JSON.parse(raw.toString());
          if (data.sdp) {
            await pc.setRemoteDescription({ sdp: data.sdp, type: data.type });
          }
          if (data.candidate) {
            await pc.addIceCandidate(
              new RTCIceCandidate({
                candidate: data.candidate,
                sdpMid: data.sdpMid,
                sdpMLineIndex: data.sdpMLineIndex,
              }),
            );
          }
        } catch (err) {
          logger.error({ err }, "Failed to process WebRTC signaling message");
        }
      })();
    });

    socket.on("close", () => {
      logger.info("WebRTC viewer disconnected");
      videoSource.stop();
      void pc.close();
    });
  });
}
