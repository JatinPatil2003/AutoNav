import type { MediaStreamTrack } from "werift";
import { IVideoSource } from "./videoSource.interface";

/**
 * NOT YET IMPLEMENTED — this is the one piece of the whole RobotServer
 * rewrite that genuinely needs a real camera to finish, so it's isolated
 * here behind IVideoSource rather than guessed at.
 *
 * The old WebServer used aiortc + OpenCV: cv2.VideoCapture reads BGR
 * frames, converts to RGB, wraps each in an av.VideoFrame, done — aiortc
 * handles encoding internally. werift's `MediaStreamTrack` (confirmed from
 * its shipped type defs, node_modules/werift/lib/webrtc/src/media/track.d.ts)
 * has no equivalent "hand me raw frames" constructor — the lowest-level hook
 * it exposes is `track.writeRtp(rtp: RtpPacket | Buffer)`. The intended
 * implementation: spawn ffmpeg reading `cameraDevice`, encode to H264,
 * packetize the Annex-B stream into RTP packets (payload type/framing per
 * RFC 6184), and feed those to `writeRtp`. That packetization step needs a
 * real camera to verify against, so it's left as this stub.
 */
export class FfmpegVideoSource implements IVideoSource {
  constructor(private readonly cameraDevice: string) {}

  getTrack(): MediaStreamTrack {
    throw new Error(
      `FfmpegVideoSource for ${this.cameraDevice} is not implemented yet — see the comment in ffmpegVideoSource.ts`,
    );
  }

  stop(): void {
    // nothing to clean up until getTrack() actually spawns ffmpeg
  }
}
