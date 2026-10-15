import type { MediaStreamTrack } from "werift";

/**
 * Produces the video track a WebRTC peer connection streams to viewers.
 * Kept behind an interface so video.wsServer.ts's signaling logic doesn't
 * depend on how the frames are actually produced.
 */
export interface IVideoSource {
  getTrack(): MediaStreamTrack;
  stop(): void;
}
