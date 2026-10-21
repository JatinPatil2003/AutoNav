import { ChildProcessByStdio, spawn } from "node:child_process";
import type { Readable } from "node:stream";
import { MediaStreamTrack, RtpHeader, RtpPacket } from "werift";
import { logger } from "../../common/utils/logger.util";
import { AnnexBParser } from "./annexBParser";
import { isVclNalUnit, packetizeNalUnit } from "./h264RtpPacketizer";
import { IVideoSource } from "./videoSource.interface";

const RTP_CLOCK_RATE_HZ = 90_000;
const SIGKILL_GRACE_MS = 2_000;

function ffmpegArgs(cameraDevice: string): string[] {
  return [
    "-f",
    "v4l2",
    "-i",
    cameraDevice,
    "-c:v",
    "libx264",
    "-preset",
    "ultrafast",
    "-tune",
    "zerolatency",
    "-profile:v",
    "baseline",
    "-pix_fmt",
    "yuv420p",
    "-g",
    "30",
    "-f",
    "h264",
    "-",
  ];
}

/**
 * Spawns ffmpeg against a V4L2 camera device and feeds its raw Annex-B
 * H264 output through AnnexBParser/h264RtpPacketizer.ts (pure, unit-tested
 * logic) into werift's MediaStreamTrack.writeRtp() — werift only accepts
 * already-packetized RTP (confirmed against its shipped types: no
 * raw-frame constructor), and its H264RtpPayload/NalUnitType helpers are
 * decode-side only, not a packetizer, so this side has to build RTP
 * packets itself.
 *
 * werift's RTCRtpSender overwrites payloadType and ssrc on every packet
 * right before sending (confirmed against its source) — this never sets
 * either field meaningfully, since whatever's set here is discarded.
 * sequenceNumber and the RTP timestamp pass through untouched and are
 * this class's responsibility to manage.
 *
 * Packets written before the peer connection's DTLS transport is
 * "connected" are silently dropped by werift (not buffered) — ffmpeg
 * still starts as soon as getTrack() is called rather than waiting for
 * that signal, accepting a brief window of wasted encoding during ICE/DTLS
 * negotiation in exchange for not threading a readiness callback through
 * video.wsServer.ts. If that startup CPU cost ever matters in practice on
 * real hardware, revisit this trade-off.
 */
export class FfmpegVideoSource implements IVideoSource {
  private track: MediaStreamTrack | undefined;
  private process: ChildProcessByStdio<null, Readable, Readable> | undefined;
  private readonly parser = new AnnexBParser();
  private sequenceNumber = Math.floor(Math.random() * 0x10000);
  private readonly initialTimestamp = Math.floor(Math.random() * 0x100000000);
  private startedAtMs = 0;
  /** Null until the first NAL unit is processed; thereafter holds the current access unit's timestamp — see sendNalUnit's doc comment for why non-VCL NALs reuse it instead of advancing. */
  private accessUnitTimestamp: number | undefined;

  constructor(private readonly cameraDevice: string) {}

  getTrack(): MediaStreamTrack {
    if (this.track) {
      return this.track;
    }

    this.track = new MediaStreamTrack({ kind: "video" });
    this.startFfmpeg();
    return this.track;
  }

  stop(): void {
    const proc = this.process;
    this.process = undefined;
    if (proc) {
      proc.kill("SIGTERM");
      const killTimer = setTimeout(() => proc.kill("SIGKILL"), SIGKILL_GRACE_MS);
      proc.once("exit", () => clearTimeout(killTimer));
    }
    this.track?.stop();
    this.track = undefined;
    this.accessUnitTimestamp = undefined;
  }

  private startFfmpeg(): void {
    this.startedAtMs = Date.now();
    const proc = spawn("ffmpeg", ffmpegArgs(this.cameraDevice), { stdio: ["ignore", "pipe", "pipe"] });
    this.process = proc;

    proc.stdout.on("data", (chunk: Buffer) => {
      for (const nal of this.parser.feed(chunk)) {
        this.sendNalUnit(nal);
      }
    });
    proc.stderr.on("data", (chunk: Buffer) => {
      logger.debug({ line: chunk.toString().trim() }, "ffmpeg (video source) stderr");
    });
    proc.on("exit", (code, signal) => {
      logger.warn({ code, signal, cameraDevice: this.cameraDevice }, "ffmpeg video source process exited");
    });
    proc.on("error", (err) => {
      logger.error({ err, cameraDevice: this.cameraDevice }, "failed to spawn ffmpeg for video source");
    });
  }

  /**
   * One VCL (slice) NAL unit per access unit is what libx264's baseline/
   * ultrafast settings produce in practice, so a fresh RTP timestamp is
   * only computed when a VCL NAL is seen; non-VCL NALs (SPS/PPS/SEI,
   * which precede the slice in decode order) reuse whatever access
   * unit's timestamp is already current.
   */
  private sendNalUnit(nal: Buffer): void {
    if (!this.track) return;

    if (isVclNalUnit(nal) || this.accessUnitTimestamp === undefined) {
      this.accessUnitTimestamp = this.computeRtpTimestamp();
    }
    const timestamp = this.accessUnitTimestamp;

    for (const { payload, marker } of packetizeNalUnit(nal)) {
      this.writeRtpPacket(payload, timestamp, marker);
    }
  }

  private writeRtpPacket(payload: Buffer, timestamp: number, marker: boolean): void {
    if (!this.track) return;

    // payloadType/ssrc are set here only because RtpHeader has no sensible
    // "unset" default — werift's RTCRtpSender overwrites both with the
    // negotiated codec's payload type and this sender's real ssrc before
    // the packet ever reaches the wire (see this class's doc comment).
    const header = new RtpHeader({
      marker,
      payloadType: 0,
      sequenceNumber: this.sequenceNumber,
      timestamp,
      ssrc: 0,
    });
    this.sequenceNumber = (this.sequenceNumber + 1) & 0xffff;
    this.track.writeRtp(new RtpPacket(header, payload));
  }

  private computeRtpTimestamp(): number {
    const elapsedSec = (Date.now() - this.startedAtMs) / 1000;
    return (this.initialTimestamp + Math.round(elapsedSec * RTP_CLOCK_RATE_HZ)) % 0x100000000;
  }
}
