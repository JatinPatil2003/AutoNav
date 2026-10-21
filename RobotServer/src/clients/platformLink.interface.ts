export type PlatformLinkFrameHandler = (frame: unknown) => void;

/**
 * This robot's persistent, authenticated link to PlatformServer's
 * `/ws/robot-link` — unlike `IPlatformClient` (one-shot REST calls),
 * "robot online" on PlatformServer *is* this socket being open, so it's
 * modeled as a connection with a lifecycle rather than a request/response
 * client. Frames are untyped at this layer (`unknown`) — validating and
 * dispatching them is `teleopLink.handler.ts`'s job, kept separate so this
 * interface stays a pure transport.
 */
export interface IPlatformLink {
  start(): void;
  stop(): void;
  isConnected(): boolean;
  send(frame: unknown): void;
  onFrame(handler: PlatformLinkFrameHandler): void;
  /** Tears down the current connection (if any) and reconnects immediately, re-reading whatever credential dial() uses at the time — needed after a credential rotation so the link doesn't keep using a stale secret until its next natural disconnect. */
  forceReconnect(): void;
}
