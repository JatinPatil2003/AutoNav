import { useEffect, useState } from "react";

interface TopicFrame<T = unknown> {
  type: string;
  data: T;
}

type Listener = (data: unknown) => void;

/**
 * Single connection to RobotServer's /ws/topics, multiplexed by `type`.
 * Segregating by topic happens here, client-side — the server just tags
 * every frame, so a new topic in the future needs no plumbing change on
 * either side. Reconnects on close with a fixed backoff since this runs
 * on a kiosk tablet that's expected to stay open indefinitely.
 */
class TopicsSocket {
  private socket: WebSocket | null = null;
  private readonly listeners = new Map<string, Set<Listener>>();
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;

  connect(): void {
    if (this.socket) return;

    const protocol = window.location.protocol === "https:" ? "wss" : "ws";
    this.socket = new WebSocket(`${protocol}://${window.location.host}/ws/topics`);

    this.socket.addEventListener("message", (event) => {
      let frame: TopicFrame;
      try {
        frame = JSON.parse(event.data);
      } catch {
        return;
      }
      for (const listener of this.listeners.get(frame.type) ?? []) {
        listener(frame.data);
      }
    });

    this.socket.addEventListener("close", () => {
      this.socket = null;
      this.reconnectTimer = setTimeout(() => this.connect(), 3000);
    });
  }

  disconnect(): void {
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.socket?.close();
    this.socket = null;
  }

  subscribe(topic: string, listener: Listener): () => void {
    this.connect();
    if (!this.listeners.has(topic)) this.listeners.set(topic, new Set());
    this.listeners.get(topic)!.add(listener);

    return () => {
      this.listeners.get(topic)?.delete(listener);
    };
  }
}

export const topicsSocket = new TopicsSocket();

/** Latest payload seen on a given topic, or `undefined` until the first frame arrives. */
export function useTopic<T>(topic: string): T | undefined {
  const [data, setData] = useState<T | undefined>(undefined);

  useEffect(() => {
    return topicsSocket.subscribe(topic, (payload) => setData(payload as T));
  }, [topic]);

  return data;
}
