import * as rclnodejs from "rclnodejs";

export interface PublishedMessage {
  type: string;
  topicName: string;
  message: unknown;
}

export class FakeRosTopicGateway {
  published: PublishedMessage[] = [];
  nodeAlive = true;
  private readonly latest = new Map<string, unknown>();

  isNodeAlive(): boolean {
    return this.nodeAlive;
  }

  publish<T>(type: string, topicName: string, message: T): void {
    this.published.push({ type, topicName, message });
  }

  subscribe<T>(_type: string, _topicName: string, _callback?: (msg: T) => void): void {
    // No-op in tests — seed what getLatest() should return via setLatest() instead.
  }

  getLatest<T>(topicName: string): T | null {
    return (this.latest.get(topicName) as T | undefined) ?? null;
  }

  /** Test helper: seeds what getLatest() returns, standing in for a real subscription's cache. */
  setLatest<T>(topicName: string, message: T): void {
    this.latest.set(topicName, message);
  }

  /** No test currently fires this timer manually — tests invoke TeleopService.publishTick() directly instead. */
  createTimer(_periodNs: bigint, _callback: () => void): rclnodejs.Timer {
    return {} as unknown as rclnodejs.Timer;
  }
}
