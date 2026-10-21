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
  private readonly callbacks = new Map<string, Array<(msg: unknown) => void>>();

  isNodeAlive(): boolean {
    return this.nodeAlive;
  }

  publish<T>(type: string, topicName: string, message: T): void {
    this.published.push({ type, topicName, message });
  }

  /**
   * Stores the callback (if given) so fireMessage() can invoke it, but
   * otherwise stays inert — most existing tests never call fireMessage()
   * and instead seed getLatest()'s return value directly via setLatest(),
   * which this doesn't change.
   */
  subscribe<T>(_type: string, topicName: string, callback?: (msg: T) => void): void {
    if (!callback) return;
    const list = this.callbacks.get(topicName) ?? [];
    list.push(callback as (msg: unknown) => void);
    this.callbacks.set(topicName, list);
  }

  getLatest<T>(topicName: string): T | null {
    return (this.latest.get(topicName) as T | undefined) ?? null;
  }

  /** Test helper: seeds what getLatest() returns, standing in for a real subscription's cache. */
  setLatest<T>(topicName: string, message: T): void {
    this.latest.set(topicName, message);
  }

  /** Test helper: simulates a new message arriving on a topic — updates getLatest() and invokes any callback registered via subscribe(), for tests (e.g. TopicFrequencyMonitor) that depend on the callback actually firing rather than just reading getLatest(). */
  fireMessage<T>(topicName: string, message: T): void {
    this.latest.set(topicName, message);
    for (const callback of this.callbacks.get(topicName) ?? []) {
      callback(message);
    }
  }

  /** No test currently fires this timer manually — tests invoke TeleopService.publishTick() directly instead. */
  createTimer(_periodNs: bigint, _callback: () => void): rclnodejs.Timer {
    return {} as unknown as rclnodejs.Timer;
  }
}
