import { Clock, SystemClock } from "../common/clock";
import { TopicGateway } from "./rosTopicGateway";

const MAX_SAMPLES = 20;

interface TrackedTopic {
  lastMessageAtMs: number | null;
  intervalsMs: number[];
}

/**
 * The native equivalent of `ros2 topic hz <topic>`, built on
 * RosTopicGateway's existing subscribe()/getLatest() primitives rather
 * than a new ROS-side mechanism — diagnostics' only job is to report what
 * these already-subscribed topics are doing, not to add new ROS
 * subscriptions beyond what telemetry.broadcaster.ts already needs
 * (though this can track any topic, including ones nothing else
 * subscribes to, since RosTopicGateway.subscribe() is idempotent and
 * shared).
 */
export class TopicFrequencyMonitor {
  private readonly tracked = new Map<string, TrackedTopic>();

  constructor(
    private readonly topicGateway: TopicGateway,
    private readonly clock: Clock = new SystemClock(),
  ) {}

  /** Starts tracking a topic's message rate. Idempotent per topic name, same as the underlying gateway's subscribe(). */
  track(type: string, topicName: string): void {
    if (this.tracked.has(topicName)) return;

    const entry: TrackedTopic = { lastMessageAtMs: null, intervalsMs: [] };
    this.tracked.set(topicName, entry);

    this.topicGateway.subscribe(type, topicName, () => {
      const now = this.clock.now();
      if (entry.lastMessageAtMs !== null) {
        entry.intervalsMs.push(now - entry.lastMessageAtMs);
        if (entry.intervalsMs.length > MAX_SAMPLES) {
          entry.intervalsMs.shift();
        }
      }
      entry.lastMessageAtMs = now;
    });
  }

  /** Rolling-average Hz over the last (up to) MAX_SAMPLES inter-message gaps. Null if this topic isn't tracked, or fewer than 2 messages have arrived yet to measure a gap from. */
  getHz(topicName: string): number | null {
    const entry = this.tracked.get(topicName);
    if (!entry || entry.intervalsMs.length === 0) {
      return null;
    }
    const avgIntervalMs = entry.intervalsMs.reduce((sum, ms) => sum + ms, 0) / entry.intervalsMs.length;
    return avgIntervalMs > 0 ? 1000 / avgIntervalMs : 0;
  }

  /** Milliseconds since the last message, or null if this topic isn't tracked or has never received one. */
  getLastMessageAgeMs(topicName: string): number | null {
    const entry = this.tracked.get(topicName);
    if (!entry || entry.lastMessageAtMs === null) {
      return null;
    }
    return this.clock.now() - entry.lastMessageAtMs;
  }
}
