import * as rclnodejs from "rclnodejs";

type AnyMessageType = rclnodejs.TypeClass<rclnodejs.MessageTypeClassName>;

/**
 * Generic pub/sub wrapper around rclnodejs, keyed by topic name. Carries no
 * feature-specific knowledge — every module-layer service that needs a ROS
 * topic goes through this instead of owning its own Publisher/Subscription.
 *
 * `type` is accepted as a plain string (cast internally) rather than
 * rclnodejs's generated union of known interface packages, so this also
 * works uniformly for project-local interface packages (see
 * RosServiceGateway, which has the same constraint for custom services).
 */
export class RosTopicGateway {
  private readonly publishers = new Map<string, rclnodejs.Publisher<AnyMessageType>>();
  private readonly subscriptions = new Map<string, rclnodejs.Subscription>();
  private readonly latest = new Map<string, unknown>();

  constructor(private readonly node: rclnodejs.Node) {}

  publish<T>(type: string, topicName: string, message: T): void {
    const publisher = this.getOrCreatePublisher(type, topicName);
    publisher.publish(message as unknown as rclnodejs.MessageType<AnyMessageType>);
  }

  /**
   * Subscribes to `topicName` once. A second call for the same topic name
   * is a no-op (the subscription — and whichever callback was passed the
   * first time — is already in place); `getLatest` keeps working
   * regardless of whether a callback was ever supplied.
   */
  subscribe<T>(type: string, topicName: string, callback?: (msg: T) => void): void {
    if (this.subscriptions.has(topicName)) {
      return;
    }

    const subscription = this.node.createSubscription(type as unknown as AnyMessageType, topicName, (msg) => {
      this.latest.set(topicName, msg);
      callback?.(msg as T);
    });
    this.subscriptions.set(topicName, subscription);
  }

  getLatest<T>(topicName: string): T | null {
    return (this.latest.get(topicName) as T | undefined) ?? null;
  }

  createTimer(periodNs: bigint, callback: () => void): rclnodejs.Timer {
    return this.node.createTimer(periodNs, callback);
  }

  /**
   * Cheap liveness probe for the underlying rclnodejs node — used by
   * teleopLink.handler's `health.query` reply so that check can stay a
   * TopicGateway caller (never importing rclnodejs directly) instead of
   * reaching past this primitive gateway.
   */
  isNodeAlive(): boolean {
    try {
      this.node.name();
      return true;
    } catch {
      return false;
    }
  }

  private getOrCreatePublisher(type: string, topicName: string): rclnodejs.Publisher<AnyMessageType> {
    let publisher = this.publishers.get(topicName);
    if (!publisher) {
      publisher = this.node.createPublisher(type as unknown as AnyMessageType, topicName);
      this.publishers.set(topicName, publisher);
    }
    return publisher;
  }
}

/**
 * RosTopicGateway's public surface, used for constructor typing in
 * services. Classes with private members can't be structurally matched by
 * a plain fake object in TypeScript, so services depend on this `Pick`
 * (derived directly from the one real class, not a hand-maintained
 * interface) rather than the concrete class — this is a TS typing
 * necessity, not a reintroduction of the per-feature gateway interfaces
 * this refactor removed.
 */
export type TopicGateway = Pick<RosTopicGateway, "publish" | "subscribe" | "getLatest" | "createTimer" | "isNodeAlive">;
