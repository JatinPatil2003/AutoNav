import { TopicGateway } from "../../ros/rosTopicGateway";
import { VelocityDto } from "./teleop.validation";

const PUBLISH_PERIOD_MS = 50; // 20Hz, matches the old WebServer's cmd_vel republish rate
const PUBLISH_PERIOD_NS = BigInt(PUBLISH_PERIOD_MS) * 1_000_000n;
const ANGULAR_SPEED = 0.6; // rad/s
const CMD_VEL_TOPIC = "cmd_vel";
const EMERGENCY_TOPIC = "/motor/emergency";

export class TeleopService {
  private linear = 0;
  private angular = 0;
  private active = false;
  private wasActive = false;

  constructor(private readonly topicGateway: TopicGateway) {
    this.topicGateway.createTimer(PUBLISH_PERIOD_NS, () => this.publishTick());
  }

  setVelocity(velocity: VelocityDto): void {
    this.linear = velocity.linear;
    this.angular = velocity.angular;
    this.active = velocity.linear !== 0 || velocity.angular !== 0;
  }

  setEmergency(status: boolean): void {
    this.topicGateway.publish("std_msgs/msg/Bool", EMERGENCY_TOPIC, { data: status });
  }

  /** Spins the robot in place `rotations` times, then stops. Used by navigation's localize-at-point flow to help AMCL converge. */
  async rotateInPlace(rotations: number): Promise<void> {
    // A touch over one full turn per rotation — gives AMCL a bit of margin
    // to converge rather than stopping at exactly 360 degrees.
    const rotationTimeSec = (2 * Math.PI + Math.PI / 3) / ANGULAR_SPEED;
    const totalMs = rotations * rotationTimeSec * 1000;

    this.setVelocity({ linear: 0, angular: ANGULAR_SPEED });
    await delay(totalMs);
    this.setVelocity({ linear: 0, angular: 0 });
  }

  /**
   * Not private: invoked by the constructor's timer in production, and
   * directly by unit tests to make a publish deterministic without waiting
   * on the real 20Hz tick.
   */
  publishTick(): void {
    if (this.active !== this.wasActive) {
      // Edge transition: publish an explicit zero once instead of letting
      // the last nonzero command keep being re-sent.
      this.topicGateway.publish("geometry_msgs/msg/Twist", CMD_VEL_TOPIC, zeroTwist());
      this.wasActive = this.active;
      return;
    }

    if (this.active) {
      this.topicGateway.publish("geometry_msgs/msg/Twist", CMD_VEL_TOPIC, {
        linear: { x: this.linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: this.angular },
      });
    }
  }
}

function zeroTwist() {
  return { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
