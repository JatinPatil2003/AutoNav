import { Clock, SystemClock } from "../../common/clock";
import { logger } from "../../common/utils/logger.util";
import { NavStatus } from "../navigation/navigation.types";
import { TopicGateway } from "../../ros/rosTopicGateway";

export interface VelocityDto {
  linear: number;
  angular: number;
}

const PUBLISH_PERIOD_MS = 50; // 20Hz, matches the old WebServer's cmd_vel republish rate
const PUBLISH_PERIOD_NS = BigInt(PUBLISH_PERIOD_MS) * 1_000_000n;
const ANGULAR_SPEED = 0.6; // rad/s
const CMD_VEL_TOPIC = "cmd_vel";
const EMERGENCY_TOPIC = "/motor/emergency";

/**
 * Default per-command deadline for `setVelocity`: 10 publish ticks at 20Hz,
 * matching the downstream `cmd_vel_timeout: 0.5` in
 * `autonav_utils/cmd_vel_republisher.py` — tolerates ordinary mobile
 * network jitter for a remote teleop client while still failing safe if
 * commands stop arriving (see `publishTick`'s staleness check below).
 */
export const REMOTE_COMMAND_TTL_MS = 500;

export class TeleopService {
  private linear = 0;
  private angular = 0;
  private active = false;
  private wasActive = false;
  private expiresAt = 0;
  private emergencyEngaged = false;

  /**
   * Resolves whether Nav2 is currently driving the robot itself (an active
   * NavigateToPose goal, which publishes its own cmd_vel via DWB) —
   * teleop must not fight that. Bound after construction via
   * `bindNavigationStatus` rather than taken as a constructor dependency:
   * NavigationService itself depends on this TeleopService instance (for
   * `rotateInPlace`), so the two can't be wired in either constructor
   * without a cycle — see app.ts's composition root.
   *
   * Deliberately NOT gated on "a launch is running" (navigation or
   * mapping): the navigation launch is already up and idle during
   * `localize`'s own `rotateInPlace` call (no goal has been sent yet), and
   * the mapping launch's entire point is to be driven around by teleop
   * while SLAM builds the map. Only an in-flight NavigateToPose goal
   * actually contends for cmd_vel.
   */
  private getNavigationStatus: (() => NavStatus) | undefined;

  constructor(
    private readonly topicGateway: TopicGateway,
    private readonly maxLinear: number,
    private readonly maxAngular: number,
    private readonly clock: Clock = new SystemClock(),
  ) {
    this.topicGateway.createTimer(PUBLISH_PERIOD_NS, () => this.publishTick());
  }

  bindNavigationStatus(getNavigationStatus: () => NavStatus): void {
    this.getNavigationStatus = getNavigationStatus;
  }

  /** This robot's real velocity envelope — the single source of truth clients should scale their joystick input against, rather than assuming a value. */
  getLimits(): { maxLinear: number; maxAngular: number } {
    return { maxLinear: this.maxLinear, maxAngular: this.maxAngular };
  }

  /**
   * Commands a velocity that expires after `ttlMs` unless refreshed by
   * another call — a per-command deadline rather than a fixed-interval
   * watchdog, so a single call (as `rotateInPlace` makes below, with its
   * own longer `ttlMs`) can still drive motion unattended for several
   * seconds while a remote client's commands still fail safe quickly.
   * Clamped to [-maxLinear, maxLinear] / [-maxAngular, maxAngular] and
   * refused outright while Nav2 is actively navigating a goal.
   */
  setVelocity(velocity: VelocityDto, ttlMs: number = REMOTE_COMMAND_TTL_MS): void {
    if (this.getNavigationStatus?.() === NavStatus.ACTIVE) {
      logger.debug({ velocity }, "teleop: refusing velocity command while Nav2 is actively navigating");
      return;
    }

    this.linear = clamp(velocity.linear, this.maxLinear);
    this.angular = clamp(velocity.angular, this.maxAngular);
    this.active = this.linear !== 0 || this.angular !== 0;
    this.expiresAt = this.clock.now() + ttlMs;
  }

  /**
   * Forces an immediate stop, bypassing the Nav2-active refusal above —
   * this is the fail-safe entry point (link drop, e-stop release, explicit
   * release frame) and must never be a no-op.
   */
  stop(): void {
    this.linear = 0;
    this.angular = 0;
    this.active = false;
  }

  setEmergency(status: boolean): void {
    this.emergencyEngaged = status;
    this.topicGateway.publish("std_msgs/msg/Bool", EMERGENCY_TOPIC, { data: status });
  }

  /** Last value this process itself commanded via setEmergency — not a readback from the ROS topic, since nothing subscribes to /motor/emergency. */
  isEmergencyEngaged(): boolean {
    return this.emergencyEngaged;
  }

  /** Spins the robot in place `rotations` times, then stops. Used by navigation's localize-at-point flow to help AMCL converge. */
  async rotateInPlace(rotations: number): Promise<void> {
    // A touch over one full turn per rotation — gives AMCL a bit of margin
    // to converge rather than stopping at exactly 360 degrees.
    const rotationTimeSec = (2 * Math.PI + Math.PI / 3) / ANGULAR_SPEED;
    const totalMs = rotations * rotationTimeSec * 1000;

    // Own TTL = duration + 1s buffer, unaffected by the shorter remote-
    // command default above (this call must keep driving unattended).
    this.setVelocity({ linear: 0, angular: ANGULAR_SPEED }, totalMs + 1_000);
    await delay(totalMs);
    this.stop();
  }

  /**
   * Not private: invoked by the constructor's timer in production, and
   * directly by unit tests to make a publish deterministic without waiting
   * on the real 20Hz tick.
   */
  publishTick(): void {
    if (this.active && this.clock.now() > this.expiresAt) {
      // Stale command — no explicit {0,0} arrived in time. Fall through to
      // the edge-triggered zero-publish below exactly as an explicit stop
      // would, rather than letting the last nonzero command keep being
      // re-sent forever (the pre-existing gap this TTL closes).
      this.active = false;
    }

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

function clamp(value: number, max: number): number {
  return Math.max(-max, Math.min(max, value));
}

function zeroTwist() {
  return { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
