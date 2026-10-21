import { useEffect, useRef, useState } from "react";
import { buildTelemetrySocketUrl } from "../../app/ws/telemetrySocketUrl";
import {
  MapSnapshotFrame,
  NavStatusFrame,
  PoseUpdateFrame,
  RobotStatusFrame,
  TelemetryFrame,
} from "./telemetry.types";

const INITIAL_RECONNECT_DELAY_MS = 1_000;
const MAX_RECONNECT_DELAY_MS = 15_000;

export interface RobotTelemetryState {
  pose: PoseUpdateFrame | null;
  map: MapSnapshotFrame | null;
  navStatus: NavStatusFrame | null;
  robotStatus: RobotStatusFrame | null;
  /** Null until the first robot_link_state frame arrives (just after the socket opens). */
  robotOnline: boolean | null;
  /** This browser's own connection to PlatformServer's relay, not the robot-link's state — can be true while robotOnline is false. */
  socketConnected: boolean;
}

/**
 * The first WebSocket client in this app (see PlaceholderPage comment
 * history) — subscribes to PlatformServer's /ws/telemetry for one robot
 * and exposes its latest pose/map/nav-status/robot-status, same cookie
 * session the rest of the app already relies on (credentials carry over
 * automatically for a same-origin-cookie WebSocket handshake). Reconnects
 * with backoff on drop, same philosophy as RobotServer's own platformLink
 * client, just without that side's fail-safe teleop.stop() concern since
 * this connection is read-only.
 */
export function useRobotTelemetry(robotId: string | null): RobotTelemetryState {
  const [state, setState] = useState<RobotTelemetryState>({
    pose: null,
    map: null,
    navStatus: null,
    robotStatus: null,
    robotOnline: null,
    socketConnected: false,
  });

  const reconnectDelayRef = useRef(INITIAL_RECONNECT_DELAY_MS);

  useEffect(() => {
    if (!robotId) {
      return;
    }

    let socket: WebSocket | undefined;
    let reconnectTimer: ReturnType<typeof setTimeout> | undefined;
    let stopped = false;

    const connect = (): void => {
      if (stopped) return;

      socket = new WebSocket(buildTelemetrySocketUrl(robotId));

      socket.onopen = () => {
        reconnectDelayRef.current = INITIAL_RECONNECT_DELAY_MS;
        setState((prev) => ({ ...prev, socketConnected: true }));
      };

      socket.onmessage = (event) => {
        let frame: TelemetryFrame;
        try {
          frame = JSON.parse(event.data);
        } catch {
          return;
        }

        setState((prev) => {
          switch (frame.type) {
            case "pose.update":
              return { ...prev, pose: frame };
            case "map.snapshot":
              return { ...prev, map: frame };
            case "nav.status":
              return { ...prev, navStatus: frame };
            case "robot.status":
              return { ...prev, robotStatus: frame };
            case "robot_link_state":
              return { ...prev, robotOnline: frame.online };
            default:
              return prev;
          }
        });
      };

      socket.onclose = () => {
        setState((prev) => ({ ...prev, socketConnected: false, robotOnline: null }));
        if (stopped) return;

        reconnectTimer = setTimeout(connect, reconnectDelayRef.current);
        reconnectDelayRef.current = Math.min(reconnectDelayRef.current * 2, MAX_RECONNECT_DELAY_MS);
      };

      socket.onerror = () => {
        // onclose always follows onerror for a WebSocket — reconnect logic lives there, not here.
      };
    };

    connect();

    return () => {
      stopped = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      socket?.close();
      setState({
        pose: null,
        map: null,
        navStatus: null,
        robotStatus: null,
        robotOnline: null,
        socketConnected: false,
      });
    };
  }, [robotId]);

  return state;
}
