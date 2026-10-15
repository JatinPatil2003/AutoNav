import { EventEmitter } from "node:events";

/**
 * Decouples the ROS layer from anything that reacts to ROS data (today:
 * the telemetry WebSocket). ROS callbacks run outside Express's request
 * lifecycle, so they publish here instead of reaching into
 * modules/telemetry directly — the old WebServer had ros/topics.py
 * importing routes/websocket.py, which made the dependency run backwards.
 */
export interface RobotEvents {
  map: MapUpdate;
  location: LocationUpdate;
  navStatus: NavStatusUpdate;
  robotStatus: RobotStatusUpdate;
}

export interface MapUpdate {
  info: {
    width: number;
    height: number;
    resolution: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

export interface LocationUpdate {
  x: number;
  y: number;
  theta: number;
}

export interface NavStatusUpdate {
  status: string;
}

export interface RobotStatusUpdate {
  internetConnectivity: boolean;
}

class TypedEventBus {
  private readonly emitter = new EventEmitter();

  emit<K extends keyof RobotEvents>(event: K, payload: RobotEvents[K]): void {
    this.emitter.emit(event, payload);
  }

  on<K extends keyof RobotEvents>(event: K, listener: (payload: RobotEvents[K]) => void): void {
    this.emitter.on(event, listener);
  }

  off<K extends keyof RobotEvents>(event: K, listener: (payload: RobotEvents[K]) => void): void {
    this.emitter.off(event, listener);
  }
}

export const eventBus = new TypedEventBus();
