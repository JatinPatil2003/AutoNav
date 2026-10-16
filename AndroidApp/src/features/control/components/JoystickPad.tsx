import { Joystick } from "react-joystick-component";

/**
 * react-joystick-component doesn't re-export its update-event type from
 * the package's main entry, so this is a minimal local shape covering the
 * two fields actually used — avoids depending on its internal file layout.
 */
interface JoystickMoveEvent {
  x: number | null;
  y: number | null;
}

interface JoystickPadProps {
  disabled?: boolean;
  onMove: (linear: number, angular: number) => void;
  onStop: () => void;
}

const SIZE = 180;
const STICK_SIZE = SIZE * 0.45;
// At full deflection the stick's center reaches the base's edge, so the
// stick itself (being STICK_SIZE wide) overhangs the base circle by
// STICK_SIZE/2 on every side — the wrapper needs that much extra room, or
// an overflow-hidden ancestor clips the stick right as it nears full travel.
const WRAPPER_SIZE = SIZE + STICK_SIZE;
// ~10Hz, matching the cadence ControlSessionService expects from clients
// (coalesced server-side to <=30Hz regardless, but no need to send faster).
const THROTTLE_MS = 100;

/**
 * react-joystick-component drives its stick via pointer events
 * (`touchAction: "none"`), so it handles touch natively inside the
 * WebView with no extra wiring — confirmed against its bundled source
 * before adopting it (see handback notes).
 */
export function JoystickPad({ disabled, onMove, onStop }: JoystickPadProps) {
  function handleMove(event: JoystickMoveEvent): void {
    const x = event.x ?? 0;
    const y = event.y ?? 0;
    // Stick forward/back -> linear velocity. Stick left/right -> angular
    // velocity; ROS REP-103 treats positive angular as counter-clockwise
    // (a left turn), so pushing the stick right (+x) maps to negative angular.
    onMove(y, -x);
  }

  return (
    // The glass backdrop is a separate, smaller sibling (not an
    // overflow-hidden wrapper around <Joystick>) so the stick can overhang
    // it freely at full deflection instead of getting clipped at the edge.
    <div className="relative" style={{ width: WRAPPER_SIZE, height: WRAPPER_SIZE }}>
      <div
        className="glass-card absolute inset-0 m-auto rounded-full shadow-xl shadow-slate-900/15"
        style={{ width: SIZE, height: SIZE }}
      />
      <div className="absolute inset-0 flex items-center justify-center">
        <Joystick
          size={SIZE}
          stickSize={STICK_SIZE}
          baseColor="rgba(10,141,124,0.14)"
          stickColor={disabled ? "#94a3b8" : "#0a8d7c"}
          throttle={THROTTLE_MS}
          disabled={disabled}
          move={handleMove}
          stop={onStop}
        />
      </div>
    </div>
  );
}
