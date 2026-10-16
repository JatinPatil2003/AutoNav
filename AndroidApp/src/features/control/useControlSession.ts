import { useCallback, useEffect, useRef, useState } from "react";
import { tokenStore } from "../../app/tokenStore";
import { setActiveControlSession } from "./activeControlSession";
import { ControlConnectionStatus, ControlSessionClient } from "./controlSession.client";
import { ServerFrame } from "./controlSession.protocol";

const WS_BASE_URL = import.meta.env.VITE_WS_BASE_URL;

export interface UseControlSessionResult {
  status: ControlConnectionStatus;
  lastServerFrame: ServerFrame | null;
  sendJoystick: (linear: number, angular: number) => void;
  sendEstop: (engaged: boolean) => void;
}

/**
 * Owns one ControlSessionClient for as long as `active` is true and
 * `robotId` is set, and registers it as the process-wide active session
 * (see activeControlSession.ts) for background-safety's e-stop-on-pause.
 *
 * `resetSignal` lets the caller force a fresh connection after a terminal
 * outcome (preempted/revoked) without needing to toggle `active` itself —
 * bumping it just remounts the effect below.
 */
export function useControlSession(
  robotId: string | null,
  active: boolean,
  resetSignal = 0,
): UseControlSessionResult {
  const [status, setStatus] = useState<ControlConnectionStatus>("idle");
  const [lastServerFrame, setLastServerFrame] = useState<ServerFrame | null>(null);
  const clientRef = useRef<ControlSessionClient | null>(null);
  const seqRef = useRef(0);

  useEffect(() => {
    if (!active || !robotId) {
      return;
    }

    seqRef.current = 0;
    const client = new ControlSessionClient({
      wsBaseUrl: WS_BASE_URL,
      robotId,
      getAccessToken: () => tokenStore.getAccessToken(),
      onStatusChange: setStatus,
      onServerFrame: setLastServerFrame,
    });

    clientRef.current = client;
    setActiveControlSession(client);
    client.connect();

    return () => {
      client.close();
      clientRef.current = null;
      setActiveControlSession(null);
      setLastServerFrame(null);
    };
  }, [robotId, active, resetSignal]);

  const sendJoystick = useCallback((linear: number, angular: number) => {
    const seq = seqRef.current++;
    clientRef.current?.send({ type: "joystick", seq, data: { linear, angular } });
  }, []);

  const sendEstop = useCallback((engaged: boolean) => {
    clientRef.current?.send({ type: "estop", data: { engaged } });
  }, []);

  return { status, lastServerFrame, sendJoystick, sendEstop };
}
