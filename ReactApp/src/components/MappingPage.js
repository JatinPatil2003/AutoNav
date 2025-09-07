import React, { useState, useEffect, useRef } from "react";
import JoystickControl from "./JoystickView";
import MappingMap from "./MappingMap";
import "./css/MappingPage.css";
import { API_FULL_URL, WS_FULL_URL } from "./env";

function MappingPage({ onBack }) {
  const [mapName, setMapName] = useState("");
  const [linear, setLinear] = useState(0.0);
  const [angular, setAngular] = useState(0.0);
  const linearRef = useRef(linear);
  const angularRef = useRef(angular);
  const wsRef = useRef(null);

  const handleStopMapping = () => {
    fetch(`${API_FULL_URL}/mapping/stop`)
      .then((response) => response.json())
      .then(() => onBack());
  };

  const handleSaveMap = () => {
    fetch(`${API_FULL_URL}/mapping/save_map`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ name: mapName }),
    }).then((response) => response.json());
  };

  const handleStartMapping = async () => {
    const response = await fetch(
      `${API_FULL_URL}/mapping/start`
    );
    console.log(response.json());
  };

  const handleJoystickControl = (linear, angular) => {
    setLinear(linear);
    setAngular(angular);
  };

  useEffect(() => {
    linearRef.current = linear;
    angularRef.current = angular;

    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(
        JSON.stringify({type: 'joystick', data: {linear: linearRef.current, angular: angularRef.current}})
      );
    }
  }, [linear, angular]);

  useEffect(() => {
    // Connect WebSocket
    const ws = new WebSocket(`${WS_FULL_URL}/joystick`);
    wsRef.current = ws;

    ws.onopen = () => {
      console.log("✅ WebSocket connected");
    };

    ws.onmessage = (event) => {
    };

    ws.onclose = () => {
      console.log("❌ WebSocket disconnected");
    };

    return () => ws.close();
  }, []);

  return (
    <div className="mapping-container">
      <div className="mapping-left">
        <div className="mapping-left-top">
          <h1>Mapping Mode</h1>
          <input
            type="text"
            placeholder="Enter map name"
            value={mapName}
            onChange={(e) => setMapName(e.target.value)}
          />
          <button onClick={handleSaveMap} disabled={!mapName}>
            Save Map
          </button>
          <button onClick={handleStopMapping}>Stop Mapping</button>
          <button onClick={handleStartMapping}>Start Mapping</button>
          <button onClick={onBack}>Back</button>
        </div>
        <div className="joystick-control">
          <JoystickControl onControl={handleJoystickControl} />
        </div>
      </div>
      <div className="mapping-right">
        <MappingMap className="mapping-map" />
      </div>
    </div>
  );
}

export default MappingPage;
