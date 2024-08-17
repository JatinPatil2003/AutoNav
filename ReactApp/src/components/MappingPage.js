import React, { useState, useEffect, useRef } from "react";
import JoystickControl from "./JoystickView";
import MappingMap from "./MappingMap";
import "./css/MappingPage.css";

function MappingPage({ onBack }) {
  const [mapName, setMapName] = useState("");
  const [linear, setLinear] = useState(0.0);
  const [angular, setAngular] = useState(0.0);
  const linearRef = useRef(linear);
  const angularRef = useRef(angular);

  const handleStopMapping = () => {
    fetch("http://13.201.82.2:5747/mapping/stop")
      .then((response) => response.json())
      .then(() => onBack());
  };

  const handleSaveMap = () => {
    fetch("http://13.201.82.2:5747/mapping/save_map", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ name: mapName }),
    }).then((response) => response.json());
  };

  const handleStartMapping = async () => {
    const response = await fetch(
      "http://13.201.82.2:5747/mapping/start"
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
  }, [linear, angular]);

  useEffect(() => {
    const setvelocity = async () => {
      try {
        const response = await fetch("http://13.201.82.2:5747/joystick/control", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ linear: linearRef.current, angular: angularRef.current }),
        });
        if (!response.ok) {
          throw new Error("Network response was not ok");
        }
        const data = await response.json();
        console.log(data);
      } catch (error) {
        console.error("Error fetching robot location:", error);
      }
    };

    setvelocity();

    const intervalId = setInterval(setvelocity, 500);

    return () => clearInterval(intervalId);
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
