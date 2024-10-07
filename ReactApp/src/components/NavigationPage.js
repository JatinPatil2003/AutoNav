import React, { useEffect, useState } from "react";
import MapPosePage from "./MapPosePage";
import "./css/NavigationPage.css";

function NavigationPage({ onBack }) {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState("");
  const [useMap, setUseMap] = useState(false);
  const [startNav, setStartNav] = useState(false);
  const [stopButton, setStopButton] = useState(true);

  useEffect(() => {
    fetch("http://3.109.213.102:5747/navigation/list/maps")
      .then((response) => response.json())
      .then((data) => setMaps(data));
  }, []);

  const handleUseMap = (map) => {
    if (map) {
      fetch("http://3.109.213.102:5747/navigation/use_map", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ name: map }),
      })
        .then((response) => response.json())
        .then(() => setUseMap(true));
    }
  };

  const handleStopNavigation = () => {
    fetch("http://3.109.213.102:5747/navigation/stop")
      .then((response) => response.json())
      .then(() => setSelectedMap(""))
      .then(() => setStopButton(true));
  };

  const handleStartNavigation = () => {
    fetch("http://3.109.213.102:5747/navigation/start", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        name: selectedMap,
      }),
    })
      .then((response) => response.json())
      .then(() => setStartNav(true))
      .then(() => setStopButton(false))
      .then(() => setUseMap(false));
  };

  if (startNav) {
    return (
      <MapPosePage mapName={selectedMap} onBack={() => {setStartNav(false); setUseMap(true);}} />
    );
  }

  const handleMapSelection = (map) => {
    setSelectedMap(map);
    console.log(map);
    handleUseMap(map);
  };

  return (
    <div className="navigation-page">
      <h1>Navigation</h1>
      <div className="map-selection">
        <select
          value={selectedMap}
          onChange={(e) => handleMapSelection(e.target.value)}
        >
          <option value="">Select Map</option>
          {maps.map((map) => (
            <option key={map} value={map}>
              {map}
            </option>
          ))}
        </select>
      </div>
      <div>
        <button class="idk" onClick={handleStartNavigation} disabled={!useMap}>
          Start Navigation
        </button>
        <button
          class="idk"
          onClick={handleStopNavigation}
          disabled={stopButton}
        >
          Stop Navigation
        </button>
      </div>
      <button class="idk" onClick={onBack}>
        Back
      </button>
    </div>
  );
}

export default NavigationPage;
