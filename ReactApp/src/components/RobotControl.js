import React, { useEffect, useState } from "react";
import NextPage from "./NextPage";
import "./css/RobotControl.css";
import image1 from "./img/fusion.png";
import image2 from "./img/real.png";

function RobotControl() {
  const [robotStatus, setRobotStatus] = useState("stopped");

  useEffect(() => {
    fetch("http://13.201.82.2:5747/robot/status")
      .then((response) => response.json())
      .then((data) => {
        if (data.status === "started") {
          setRobotStatus("started");
        }
      });
  }, []);

  const handleStart = () => {
    fetch("http://13.201.82.2:5747/robot/start")
      .then((response) => response.json())
      .then(() => setRobotStatus("started"));
  };

  const handleStop = () => {
    fetch("http://13.201.82.2:5747/robot/stop")
      .then((response) => response.json())
      .then(() => setRobotStatus("stopped"));
  };

  if (robotStatus === "started") {
    return <NextPage onBack={() => setRobotStatus("stopped")} />;
  }

  return (
    <div className="robot-control">
      <h1>Robot Control</h1>
      <div className="button-container">
        <button onClick={handleStart}>Start Robot</button>
        <button onClick={handleStop}>Stop Robot</button>
      </div>
      <div className="image-container">
        <img className="img1" src={image1} alt="Image 1" />
        <img className="img2" src={image2} alt="Image 2" />
      </div>
    </div>
  );
}

export default RobotControl;
