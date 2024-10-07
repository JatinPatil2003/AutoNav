import React, { useState, useEffect, useRef } from "react";
import robotIconSrc from "./icons/robot.png";

function MappingMap() {
  const [mapData, setMapData] = useState(null);
  const [robotLocation, setRobotLocation] = useState(null);
  const [robotIcon, setRobotIcon] = useState(null);
  const canvasRef = useRef(null);

  useEffect(() => {
    const fetchMapData = async () => {
      try {
        const response = await fetch(
          "http://3.109.213.102:5747/mapping/current/map"
        );
        if (!response.ok) {
          throw new Error("Network response was not ok");
        }
        const data = await response.json();
        setMapData(data);
      } catch (error) {
        console.error("Error fetching map data:", error);
      }
    };

    fetchMapData();

    const intervalId = setInterval(fetchMapData, 3000);

    return () => clearInterval(intervalId);
  }, []);

  useEffect(() => {
    const fetchRobotLocation = async () => {
      try {
        const response = await fetch(
          "http://3.109.213.102:5747/mapping/current/location"
        );
        if (!response.ok) {
          throw new Error("Network response was not ok");
        }
        const data = await response.json();
        setRobotLocation(data);
        // console.log(data);
      } catch (error) {
        console.error("Error fetching robot location:", error);
      }
    };

    fetchRobotLocation();

    const intervalId = setInterval(fetchRobotLocation, 250);

    return () => clearInterval(intervalId);
  }, []);

  useEffect(() => {
    const loadRobotIcon = () => {
      const icon = new Image();
      icon.src = robotIconSrc; // Update with the correct path to your icon
      icon.onload = () => setRobotIcon(icon);
    };

    loadRobotIcon();
  }, []);

  const getColorForOccupancy = (value) => {
    if (value === 0) return "white"; // Free space
    if (value === 100) return "black"; // Occupied
    return "gray"; // Unknown
  };

  useEffect(() => {
    const drawRobot = (ctx, location, mapInfo, cellWidth, cellHeight) => {
      const { x, y, theta } = location;
      // const robotX = (x - mapInfo.origin.position.x) / mapInfo.resolution * cellWidth;
      // const robotY = (y - mapInfo.origin.position.y) / mapInfo.resolution * cellHeight;
      const robotX =
        (mapInfo.width - ((x - mapInfo.origin.position.x) / mapInfo.resolution)) * cellWidth;
      const robotY =
        ((y - mapInfo.origin.position.y) / mapInfo.resolution) * cellHeight;

      console.log(x, y, robotX, robotY);
      console.log(mapInfo.origin.position.x, mapInfo.origin.position.y)
      // Draw robot icon
      const iconSize = 25; // Adjust the icon size if needed
      ctx.save();
      ctx.translate(robotX, robotY);
      ctx.rotate(-theta - Math.PI / 2); // Rotate the icon based on the robot's orientation
      ctx.drawImage(
        robotIcon,
        -iconSize / 2,
        -iconSize / 2,
        iconSize,
        iconSize
      );
      ctx.restore();
    };

    const loadMap = (ctx, mapData, mapInfo, cellWidth, cellHeight) => {
      // Loop through map data and draw onto canvas
      for (let y = 0; y < mapInfo.height; y++) {
        for (let x = 0; x < mapInfo.width; x++) {
          const val = mapData[mapInfo.width - x + y * mapInfo.width];

          // Set fill color based on occupancy value
          ctx.fillStyle = getColorForOccupancy(val);

          // Draw rectangle representing map cell
          ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
        }
      }
    };

    if (mapData && robotIcon) {
      const canvas = canvasRef.current;
      const ctx = canvas.getContext("2d");
      const { width, height } = mapData.info;
      const { innerWidth: window_width, innerHeight: window_height } = window;

      // Set canvas dimensions
      canvas.width = window_width;
      canvas.height = (height * window_width) / width;
      if (canvas.height >= window_height - height / 1.5) {
        canvas.height = window_height - height / 1.5;
        canvas.width = (width * window_height) / height - width / 1.5;
      }
      // canvas.width = width;
      // canvas.height = height;

      // Define cell size based on canvas size and map dimensions
      const cellWidth = canvas.width / width;
      const cellHeight = canvas.height / height;

      // Clear the canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Draw the map data
      loadMap(ctx, mapData.data, mapData.info, cellWidth, cellHeight);

      if (robotLocation) {
        drawRobot(ctx, robotLocation, mapData.info, cellWidth, cellHeight);
      }
    }
  }, [mapData, robotIcon, robotLocation]);

  return (
    <div>
      <canvas
        ref={canvasRef}
        style={{ border: "1px solid black" }}
      />
    </div>
  );
}

export default MappingMap;
