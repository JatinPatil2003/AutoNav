import React, {useState} from "react";
import { Joystick } from "react-joystick-component";

function JoystickControl({ onControl}) {

  const [sendVel, setSendVel] = useState(false);

  const handleMove = (data) => {
    if(sendVel){
      const { x, y } = data;
      const linear = y * 0.5; // Adjust as necessary
      const angular = x * -1; // Adjust as necessary
      onControl(linear, angular);
      console.log(`Linear: ${linear}, Angular: ${angular}`);
    };
  };

  const handleStop = (data) => {
    const linear = 0.0; // Adjust as necessary
    const angular = 0.0; // Adjust as necessary
    onControl(linear, angular);
    console.log(`Linear: ${linear}, Angular: ${angular}`);
    setSendVel(false);
  };

  const handleStart = (data) => {
    setSendVel(true);
  };

  return (
    <Joystick
      size={150}
      stickSize={80}
      baseColor="lightblue"
      stickColor="blue"
      move={handleMove}
      stop={handleStop}
      start={handleStart}
      minDistance={70}
    />
  );
}

export default JoystickControl;
