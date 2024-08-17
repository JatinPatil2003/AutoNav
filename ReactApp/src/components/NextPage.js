import React, { useState } from "react";
import NavigationPage from "./NavigationPage";
import MappingPage from "./MappingPage";
import "./css/NextPage.css";

function NextPage({ onBack }) {
  const [operation, setOperation] = useState(null);

  if (operation === "navigation") {
    return (
      <NavigationPage
        onBack={() => {
          setOperation(null);
          // onBack();
        }}
      />
    );
  }

  if (operation === "mapping") {
    return (
      <MappingPage
        onBack={() => {
          setOperation(null);
          // onBack();
        }}
      />
    );
  }

  return (
    <div className="robot-operations">
      <h1>Select Operations</h1>
      <div>
        <button onClick={() => setOperation("mapping")}>Mapping</button>
        <button onClick={() => setOperation("navigation")}>Navigation</button>
      </div>
      <div>
        <button onClick={onBack}>Back</button>
      </div>
    </div>
  );
}

export default NextPage;
