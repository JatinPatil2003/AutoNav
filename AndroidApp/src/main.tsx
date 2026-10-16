import React from "react";
import ReactDOM from "react-dom/client";
import { Provider } from "react-redux";
import { App } from "./App";
import { store } from "./app/store";
import { registerBackButtonHandler } from "./native/backButton";
import { registerBackgroundSafety } from "./native/backgroundSafety";
import { router } from "./router";
import "./styles/tailwind.css";

registerBackgroundSafety();
registerBackButtonHandler(router);

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <Provider store={store}>
      <App />
    </Provider>
  </React.StrictMode>,
);
