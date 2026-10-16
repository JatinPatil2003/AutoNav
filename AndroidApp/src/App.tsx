import { RouterProvider } from "react-router-dom";
import { RobotSelectionProvider } from "./app/robotSelection/RobotSelectionProvider";
import { router } from "./router";

export function App() {
  return (
    <RobotSelectionProvider>
      <RouterProvider router={router} />
    </RobotSelectionProvider>
  );
}
