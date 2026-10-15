import { RouterProvider } from "react-router-dom";
import { router } from "./router";
import { ThemeProvider } from "./app/theme/ThemeProvider";

export function App() {
  return (
    <ThemeProvider>
      <RouterProvider router={router} />
    </ThemeProvider>
  );
}
