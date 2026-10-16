import { App as CapacitorApp } from "@capacitor/app";
import { Capacitor } from "@capacitor/core";
import type { AppRouter } from "../router";

/**
 * Routes Android's hardware back button through the app's own router
 * history instead of the WebView's default back-navigation (which would
 * otherwise behave unpredictably across routes). Exits the app only once
 * there's nowhere left to go back to.
 */
export function registerBackButtonHandler(router: AppRouter): void {
  if (!Capacitor.isNativePlatform()) return;

  void CapacitorApp.addListener("backButton", ({ canGoBack }) => {
    if (canGoBack) {
      router.navigate(-1);
    } else {
      void CapacitorApp.exitApp();
    }
  });
}
