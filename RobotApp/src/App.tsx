import { useState } from "react";
import { FullPageSpinner } from "./components/ui/Spinner";
import { UserPickerPage } from "./features/auth/UserPickerPage";
import { IdentifyPage } from "./features/auth/IdentifyPage";
import { PinLoginPage } from "./features/auth/PinLoginPage";
import { PinSetupWizard } from "./features/auth/PinSetupWizard";
import { useSession } from "./features/auth/useSession";
import { RobotAppHomePage } from "./features/home/RobotAppHomePage";
import { SettingsPage } from "./features/settings/SettingsPage";
import { WifiSettingsPage } from "./features/settings/WifiSettingsPage";

type UnauthFlow = "picker" | "identify" | "pin-login" | "pin-setup" | "wifi";
type AuthedView = "home" | "settings" | "wifi";

export function App() {
  const { status, user, refresh, logout } = useSession();
  const [flow, setFlow] = useState<UnauthFlow>("picker");
  const [email, setEmail] = useState("");
  const [name, setName] = useState<string | undefined>();
  const [avatarUrl, setAvatarUrl] = useState<string | undefined>();
  const [pinLoginNotice, setPinLoginNotice] = useState<string | undefined>();
  const [authedView, setAuthedView] = useState<AuthedView>("home");

  if (status === "loading") {
    return <FullPageSpinner />;
  }

  if (status === "authenticated" && user) {
    if (authedView === "wifi") {
      return <WifiSettingsPage onBack={() => setAuthedView("settings")} />;
    }
    if (authedView === "settings") {
      return (
        <SettingsPage onBack={() => setAuthedView("home")} onOpenWifi={() => setAuthedView("wifi")} />
      );
    }
    return (
      <RobotAppHomePage
        user={user}
        onLogout={async () => {
          await logout();
          setFlow("picker");
        }}
        onOpenSettings={() => setAuthedView("settings")}
      />
    );
  }

  if (flow === "wifi") {
    return <WifiSettingsPage onBack={() => setFlow("picker")} />;
  }

  if (flow === "picker") {
    return (
      <UserPickerPage
        onPickUser={(pickedEmail, pickedName, pickedAvatarUrl) => {
          setEmail(pickedEmail);
          setName(pickedName);
          setAvatarUrl(pickedAvatarUrl);
          setPinLoginNotice(undefined);
          setFlow("pin-login");
        }}
        onNewUser={() => setFlow("identify")}
        onOpenWifi={() => setFlow("wifi")}
      />
    );
  }

  if (flow === "identify") {
    return (
      <IdentifyPage
        onIdentified={(identifiedEmail, pinSet, identifiedName, identifiedAvatarUrl) => {
          setEmail(identifiedEmail);
          setName(identifiedName);
          setAvatarUrl(identifiedAvatarUrl);
          setPinLoginNotice(undefined);
          setFlow(pinSet ? "pin-login" : "pin-setup");
        }}
        onBack={() => setFlow("picker")}
      />
    );
  }

  if (flow === "pin-login") {
    return (
      <PinLoginPage
        email={email}
        name={name}
        avatarUrl={avatarUrl}
        onBack={() => setFlow("picker")}
        onLoggedIn={refresh}
        onForgotPin={() => {
          setPinLoginNotice(undefined);
          setFlow("pin-setup");
        }}
        notice={pinLoginNotice}
      />
    );
  }

  return (
    <PinSetupWizard
      email={email}
      name={name}
      avatarUrl={avatarUrl}
      onBack={() => setFlow("picker")}
      onPinSet={() => {
        setPinLoginNotice("PIN set — log in with it now.");
        setFlow("pin-login");
      }}
    />
  );
}
