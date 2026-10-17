import { useState } from "react";
import { AnimatePresence, motion } from "framer-motion";
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

  let key: string;
  let screen: JSX.Element;

  if (status === "authenticated" && user) {
    if (authedView === "wifi") {
      key = "authed-wifi";
      screen = <WifiSettingsPage onBack={() => setAuthedView("settings")} />;
    } else if (authedView === "settings") {
      key = "authed-settings";
      screen = (
        <SettingsPage onBack={() => setAuthedView("home")} onOpenWifi={() => setAuthedView("wifi")} />
      );
    } else {
      key = "authed-home";
      screen = (
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
  } else if (flow === "wifi") {
    key = "wifi";
    screen = <WifiSettingsPage onBack={() => setFlow("picker")} />;
  } else if (flow === "picker") {
    key = "picker";
    screen = (
      <UserPickerPage
        onPickUser={(pickedEmail, pickedName, pickedPinSet, pickedAvatarUrl) => {
          setEmail(pickedEmail);
          setName(pickedName);
          setAvatarUrl(pickedAvatarUrl);
          setPinLoginNotice(undefined);
          setFlow(pickedPinSet ? "pin-login" : "pin-setup");
        }}
        onNewUser={() => setFlow("identify")}
        onOpenWifi={() => setFlow("wifi")}
      />
    );
  } else if (flow === "identify") {
    key = "identify";
    screen = (
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
  } else if (flow === "pin-login") {
    key = "pin-login";
    screen = (
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
  } else {
    key = "pin-setup";
    screen = (
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

  return (
    <AnimatePresence mode="wait">
      <motion.div
        key={key}
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        exit={{ opacity: 0 }}
        transition={{ duration: 0.15 }}
        className="h-screen w-full"
      >
        {screen}
      </motion.div>
    </AnimatePresence>
  );
}
