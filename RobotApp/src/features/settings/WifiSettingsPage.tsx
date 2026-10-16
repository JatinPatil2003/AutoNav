import { useEffect, useState } from "react";
import { ArrowLeft, Lock, ShieldCheck, Signal, Wifi as WifiIcon } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
import { AppShell } from "../../components/ui/AppShell";
import { getApiErrorMessage } from "../../api/apiError";
import { CurrentWifi, WifiNetwork, networkClient } from "../../api/networkClient";

interface WifiSettingsPageProps {
  onBack: () => void;
}

const SECURED_PATTERN = /wpa|wep/i;

/**
 * Standalone wifi setup screen — reachable both pre-login (from
 * UserPickerPage, since a robot may need internet before anyone can even
 * log in) and from the authenticated SettingsPage. No session required:
 * see RobotServer's /api/system/network/* routes.
 */
export function WifiSettingsPage({ onBack }: WifiSettingsPageProps) {
  const [networks, setNetworks] = useState<WifiNetwork[] | null>(null);
  const [current, setCurrent] = useState<CurrentWifi | null>(null);
  const [selected, setSelected] = useState<WifiNetwork | null>(null);
  const [password, setPassword] = useState("");
  const [isConnecting, setIsConnecting] = useState(false);
  const [isVpnLoading, setIsVpnLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    refresh();
  }, []);

  async function refresh() {
    try {
      const [{ networks: list }, currentWifi] = await Promise.all([
        networkClient.listWifiNetworks(),
        networkClient.getCurrentWifi(),
      ]);
      setNetworks(list);
      setCurrent(currentWifi);
    } catch (err) {
      setError(getApiErrorMessage(err));
    }
  }

  async function handleConnect() {
    if (!selected) return;
    setError(null);
    setIsConnecting(true);
    try {
      const result = await networkClient.connectWifi(selected.ssid, password || undefined);
      if (!result.success) throw new Error(result.message);
      setSelected(null);
      setPassword("");
      await refresh();
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsConnecting(false);
    }
  }

  async function handleVpn(action: "connect" | "disconnect") {
    setError(null);
    setIsVpnLoading(true);
    try {
      const result = await networkClient.controlVpn(action);
      if (!result.success) throw new Error(result.message);
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsVpnLoading(false);
    }
  }

  return (
    <AppShell>
      <Card>
        <div className="mb-4 flex h-11 w-11 items-center justify-center rounded-xl bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
          <WifiIcon className="h-5 w-5 text-white" />
        </div>
        <h1 className="mb-1 text-2xl font-bold tracking-tight text-slate-900 dark:text-slate-50">Wifi</h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
          {current?.connected ? `Connected to ${current.ssid}` : "Not connected"}
        </p>

        {error && (
          <p className="mb-4 rounded-lg bg-red-50 px-3 py-2 text-sm text-red-600 dark:bg-red-900/20 dark:text-red-400">
            {error}
          </p>
        )}

        {!networks && !error && (
          <div className="mb-4 flex justify-center">
            <span className="h-6 w-6 animate-spin rounded-full border-2 border-brand-600 border-t-transparent" />
          </div>
        )}

        {networks && (
          <ul className="mb-4 flex max-h-64 flex-col gap-2 overflow-y-auto">
            {networks.map((network) => (
              <li key={network.ssid}>
                <button
                  type="button"
                  onClick={() => {
                    setSelected(network);
                    setPassword("");
                    setError(null);
                  }}
                  className="flex w-full items-center justify-between gap-2 rounded-xl border border-slate-200/70 bg-white/60 px-4 py-3 text-left transition-all hover:-translate-y-0.5 hover:border-brand-300 hover:bg-white hover:shadow-md hover:shadow-brand-900/5 dark:border-slate-700/70 dark:bg-slate-800/40 dark:hover:bg-slate-800"
                >
                  <span className="flex items-center gap-2 text-sm font-medium text-slate-900 dark:text-slate-100">
                    {SECURED_PATTERN.test(network.security) && (
                      <Lock className="h-3.5 w-3.5 flex-shrink-0 text-slate-400" />
                    )}
                    {network.ssid}
                  </span>
                  <span className="flex items-center gap-1 text-xs font-medium text-slate-400">
                    <Signal className="h-3.5 w-3.5" />
                    {network.signal}%
                  </span>
                </button>
              </li>
            ))}
            {networks.length === 0 && (
              <p className="text-sm text-slate-500 dark:text-slate-400">No networks found.</p>
            )}
          </ul>
        )}

        {selected && (
          <div className="mb-4 flex flex-col gap-3 rounded-xl border border-slate-200/70 bg-white/60 p-4 dark:border-slate-700/70 dark:bg-slate-800/40">
            <p className="text-sm font-semibold text-slate-900 dark:text-slate-100">{selected.ssid}</p>
            {SECURED_PATTERN.test(selected.security) && (
              <Input
                label="Password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
              />
            )}
            <div className="flex gap-2">
              <Button isLoading={isConnecting} onClick={handleConnect} className="flex-1">
                Connect
              </Button>
              <Button variant="ghost" onClick={() => setSelected(null)}>
                Cancel
              </Button>
            </div>
          </div>
        )}

        <div className="mb-6 flex items-center justify-between rounded-xl border border-slate-200/70 bg-white/60 px-4 py-3 dark:border-slate-700/70 dark:bg-slate-800/40">
          <span className="flex items-center gap-2 text-sm font-medium text-slate-900 dark:text-slate-100">
            <ShieldCheck className="h-4 w-4 text-slate-400" />
            VPN
          </span>
          <div className="flex gap-2">
            <Button variant="secondary" isLoading={isVpnLoading} onClick={() => handleVpn("connect")}>
              Connect
            </Button>
            <Button variant="ghost" isLoading={isVpnLoading} onClick={() => handleVpn("disconnect")}>
              Disconnect
            </Button>
          </div>
        </div>

        <Button variant="ghost" className="w-full" onClick={onBack}>
          <ArrowLeft className="h-4 w-4" /> Back
        </Button>
      </Card>
    </AppShell>
  );
}
