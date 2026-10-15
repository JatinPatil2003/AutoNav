import { useEffect, useState } from "react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
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
    <div className="flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-sm">
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">Wifi</h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
          {current?.connected ? `Connected to ${current.ssid}` : "Not connected"}
        </p>

        {error && <p className="mb-4 text-sm text-red-600 dark:text-red-400">{error}</p>}

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
                  className="flex w-full items-center justify-between rounded-md border border-slate-200 px-4 py-3 text-left transition-colors hover:bg-slate-100 dark:border-slate-700 dark:hover:bg-slate-800"
                >
                  <span className="text-sm font-medium text-slate-900 dark:text-slate-100">
                    {network.ssid}
                  </span>
                  <span className="text-xs text-slate-500 dark:text-slate-400">{network.signal}%</span>
                </button>
              </li>
            ))}
            {networks.length === 0 && (
              <p className="text-sm text-slate-500 dark:text-slate-400">No networks found.</p>
            )}
          </ul>
        )}

        {selected && (
          <div className="mb-4 flex flex-col gap-3 rounded-md border border-slate-200 p-4 dark:border-slate-700">
            <p className="text-sm font-medium text-slate-900 dark:text-slate-100">{selected.ssid}</p>
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

        <div className="mb-6 flex items-center justify-between rounded-md border border-slate-200 px-4 py-3 dark:border-slate-700">
          <span className="text-sm font-medium text-slate-900 dark:text-slate-100">VPN</span>
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
          Back
        </Button>
      </Card>
    </div>
  );
}
