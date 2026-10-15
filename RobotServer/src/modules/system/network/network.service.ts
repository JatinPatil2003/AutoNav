import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { logger } from "../../../common/utils/logger.util";
import { AppError } from "../../../common/errors";
import { VpnControlDto, WifiConnectDto } from "./network.validation";

const execFileAsync = promisify(execFile);

// The OpenVPN connection profile this robot was provisioned with — see
// "docs/OpenVPN Access Server Dynamic Robot DNS Setup.md". One profile
// name across every robot (it's the VPN config, not a per-robot id).
const VPN_SYSTEMD_UNIT = "openvpn-client@autonavvpn.service";

export interface WifiNetworkDto {
  ssid: string;
  signal: number;
  security: string;
}

export interface CurrentWifiDto {
  connected: boolean;
  ssid?: string;
}

export interface NetworkCommandResult {
  success: boolean;
  message: string;
}

/**
 * Thin wrapper around `nmcli` (NetworkManager, already installed on the
 * robot image) and the VPN's systemd unit. Every nmcli/systemctl call goes
 * through execFile with an argument array — never a shell string — so an
 * SSID or password can never be interpreted as shell syntax.
 */
export class NetworkService {
  async listWifiNetworks(): Promise<WifiNetworkDto[]> {
    const { stdout } = await this.run("nmcli", ["-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list"]);
    return this.parseTerseRows(stdout)
      .filter((fields) => fields[0])
      .map(([ssid, signal, security]) => ({
        ssid,
        signal: Number.parseInt(signal, 10) || 0,
        security: security || "--",
      }));
  }

  async getCurrentWifi(): Promise<CurrentWifiDto> {
    const { stdout } = await this.run("nmcli", ["-t", "-f", "ACTIVE,SSID", "dev", "wifi", "list"]);
    const active = this.parseTerseRows(stdout).find(([isActive]) => isActive === "yes");
    return active ? { connected: true, ssid: active[1] } : { connected: false };
  }

  async connectWifi(input: WifiConnectDto): Promise<NetworkCommandResult> {
    const args = ["dev", "wifi", "connect", input.ssid];
    if (input.password) {
      args.push("password", input.password);
    }

    try {
      const { stdout } = await this.run("nmcli", args);
      return { success: true, message: stdout.trim() || "Connected" };
    } catch (err) {
      logger.warn({ err, ssid: input.ssid }, "networkService: wifi connect failed");
      return { success: false, message: this.errorMessage(err) };
    }
  }

  async controlVpn(input: VpnControlDto): Promise<NetworkCommandResult> {
    const systemctlAction = input.action === "connect" ? "start" : "stop";

    try {
      await this.run("systemctl", [systemctlAction, VPN_SYSTEMD_UNIT]);
      return { success: true, message: `VPN ${input.action}ed` };
    } catch (err) {
      logger.warn({ err, action: input.action }, "networkService: vpn control failed");
      return { success: false, message: this.errorMessage(err) };
    }
  }

  private async run(command: string, args: string[]): Promise<{ stdout: string; stderr: string }> {
    try {
      return await execFileAsync(command, args, { timeout: 15_000 });
    } catch (err) {
      throw new AppError(`${command} failed: ${this.errorMessage(err)}`, 502);
    }
  }

  /** nmcli's terse (-t) output escapes literal ':' inside a field as '\:' — split on unescaped colons only. */
  private parseTerseRows(stdout: string): string[][] {
    return stdout
      .split("\n")
      .filter((line) => line.length > 0)
      .map((line) => line.split(/(?<!\\):/).map((field) => field.replace(/\\:/g, ":")));
  }

  private errorMessage(err: unknown): string {
    if (err instanceof Error) return err.message;
    return String(err);
  }
}
