import { z } from "zod";

export const wifiConnectSchema = z.object({
  ssid: z.string().min(1).max(32),
  password: z.string().min(8).max(63).optional(),
});
export type WifiConnectDto = z.infer<typeof wifiConnectSchema>;

export const vpnControlSchema = z.object({
  action: z.enum(["connect", "disconnect"]),
});
export type VpnControlDto = z.infer<typeof vpnControlSchema>;
