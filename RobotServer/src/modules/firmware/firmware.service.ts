import { ServiceGateway } from "../../ros/rosServiceGateway";
import { BrightnessDto, LedPatternDto } from "./firmware.validation";

export interface FirmwareCommandResult {
  success: boolean;
  message: string;
}

export interface BatteryStatusResult extends FirmwareCommandResult {
  voltageVolts: number;
  percentage: number;
  charging: boolean;
  /** True until the ESP32 has a real voltage-sense ADC wired up — see autonav_firmware's esp32_protocol.hpp. Never present this as a live reading while true. */
  isDefault: boolean;
}

const SET_LED_PATTERN_SERVICE = "/esp32_bridge_node/led/set_pattern";
const SET_BRIGHTNESS_SERVICE = "/esp32_bridge_node/led/set_brightness";
const GET_BATTERY_STATUS_SERVICE = "/esp32_bridge_node/get_battery_status";

export class FirmwareService {
  constructor(private readonly serviceGateway: ServiceGateway) {}

  setLedPattern(pattern: LedPatternDto): Promise<FirmwareCommandResult> {
    const useCustomParams = pattern.hue !== undefined;
    return this.serviceGateway.call(
      "autonav_msgs/srv/SetLedPattern",
      SET_LED_PATTERN_SERVICE,
      {
        pattern_id: pattern.patternId,
        use_custom_params: useCustomParams,
        hue: pattern.hue ?? 0,
        sat: pattern.sat ?? 0,
        val: pattern.val ?? 0,
        speed_ms: pattern.speedMs ?? 0,
      },
    );
  }

  setBrightness(brightness: BrightnessDto): Promise<FirmwareCommandResult> {
    return this.serviceGateway.call(
      "autonav_msgs/srv/SetBrightness",
      SET_BRIGHTNESS_SERVICE,
      { brightness: brightness.brightness },
    );
  }

  async getBatteryStatus(): Promise<BatteryStatusResult> {
    const response = await this.serviceGateway.call<
      Record<string, never>,
      { success: boolean; message: string; voltage_volts: number; percentage: number; charging: boolean; is_default: boolean }
    >("autonav_msgs/srv/GetBatteryStatus", GET_BATTERY_STATUS_SERVICE, {});

    return {
      success: response.success,
      message: response.message,
      voltageVolts: response.voltage_volts,
      percentage: response.percentage,
      charging: response.charging,
      isDefault: response.is_default,
    };
  }
}
