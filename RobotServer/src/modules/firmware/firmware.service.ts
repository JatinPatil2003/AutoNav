import { ServiceGateway } from "../../ros/rosServiceGateway";
import { BrightnessDto, LedPatternDto } from "./firmware.validation";

export interface FirmwareCommandResult {
  success: boolean;
  message: string;
}

const SET_LED_PATTERN_SERVICE = "/esp32_bridge_node/led/set_pattern";
const SET_BRIGHTNESS_SERVICE = "/esp32_bridge_node/led/set_brightness";

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
}
