import { FirmwareService } from "../../src/modules/firmware/firmware.service";
import { FakeRosServiceGateway } from "./fakes/FakeRosServiceGateway";

describe("FirmwareService", () => {
  const SET_LED_PATTERN_SERVICE = "/esp32_bridge_node/led/set_pattern";

  it("requests the ESP32's hardcoded default params when only patternId is given", async () => {
    const serviceGateway = new FakeRosServiceGateway();
    serviceGateway.setResponse(SET_LED_PATTERN_SERVICE, { success: true, message: "OK" });
    const service = new FirmwareService(serviceGateway);

    await service.setLedPattern({ patternId: 2 });

    expect(serviceGateway.calls[0].request).toEqual({
      pattern_id: 2,
      use_custom_params: false,
      hue: 0,
      sat: 0,
      val: 0,
      speed_ms: 0,
    });
  });

  it("forwards explicit hue/sat/val/speedMs as a custom override", async () => {
    const serviceGateway = new FakeRosServiceGateway();
    serviceGateway.setResponse(SET_LED_PATTERN_SERVICE, { success: true, message: "OK" });
    const service = new FirmwareService(serviceGateway);

    await service.setLedPattern({ patternId: 3, hue: 86, sat: 200, val: 255, speedMs: 40 });

    expect(serviceGateway.calls[0].request).toEqual({
      pattern_id: 3,
      use_custom_params: true,
      hue: 86,
      sat: 200,
      val: 255,
      speed_ms: 40,
    });
  });
});
