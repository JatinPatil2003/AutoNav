import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { IPlatformLink, PlatformLinkFrameHandler } from "../../src/clients/platformLink.interface";
import { MaintenanceService, REBOOT_TRIGGER_FILE } from "../../src/modules/system/maintenance/maintenance.service";
import { FakePlatformClient } from "./fakes/FakePlatformClient";

class FakePlatformLink implements IPlatformLink {
  forceReconnectCalls = 0;
  start(): void {}
  stop(): void {}
  isConnected(): boolean {
    return true;
  }
  send(_frame: unknown): void {}
  onFrame(_handler: PlatformLinkFrameHandler): void {}
  forceReconnect(): void {
    this.forceReconnectCalls += 1;
  }
}

class FakeSecretStore {
  current = "old-secret";
  rotateCalls: string[] = [];
  getSecret(): string {
    return this.current;
  }
  rotate(newSecret: string): void {
    this.current = newSecret;
    this.rotateCalls.push(newSecret);
  }
}

describe("MaintenanceService", () => {
  let runtimeDir: string;
  let platformClient: FakePlatformClient;
  let secretStore: FakeSecretStore;
  let link: FakePlatformLink;
  let service: MaintenanceService;

  beforeEach(async () => {
    runtimeDir = await fs.mkdtemp(path.join(os.tmpdir(), "autonav-maintenance-test-"));
    platformClient = new FakePlatformClient();
    secretStore = new FakeSecretStore();
    link = new FakePlatformLink();
    service = new MaintenanceService(platformClient, runtimeDir, secretStore);
    service.setPlatformLink(link);
  });

  afterEach(async () => {
    await fs.rm(runtimeDir, { recursive: true, force: true });
  });

  it("writes a reboot trigger file onto the runtime dir", async () => {
    await service.requestReboot();

    const contents = await fs.readFile(path.join(runtimeDir, REBOOT_TRIGGER_FILE), "utf-8");
    expect(() => new Date(contents).toISOString()).not.toThrow();
  });

  it("creates the runtime dir if it doesn't exist yet", async () => {
    const nestedDir = path.join(runtimeDir, "nested", "deeper");
    service = new MaintenanceService(platformClient, nestedDir, secretStore);

    await service.requestReboot();

    const contents = await fs.readFile(path.join(nestedDir, REBOOT_TRIGGER_FILE), "utf-8");
    expect(contents).toBeTruthy();
  });

  it("rotates credentials: fetches a new secret, applies it, and forces a reconnect", async () => {
    const result = await service.rotateCredentials();

    expect(result.linkSecret).toBe("fake-rotated-secret");
    expect(secretStore.rotateCalls).toEqual(["fake-rotated-secret"]);
    expect(secretStore.current).toBe("fake-rotated-secret");
    expect(link.forceReconnectCalls).toBe(1);
  });

  it("does not force a reconnect if the platform link hasn't been wired in yet", async () => {
    service = new MaintenanceService(platformClient, runtimeDir, secretStore);
    // Deliberately no setPlatformLink() call — exercises the pre-boot window.

    await expect(service.rotateCredentials()).resolves.toEqual({ linkSecret: "fake-rotated-secret" });
  });
});
