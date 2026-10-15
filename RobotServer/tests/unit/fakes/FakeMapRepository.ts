import { IMapRepository } from "../../../src/modules/maps/map.repository.interface";

export class FakeMapRepository implements IMapRepository {
  existingMaps = new Set<string>();
  saved: string[] = [];

  async save(mapName: string): Promise<void> {
    this.saved.push(mapName);
    this.existingMaps.add(mapName);
  }

  async load(mapName: string): Promise<boolean> {
    return this.existingMaps.has(mapName);
  }

  async list(): Promise<string[]> {
    return [...this.existingMaps];
  }
}
