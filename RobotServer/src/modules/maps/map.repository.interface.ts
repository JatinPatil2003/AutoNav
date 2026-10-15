export interface IMapRepository {
  /** Reads `${mapsDir}/${mapName}.{png,yaml}` off disk, stores them in GridFS, and removes the local copies. */
  save(mapName: string, mapsDir: string): Promise<void>;
  /** Writes `${mapsDir}/${mapName}.{png,yaml}` from GridFS back to disk. Returns false if the map doesn't exist. */
  load(mapName: string, mapsDir: string): Promise<boolean>;
  list(): Promise<string[]>;
}
