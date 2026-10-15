import { createReadStream, createWriteStream } from "node:fs";
import { rm } from "node:fs/promises";
import path from "node:path";
import mongoose from "mongoose";
import { IMapRepository } from "./map.repository.interface";

/**
 * Map blobs (PNG + YAML from `nav2_map_server`) live in GridFS rather than
 * on the container's own disk — the same persistence choice the old
 * WebServer made, kept here because it survives a container recreate and
 * makes maps visible from any container that shares this Mongo instance.
 */
export class GridFsMapRepository implements IMapRepository {
  private bucket(): mongoose.mongo.GridFSBucket {
    const db = mongoose.connection.db;
    if (!db) {
      throw new Error("Mongo connection is not established yet");
    }
    return new mongoose.mongo.GridFSBucket(db, { bucketName: "maps" });
  }

  async save(mapName: string, mapsDir: string): Promise<void> {
    const bucket = this.bucket();

    for (const ext of ["png", "yaml"] as const) {
      const filename = `${mapName}.${ext}`;
      const localPath = path.join(mapsDir, filename);

      const existing = await bucket.find({ filename }).toArray();
      await Promise.all(existing.map((file) => bucket.delete(file._id)));

      await new Promise<void>((resolve, reject) => {
        createReadStream(localPath)
          .pipe(bucket.openUploadStream(filename))
          .on("finish", resolve)
          .on("error", reject);
      });

      await rm(localPath, { force: true });
    }
  }

  async load(mapName: string, mapsDir: string): Promise<boolean> {
    const bucket = this.bucket();
    let foundAny = false;

    for (const ext of ["png", "yaml"] as const) {
      const filename = `${mapName}.${ext}`;
      const [file] = await bucket.find({ filename }).toArray();
      if (!file) {
        continue;
      }
      foundAny = true;

      const localPath = path.join(mapsDir, filename);
      await new Promise<void>((resolve, reject) => {
        bucket
          .openDownloadStreamByName(filename)
          .pipe(createWriteStream(localPath))
          .on("finish", resolve)
          .on("error", reject);
      });
    }

    return foundAny;
  }

  async list(): Promise<string[]> {
    const bucket = this.bucket();
    const files = await bucket.find().toArray();
    const names = new Set(files.map((file) => file.filename.replace(/\.(png|yaml)$/, "")));
    return [...names];
  }
}
