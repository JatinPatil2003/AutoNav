import mongoose from "mongoose";
import { MongoMemoryServer } from "mongodb-memory-server";
import request from "supertest";
import { Express } from "express";

let mongoServer: MongoMemoryServer;
let app: Express;

beforeAll(async () => {
  mongoServer = await MongoMemoryServer.create();
  process.env.MONGO_URI = mongoServer.getUri();

  await mongoose.connect(mongoServer.getUri());

  // Imported after MONGO_URI is set so config/env.ts validates against it.
  const { createApp } = await import("../../src/app");
  app = createApp().app;
});

afterAll(async () => {
  await mongoose.disconnect();
  await mongoServer.stop();
});

afterEach(async () => {
  const collections = mongoose.connection.collections;
  for (const key of Object.keys(collections)) {
    await collections[key].deleteMany({});
  }
});

describe("Auth routes", () => {
  it("signs up, fetches /me, logs out, then rejects /me", async () => {
    const agent = request.agent(app);

    const signupRes = await agent
      .post("/api/auth/signup")
      .send({ email: "e2e@example.com", password: "password123", name: "E2E User" });

    expect(signupRes.status).toBe(201);
    expect(signupRes.body.user.email).toBe("e2e@example.com");

    const meRes = await agent.get("/api/users/me");
    expect(meRes.status).toBe(200);
    expect(meRes.body.user.robots).toEqual([]);

    const logoutRes = await agent.post("/api/auth/logout");
    expect(logoutRes.status).toBe(204);

    const meAfterLogoutRes = await agent.get("/api/users/me");
    expect(meAfterLogoutRes.status).toBe(401);
  });

  it("rejects duplicate signups", async () => {
    const agent = request.agent(app);
    await agent
      .post("/api/auth/signup")
      .send({ email: "dup@example.com", password: "password123", name: "Dup" });

    const secondRes = await agent
      .post("/api/auth/signup")
      .send({ email: "dup@example.com", password: "password123", name: "Dup" });

    expect(secondRes.status).toBe(409);
  });

  it("rejects login with a bad password", async () => {
    const agent = request.agent(app);
    await agent
      .post("/api/auth/signup")
      .send({ email: "badpw@example.com", password: "password123", name: "BadPw" });

    const loginRes = await agent
      .post("/api/auth/login")
      .send({ email: "badpw@example.com", password: "wrong-password" });

    expect(loginRes.status).toBe(401);
  });

  it("refreshes the access token via the refresh cookie", async () => {
    const agent = request.agent(app);
    await agent
      .post("/api/auth/signup")
      .send({ email: "refresh@example.com", password: "password123", name: "Refresh" });

    const refreshRes = await agent.post("/api/auth/refresh");
    expect(refreshRes.status).toBe(200);

    const meRes = await agent.get("/api/users/me");
    expect(meRes.status).toBe(200);
  });
});
