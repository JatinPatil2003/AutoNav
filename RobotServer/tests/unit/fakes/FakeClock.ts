import { Clock } from "../../../src/common/clock";

/** Deterministic clock for tests — advance it explicitly instead of sleeping. */
export class FakeClock implements Clock {
  constructor(private current = 0) {}

  now(): number {
    return this.current;
  }

  advanceBy(ms: number): void {
    this.current += ms;
  }
}
