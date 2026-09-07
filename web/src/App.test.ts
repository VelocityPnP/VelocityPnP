import { describe, expect, it } from "vitest";
import {
  canResolveJobStep,
  canStartJob,
  completedJobStepCount,
  pretty,
} from "./App";

describe("pretty", () => {
  it("formats API identifiers", () => {
    expect(pretty("machine.homing_completed")).toBe("Machine Homing Completed");
  });
});

describe("job action state", () => {
  it("starts only from a ready and fine-homed machine", () => {
    expect(canStartJob({
      state: "ready", homed: true, calibration_revision: "calibration-id",
    }, false, false)).toBe(true);
    expect(canStartJob({
      state: "disconnected", homed: false, calibration_revision: null,
    }, false, false)).toBe(false);
    expect(canStartJob({
      state: "ready", homed: true, calibration_revision: null,
    }, false, false)).toBe(false);
  });

  it("counts skipped steps as complete", () => {
    expect(completedJobStepCount([
      { status: "succeeded" },
      { status: "skipped" },
      { status: "pending" },
    ])).toBe(2);
  });

  it("offers recovery only for a run that needs attention", () => {
    expect(canResolveJobStep("needs_attention", "needs_attention")).toBe(true);
    expect(canResolveJobStep("cancelled", "needs_attention")).toBe(false);
    expect(canResolveJobStep("needs_attention", "pending")).toBe(false);
  });
});
