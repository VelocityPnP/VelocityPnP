import { describe, expect, it } from "vitest";
import { pretty } from "./App";

describe("pretty", () => {
  it("formats API identifiers", () => {
    expect(pretty("machine.homing_completed")).toBe("Machine Homing Completed");
  });
});
