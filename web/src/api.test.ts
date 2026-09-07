import { afterEach, describe, expect, it, vi } from "vitest";
import { api, formatErrorMessage } from "./api";

const lease = {
  id: "lease-id",
  client_name: "test",
  acquired_at: "2026-09-07T18:00:00Z",
  expires_at: "2026-09-07T18:01:00Z",
};

afterEach(() => vi.unstubAllGlobals());

describe("formatErrorMessage", () => {
  it("adds sentence capitalization and punctuation", () => {
    expect(formatErrorMessage("control lease does not match"))
      .toBe("Control lease does not match.");
  });

  it("preserves an error that is already formatted", () => {
    expect(formatErrorMessage("Camera is not attached!"))
      .toBe("Camera is not attached!");
  });
});

describe("acquireOrRenewLease", () => {
  it("replaces a stale lease reported by the machine status", async () => {
    const replacement = { ...lease, id: "replacement-lease" };
    const fetch = vi.fn()
      .mockResolvedValueOnce(new Response(JSON.stringify({
        machine: { control_lease: lease },
      }), { status: 200 }))
      .mockResolvedValueOnce(new Response(JSON.stringify({
        error: { message: "control lease does not match" },
      }), { status: 400 }))
      .mockResolvedValueOnce(new Response(JSON.stringify({
        lease: replacement,
      }), { status: 200 }));
    vi.stubGlobal("fetch", fetch);

    await expect(api.acquireOrRenewLease("test", 900))
      .resolves.toEqual(replacement);
    expect(fetch).toHaveBeenCalledTimes(3);
  });
});
