import { afterEach, describe, expect, it, vi } from "vitest";
import { api, formatErrorMessage, waitForOperation } from "./api";

const lease = {
  id: "lease-id",
  client_name: "test",
  acquired_at: "2026-09-07T18:00:00Z",
  expires_at: "2026-09-07T18:01:00Z",
};

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

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

describe("API response validation", () => {
  it("reports a web fallback response as a daemon version mismatch", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue(
      new Response("<!doctype html><title>VelocityPnP</title>", { status: 200 }),
    ));

    await expect(api.stripFeederOccupancyReview("registration-id"))
      .rejects.toMatchObject({
        status: 502,
        message: "The daemon returned text/plain;charset=UTF-8 for GET "
          + "/api/v1/strip-feeder-registrations/registration-id/occupancy-review. "
          + "The web UI and daemon may be different versions.",
      });
    expect(fetch).toHaveBeenCalledWith(
      "/api/v1/strip-feeder-registrations/registration-id/occupancy-review",
      { cache: "no-store" },
    );
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

describe("startWorkspaceSurvey", () => {
  it("estimates a focused scan with the same survey specification", async () => {
    const estimate = {
      mode: "survey",
      artifact_retention: "transient_sources",
      planned_tiles: 24,
      estimated_duration_seconds: 75,
    };
    const fetch = vi.fn().mockResolvedValue(
      new Response(JSON.stringify(estimate), { status: 200 }),
    );
    vi.stubGlobal("fetch", fetch);
    const region = {
      x_min_mm: 10, x_max_mm: 30, y_min_mm: 40, y_max_mm: 80,
    };

    await expect(api.estimateWorkspaceSurvey(
      "full-scan", "calibration-id", region,
    )).resolves.toEqual(estimate);

    const [url, init] = fetch.mock.calls[0] as [string, RequestInit];
    expect(url).toBe("/api/v1/workspace-scans/estimate");
    expect(JSON.parse(String(init.body))).toMatchObject({
      calibration_id: "calibration-id",
      mode: "survey",
      artifact_retention: "transient_sources",
      region_preset: "installed_plates",
      parent_scan_id: "full-scan",
      output_mm_per_pixel: .05,
      region,
    });
  });

  it("links a focused scan to its full workspace scan", async () => {
    const response = {
      scan: { id: "local-scan" },
      operation: {
        id: "scan-operation", kind: "vision.workspace_scan",
        status: "pending", message: null,
      },
    };
    const fetch = vi.fn().mockResolvedValue(
      new Response(JSON.stringify(response), { status: 202 }),
    );
    vi.stubGlobal("fetch", fetch);
    const region = {
      x_min_mm: 10, x_max_mm: 30, y_min_mm: 40, y_max_mm: 80,
    };

    await expect(api.startWorkspaceSurvey(
      "full-scan", "calibration-id", region, lease.id,
    )).resolves.toEqual(response);

    const [url, init] = fetch.mock.calls[0] as [string, RequestInit];
    expect(url).toBe("/api/v1/workspace-scans");
    expect(JSON.parse(String(init.body))).toMatchObject({
      lease_id: lease.id,
      calibration_id: "calibration-id",
      mode: "survey",
      artifact_retention: "transient_sources",
      region_preset: "installed_plates",
      parent_scan_id: "full-scan",
      output_mm_per_pixel: .05,
      region,
    });
  });
});

describe("waitForOperation", () => {
  it("reports each polled operation state", async () => {
    vi.useFakeTimers();
    const running = {
      id: "operation-id", kind: "calibration.test", status: "running", message: null,
    } as const;
    const succeeded = { ...running, status: "succeeded" as const, message: "Complete" };
    vi.stubGlobal("fetch", vi.fn()
      .mockResolvedValueOnce(new Response(JSON.stringify(running), { status: 200 }))
      .mockResolvedValueOnce(new Response(JSON.stringify(succeeded), { status: 200 })));
    const updates: string[] = [];

    const result = waitForOperation(
      { ...running, status: "pending" },
      (current) => updates.push(current.status),
    );
    await vi.advanceTimersByTimeAsync(240);

    await expect(result).resolves.toEqual(succeeded);
    expect(updates).toEqual(["pending", "running", "succeeded"]);
  });
});
