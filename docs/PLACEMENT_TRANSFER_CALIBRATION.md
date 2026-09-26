# Placement-transfer calibration

## Decision

Add a first-class `PlacementTransfer` calibration step between the nozzle
height model and final known-board validation.

This step calibrates the end-to-end XY transfer from bottom-camera part
alignment to top-camera placement. It does not qualify a feeder. It does not
return parts to feeder pockets. It consumes parts and empty board targets.

Keep final validation separate. Calibration data can fit a correction. Final
validation must use different placements and must not change the correction.

Release and seating height are also separate. The existing placement-process
estimate controls those values. Placement-transfer calibration reports release
failures, but it does not tune Z.

## Existing components

The guarded production placement path already supplies the required evidence:

- a live top-camera source measurement;
- initial and final bottom-camera held-part measurements;
- local top-camera registration of the bare board target;
- verified release and a clear-nozzle result;
- a top-camera result image and placement center estimate; and
- persistent execution and image records.

The nozzle height model already has a per-nozzle working-plane correction.
`NozzleHeightModel::observe_post_correction_residual` updates this term. The
current manual nozzle-height observation endpoint uses the same operation.

The new step must call the guarded placement implementation. It must not copy
the pickup, alignment, release, or inspection motion into a second executor.

## Coordinate contract

`PlacedPartInspectionReport.offset_x_mm` and `offset_y_mm` are recenter
commands. They describe the machine correction that would move the observed
part center onto the registered target. They are not signed physical part
displacements.

The calibration algorithm must use these values directly:

```text
candidate working-plane correction
    = current working-plane correction
    + robust mean of top-inspection recenter commands
```

The web interface must show these as `recenter X/Y`. If it also shows physical
part displacement, it must label that value separately and use the opposite
sign. Stored reports must not use an unlabeled `Y error` field.

## Run sequence

```text
preflight
   |
   v
baseline placements ---- no stable measurement ----> stop for review
   |
   v
bounded candidate correction
   |
   v
independent check placements ---- fails ----> one more bounded round or stop
   |
   v
commit correction and qualification evidence
```

Use three baseline placements and three check placements by default. Allow a
minimum of two in each set. Allow no more than two adjustment rounds. The
preflight must show the maximum number of parts and board targets that the run
can consume.

If the baseline already passes the residual limits, record a zero-change
qualification. No check set is necessary because the baseline did not fit a
new value.

When the baseline causes an adjustment, never use a baseline image in the
check set.

## Target selection

All samples in one run must have the same:

- calibration revision;
- board instance and measured board surface;
- part key and package;
- nozzle and installed tip;
- nominal part rotation;
- placement surface offset;
- seating and release schedule; and
- feeder registration.

The server can accept an ordered list of placement references. It must also be
able to select eligible unused references from a job. It must validate the
complete list before motion starts.

The baseline and check sets must have similar board-position centroids. This
limits distortion and board-region differences. The first version can reject
a set when the two centroids differ by more than a profile limit.

Each target must be empty and must have no earlier physical board contact.
Normal placement rules reserve and consume feeder pockets. A failed pickup can
release its reservation. Board contact always consumes the board target.

## Measurement gates

A placement can supply an XY sample only when all these checks pass:

- local board-target registration passed;
- bottom-camera alignment passed;
- the final held-part residual is within the profile limit;
- the release sensor and bottom camera confirm release;
- the nozzle is clear;
- deterministic top vision detected one flat package;
- the estimated center and recenter command are finite;
- rotation is within the calibration limit; and
- learned and deterministic placement results agree when a learned model is
  active.

An execution does not have to pass the normal production center tolerance to
supply a calibration sample. A large but well-measured offset is the input to
this step. Missing, on-edge, stacked, ambiguous, or weak-image results cannot
supply a sample.

The run must stop for review when top inspection is uncertain. It must not
infer a correction from a failed detection.

## Adjustment policy

For each round, calculate a robust center from its recenter vectors. Report
the vector for every sample, the robust center, radial residuals, and the
maximum distance from the robust center.

Reject the round when:

- fewer than the configured sample count are usable;
- sample spread exceeds the configured limit;
- one suggested adjustment exceeds the configured bound; or
- the measurements do not have one common direction.

Apply the full robust recenter vector as the first candidate. The profile must
bound each candidate and the cumulative correction. Do not silently clamp an
out-of-range result. A result outside the bound indicates a calibration or
measurement fault.

A check round passes only when its mean recenter magnitude and its p95 radial
residual are within profile limits. Suggested initial limits are 0.05 mm for
the mean and 0.10 mm for p95. These limits must be profile data, not API input.

Rotation remains an acceptance measurement in version 1. Do not adjust nozzle
rotation from symmetric resistor images.

## Provisional correction and rollback

Do not write a candidate correction into the calibration snapshot before its
check round passes.

Add an internal `working_plane_correction_delta_mm` to the prepared placement
context. Only the placement-transfer runner can set it. The normal production
API cannot accept this override. The physical placement transform applies the
delta at the same point as the stored nozzle working-plane correction.

This rule gives these properties:

- a daemon crash leaves the calibration snapshot unchanged;
- a failed check needs no database rollback;
- production placement cannot use an unverified correction; and
- the check images measure the exact candidate that will be committed.

After a check passes, update the nozzle height model once. Recompute its
runtime tip transform. Then record qualification evidence against the new
runtime transform.

## Calibration model

Add these calibration artifact IDs:

- `PlacementTransferN1`; and
- `PlacementTransferN2`.

Each artifact depends on the matching runtime tip transform, the top-camera
planar model, and the matching vacuum characterization. It records the final
correction and links to every placement execution that supplied evidence.

The runtime tip transform does not depend on placement-transfer evidence. This
avoids a dependency cycle. Production-readiness checks must explicitly require
current placement-transfer evidence for each production nozzle.

Add `CalibrationStepId::PlacementTransfer`. It depends on the nozzle height
model and vacuum thresholds. Final `Validation` depends on
`PlacementTransfer`.

Add `CalibrationRunPhase::PlacementTransferCalibration` and a typed
`PlacementTransferCalibrationReport` to the persistent calibration run. The
report contains:

- job, board, feeder, part, nozzle, and target IDs;
- initial and final working-plane corrections;
- per-round candidate deltas;
- child placement execution IDs;
- recenter vectors and measurement gates;
- before-and-after summary statistics;
- the committed artifact fingerprint; and
- a passed, rejected, or interrupted result.

## API and interface

Add this endpoint:

```text
POST /api/v1/calibrations/{id}/placement-transfer-runs
```

The request supplies the control lease, job, nozzle, optional ordered target
references, and the external safety confirmation. It does not supply
correction values or acceptance limits.

The existing calibration-run endpoints expose progress and the final report.
The calibration page must show:

- the selected part, nozzle, feeder, board, and target count;
- the maximum part consumption before start;
- each placement and its evidence status;
- baseline and check recenter vectors;
- the provisional and committed corrections;
- intended and detected center overlays; and
- the reason for a stop or rejection.

The start confirmation must say that the operation consumes parts and board
targets. It must confirm that the PCB and feeders are fixed, the selected
targets are empty, the nozzles are level, the travel path is clear, and power
isolation is reachable.

## Implementation order

1. Add pure summary and adjustment types in the domain crate. Test sign,
   spread, limits, convergence, and insufficient evidence.
2. Add the internal provisional correction to guarded placement. Test that
   normal production requests cannot set it.
3. Add the persistent placement-transfer run and artifact records. Test that
   an interrupted run does not change the snapshot.
4. Add the calibration endpoint and reuse the guarded executor for each child
   placement.
5. Add the calibration-page controls, progress, and image overlays.
6. Run a same-part, same-angle hardware test. Compare the committed result
   with a later independent validation set.
