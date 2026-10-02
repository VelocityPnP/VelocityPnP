# Placement-transfer calibration

## Decision

Add a first-class `PlacementTransfer` calibration step between the nozzle
height model and final known-board validation.

This step calibrates the end-to-end XY transfer from bottom-camera part
alignment to top-camera placement. It does not qualify a feeder. It does not
return parts to feeder pockets. It consumes parts and empty board targets.

Keep final validation separate. Calibration data can fit a correction. Final
validation must use different placements and must not change the correction.

## Implementation and hardware status

The daemon implements the placement-transfer artifact, report, calibration and
validation endpoints, provisional correction, bounded rounds, and continuation
of a rejected calibration run.
The continuation accepts recomputed, compatible evidence from completed
rounds and validated partial-round samples. This prevents a failed later
sample from causing a second placement on an occupied target. Normal
production requests cannot supply a correction override.

The 2026-09-26 N1 hardware run completed one three-part baseline. Its robust
physical part offset was X+0.091 and Y+0.091 mm. The p95 radial offset was
0.215 mm. The samples had a common direction and passed the 0.100 mm spread
limit, so the runner prepared the provisional X+0.091 and Y+0.091 mm
correction.

The 2026-09-27 corrected check completed R29, R32, and R26 with the provisional
correction. Their signed offsets were X-0.062/Y+0.025,
X-0.053/Y+0.038, and X+0.030/Y-0.056 mm. All radial errors were at most
0.067 mm. The robust residual was X-0.053/Y+0.025 mm, or 0.059 mm.

The check did not pass. The left and right board columns had opposite residual
directions, and maximum sample deviation was 0.116 mm. This exceeds the
0.100 mm stability limit. No placement-transfer artifact was committed. The
result does not support a second global offset adjustment.

The 2026-10-02 run showed that the earlier decision policy mixed transfer bias
with final process capability. Its zero-correction R33, R36, and R38 baseline
had a 0.033 mm robust offset, 0.068 mm maximum sample deviation, and 0.1003 mm
p95 radial offset. The robust offset and spread passed, but the p95 missed its
0.100 mm limit by 0.0003 mm. Version 1 therefore tested an unnecessary
X-0.010/Y-0.032 mm correction. The corrected check had no common residual
direction. No artifact was committed.

Version 2 separates these decisions. Calibration qualifies a small systematic
offset with bounded spread. It records p95 variation but does not try to remove
that variation with a constant correction. A separate validation-only run uses
new targets, applies the stored correction, requires the p95 limit, and cannot
change calibration. The web interface starts both modes and opens the active
model overlay for each sample. Deterministic CV remains an optional diagnostic.

The 2026-10-02 N1 version 2 run measured a zero-correction baseline on R24,
R22, and R1. Its robust physical offset was X+0.029/Y-0.174 mm, its p95 radial
error was 0.226 mm, and its maximum sample deviation was 0.049 mm. The check
round applied that provisional correction to R23, R21, and R20. The corrected
radial errors were 0.079, 0.038, and 0.071 mm. The corrected p95 was 0.079 mm.

The run did not commit the correction. Its robust check center was
X+0.036/Y-0.038 mm, or 0.052 mm, against the 0.050 mm systematic limit. Its
maximum sample deviation was 0.100084 mm against the 0.100 mm spread limit.
The measurements did not have one common direction. This is a narrow formal
gate failure, not evidence for another global offset change. Independent
validation could not run because there is no current committed N1 transfer
artifact.

The same run exposed poor source-pick centering in several feeder pockets.
Calibration pickup can use a bounded retry grid and a 0.200 mm seating move.
The Y-0.750 mm retry now runs before Y-0.500 mm because the smaller move often
made partial contact and stopped the pocket after moving the part. These retry
and seating changes apply only to placement-transfer calibration.

Release and seating height are also separate. The existing placement-process
estimate controls those values. Placement-transfer calibration reports release
failures, but it does not tune Z.

## Existing components

The production placement path already supplies the required evidence:

- a live top-camera source measurement;
- initial and final bottom-camera held-part measurements;
- local top-camera registration of the bare board target;
- verified release and a clear-nozzle result;
- a top-camera result image and placement center estimate; and
- persistent execution and image records.

Legacy corrections remain in the nozzle height model. New qualified
corrections are separate placement-transfer artifacts. Runtime placement adds
the two terms. This avoids invalidating vacuum evidence when this step runs.

The new step must call the production placement implementation. It must not copy
the pickup, alignment, release, or inspection motion into a second executor.

## Coordinate contract

`PlacedPartInspectionReport.offset_x_mm` and `offset_y_mm` are signed physical
part-center displacements from the registered target. They are not direct head
move commands.

The calibration algorithm must use these values directly:

```text
candidate placement-transfer correction
    = current placement-transfer correction
    + robust center of top-inspection physical part offsets
```

The placement transform subtracts this stored correction when it calculates
the head position. Thus, a positive physical Y error causes a negative Y head
move on the next round. The web interface must show `physical part offset` and
`stored correction delta` separately. Stored reports must not use an unlabeled
`Y error` field.

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
   |
   v
new validation placements ---- fails ----> keep correction; report capability
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
- the active inspection detected one flat package;
- the estimated center and physical part offset are finite;
- rotation is within the calibration limit; and
- a promoted learned result passed its occupancy and confidence control gates
  when it supplied the active center.

The paired deterministic center remains diagnostic evidence. It does not
reject a promoted learned center only because the two localization methods
disagree. This is required when deterministic thresholding locks onto a pad or
tape edge while the reviewed learned box remains on the package.

An execution does not have to pass the normal production center tolerance to
supply a calibration sample. A large but well-measured offset is the input to
this step. Missing, on-edge, stacked, ambiguous, or weak-image results cannot
supply a sample.

The run must stop for review when top inspection is uncertain. It must not
infer a correction from a failed detection.

## Adjustment policy

For each round, calculate a robust center from its physical offset vectors.
Report the vector for every sample, the robust center, radial residuals, and
the maximum distance from the robust center.

Reject the round when:

- fewer than the configured sample count are usable;
- sample spread exceeds the configured limit;
- one suggested adjustment exceeds the configured bound; or
- the measurements do not have one common direction.

Add the full robust physical offset vector to the stored correction. The
placement transform subtracts that correction from the next head command. The
profile must bound each candidate and the cumulative correction. Do not
silently clamp an out-of-range result. A result outside the bound indicates a
calibration or measurement fault.

A calibration round passes when its robust physical offset and maximum sample
deviation are within profile limits. The p95 radial result is retained as a
process-capability diagnostic. If the systematic offset is outside its limit,
the runner can adjust only when the samples have a common direction and bounded
spread.

A validation-only round passes only when the robust physical offset, maximum
sample deviation, and p95 radial residual are all within profile limits. The
initial profile uses 0.05 mm for systematic offset and 0.10 mm for p95. These
limits and the validation sample count are profile data, not API input.

Rotation remains an acceptance measurement in version 1. Do not adjust nozzle
rotation from symmetric resistor images.

## Provisional correction and rollback

Do not write a candidate correction into the calibration snapshot before its
check round passes.

Add an internal provisional placement-transfer correction to the prepared
placement context. Only the placement-transfer runner can set it. The normal
production API cannot accept this override. The physical placement transform
applies it with the stored nozzle working-plane correction.

This rule gives these properties:

- a daemon crash leaves the calibration snapshot unchanged;
- a failed check needs no database rollback;
- production placement cannot use an unverified correction; and
- the check images measure the exact candidate that will be committed.

After a check passes, store the correction in the matching placement-transfer
artifact. Do not mutate the nozzle height model or invalidate its dependent
vacuum artifacts.

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
model and vacuum sensor characterization. Final `Validation` depends on
`PlacementTransfer`.

Add `CalibrationRunPhase::PlacementTransferCalibration` and a typed
`PlacementTransferCalibrationReport` to the persistent calibration run. The
report contains:

- job, board, feeder, part, nozzle, and target IDs;
- initial and final placement-transfer corrections;
- per-round candidate deltas;
- child placement execution IDs;
- physical offset vectors, stored correction deltas, and measurement gates;
- before-and-after summary statistics;
- the committed artifact fingerprint; and
- a passed, rejected, or interrupted result.

## API and interface

Add this endpoint:

```text
POST /api/v1/calibrations/{id}/placement-transfer-runs
POST /api/v1/calibrations/{id}/placement-transfer-validation-runs
```

The request supplies the control lease, job, nozzle, optional ordered target
references, and the external safety confirmation. It does not supply
correction values or acceptance limits.

The validation endpoint requires a current per-nozzle placement-transfer
artifact. It rejects continuation input and never writes a correction.

The existing calibration-run endpoints expose progress and the final report.
The calibration page must show:

- the selected part, nozzle, feeder, board, and target count;
- the maximum part consumption before start;
- each placement and its evidence status;
- baseline and check physical offset vectors;
- the provisional and committed corrections;
- the active intended and detected center overlay, with deterministic CV as a
  separate diagnostic; and
- the reason for a stop or rejection.

The start confirmation must say that the operation consumes parts and board
targets. It must confirm that the PCB and feeders are fixed, the selected
targets are empty, the nozzles are level, the travel path is clear, and power
isolation is reachable.

## Original implementation order

1. Add pure summary and adjustment types in the domain crate. Test sign,
   spread, limits, convergence, and insufficient evidence.
2. Add the internal provisional correction to production placement. Test that
   normal production requests cannot set it.
3. Add the persistent placement-transfer run and artifact records. Test that
   an interrupted run does not change the snapshot.
4. Add the calibration endpoint and reuse the production executor for each child
   placement.
5. Add the calibration-page controls, progress, and image overlays.
6. Run a same-part, same-angle hardware test. Compare the committed result
   with a later independent validation set.
