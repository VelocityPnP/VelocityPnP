use anyhow::{bail, Context, Result};
use serde::Deserialize;
use std::env;
use std::fs;
use std::path::Path;
use velocitypnp_domain::{
    StripPocketOccupancy, WorkspaceBoardCandidate, WorkspaceBoardDiscoveryReport,
};
use velocitypnp_vision::{analyze_strip_feeder_candidate, decode_gray_png};

#[derive(Debug, Deserialize)]
struct GroundTruthFixture {
    schema_version: u32,
    source_candidate_index: u32,
    lanes: Vec<GroundTruthLane>,
}

#[derive(Debug, Deserialize)]
struct GroundTruthLane {
    feeder_number: u32,
    occupancy: String,
}

#[derive(Debug, Default)]
struct Evaluation {
    correct_confident: usize,
    incorrect_confident: usize,
    uncertain: usize,
    omitted: usize,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    let [image_path, discovery_path, fixture_path] = arguments.as_slice() else {
        bail!(
            "usage: evaluate_strip_feeder_ground_truth \
             EVIDENCE.png BOARD_DISCOVERY.json GROUND_TRUTH.json"
        );
    };
    let image =
        fs::read(image_path).with_context(|| format!("reading evidence image {image_path}"))?;
    let discovery: WorkspaceBoardDiscoveryReport = read_json(discovery_path)?;
    let fixture: GroundTruthFixture = read_json(fixture_path)?;
    if fixture.schema_version != 1 {
        bail!("unsupported ground-truth schema {}", fixture.schema_version);
    }
    let candidate: WorkspaceBoardCandidate = discovery
        .candidates
        .into_iter()
        .find(|candidate| candidate.candidate_index == fixture.source_candidate_index)
        .context("ground-truth candidate is absent from board discovery")?;
    let analysis = analyze_strip_feeder_candidate(&decode_gray_png(&image)?, &candidate)?;
    if analysis.proposals.len() != fixture.lanes.len() {
        bail!(
            "detected {} lanes; fixture has {} lanes",
            analysis.proposals.len(),
            fixture.lanes.len()
        );
    }

    let mut total = Evaluation::default();
    for (proposal, truth) in analysis.proposals.iter().zip(&fixture.lanes) {
        if proposal.proposal_index + 1 != truth.feeder_number {
            bail!(
                "proposal {} does not match feeder {}",
                proposal.proposal_index + 1,
                truth.feeder_number
            );
        }
        let expected = truth.occupancy.as_bytes();
        let detected = proposal
            .pocket_observations
            .iter()
            .map(|observation| observation.occupancy)
            .collect::<Vec<_>>();
        if detected.len() > expected.len() {
            bail!(
                "feeder {} detected {} pockets; fixture has {}",
                truth.feeder_number,
                detected.len(),
                expected.len()
            );
        }
        let (offset, lane) = best_alignment(expected, &detected);
        println!(
            "feeder {}: detected {}/{}, offset {}, correct {}, incorrect {}, \
             uncertain {}",
            truth.feeder_number,
            detected.len(),
            expected.len(),
            offset,
            lane.correct_confident,
            lane.incorrect_confident,
            lane.uncertain,
        );
        total.correct_confident += lane.correct_confident;
        total.incorrect_confident += lane.incorrect_confident;
        total.uncertain += lane.uncertain;
        total.omitted += expected.len() - detected.len();
    }
    println!(
        "total: correct {}, incorrect {}, uncertain {}, omitted {}",
        total.correct_confident, total.incorrect_confident, total.uncertain, total.omitted,
    );
    if total.incorrect_confident > 0 {
        bail!(
            "{} confident occupancy labels disagree with ground truth",
            total.incorrect_confident
        );
    }
    Ok(())
}

fn read_json<T: for<'de> Deserialize<'de>>(path: impl AsRef<Path>) -> Result<T> {
    let path = path.as_ref();
    let data = fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    serde_json::from_slice(&data).with_context(|| format!("decoding {}", path.display()))
}

fn best_alignment(expected: &[u8], detected: &[StripPocketOccupancy]) -> (usize, Evaluation) {
    (0..=expected.len() - detected.len())
        .map(|offset| {
            let mut result = Evaluation::default();
            for (expected, detected) in expected[offset..].iter().zip(detected) {
                match detected {
                    StripPocketOccupancy::Uncertain => result.uncertain += 1,
                    StripPocketOccupancy::Occupied if *expected == b'P' => {
                        result.correct_confident += 1;
                    }
                    StripPocketOccupancy::Empty if *expected == b'E' => {
                        result.correct_confident += 1;
                    }
                    _ => result.incorrect_confident += 1,
                }
            }
            (offset, result)
        })
        .max_by(|first, second| {
            first
                .1
                .correct_confident
                .cmp(&second.1.correct_confident)
                .then_with(|| {
                    second
                        .1
                        .incorrect_confident
                        .cmp(&first.1.incorrect_confident)
                })
                .then_with(|| second.0.cmp(&first.0))
        })
        .expect("an alignment always exists")
}
