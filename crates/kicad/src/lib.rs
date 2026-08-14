//! Direct, dependency-light KiCad board importer.

use anyhow::{anyhow, bail, Context, Result};
use std::collections::{BTreeMap, HashMap};
use std::path::Path;
use uuid::Uuid;
use velocitypnp_domain::{BoardDesign, BoardSide, Placement, Pose};

#[derive(Debug, Clone, PartialEq)]
enum SExpr {
    Atom(String),
    List(Vec<SExpr>),
}

impl SExpr {
    fn atom(&self) -> Option<&str> {
        match self {
            Self::Atom(value) => Some(value),
            Self::List(_) => None,
        }
    }

    fn items(&self) -> Option<&[SExpr]> {
        match self {
            Self::List(items) => Some(items),
            Self::Atom(_) => None,
        }
    }

    fn named(&self, name: &str) -> bool {
        self.items()
            .and_then(|items| items.first())
            .and_then(Self::atom)
            == Some(name)
    }

    fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a SExpr> {
        self.items()
            .into_iter()
            .flatten()
            .filter(move |value| value.named(name))
    }
}

pub fn import_board_file(path: &Path) -> Result<BoardDesign> {
    let source = std::fs::read_to_string(path)
        .with_context(|| format!("reading KiCad board {}", path.display()))?;
    let name = path.file_stem().and_then(|v| v.to_str()).unwrap_or("board");
    import_board(&source, name)
}

pub fn import_board(source: &str, name: &str) -> Result<BoardDesign> {
    let root = Parser::new(source).parse()?;
    if !root.named("kicad_pcb") {
        bail!("expected a kicad_pcb root expression");
    }

    let source_version = root.children_named("version").next().and_then(second_u32);
    let mut placements = Vec::new();
    let mut warnings = Vec::new();
    for footprint in root
        .items()
        .into_iter()
        .flatten()
        .filter(|item| item.named("footprint") || item.named("module"))
    {
        match parse_footprint(footprint) {
            Ok(placement) => placements.push(placement),
            Err(error) => warnings.push(format!("Skipped footprint: {error}")),
        }
    }

    let edge_points = collect_edge_points(&root);
    let (width_mm, height_mm) = dimensions(&edge_points);
    if edge_points.is_empty() {
        warnings.push(
            "No usable Edge.Cuts geometry was found; board fit must be confirmed manually".into(),
        );
    }
    if placements.is_empty() {
        warnings.push("The board contains no importable footprints".into());
    }

    Ok(BoardDesign {
        id: Uuid::new_v4(),
        name: name.into(),
        source_format: "kicad_pcb".into(),
        source_version,
        width_mm,
        height_mm,
        placements,
        warnings,
    })
}

fn parse_footprint(expr: &SExpr) -> Result<Placement> {
    let items = expr.items().context("footprint is not a list")?;
    let package = items
        .get(1)
        .and_then(SExpr::atom)
        .unwrap_or("unknown")
        .to_string();
    let layer = expr
        .children_named("layer")
        .next()
        .and_then(second_atom)
        .unwrap_or("F.Cu");
    let side = if layer.starts_with("B.") {
        BoardSide::Bottom
    } else {
        BoardSide::Top
    };
    let at = expr
        .children_named("at")
        .next()
        .context("footprint has no at position")?;
    let at_items = at.items().unwrap_or_default();
    let x_mm = number(at_items.get(1)).context("invalid footprint X")?;
    let y_mm = number(at_items.get(2)).context("invalid footprint Y")?;
    let rotation_deg = number(at_items.get(3)).unwrap_or(0.0);

    let mut properties = BTreeMap::new();
    for property in expr.children_named("property") {
        if let Some(values) = property.items() {
            if let (Some(key), Some(value)) = (
                values.get(1).and_then(SExpr::atom),
                values.get(2).and_then(SExpr::atom),
            ) {
                properties.insert(key.to_string(), value.to_string());
            }
        }
    }
    for text in expr.children_named("fp_text") {
        if let Some(values) = text.items() {
            if let (Some(kind), Some(value)) = (
                values.get(1).and_then(SExpr::atom),
                values.get(2).and_then(SExpr::atom),
            ) {
                if kind == "reference" {
                    properties
                        .entry("Reference".into())
                        .or_insert_with(|| value.into());
                } else if kind == "value" {
                    properties
                        .entry("Value".into())
                        .or_insert_with(|| value.into());
                }
            }
        }
    }

    let reference = properties
        .get("Reference")
        .cloned()
        .filter(|value| !value.is_empty())
        .ok_or_else(|| anyhow!("footprint {package} has no Reference property"))?;
    let value = properties.get("Value").cloned().unwrap_or_default();
    let attributes = expr
        .children_named("attr")
        .next()
        .and_then(SExpr::items)
        .unwrap_or_default()
        .iter()
        .skip(1)
        .filter_map(SExpr::atom)
        .collect::<Vec<_>>();
    let enabled = !attributes
        .iter()
        .any(|attribute| matches!(*attribute, "exclude_from_pos_files" | "dnp" | "board_only"));
    let lower_package = package.to_ascii_lowercase();
    let fiducial =
        reference.to_ascii_uppercase().starts_with("FID") || lower_package.contains("fiducial");

    Ok(Placement {
        reference,
        value,
        package,
        side,
        pose: Pose {
            x_mm,
            y_mm,
            z_mm: 0.0,
            rotation_deg,
        },
        enabled,
        fiducial,
        properties,
    })
}

/// Enrich placements from a permissive CSV BOM. Recognized headers include
/// Reference/Designator, MPN, Height, and DNP/Populate.
pub fn merge_bom(board: &mut BoardDesign, csv_source: &str) -> Result<Vec<String>> {
    let mut reader = csv::ReaderBuilder::new()
        .flexible(true)
        .from_reader(csv_source.as_bytes());
    let headers = reader.headers()?.clone();
    let header_map = headers
        .iter()
        .enumerate()
        .map(|(index, value)| (normalize_header(value), index))
        .collect::<HashMap<_, _>>();
    let reference_column = find_column(
        &header_map,
        &["reference", "references", "designator", "ref"],
    )
    .context("BOM must contain a Reference or Designator column")?;
    let mut unmatched = Vec::new();
    for record in reader.records() {
        let record = record?;
        let references = record.get(reference_column).unwrap_or_default();
        for reference in split_references(references) {
            let Some(placement) = board
                .placements
                .iter_mut()
                .find(|p| p.reference == reference)
            else {
                unmatched.push(reference.to_string());
                continue;
            };
            copy_column(
                &record,
                &header_map,
                &["mpn", "manufacturerpartnumber"],
                placement,
                "MPN",
            );
            copy_column(
                &record,
                &header_map,
                &["height", "heightmm"],
                placement,
                "Height",
            );
            if let Some(index) = find_column(&header_map, &["dnp", "donotpopulate", "populate"]) {
                let value = record
                    .get(index)
                    .unwrap_or_default()
                    .trim()
                    .to_ascii_lowercase();
                let header = normalize_header(headers.get(index).unwrap_or_default());
                placement.enabled = if header == "populate" {
                    !matches!(value.as_str(), "no" | "false" | "0" | "dnp")
                } else {
                    !matches!(value.as_str(), "yes" | "true" | "1" | "dnp")
                };
            }
        }
    }
    Ok(unmatched)
}

fn copy_column(
    record: &csv::StringRecord,
    headers: &HashMap<String, usize>,
    names: &[&str],
    placement: &mut Placement,
    key: &str,
) {
    if let Some(value) = find_column(headers, names).and_then(|index| record.get(index)) {
        if !value.trim().is_empty() {
            placement.properties.insert(key.into(), value.trim().into());
        }
    }
}

fn find_column(headers: &HashMap<String, usize>, names: &[&str]) -> Option<usize> {
    names.iter().find_map(|name| headers.get(*name).copied())
}

fn normalize_header(value: &str) -> String {
    value
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

fn split_references(value: &str) -> impl Iterator<Item = &str> {
    value
        .split([',', ';', ' '])
        .map(str::trim)
        .filter(|value| !value.is_empty())
}

fn collect_edge_points(root: &SExpr) -> Vec<(f64, f64)> {
    let mut points = Vec::new();
    for item in root.items().into_iter().flatten() {
        let is_edge = item
            .children_named("layer")
            .any(|layer| second_atom(layer) == Some("Edge.Cuts"));
        if !is_edge {
            continue;
        }
        collect_coordinates(item, &mut points);
    }
    points
}

fn collect_coordinates(expr: &SExpr, points: &mut Vec<(f64, f64)>) {
    let Some(items) = expr.items() else { return };
    if matches!(
        items.first().and_then(SExpr::atom),
        Some("start" | "end" | "mid" | "center" | "xy")
    ) {
        if let (Some(x), Some(y)) = (number(items.get(1)), number(items.get(2))) {
            points.push((x, y));
        }
    }
    for child in items {
        collect_coordinates(child, points);
    }
}

fn dimensions(points: &[(f64, f64)]) -> (Option<f64>, Option<f64>) {
    if points.is_empty() {
        return (None, None);
    }
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in points {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    (Some(max_x - min_x), Some(max_y - min_y))
}

fn second_atom(expr: &SExpr) -> Option<&str> {
    expr.items()?.get(1)?.atom()
}

fn second_u32(expr: &SExpr) -> Option<u32> {
    second_atom(expr)?.parse().ok()
}

fn number(value: Option<&SExpr>) -> Option<f64> {
    value?.atom()?.parse().ok()
}

struct Parser<'a> {
    source: &'a [u8],
    index: usize,
}

impl<'a> Parser<'a> {
    fn new(source: &'a str) -> Self {
        Self {
            source: source.as_bytes(),
            index: 0,
        }
    }

    fn parse(mut self) -> Result<SExpr> {
        self.skip_space_and_comments();
        let value = self.expression()?;
        self.skip_space_and_comments();
        if self.index != self.source.len() {
            bail!(
                "unexpected data after root expression at byte {}",
                self.index
            );
        }
        Ok(value)
    }

    fn expression(&mut self) -> Result<SExpr> {
        self.skip_space_and_comments();
        match self.peek() {
            Some(b'(') => self.list(),
            Some(b'"') => self.string().map(SExpr::Atom),
            Some(_) => self.atom().map(SExpr::Atom),
            None => bail!("unexpected end of KiCad file"),
        }
    }

    fn list(&mut self) -> Result<SExpr> {
        self.index += 1;
        let mut values = Vec::new();
        loop {
            self.skip_space_and_comments();
            match self.peek() {
                Some(b')') => {
                    self.index += 1;
                    return Ok(SExpr::List(values));
                }
                Some(_) => values.push(self.expression()?),
                None => bail!("unterminated list in KiCad file"),
            }
        }
    }

    fn string(&mut self) -> Result<String> {
        self.index += 1;
        let mut value = String::new();
        while let Some(byte) = self.peek() {
            self.index += 1;
            match byte {
                b'"' => return Ok(value),
                b'\\' => {
                    let escaped = self.peek().context("unterminated string escape")?;
                    self.index += 1;
                    value.push(match escaped {
                        b'n' => '\n',
                        b'r' => '\r',
                        b't' => '\t',
                        other => other as char,
                    });
                }
                other => value.push(other as char),
            }
        }
        bail!("unterminated string in KiCad file")
    }

    fn atom(&mut self) -> Result<String> {
        let start = self.index;
        while let Some(byte) = self.peek() {
            if byte.is_ascii_whitespace() || matches!(byte, b'(' | b')' | b';') {
                break;
            }
            self.index += 1;
        }
        if self.index == start {
            bail!("expected token at byte {}", self.index);
        }
        Ok(String::from_utf8(self.source[start..self.index].to_vec())?)
    }

    fn skip_space_and_comments(&mut self) {
        loop {
            while self.peek().is_some_and(|byte| byte.is_ascii_whitespace()) {
                self.index += 1;
            }
            if self.peek() == Some(b';') {
                while self.peek().is_some_and(|byte| byte != b'\n') {
                    self.index += 1;
                }
            } else {
                break;
            }
        }
    }

    fn peek(&self) -> Option<u8> {
        self.source.get(self.index).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOARD: &str = r#"
      (kicad_pcb (version 20240108)
        (footprint "Resistor_SMD:R_0402_1005Metric"
          (layer "F.Cu") (at 10.25 20.5 90)
          (property "Reference" "R1") (property "Value" "10k") (attr smd))
        (footprint "Fiducial:Fiducial_1mm_Mask2mm"
          (layer "B.Cu") (at 5 7) (property "Reference" "FID1")
          (property "Value" "Fiducial") (attr smd exclude_from_pos_files))
        (gr_rect (start 0 0) (end 50 30) (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts")))
    "#;

    #[test]
    fn imports_placements_sides_attributes_and_dimensions() {
        let board = import_board(BOARD, "test").unwrap();
        assert_eq!(board.source_version, Some(20240108));
        assert_eq!(board.placements.len(), 2);
        assert_eq!(board.placements[0].reference, "R1");
        assert_eq!(board.placements[0].pose.rotation_deg, 90.0);
        assert_eq!(board.placements[1].side, BoardSide::Bottom);
        assert!(board.placements[1].fiducial);
        assert!(!board.placements[1].enabled);
        assert_eq!(board.width_mm, Some(50.0));
        assert_eq!(board.height_mm, Some(30.0));
    }

    #[test]
    fn merges_optional_bom() {
        let mut board = import_board(BOARD, "test").unwrap();
        let unmatched = merge_bom(
            &mut board,
            "Reference,MPN,Height,DNP\nR1,RC0402-10K,0.35,no\nX1,nope,1,yes\n",
        )
        .unwrap();
        assert_eq!(board.placements[0].properties["MPN"], "RC0402-10K");
        assert!(board.placements[0].enabled);
        assert_eq!(unmatched, vec!["X1"]);
    }

    #[test]
    fn rejects_non_kicad_root() {
        assert!(import_board("(not_a_board)", "bad").is_err());
    }
}
