//! One-way, read-only migration from user-owned OpenPnP XML. Imported values
//! are calibration seeds only and never confer production-ready status.

use anyhow::{Context, Result};
use roxmltree::{Document, Node};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenPnpImport {
    pub source_format: String,
    pub axes: Vec<AxisSeed>,
    pub cameras: Vec<CameraSeed>,
    pub nozzles: Vec<NozzleSeed>,
    pub feeders: Vec<FeederSeed>,
    pub parts: Vec<PartSeed>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisSeed {
    pub name: String,
    pub kind: Option<String>,
    pub controller_letter: Option<String>,
    pub home_coordinate_mm: Option<f64>,
    pub soft_limit_low_mm: Option<f64>,
    pub soft_limit_high_mm: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraSeed {
    pub name: String,
    pub looking: Option<String>,
    pub unique_id: Option<String>,
    pub flip_x: bool,
    pub flip_y: bool,
    pub units_per_pixel_mm: Option<(f64, f64)>,
    pub head_offsets_mm: Option<PoseSeed>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NozzleSeed {
    pub name: String,
    pub head_offsets_mm: Option<PoseSeed>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeederSeed {
    pub id: Option<String>,
    pub name: Option<String>,
    pub implementation: Option<String>,
    pub part_id: Option<String>,
    pub location_mm: Option<PoseSeed>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartSeed {
    pub id: String,
    pub name: Option<String>,
    pub package_id: Option<String>,
    pub height_mm: Option<f64>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PoseSeed {
    pub x_mm: f64,
    pub y_mm: f64,
    pub z_mm: f64,
    pub rotation_deg: f64,
}

pub fn import_openpnp(machine_xml: &str, parts_xml: Option<&str>) -> Result<OpenPnpImport> {
    let machine = Document::parse(machine_xml).context("parsing OpenPnP machine.xml")?;
    let root = machine.root_element();
    anyhow::ensure!(
        root.tag_name().name() == "openpnp-machine",
        "expected an openpnp-machine root element"
    );
    let axes: Vec<AxisSeed> = elements(&machine, "axis")
        .filter_map(|node| {
            let name = node.attribute("name")?.to_string();
            Some(AxisSeed {
                name,
                kind: attribute(node, "type"),
                controller_letter: attribute(node, "letter"),
                home_coordinate_mm: child_value(node, "home-coordinate"),
                soft_limit_low_mm: child_value(node, "soft-limit-low"),
                soft_limit_high_mm: child_value(node, "soft-limit-high"),
            })
        })
        .collect();
    let cameras: Vec<CameraSeed> = elements(&machine, "camera")
        .filter_map(|node| {
            Some(CameraSeed {
                name: node.attribute("name")?.to_string(),
                looking: attribute(node, "looking"),
                unique_id: attribute(node, "unique-id"),
                flip_x: node.attribute("flip-x") == Some("true"),
                flip_y: node.attribute("flip-y") == Some("true"),
                units_per_pixel_mm: child(node, "units-per-pixel")
                    .and_then(|value| Some((number(value, "x")?, number(value, "y")?))),
                head_offsets_mm: child(node, "head-offsets").and_then(pose),
            })
        })
        .collect();
    let nozzles = elements(&machine, "nozzle")
        .filter_map(|node| {
            Some(NozzleSeed {
                name: node.attribute("name")?.to_string(),
                head_offsets_mm: child(node, "head-offsets").and_then(pose),
            })
        })
        .collect();
    let feeders = elements(&machine, "feeder")
        .map(|node| FeederSeed {
            id: attribute(node, "id"),
            name: attribute(node, "name"),
            implementation: attribute(node, "class"),
            part_id: attribute(node, "part-id"),
            location_mm: child(node, "location").and_then(pose),
        })
        .collect();

    let parts = parts_xml.map(import_parts).transpose()?.unwrap_or_default();
    let mut warnings = vec![
        "Imported geometry is a migration seed; run the complete VelocityPnP calibration workflow before motion".into(),
    ];
    if cameras.is_empty() {
        warnings.push("No OpenPnP camera definitions were found".into());
    }
    if axes.is_empty() {
        warnings.push("No OpenPnP axis definitions were found".into());
    }
    Ok(OpenPnpImport {
        source_format: "openpnp_xml".into(),
        axes,
        cameras,
        nozzles,
        feeders,
        parts,
        warnings,
    })
}

fn import_parts(source: &str) -> Result<Vec<PartSeed>> {
    let document = Document::parse(source).context("parsing OpenPnP parts.xml")?;
    Ok(elements(&document, "part")
        .filter_map(|node| {
            Some(PartSeed {
                id: node.attribute("id")?.to_string(),
                name: attribute(node, "name"),
                package_id: attribute(node, "package-id"),
                height_mm: child_value(node, "height"),
            })
        })
        .collect())
}

fn elements<'a>(document: &'a Document<'a>, name: &'a str) -> impl Iterator<Item = Node<'a, 'a>> {
    document
        .descendants()
        .filter(move |node| node.is_element() && node.tag_name().name() == name)
}

fn child<'a>(node: Node<'a, 'a>, name: &str) -> Option<Node<'a, 'a>> {
    node.children()
        .find(|child| child.is_element() && child.tag_name().name() == name)
}

fn child_value(node: Node<'_, '_>, name: &str) -> Option<f64> {
    child(node, name).and_then(|value| number(value, "value"))
}

fn number(node: Node<'_, '_>, attribute: &str) -> Option<f64> {
    node.attribute(attribute)?.parse().ok()
}

fn attribute(node: Node<'_, '_>, name: &str) -> Option<String> {
    node.attribute(name).map(str::to_string)
}

fn pose(node: Node<'_, '_>) -> Option<PoseSeed> {
    Some(PoseSeed {
        x_mm: number(node, "x")?,
        y_mm: number(node, "y")?,
        z_mm: number(node, "z")?,
        rotation_deg: number(node, "rotation").unwrap_or(0.0),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imports_machine_and_part_seeds_without_marking_them_calibrated() {
        let machine = r#"<openpnp-machine><machine><axes>
          <axis name="x" type="X" letter="X"><home-coordinate value="0"/><soft-limit-low value="0"/><soft-limit-high value="433"/></axis>
        </axes><heads><head><nozzles><nozzle name="N1"><head-offsets x="-23" y="-66" z="0" rotation="0"/></nozzle></nozzles></head></heads>
        <cameras><camera name="Top" looking="Down" unique-id="camera-id" flip-x="true"><units-per-pixel x="0.013" y="0.014"/><head-offsets x="0" y="0" z="0" rotation="0"/></camera></cameras>
        <feeders><feeder class="ReferenceStripFeeder" id="F1" name="R1" part-id="P1"><location x="10" y="20" z="3" rotation="90"/></feeder></feeders>
        </machine></openpnp-machine>"#;
        let parts = r#"<openpnp-parts><parts><part id="P1" name="10k" package-id="R0402"><height value="0.4"/></part></parts></openpnp-parts>"#;
        let imported = import_openpnp(machine, Some(parts)).unwrap();
        assert_eq!(imported.axes[0].soft_limit_high_mm, Some(433.0));
        assert_eq!(imported.cameras[0].unique_id.as_deref(), Some("camera-id"));
        assert_eq!(imported.nozzles[0].head_offsets_mm.unwrap().x_mm, -23.0);
        assert_eq!(imported.feeders[0].part_id.as_deref(), Some("P1"));
        assert_eq!(imported.parts[0].height_mm, Some(0.4));
        assert!(imported.warnings[0].contains("calibration"));
    }

    #[test]
    fn rejects_unrelated_xml() {
        assert!(import_openpnp("<not-openpnp/>", None).is_err());
    }
}
