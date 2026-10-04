use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};

const CACHE_SCHEMA_VERSION: u32 = 2;
const PACKAGE_SCHEMA_VERSION: u32 = 1;
const REQUIRED_ROLES: [&str; 3] = ["held_part", "placement_verification", "source_component"];
const REQUIRED_PACKAGE_FILES: [&str; 5] = [
    "LICENSE",
    "best.onnx",
    "best.pt",
    "model-card.md",
    "model-manifest.json",
];

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ModelSelectionMode {
    Disabled,
    Explicit,
    Managed,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SelectedModelPackage {
    pub id: String,
    pub status: String,
    pub contract_version: String,
    pub archive_sha256: String,
    pub manifest_sha256: String,
    pub onnx_sha256: String,
    pub checkpoint_sha256: String,
    pub dataset_content_sha256: String,
    #[serde(skip)]
    package_dir: PathBuf,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ModelSelectionStatus {
    pub mode: ModelSelectionMode,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lock_sha256: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub catalog_commit: Option<String>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub models: BTreeMap<String, SelectedModelPackage>,
}

impl ModelSelectionStatus {
    pub fn disabled() -> Self {
        Self {
            mode: ModelSelectionMode::Disabled,
            lock_sha256: None,
            catalog_commit: None,
            models: BTreeMap::new(),
        }
    }

    pub fn explicit() -> Self {
        Self {
            mode: ModelSelectionMode::Explicit,
            lock_sha256: None,
            catalog_commit: None,
            models: BTreeMap::new(),
        }
    }

    pub fn package_dir(&self, role: &str) -> Option<&Path> {
        self.models
            .get(role)
            .map(|model| model.package_dir.as_path())
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CacheState {
    schema_version: u32,
    current_lock_sha256: Option<String>,
    history: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelLock {
    schema_version: u32,
    selection_policy: String,
    catalog: LockCatalog,
    models: Vec<LockModel>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct LockCatalog {
    repository: String,
    commit: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct LockModel {
    role: String,
    id: String,
    status: String,
    contract_version: String,
    package_root: String,
    release_tag: String,
    dataset_content_sha256: String,
    manifest_sha256: String,
    onnx_sha256: String,
    checkpoint_sha256: String,
    asset: LockAsset,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct LockAsset {
    name: String,
    size_bytes: u64,
    sha256: String,
    api_url: String,
    download_url: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PackageMetadata {
    schema_version: u32,
    id: String,
    family: String,
    version: String,
    release_tag: String,
    contract_version: String,
    license: serde_json::Value,
    manifest_sha256: String,
    files: Vec<PackageFile>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PackageFile {
    path: String,
    sha256: String,
    size_bytes: u64,
}

#[derive(Debug, Deserialize)]
struct ModelManifest {
    contract_version: String,
    dataset: ManifestDataset,
    artifacts: ManifestArtifacts,
}

#[derive(Debug, Deserialize)]
struct ManifestDataset {
    content_sha256: String,
}

#[derive(Debug, Deserialize)]
struct ManifestArtifacts {
    checkpoint: ManifestArtifact,
    onnx: ManifestArtifact,
}

#[derive(Debug, Deserialize)]
struct ManifestArtifact {
    path: String,
    sha256: String,
    size_bytes: u64,
}

pub fn load_active_model_selection(cache_root: &Path) -> Result<Option<ModelSelectionStatus>> {
    let state_path = cache_root.join("state.json");
    if !state_path.is_file() {
        return Ok(None);
    }
    let state: CacheState = read_json(&state_path)?;
    ensure!(
        state.schema_version == CACHE_SCHEMA_VERSION,
        "model cache state schema differs"
    );
    for digest in state.history.iter().chain(&state.current_lock_sha256) {
        validate_sha256(digest, "model cache lock digest")?;
    }
    let Some(lock_sha256) = state.current_lock_sha256 else {
        return Ok(None);
    };

    let lock_path = cache_root.join("locks").join(format!("{lock_sha256}.json"));
    let lock_bytes = fs::read(&lock_path)
        .with_context(|| format!("cannot read selected model lock {}", lock_path.display()))?;
    ensure!(
        sha256_bytes(&lock_bytes) == lock_sha256,
        "selected model lock checksum differs"
    );
    let lock: ModelLock = serde_json::from_slice(&lock_bytes)
        .with_context(|| format!("cannot decode selected model lock {}", lock_path.display()))?;
    validate_lock(&lock)?;

    let canonical_cache = cache_root
        .canonicalize()
        .with_context(|| format!("cannot resolve model cache {}", cache_root.display()))?;
    let mut models = BTreeMap::new();
    for model in &lock.models {
        let package = canonical_cache
            .join("packages")
            .join(&model.asset.sha256)
            .join(&model.package_root);
        let package = verify_cached_package(&canonical_cache, &package, model)?;
        let selected = SelectedModelPackage {
            id: model.id.clone(),
            status: model.status.clone(),
            contract_version: model.contract_version.clone(),
            archive_sha256: model.asset.sha256.clone(),
            manifest_sha256: model.manifest_sha256.clone(),
            onnx_sha256: model.onnx_sha256.clone(),
            checkpoint_sha256: model.checkpoint_sha256.clone(),
            dataset_content_sha256: model.dataset_content_sha256.clone(),
            package_dir: package,
        };
        ensure!(
            models.insert(model.role.clone(), selected).is_none(),
            "duplicate model role: {}",
            model.role
        );
    }

    Ok(Some(ModelSelectionStatus {
        mode: ModelSelectionMode::Managed,
        lock_sha256: Some(lock_sha256),
        catalog_commit: Some(lock.catalog.commit),
        models,
    }))
}

fn validate_lock(lock: &ModelLock) -> Result<()> {
    ensure!(
        lock.schema_version == CACHE_SCHEMA_VERSION,
        "selected model lock schema differs"
    );
    ensure!(
        lock.selection_policy == "runtime",
        "selected model lock does not permit runtime use"
    );
    ensure!(
        !lock.catalog.repository.is_empty(),
        "selected model catalog repository is empty"
    );
    validate_git_commit(&lock.catalog.commit, "selected model catalog commit")?;
    ensure!(
        lock.models.len() == REQUIRED_ROLES.len(),
        "selected model roles differ"
    );

    let mut roles = BTreeSet::new();
    let mut ids = BTreeSet::new();
    for model in &lock.models {
        ensure!(model.status == "approved", "{} is not approved", model.id);
        ensure!(
            REQUIRED_ROLES.contains(&model.role.as_str()),
            "unknown model role: {}",
            model.role
        );
        ensure!(roles.insert(model.role.as_str()), "duplicate model role");
        ensure!(ids.insert(model.id.as_str()), "duplicate model ID");
        ensure!(
            !model.contract_version.is_empty(),
            "{} has no contract version",
            model.id
        );
        let (family, version) = model
            .id
            .split_once('/')
            .context("model ID must be family/version")?;
        ensure!(
            !family.is_empty() && !version.is_empty(),
            "model ID must be family/version"
        );
        ensure!(
            model.package_root == format!("{family}-{version}"),
            "{} package root differs",
            model.id
        );
        validate_file_name(&model.package_root, "model package root")?;
        ensure!(
            !model.release_tag.is_empty(),
            "{} release tag is empty",
            model.id
        );
        validate_sha256(&model.dataset_content_sha256, "dataset checksum")?;
        validate_sha256(&model.manifest_sha256, "manifest checksum")?;
        validate_sha256(&model.onnx_sha256, "ONNX checksum")?;
        validate_sha256(&model.checkpoint_sha256, "checkpoint checksum")?;
        validate_sha256(&model.asset.sha256, "archive checksum")?;
        ensure!(model.asset.size_bytes > 0, "{} archive is empty", model.id);
        ensure!(
            model.asset.name == format!("{}.tar.gz", model.package_root),
            "{} archive name differs",
            model.id
        );
        ensure!(
            model.asset.api_url.starts_with("https://api.github.com/"),
            "{} asset API URL is invalid",
            model.id
        );
        ensure!(
            model.asset.download_url.starts_with("https://github.com/"),
            "{} download URL is invalid",
            model.id
        );
    }
    ensure!(
        roles == REQUIRED_ROLES.into_iter().collect(),
        "selected model roles differ"
    );
    Ok(())
}

fn verify_cached_package(cache_root: &Path, package: &Path, model: &LockModel) -> Result<PathBuf> {
    let metadata = fs::symlink_metadata(package)
        .with_context(|| format!("cached model package is absent: {}", package.display()))?;
    ensure!(
        metadata.is_dir() && !metadata.file_type().is_symlink(),
        "cached model package is not a safe directory"
    );
    let package = package
        .canonicalize()
        .with_context(|| format!("cannot resolve cached model package {}", package.display()))?;
    ensure!(
        package.starts_with(cache_root),
        "cached model package resolves outside the cache"
    );

    let metadata_path = package.join("package-metadata.json");
    let package_metadata: PackageMetadata = read_json(&metadata_path)?;
    ensure!(
        package_metadata.schema_version == PACKAGE_SCHEMA_VERSION,
        "unsupported package metadata schema"
    );
    ensure!(package_metadata.id == model.id, "package model ID differs");
    ensure!(
        package_metadata.contract_version == model.contract_version,
        "package model contract differs"
    );
    ensure!(
        package_metadata.manifest_sha256 == model.manifest_sha256,
        "package manifest checksum differs"
    );
    ensure!(
        package_metadata.release_tag == model.release_tag,
        "package release tag differs"
    );
    ensure!(
        model.id == format!("{}/{}", package_metadata.family, package_metadata.version),
        "package family or version differs"
    );
    ensure!(
        package_metadata.license.is_object(),
        "package license metadata is invalid"
    );

    let actual_files = package_files(&package)?;
    let mut inventory = BTreeMap::new();
    for file in &package_metadata.files {
        validate_file_name(&file.path, "package file")?;
        validate_sha256(&file.sha256, "package file checksum")?;
        ensure!(
            inventory.insert(file.path.as_str(), file).is_none(),
            "duplicate package file: {}",
            file.path
        );
    }
    let expected_files: BTreeSet<String> =
        inventory.keys().map(|name| (*name).to_owned()).collect();
    ensure!(
        actual_files == expected_files,
        "cached model package file inventory differs"
    );
    for required in REQUIRED_PACKAGE_FILES {
        ensure!(
            expected_files.contains(required),
            "cached model package is missing {required}"
        );
    }
    for file in package_metadata.files {
        let path = package.join(&file.path);
        let bytes = fs::read(&path)
            .with_context(|| format!("cannot read cached package file {}", path.display()))?;
        ensure!(
            u64::try_from(bytes.len())? == file.size_bytes,
            "cached package file size differs: {}",
            file.path
        );
        ensure!(
            sha256_bytes(&bytes) == file.sha256,
            "cached package file checksum differs: {}",
            file.path
        );
    }

    let manifest_path = package.join("model-manifest.json");
    let manifest_bytes = fs::read(&manifest_path)
        .with_context(|| format!("cannot read {}", manifest_path.display()))?;
    ensure!(
        sha256_bytes(&manifest_bytes) == model.manifest_sha256,
        "cached model manifest checksum differs"
    );
    let manifest: ModelManifest = serde_json::from_slice(&manifest_bytes)
        .with_context(|| format!("cannot decode {}", manifest_path.display()))?;
    ensure!(
        manifest.contract_version == model.contract_version,
        "cached model manifest contract differs"
    );
    ensure!(
        manifest.dataset.content_sha256 == model.dataset_content_sha256,
        "cached model dataset checksum differs"
    );
    verify_manifest_artifact(
        &package,
        &manifest.artifacts.onnx,
        &model.onnx_sha256,
        "ONNX",
    )?;
    verify_manifest_artifact(
        &package,
        &manifest.artifacts.checkpoint,
        &model.checkpoint_sha256,
        "checkpoint",
    )?;
    Ok(package)
}

fn package_files(package: &Path) -> Result<BTreeSet<String>> {
    let mut names = BTreeSet::new();
    for entry in fs::read_dir(package)
        .with_context(|| format!("cannot list cached package {}", package.display()))?
    {
        let entry = entry?;
        let file_type = entry.file_type()?;
        ensure!(
            file_type.is_file() && !file_type.is_symlink(),
            "cached model package contains a non-file"
        );
        let name = entry.file_name();
        let name = name
            .to_str()
            .context("cached package file name is not UTF-8")?;
        if name != "package-metadata.json" {
            names.insert(name.to_owned());
        }
    }
    Ok(names)
}

fn verify_manifest_artifact(
    package: &Path,
    artifact: &ManifestArtifact,
    locked_sha256: &str,
    kind: &str,
) -> Result<()> {
    validate_file_name(&artifact.path, "model artifact")?;
    validate_sha256(&artifact.sha256, "model artifact checksum")?;
    ensure!(
        artifact.sha256 == locked_sha256,
        "cached model {kind} checksum differs from the lock"
    );
    let path = package.join(&artifact.path);
    let bytes = fs::read(&path)
        .with_context(|| format!("cannot read cached model artifact {}", path.display()))?;
    ensure!(
        u64::try_from(bytes.len())? == artifact.size_bytes,
        "cached model {kind} size differs"
    );
    ensure!(
        sha256_bytes(&bytes) == artifact.sha256,
        "cached model {kind} checksum differs"
    );
    Ok(())
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T> {
    let bytes = fs::read(path).with_context(|| format!("cannot read {}", path.display()))?;
    serde_json::from_slice(&bytes).with_context(|| format!("cannot decode {}", path.display()))
}

fn validate_file_name(value: &str, context: &str) -> Result<()> {
    ensure!(
        Path::new(value)
            .components()
            .all(|component| matches!(component, Component::Normal(_)))
            && Path::new(value).components().count() == 1,
        "{context} must be one safe file name"
    );
    Ok(())
}

fn validate_sha256(value: &str, context: &str) -> Result<()> {
    ensure!(
        value.len() == 64
            && value
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        "{context} is invalid"
    );
    Ok(())
}

fn validate_git_commit(value: &str, context: &str) -> Result<()> {
    ensure!(
        value.len() == 40
            && value
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        "{context} is invalid"
    );
    Ok(())
}

fn sha256_bytes(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use tempfile::TempDir;

    struct CacheFixture {
        root: TempDir,
        lock_sha256: String,
        onnx_sha256: String,
    }

    impl CacheFixture {
        fn new(policy: &str, status: &str) -> Self {
            let root = TempDir::new().unwrap();
            let mut models = Vec::new();
            let mut first_onnx_sha256 = String::new();
            for role in REQUIRED_ROLES {
                let family = role.replace('_', "-");
                let id = format!("{family}/1");
                let package_root = format!("{family}-1");
                let checkpoint = format!("checkpoint-{role}").into_bytes();
                let onnx = format!("onnx-{role}").into_bytes();
                let checkpoint_sha256 = sha256_bytes(&checkpoint);
                let onnx_sha256 = sha256_bytes(&onnx);
                if first_onnx_sha256.is_empty() {
                    first_onnx_sha256.clone_from(&onnx_sha256);
                }
                let dataset_sha256 = sha256_bytes(format!("dataset-{role}").as_bytes());
                let manifest = json!({
                    "contract_version": format!("velocitypnp.{family}.v1"),
                    "dataset": {"content_sha256": dataset_sha256},
                    "artifacts": {
                        "checkpoint": {
                            "path": "best.pt",
                            "sha256": checkpoint_sha256,
                            "size_bytes": checkpoint.len(),
                        },
                        "onnx": {
                            "path": "best.onnx",
                            "sha256": onnx_sha256,
                            "size_bytes": onnx.len(),
                        },
                    },
                });
                let manifest = serde_json::to_vec(&manifest).unwrap();
                let manifest_sha256 = sha256_bytes(&manifest);
                let files = BTreeMap::from([
                    ("LICENSE", b"license".to_vec()),
                    ("best.onnx", onnx),
                    ("best.pt", checkpoint),
                    ("model-card.md", b"card".to_vec()),
                    ("model-manifest.json", manifest),
                ]);
                let archive_sha256 = sha256_bytes(format!("archive-{role}").as_bytes());
                let package = root
                    .path()
                    .join("packages")
                    .join(&archive_sha256)
                    .join(&package_root);
                fs::create_dir_all(&package).unwrap();
                for (name, bytes) in &files {
                    fs::write(package.join(name), bytes).unwrap();
                }
                let inventory: Vec<_> = files
                    .iter()
                    .map(|(name, bytes)| {
                        json!({
                            "path": name,
                            "sha256": sha256_bytes(bytes),
                            "size_bytes": bytes.len(),
                        })
                    })
                    .collect();
                fs::write(
                    package.join("package-metadata.json"),
                    serde_json::to_vec(&json!({
                        "schema_version": 1,
                        "id": id,
                        "family": family,
                        "version": "1",
                        "release_tag": "models-1",
                        "contract_version": format!("velocitypnp.{family}.v1"),
                        "license": {"artifact": "Apache-2.0"},
                        "manifest_sha256": manifest_sha256,
                        "files": inventory,
                    }))
                    .unwrap(),
                )
                .unwrap();
                models.push(json!({
                    "role": role,
                    "id": format!("{family}/1"),
                    "status": status,
                    "contract_version": format!("velocitypnp.{family}.v1"),
                    "package_root": package_root,
                    "release_tag": "models-1",
                    "dataset_content_sha256": dataset_sha256,
                    "manifest_sha256": manifest_sha256,
                    "onnx_sha256": onnx_sha256,
                    "checkpoint_sha256": checkpoint_sha256,
                    "asset": {
                        "name": format!("{family}-1.tar.gz"),
                        "size_bytes": 1,
                        "sha256": archive_sha256,
                        "api_url": "https://api.github.com/repos/test/models/releases/assets/1",
                        "download_url": format!(
                            "https://github.com/test/models/releases/download/models-1/{family}-1.tar.gz"
                        ),
                    },
                }));
            }
            let lock = serde_json::to_vec(&json!({
                "schema_version": 2,
                "selection_policy": policy,
                "catalog": {
                    "repository": "test/models",
                    "commit": "1".repeat(40),
                },
                "models": models,
            }))
            .unwrap();
            let lock_sha256 = sha256_bytes(&lock);
            fs::create_dir_all(root.path().join("locks")).unwrap();
            fs::write(
                root.path()
                    .join("locks")
                    .join(format!("{lock_sha256}.json")),
                lock,
            )
            .unwrap();
            fs::write(
                root.path().join("state.json"),
                serde_json::to_vec(&json!({
                    "schema_version": 2,
                    "current_lock_sha256": lock_sha256,
                    "history": [],
                }))
                .unwrap(),
            )
            .unwrap();
            Self {
                root,
                lock_sha256,
                onnx_sha256: first_onnx_sha256,
            }
        }
    }

    #[test]
    fn absent_state_has_no_active_selection() {
        let root = TempDir::new().unwrap();
        assert!(load_active_model_selection(root.path()).unwrap().is_none());
    }

    #[test]
    fn approved_runtime_selection_verifies_every_package() {
        let fixture = CacheFixture::new("runtime", "approved");
        let selection = load_active_model_selection(fixture.root.path())
            .unwrap()
            .unwrap();

        assert_eq!(selection.mode, ModelSelectionMode::Managed);
        assert_eq!(
            selection.lock_sha256.as_deref(),
            Some(fixture.lock_sha256.as_str())
        );
        assert_eq!(selection.models.len(), 3);
        assert_eq!(
            selection.models["held_part"].onnx_sha256,
            fixture.onnx_sha256
        );
        assert!(selection.package_dir("source_component").unwrap().is_dir());
    }

    #[test]
    fn cache_only_or_unapproved_selection_is_rejected() {
        let cache_only = CacheFixture::new("cache_only", "approved");
        assert!(load_active_model_selection(cache_only.root.path())
            .unwrap_err()
            .to_string()
            .contains("does not permit runtime use"));

        let candidate = CacheFixture::new("runtime", "candidate");
        assert!(load_active_model_selection(candidate.root.path())
            .unwrap_err()
            .to_string()
            .contains("is not approved"));
    }

    #[test]
    fn changed_cached_payload_is_rejected() {
        let fixture = CacheFixture::new("runtime", "approved");
        let selection = load_active_model_selection(fixture.root.path())
            .unwrap()
            .unwrap();
        let package = selection.package_dir("held_part").unwrap();
        fs::write(package.join("best.onnx"), b"changed").unwrap();

        assert!(load_active_model_selection(fixture.root.path())
            .unwrap_err()
            .to_string()
            .contains("size differs"));
    }
}
