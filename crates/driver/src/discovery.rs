use anyhow::Result;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::{DeviceIdentity, DeviceKind};

pub fn discover_linux_devices() -> Result<Vec<DeviceIdentity>> {
    let mut devices = Vec::new();
    let serial_links = symlink_map(Path::new("/dev/serial/by-id"));
    let video_links = symlink_map(Path::new("/dev/v4l/by-id"));

    for entry in glob_class("/sys/class/tty", "ttyACM")? {
        let name = file_name(&entry);
        let device_path = format!("/dev/{name}");
        let usb = usb_properties(&entry.join("device"));
        devices.push(DeviceIdentity {
            kind: DeviceKind::SerialController,
            stable_path: serial_links
                .get(&canonical_string(Path::new(&device_path)))
                .cloned(),
            path: device_path,
            vendor_id: usb.get("idVendor").cloned(),
            product_id: usb.get("idProduct").cloned(),
            serial: usb.get("serial").cloned(),
            product: usb.get("product").cloned(),
            capture_capable: None,
            suggested_role: Some("machine_controller".into()),
        });
    }

    for entry in glob_class("/sys/class/video4linux", "video")? {
        let name = file_name(&entry);
        let device_path = format!("/dev/{name}");
        let product = read_trimmed(&entry.join("name"));
        let usb = usb_properties(&entry.join("device"));
        let lower = product.as_deref().unwrap_or_default().to_ascii_lowercase();
        let suggested_role = if lower.contains("top") {
            Some("top_camera".into())
        } else if lower.contains("bottom") {
            Some("bottom_camera".into())
        } else {
            None
        };
        // UVC devices commonly expose a capture node (index 0) and a metadata
        // node (index 1). Avoid offering metadata nodes as cameras.
        let capture_capable = read_trimmed(&entry.join("index")).map(|index| index == "0");
        devices.push(DeviceIdentity {
            kind: DeviceKind::Camera,
            stable_path: video_links
                .get(&canonical_string(Path::new(&device_path)))
                .cloned(),
            path: device_path,
            vendor_id: usb.get("idVendor").cloned(),
            product_id: usb.get("idProduct").cloned(),
            serial: usb.get("serial").cloned(),
            product: product.or_else(|| usb.get("product").cloned()),
            capture_capable,
            suggested_role,
        });
    }
    devices.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(devices)
}

fn glob_class(root: &str, prefix: &str) -> Result<Vec<PathBuf>> {
    let root = Path::new(root);
    if !root.exists() {
        return Ok(Vec::new());
    }
    let mut result = fs::read_dir(root)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| file_name(path).starts_with(prefix))
        .collect::<Vec<_>>();
    result.sort();
    Ok(result)
}

fn symlink_map(root: &Path) -> HashMap<String, String> {
    let mut result = HashMap::new();
    let Ok(entries) = fs::read_dir(root) else {
        return result;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        result.insert(canonical_string(&path), path.to_string_lossy().into_owned());
    }
    result
}

fn canonical_string(path: &Path) -> String {
    fs::canonicalize(path)
        .unwrap_or_else(|_| path.to_path_buf())
        .to_string_lossy()
        .into_owned()
}

fn usb_properties(start: &Path) -> HashMap<String, String> {
    let Ok(mut current) = fs::canonicalize(start) else {
        return HashMap::new();
    };
    loop {
        if current.join("idVendor").exists() {
            return ["idVendor", "idProduct", "serial", "product", "manufacturer"]
                .into_iter()
                .filter_map(|key| read_trimmed(&current.join(key)).map(|value| (key.into(), value)))
                .collect();
        }
        if !current.pop() {
            return HashMap::new();
        }
    }
}

fn read_trimmed(path: &Path) -> Option<String> {
    fs::read_to_string(path)
        .ok()
        .map(|value| value.trim().to_string())
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_class_directory_is_empty() {
        assert!(glob_class("/definitely/not/a/real/sysfs/path", "video")
            .unwrap()
            .is_empty());
    }
}
