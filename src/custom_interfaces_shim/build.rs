use std::path::Path;

fn main() {
    let dependencies: &[&str] = &["std_msgs", "custom_interfaces"];

    safe_drive_msg::depends(
        &Path::new("../custom_interfaces/bindings")
            .canonicalize()
            .expect("Failed to canonicalize path (useful to prevent other build errors)"),
        dependencies,
        safe_drive_msg::SafeDrive::Version("0.4.3"),
    )
    .expect("Failed to build `custom_interfaces` bindings through `safe_drive_msg`!");
}
