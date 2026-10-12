//! Embeds the application icon and version information into the Windows executable.

fn main() {
    println!("cargo:rerun-if-changed=../../assets/icon.ico");
    println!("cargo:rerun-if-changed=build.rs");
    #[cfg(windows)]
    {
        if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
            let mut res = winresource::WindowsResource::new();
            res.set_icon("../../assets/icon.ico");
            res.set("ProductName", "OpenSlicer");
            res.set("FileDescription", "OpenSlicer: split large 3D models into printable parts");
            res.set("LegalCopyright", "MIT OR Apache-2.0");
            if let Err(e) = res.compile() {
                // A missing resource compiler must not break the build; the exe just keeps the default icon.
                println!("cargo:warning=could not embed the Windows icon: {e}");
            }
        }
    }
}
