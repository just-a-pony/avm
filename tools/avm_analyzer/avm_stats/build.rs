use std::io::Result;

fn main() -> Result<()> {
    prost_build::Config::new()
        .type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]")
        .compile_protos(&["../../extract_proto/avm_frame.proto"], &["../../extract_proto"])?;
    Ok(())
}
