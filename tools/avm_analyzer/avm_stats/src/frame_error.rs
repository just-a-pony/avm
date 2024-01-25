use thiserror::Error;

#[derive(Error, Debug, Clone)]
pub enum FrameError {
    #[error("Badly formed frame: {0}")]
    BadFrame(String),
    #[error("Badly formed superblock: {0}")]
    BadSuperblock(String),
    #[error("Badly formed coding unit: {0}")]
    BadCodingUnit(String),
    #[error("Badly formed transform unit: {0}")]
    BadTransformUnit(String),
    #[error("Badly formed symbol: {0}")]
    BadSymbol(String),
    #[error("Badly formed pixel buffer: {0}")]
    BadPixelBuffer(String),
    #[error("Missing pixel buffer: {0}")]
    MissingPixelBuffer(String),
    #[error("Internal error: {0}")]
    Internal(String),
    #[error("Unknown frame error: {0}")]
    Unknown(String),
}
