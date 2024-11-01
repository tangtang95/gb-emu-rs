use thiserror::Error;

#[derive(Error, Debug)]
pub enum CpuError {
    #[error("Decode require other bytes ({bytes_len})")]
    DecodeRequireBytes { bytes_len: u8 },
    #[error("Invalid register id: {id}")]
    InvalidRegisterId { id: u8 }
}
