#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockDeviceError {
    /// Error during writing; most likely value read back after write was wrong
    WriteError,

    /// Address is invalid or out of range
    InvalidAddress,
}

// TODO: async these
pub trait BlockDevice {
    /// The number of bytes per block. This determines the size of the buffer passed
    /// to read/write functions
    const BLOCK_BYTES: usize;

    /// Read the block indicated by `lba` into the provided buffer
    fn read_block(&mut self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError>;

    /// Write the `block` buffer to the block indicated by `lba`
    fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError>;

    /// Get the maxium valid lba (logical block address)
    fn max_lba(&self) -> u32;
}
