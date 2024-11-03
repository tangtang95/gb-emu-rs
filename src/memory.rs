use thiserror::Error;

use crate::hardware::HardwareMode;

enum MemoryType {
    /// Non-switchable ROM bank
    Rom0,
    /// Switchable ROM bank
    Rom1,
    /// Video RAM (switchable in GBC mode)
    VRam,
    /// External RAM in cartridge
    SRam,
    /// Work RAM
    WRam0,
    /// Work RAM (switchable in GBC mode)
    WRam1,
    /// Mirror of C000h - DDFFh
    Echo,
    /// Object Attribute Memory: Sprite information table
    Oam,
    /// Not Usable
    Unused,
    /// I/O registers
    IO,
    /// High RAM (Internal CPU RAM)
    HRam,
    /// Interrupt enable flags
    IE,
}

impl From<u16> for MemoryType {
    fn from(value: u16) -> Self {
        match value {
            0x0000..=0x3FFF => MemoryType::Rom0,
            0x4000..=0x7FFF => MemoryType::Rom1,
            0x8000..=0x9FFF => MemoryType::VRam,
            0xA000..=0xBFFF => MemoryType::SRam,
            0xC000..=0xCFFF => MemoryType::WRam0,
            0xD000..=0xDFFF => MemoryType::WRam1,
            0xE000..=0xFDFF => MemoryType::Echo,
            0xFE00..=0xFE9F => MemoryType::Oam,
            0xFEA0..=0xFEFF => MemoryType::Unused,
            0xFF00..=0xFF7F => MemoryType::IO,
            0xFF80..=0xFFFE => MemoryType::HRam,
            0xFFFF => MemoryType::IE,
        }
    }
}

#[derive(Error, Debug)]
pub enum MemoryError {
    #[error("Invalid boot ROM bytes length: {0} bytes")]
    InvalidBootBytesLen(usize),
}

/// 16 bit memory
pub struct Memory {
    hw_mode: HardwareMode,
    memory: [u8; 65536],
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            hw_mode: HardwareMode::DMG,
            memory: [0u8; 65536],
        }
    }
}

impl Memory {
    pub fn load_boot_rom(&mut self, boot_rom_bytes: &[u8]) -> Result<(), MemoryError> {
        match self.hw_mode {
            HardwareMode::DMG => {
                if boot_rom_bytes.len() != 0xFF {
                    Err(MemoryError::InvalidBootBytesLen(boot_rom_bytes.len()))
                } else {
                    self.memory[0..=0xFF].copy_from_slice(boot_rom_bytes);
                    Ok(())
                }
            }
        }
    }

    /// Read used by CPU
    pub fn read_u8(&self, address: u16) -> u8 {
        match MemoryType::from(address) {
            MemoryType::Rom0
            | MemoryType::Rom1
            | MemoryType::VRam
            | MemoryType::SRam
            | MemoryType::WRam0
            | MemoryType::WRam1
            | MemoryType::Oam
            // TODO: unreadable IO bits
            | MemoryType::IO
            | MemoryType::HRam
            // TODO: unreadable IE register bits
            | MemoryType::IE => self.memory[usize::from(address)],
            MemoryType::Echo => {
                assert!((0xE000..=0xFDFF).contains(&address));
                match self.hw_mode {
                    // DMG Normal cartridge behavior
                    HardwareMode::DMG => self.memory[usize::from(address - 0x2000)],
                }
            }
            MemoryType::Unused => {
                assert!((0xFEA0..=0xFEFF).contains(&address));
                match self.hw_mode {
                    HardwareMode::DMG => 0x00,
                }
            }
        }
    }

    /// Write used by CPU
    pub fn write_u8(&mut self, address: u16, value: u8) {
        match MemoryType::from(address) {
            MemoryType::Rom0
            | MemoryType::Rom1
            | MemoryType::VRam
            | MemoryType::SRam
            | MemoryType::WRam0
            | MemoryType::WRam1
            | MemoryType::Oam
            | MemoryType::IO
            | MemoryType::HRam
            | MemoryType::IE => self.memory[usize::from(address)] = value,
            MemoryType::Echo => {
                assert!((0xE000..=0xFDFF).contains(&address));
                match self.hw_mode {
                    // DMG Normal cartridge behavior
                    HardwareMode::DMG => self.memory[usize::from(address - 0x2000)] = value,
                }
            }
            MemoryType::Unused => {
                assert!((0xFEA0..=0xFEFF).contains(&address));
                match self.hw_mode {
                    // DMG: Writes ignored
                    HardwareMode::DMG => {}
                }
            }
        }
    }
}
