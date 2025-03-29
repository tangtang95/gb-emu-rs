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
    memory: Box<[u8; 65536]>,
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            hw_mode: HardwareMode::DMG,
            memory: Box::new([0u8; 65536]),
        }
    }
}

impl Memory {
    /// Based on table: https://gbdev.io/pandocs/Power_Up_Sequence.html#hardware-registers
    pub fn reset_to_after_boot_rom(&mut self) {
        match self.hw_mode {
            HardwareMode::DMG => {
                self.memory[0xFF00] = 0xCF; // P1
                self.memory[0xFF01] = 0x00; // SB
                self.memory[0xFF02] = 0x7E; // SC
                self.memory[0xFF04] = 0xAB; // DIV
                self.memory[0xFF05] = 0x00; // TIMA
                self.memory[0xFF06] = 0x00; // TMA
                self.memory[0xFF07] = 0xF8; // TAC
                self.memory[0xFF0F] = 0xE1; // IF
                self.memory[0xFF10] = 0x80; // NR10
                self.memory[0xFF11] = 0xBF; // NR11
                self.memory[0xFF12] = 0xF3; // NR12
                self.memory[0xFF14] = 0xBF; // NR14
                self.memory[0xFF16] = 0x3F; // NR21
                self.memory[0xFF17] = 0x00; // NR22
                self.memory[0xFF18] = 0xFF; // NR23
                self.memory[0xFF19] = 0xBF; // NR24
                self.memory[0xFF1A] = 0x7F; // NR30
                self.memory[0xFF1B] = 0xFF; // NR31
                self.memory[0xFF1C] = 0x9F; // NR32
                self.memory[0xFF1D] = 0xFF; // NR33
                self.memory[0xFF1E] = 0xBF; // NR34
                self.memory[0xFF20] = 0xFF; // NR41
                self.memory[0xFF21] = 0x00; // NR42
                self.memory[0xFF22] = 0x00; // NR43
                self.memory[0xFF23] = 0xBF; // NR44
                self.memory[0xFF24] = 0x77; // NR50
                self.memory[0xFF25] = 0xF3; // NR51
                self.memory[0xFF26] = 0xF1; // NR52 (DMG)
                self.memory[0xFF40] = 0x91; // LCDC
                self.memory[0xFF41] = 0x85; // STAT
                self.memory[0xFF42] = 0x00; // SCY
                self.memory[0xFF43] = 0x00; // SCX
                self.memory[0xFF45] = 0x00; // LYC
                self.memory[0xFF46] = 0xFF; // DMA
                self.memory[0xFF47] = 0xFC; // BGP
                self.memory[0xFF48] = 0xFF; // OBP0??
                self.memory[0xFF49] = 0xFF; // OBP1??
                self.memory[0xFF4A] = 0x00; // WY
                self.memory[0xFF4B] = 0x00; // WX
                self.memory[0xFFFF] = 0x00; // IE
            }
        }
    }

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

    pub fn load_rom(&mut self, rom_bytes: &[u8]) -> Result<(), MemoryError> {
        match self.hw_mode {
            HardwareMode::DMG => {
                self.memory[0x100..(rom_bytes.len()+0x100)].copy_from_slice(rom_bytes);
                Ok(())
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
