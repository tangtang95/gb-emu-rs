
/// 16 bit memory
pub struct Memory {
    memory: [u8; 65536],
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            memory: [0u8; 65536],
        }
    }
}

impl Memory {
    pub fn read_u8(&self, address: u16) -> u8 {
        self.memory[usize::from(address)]
    }

    pub fn write_u8(&mut self, address: u16, value: u8) {
        self.memory[usize::from(address)] = value;
    }
}

