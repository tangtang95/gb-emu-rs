mod error;

use crate::cpu::error::CpuError;

enum ValueOrReg8 {
    Reg8(Register8),
    Value(u8),
}

enum Register8or16 {
    Reg8(Register8),
    Reg16(Register16),
}

enum Register8 {
    B,
    C,
    D,
    E,
    H,
    L,
    HlAddress,
    A,
}

impl TryFrom<u8> for Register8 {
    type Error = CpuError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::B),
            1 => Ok(Self::C),
            2 => Ok(Self::D),
            3 => Ok(Self::E),
            4 => Ok(Self::H),
            5 => Ok(Self::L),
            6 => Ok(Self::HlAddress),
            7 => Ok(Self::A),
            r => Err(CpuError::InvalidRegisterId { id: r }),
        }
    }
}

enum Register16 {
    BC,
    DE,
    HL,
    SP,
}

impl TryFrom<u8> for Register16 {
    type Error = CpuError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::BC),
            1 => Ok(Self::DE),
            2 => Ok(Self::HL),
            3 => Ok(Self::SP),
            r => Err(CpuError::InvalidRegisterId { id: r }),
        }
    }
}

enum RegisterStack {
    BC,
    DE,
    HL,
    AF,
}

enum RegisterMemory {
    BC,
    DE,
    HLPlus,
    HLMinus,
}

impl TryFrom<u8> for RegisterMemory {
    type Error = CpuError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::BC),
            1 => Ok(Self::DE),
            2 => Ok(Self::HLPlus),
            3 => Ok(Self::HLMinus),
            r => Err(CpuError::InvalidRegisterId { id: r }),
        }
    }
}

enum RegisterCond {
    NZ,
    Z,
    NC,
    C,
}

impl TryFrom<u8> for RegisterCond {
    type Error = CpuError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::NZ),
            1 => Ok(Self::Z),
            2 => Ok(Self::NC),
            3 => Ok(Self::C),
            r => Err(CpuError::InvalidRegisterId { id: r }),
        }
    }
}

enum CarryOption {
    With,
    Without,
}

/// Operation code organized based on https://rgbds.gbdev.io/docs/v0.8.0/gbz80.7
/// https://gbdev.io/pandocs/CPU_Instruction_Set.html
enum Opcode {
    // --- Arithmetic and logic ---
    /// ADD a, imm8 and ADD a, r8 and ADC a, r8 and ADC a, imm8
    AddRegA(ValueOrReg8, CarryOption),
    /// SUB a, imm8 and SUB a, r8 and ADC a, r8 and ADC a, imm8
    SubtractRegA(ValueOrReg8, CarryOption),
    /// AND a, imm8 and AND a, r8
    BitwiseAndRegA(ValueOrReg8),
    /// XOR a, imm8 and XOR a, r8
    BitwiseXorRegA(ValueOrReg8),
    /// OR a, imm8 and OR a, r8
    BitwiseOrRegA(ValueOrReg8),
    /// CP a, imm8 and CP a, r8
    ComparingRegA(ValueOrReg8),
    /// ADD hl, r16
    AddToHl(Register16),
    /// INC r8 and INC r16
    Increment(Register8or16),
    /// DEC r8 and DEC r16
    Decrement(Register8or16),

    // --- Bit operations ---
    /// BIT b3,r8
    BitTest { operand: Register8, bit_idx: u8 },
    /// RES b3,r8
    BitReset { operand: Register8, bit_idx: u8 },
    /// SET b3, r8
    BitSet { operand: Register8, bit_idx: u8 },
    /// SWAP r8
    BitSwap(Register8),

    // --- Bit shift operations ---
    /// RL r8 and RLC r8 and RLCA and RLA
    BitRotateLeft(Register8, CarryOption),
    /// RR r8 and RRC r8 and RRCA and RRA
    BitRotateRight(Register8, CarryOption),
    /// SLA r8
    BitShiftLeft(Register8),
    /// SRA r8
    BitShiftRight(Register8),
    /// SRL r8
    BitShiftLogicRight(Register8),

    // --- Load instructions ---
    /// LD r8, imm8 and LD r8, r8
    Load8 { src: ValueOrReg8, dest: Register8 },
    /// LD r16, imm16
    Load16 { src: u16, dest: Register16 },
    /// LD \[r16mem\], a
    LoadMemFromA { dest_addr: RegisterMemory },
    /// LD a, \[r16mem\]
    LoadAFromMem { src_addr: RegisterMemory },
    /// LDH \[c\], a
    LoadHighAToAddrC,
    /// LDH a, \[c\]
    LoadHighAddrCToA,
    /// LDH \[imm8\], a
    LoadHighFromA { src_addr: u8 },
    /// LDH a, \[imm8\]
    LoadHighToA { dest_addr: u8 },
    /// LD \[imm16\], a
    LoadFromA { dest_addr: u16 },
    /// LD a, \[imm16\]
    LoadToA { src_addr: u16 },

    // --- Jumps and subroutines ---
    /// JR imm8 and JR cond, imm8
    JumpRelative {
        offset: u8,
        cond: Option<RegisterCond>,
    },
    /// JP imm16 and JP cond, imm16
    Jump {
        addr: u16,
        cond: Option<RegisterCond>,
    },
    /// JP hl
    JumpHl,
    /// RET and RET cond
    Return(Option<RegisterCond>),
    /// RETI
    ReturnInterrupts,
    /// CALL imm16 and CALL cond, imm16
    Call {
        addr: u16,
        cond: Option<RegisterCond>,
    },
    /// RST tgt3
    RstCallVec { tgt3: u8 },

    // --- Stack operations ---
    /// LD hl, sp + imm8
    LoadHlFromSpOffset { offset: u8 },
    /// ADD sp, imm8
    AddSp(u8),
    /// LD \[imm16\], sp
    LoadFromSp { dest_addr: u16 },
    /// LD sp, hl
    LoadSpFromHl,
    /// PUSH r16stk
    Push(RegisterStack),
    /// POP r16stk,
    Pop(RegisterStack),

    // --- Miscellaneous ---
    /// CCF
    ComplementCarryFlag,
    /// CPL
    ComplementAccumulator,
    /// DAA
    DecimalAdjustAccumulator,
    /// DI
    DisableInterrupts,
    /// EI
    EnableInterrupts,
    /// HALT
    Halt,
    /// NOP
    Nop,
    /// SCF
    SetCarryFlag,
    /// STOP
    Stop,
}

#[derive(Default)]
struct Register {
    value: u16,
}

impl Register {
    fn new(value: u16) -> Self {
        Register { value }
    }

    fn as_u16(&self) -> u16 {
        self.value
    }

    fn high(&self) -> u8 {
        (self.value >> 8) as u8
    }

    fn low(&self) -> u8 {
        (self.value & 0xFF) as u8
    }

    fn from_bytes(high: u8, low: u8) -> Self {
        let value = ((high as u16) << 8) | (low as u16);
        Register { value }
    }
}

#[derive(Default)]
struct RegisterAF {
    pub acc: u8,
    flags: u8,
}

impl RegisterAF {
    fn new(acc: u8, flags: u8) -> Self {
        Self { acc, flags }
    }

    /// Get zero flag
    fn z_flag(&self) -> bool {
        self.flags & (1 << 7) > 1
    }

    /// Get subtraction flag (BCD)
    fn n_flag(&self) -> bool {
        self.flags & (1 << 6) > 1
    }

    /// Get half carry flag (BCD)
    fn h_flag(&self) -> bool {
        self.flags & (1 << 5) > 1
    }

    /// Get carry flag
    fn c_flag(&self) -> bool {
        self.flags & (1 << 4) > 1
    }
}

#[derive(Default)]
struct Cpu {
    /// Program Counter
    pc: u16,
    /// Stack Pointer
    sp: u16,
    /// AF register
    af: RegisterAF,
    /// BF register
    bc: Register,
    /// DE register
    de: Register,
    /// HL register
    hl: Register,
}

/// 16 bit memory
struct Memory {
    memory: [u8; 65536],
}
struct CpuExternal<'a> {
    memory: &'a mut Memory,
}

impl Memory {
    fn read_u8(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            memory: [0u8; 65536],
        }
    }
}

impl Cpu {
    pub fn next(&mut self, external: &mut CpuExternal) {
        let bytes = vec![self.fetch(external.memory)];
        let opcode = match self.decode(bytes.as_ref()) {
            Ok(opcode) => opcode,
            Err(_) => {
                todo!()
            }
        };
        self.execute(opcode, external);
    }

    fn fetch(&mut self, memory: &mut Memory) -> u8 {
        let byte = memory.read_u8(self.pc);
        self.pc += 1;
        byte
    }

    fn decode(&self, bytes: &[u8]) -> Result<Opcode, CpuError> {
        let first_code = bytes
            .first()
            .ok_or(CpuError::DecodeRequireBytes { bytes_len: 1 })?
            .to_owned();
        match first_code {
            0b00000000 => Opcode::Nop,
            // LD r16, imm16
            b if b & 0b11001111 == 0b00000001 => Opcode::Load16 {
                src: self.arg_u16(bytes)?,
                dest: Register16::try_from(bits54(b))?,
            },
            // LD [r16mem], a
            b if b & 0b11001111 == 0b00000010 => Opcode::LoadMemFromA {
                dest_addr: RegisterMemory::try_from(bits54(b))?,
            },
            // LD a, [r16mem]
            b if b & 0b11001111 == 0b00001010 => Opcode::LoadAFromMem {
                src_addr: RegisterMemory::try_from(bits54(b))?,
            },
            // LD [imm16], sp
            0b00001000 => Opcode::LoadFromSp {
                dest_addr: self.arg_u16(bytes)?,
            },
            // INC r16
            b if b & 0b11001111 == 0b00000011 => {
                Opcode::Increment(Register8or16::Reg16(Register16::try_from(bits54(b))?))
            }
            // DEC r16
            b if b & 0b11001111 == 0b00001011 => {
                Opcode::Decrement(Register8or16::Reg16(Register16::try_from(bits54(b))?))
            }
            // ADD hl, r16
            b if b & 0b11001111 == 0b00001001 => Opcode::AddToHl(Register16::try_from(bits54(b))?),
            // INC r8
            b if b & 0b11000111 == 0b00000100 => {
                Opcode::Increment(Register8or16::Reg8(Register8::try_from(bits543(b))?))
            }
            // DEC r8
            b if b & 0b11000111 == 0b00000101 => {
                Opcode::Decrement(Register8or16::Reg8(Register8::try_from(bits543(b))?))
            }
            // LD r8, imm8
            0b00000111 => Opcode::BitRotateLeft(Register8::A, CarryOption::Without),
            0b00001111 => Opcode::BitRotateRight(Register8::A, CarryOption::Without),
            0b00010111 => Opcode::BitRotateLeft(Register8::A, CarryOption::With),
            0b00011111 => Opcode::BitRotateRight(Register8::A, CarryOption::With),
            0b00100111 => Opcode::DecimalAdjustAccumulator,
            0b00101111 => Opcode::ComplementAccumulator,
            0b00110111 => Opcode::SetCarryFlag,
            0b00111111 => Opcode::ComplementCarryFlag,
            // JR imm8
            0b00011000 => Opcode::JumpRelative {
                offset: self.arg_u8(bytes)?,
                cond: None,
            },
            // JR cond, imm8
            b if b & 0b11100111 == 0b00100000 => Opcode::JumpRelative {
                offset: self.arg_u8(bytes)?,
                cond: Some(RegisterCond::try_from(bits43(b))?),
            },
            0b00010000 => Opcode::Stop,
            // LD r8, r8
            b if (b & 0b11000000 == 0b01000000) && b != 0b01110110 => Opcode::Load8 {
                src: ValueOrReg8::Reg8(Register8::try_from(bits543(b))?),
                dest: Register8::try_from(bits210(b))?,
            },
            0b01110110 => Opcode::Halt,
            // ADD a, r8
            b if b & 0b11111000 == 0b10000000 => Opcode::AddRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(b))?),
                CarryOption::Without,
            ),
            // ADC a, r8
            b if b & 0b11111000 == 0b10001000 => Opcode::AddRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(b))?),
                CarryOption::With,
            ),
            // SUB a, r8
            b if b & 0b11111000 == 0b10010000 => Opcode::SubtractRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(b))?),
                CarryOption::Without,
            ),
            // SBC a, r8
            b if b & 0b11111000 == 0b10011000 => Opcode::SubtractRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(b))?),
                CarryOption::With,
            ),
            // AND a, r8
            b if b & 0b11111000 == 0b10100000 => {
                Opcode::BitwiseAndRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(b))?))
            }
            // XOR a, r8
            b if b & 0b11111000 == 0b10101000 => {
                Opcode::BitwiseXorRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(b))?))
            }
            // OR a, r8
            b if b & 0b11111000 == 0b10110000 => {
                Opcode::BitwiseOrRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(b))?))
            }
            // CP a, r8
            b if b & 0b11111000 == 0b10111000 => {
                Opcode::ComparingRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(b))?))
            }
            // ADD a, imm8
            0b11000110 => Opcode::AddRegA(
                ValueOrReg8::Value(self.arg_u8(bytes)?),
                CarryOption::Without,
            ),
            // ADC a, imm8
            0b11001110 => {
                Opcode::AddRegA(ValueOrReg8::Value(self.arg_u8(bytes)?), CarryOption::With)
            }
            // SUB a, imm8
            0b11010110 => Opcode::SubtractRegA(
                ValueOrReg8::Value(self.arg_u8(bytes)?),
                CarryOption::Without,
            ),
            // SBC a, imm8
            0b11011110 => {
                Opcode::SubtractRegA(ValueOrReg8::Value(self.arg_u8(bytes)?), CarryOption::With)
            }
            // AND a, imm8
            0b11100110 => Opcode::BitwiseAndRegA(ValueOrReg8::Value(self.arg_u8(bytes)?)),
            // XOR a, imm8
            0b11101110 => Opcode::BitwiseXorRegA(ValueOrReg8::Value(self.arg_u8(bytes)?)),
            // OR a, imm8
            0b11110110 => Opcode::BitwiseOrRegA(ValueOrReg8::Value(self.arg_u8(bytes)?)),
            // CP a, imm8
            0b11111110 => Opcode::ComparingRegA(ValueOrReg8::Value(self.arg_u8(bytes)?)),
            // RET cond
            b if b & 0b11100111 == 0b11000000 => {
                Opcode::Return(Some(RegisterCond::try_from(bits43(b))?))
            }
            // RET
            0b11001001 => Opcode::Return(None),
            // RETI
            0b11011001 => Opcode::ReturnInterrupts,
            // JP cond, imm16
            b if b & 0b11100111 == 0b11000010 => Opcode::Jump {
                addr: self.arg_u16(bytes)?,
                cond: Some(RegisterCond::try_from(bits43(b))?),
            },
            // JP imm16
            0b11000011 => Opcode::Jump {
                addr: self.arg_u16(bytes)?,
                cond: None,
            },
            // JP hl
            0b11101001 => Opcode::JumpHl,
            // CALL cond, imm16
            b if b & 0b11100111 == 0b11000100 => Opcode::Call {
                addr: self.arg_u16(bytes)?,
                cond: Some(RegisterCond::try_from(bits43(b))?),
            },
            // CALL imm16
            0b11001101 => Opcode::Call {
                addr: self.arg_u16(bytes)?,
                cond: None,
            },
            // RST tgt3
            b if b & 0b11000111 == 0b11000111 => Opcode::RstCallVec { tgt3: bits543(b) },
            // POP r16stk
            0b11000001 => Opcode::Pop(RegisterStack::BC),
            0b11010001 => Opcode::Pop(RegisterStack::DE),
            0b11100001 => Opcode::Pop(RegisterStack::HL),
            0b11110001 => Opcode::Pop(RegisterStack::AF),
            // PUSH r16stk
            0b11000101 => Opcode::Push(RegisterStack::BC),
            0b11010101 => Opcode::Push(RegisterStack::DE),
            0b11100101 => Opcode::Push(RegisterStack::HL),
            0b11110101 => Opcode::Push(RegisterStack::AF),
            // prefix instructions
            0b11001011 => {
                let arg = self.arg_u8(bytes)?;
                match arg {
                    0b00000000..=0b00000111 => Opcode::BitRotateLeft(
                        Register8::try_from(bits210(arg))?,
                        CarryOption::Without,
                    ),
                    0b00001000..=0b00001111 => Opcode::BitRotateRight(
                        Register8::try_from(bits210(arg))?,
                        CarryOption::Without,
                    ),
                    0b00010000..=0b00010111 => {
                        Opcode::BitRotateLeft(Register8::try_from(bits210(arg))?, CarryOption::With)
                    }
                    0b00011000..=0b00011111 => Opcode::BitRotateRight(
                        Register8::try_from(bits210(arg))?,
                        CarryOption::With,
                    ),
                    0b00100000..=0b00100111 => {
                        Opcode::BitShiftLeft(Register8::try_from(bits210(arg))?)
                    }
                    0b00101000..=0b00101111 => {
                        Opcode::BitShiftRight(Register8::try_from(bits210(arg))?)
                    }
                    0b00110000..=0b00110111 => Opcode::BitSwap(Register8::try_from(bits210(arg))?),
                    0b00111000..=0b00111111 => {
                        Opcode::BitShiftLogicRight(Register8::try_from(bits210(arg))?)
                    }
                    0b01000000..=0b01111111 => Opcode::BitTest {
                        operand: Register8::try_from(bits210(arg))?,
                        bit_idx: bits543(arg),
                    },
                    0b10000000..=0b10111111 => Opcode::BitReset {
                        operand: Register8::try_from(bits210(arg))?,
                        bit_idx: bits543(arg),
                    },
                    0b11000000..=0b11111111 => Opcode::BitSet {
                        operand: Register8::try_from(bits210(arg))?,
                        bit_idx: bits543(arg),
                    },
                }
            }
            // LDH [c], a
            0b11100010 => Opcode::LoadHighAToAddrC,
            // LDH [imm8], a
            0b11100000 => Opcode::LoadHighFromA {
                src_addr: self.arg_u8(bytes)?,
            },
            // LD [imm16], a
            0b11101010 => Opcode::LoadFromA {
                dest_addr: self.arg_u16(bytes)?,
            },
            // LDH a, [c]
            0b11110010 => Opcode::LoadHighAddrCToA,
            // LDH a, [imm8]
            0b11110000 => Opcode::LoadHighToA {
                dest_addr: self.arg_u8(bytes)?,
            },
            // LD a, [imm16]
            0b11111010 => Opcode::LoadToA {
                src_addr: self.arg_u16(bytes)?,
            },
            // ADD sp, imm8
            0b11101000 => Opcode::AddSp(self.arg_u8(bytes)?),
            // LD hl, sp + imm8
            0b11111000 => Opcode::LoadHlFromSpOffset {
                offset: self.arg_u8(bytes)?,
            },
            // LD sp, hl
            0b11111001 => Opcode::LoadSpFromHl,
            // DI
            0b11110011 => Opcode::DisableInterrupts,
            // EI
            0b11111011 => Opcode::EnableInterrupts,
            _ => unreachable!(),
        };
        todo!()
    }

    fn execute(&self, opcode: Opcode, external: &mut CpuExternal) {
        todo!()
    }

    fn arg_u8(&self, bytes: &[u8]) -> Result<u8, CpuError> {
        bytes
            .get(1)
            .ok_or(CpuError::DecodeRequireBytes {
                bytes_len: 2 - bytes.len() as u8,
            })
            .copied()
    }

    fn arg_u16(&self, bytes: &[u8]) -> Result<u16, CpuError> {
        bytes
            .get(1..2)
            .ok_or(CpuError::DecodeRequireBytes {
                bytes_len: 3 - bytes.len() as u8,
            })
            .map(|b| ((b[1] as u16) << 8) | (b[0] as u16))
    }
}

fn bits43(byte: u8) -> u8 {
    (byte & 0b00011000) >> 3
}

fn bits54(byte: u8) -> u8 {
    (byte & 0b00110000) >> 4
}

fn bits210(byte: u8) -> u8 {
    byte & 0b00000111
}

fn bits543(byte: u8) -> u8 {
    (byte & 0b00111000) >> 3
}
