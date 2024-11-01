mod error;
mod opcode;

use error::CpuError;
use opcode::{
    CarryOption, Opcode, Register16, Register8, Register8or16, RegisterCond, RegisterMemory,
    RegisterStack, ValueOrReg8,
};

use crate::memory::Memory;

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

    fn set_z_flag(&mut self, z: bool) {
        self.flags |= u8::from(z) << 7;
    }

    fn set_n_flag(&mut self, n: bool) {
        self.flags |= u8::from(n) << 6;
    }

    fn set_h_flag(&mut self, h: bool) {
        self.flags |= u8::from(h) << 5;
    }

    fn set_c_flag(&mut self, c: bool) {
        self.flags |= u8::from(c) << 4;
    }
}

#[derive(Default)]
pub struct Cpu {
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

pub struct CpuExternal<'a> {
    memory: &'a mut Memory,
}

impl<'a> CpuExternal<'a> {
    pub fn new(memory: &'a mut Memory) -> Self {
        Self { memory }
    }
}

impl Cpu {
    pub fn next(&mut self, external: &mut CpuExternal) -> Result<(), CpuError> {
        let mut bytes = vec![self.fetch(external.memory)];
        let opcode = match self.decode(bytes.as_ref()) {
            Ok(opcode) => opcode,
            Err(CpuError::DecodeNotEnoughBytes { bytes_len }) => {
                (0..bytes_len).for_each(|_| bytes.push(self.fetch(external.memory)));
                self.decode(bytes.as_ref())?
            }
            Err(_) => todo!(),
        };
        self.execute(opcode, external);
        Ok(())
    }

    fn fetch(&mut self, memory: &mut Memory) -> u8 {
        let byte = memory.read_u8(self.pc);
        self.pc += 1;
        byte
    }

    fn decode(&self, bytes: &[u8]) -> Result<Opcode, CpuError> {
        let op_byte = bytes
            .first()
            .ok_or(CpuError::DecodeNotEnoughBytes { bytes_len: 1 })?
            .to_owned();
        let opcode = match op_byte {
            // Invalid opcodes
            0xD3 | 0xDB | 0xDD | 0xE3 | 0xE4 | 0xEB | 0xEC | 0xED | 0xF4 | 0xFC | 0xFD => {
                return Err(CpuError::InvalidOpcode(op_byte))
            }
            0b00000000 => Opcode::Nop,
            // LD r16, imm16 (00xx0001)
            0b00000001 | 0b00010001 | 0b0100001 | 0b00110001 => Opcode::Load16 {
                src: self.arg_u16(bytes)?,
                dest: Register16::try_from(bits54(op_byte))?,
            },
            // LD [r16mem], a (00xx0010)
            0b00000010 | 0b00010010 | 0b0100010 | 0b00110010 => Opcode::LoadMemFromA {
                dest_addr: RegisterMemory::try_from(bits54(op_byte))?,
            },
            // LD a, [r16mem] (00xx1010)
            0b00001010 | 0b00011010 | 0b0101010 | 0b00111010 => Opcode::LoadAFromMem {
                src_addr: RegisterMemory::try_from(bits54(op_byte))?,
            },
            // LD [imm16], sp
            0b00001000 => Opcode::LoadFromSp {
                dest_addr: self.arg_u16(bytes)?,
            },
            // INC r16 (00xx0011)
            0b00000011 | 0b00010011 | 0b0100011 | 0b00110011 => {
                Opcode::Increment(Register8or16::Reg16(Register16::try_from(bits54(op_byte))?))
            }
            // DEC r16 (00xx1011)
            0b00001011 | 0b00011011 | 0b0101011 | 0b00111011 => {
                Opcode::Decrement(Register8or16::Reg16(Register16::try_from(bits54(op_byte))?))
            }
            // ADD hl, r16 (00xx1001)
            0b00001001 | 0b00011001 | 0b0101001 | 0b00111001 => {
                Opcode::AddToHl(Register16::try_from(bits54(op_byte))?)
            }
            // INC r8 (00xxx100)
            0b00000100 | 0b00001100 | 0b0010100 | 0b00011100 | 0b00100100 | 0b00101100
            | 0b0110100 | 0b00111100 => {
                Opcode::Increment(Register8or16::Reg8(Register8::try_from(bits543(op_byte))?))
            }
            // DEC r8
            0b00000101 | 0b00001101 | 0b0010101 | 0b00011101 | 0b00100101 | 0b00101101
            | 0b0110101 | 0b00111101 => {
                Opcode::Decrement(Register8or16::Reg8(Register8::try_from(bits543(op_byte))?))
            }
            // LD r8, imm8
            0b00000110 | 0b00001110 | 0b0010110 | 0b00011110 | 0b00100110 | 0b00101110
            | 0b0110110 | 0b00111110 => Opcode::Load8 {
                src: ValueOrReg8::Value(self.arg_u8(bytes)?),
                dest: Register8::try_from(bits543(op_byte))?,
            },
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
            0b00100000 | 0b00101000 | 0b00110000 | 0b00111000 => Opcode::JumpRelative {
                offset: self.arg_u8(bytes)?,
                cond: Some(RegisterCond::try_from(bits43(op_byte))?),
            },
            0b00010000 => Opcode::Stop,
            // HALT (NOTE: must be before LD r8, r8)
            0b01110110 => Opcode::Halt,
            // LD r8, r8 (cover also HALT)
            0b01000000..=0b01111111 => Opcode::Load8 {
                src: ValueOrReg8::Reg8(Register8::try_from(bits543(op_byte))?),
                dest: Register8::try_from(bits210(op_byte))?,
            },
            // ADD a, r8
            0b10000000..=0b10000111 => Opcode::AddRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?),
                CarryOption::Without,
            ),
            // ADC a, r8
            0b10001000..=0b10001111 => Opcode::AddRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?),
                CarryOption::With,
            ),
            // SUB a, r8
            0b10010000..=0b10010111 => Opcode::SubtractRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?),
                CarryOption::Without,
            ),
            // SBC a, r8
            0b10011000..=0b10011111 => Opcode::SubtractRegA(
                ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?),
                CarryOption::With,
            ),
            // AND a, r8
            0b10100000..=0b10100111 => {
                Opcode::BitwiseAndRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?))
            }
            // XOR a, r8
            0b10101000..=0b10101111 => {
                Opcode::BitwiseXorRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?))
            }
            // OR a, r8
            0b10110000..=0b10110111 => {
                Opcode::BitwiseOrRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?))
            }
            // CP a, r8
            0b10111000..=0b10111111 => {
                Opcode::ComparingRegA(ValueOrReg8::Reg8(Register8::try_from(bits210(op_byte))?))
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
            0b11000000 | 0b11001000 | 0b11010000 | 0b11011000 => {
                Opcode::Return(Some(RegisterCond::try_from(bits43(op_byte))?))
            }
            // RET
            0b11001001 => Opcode::Return(None),
            // RETI
            0b11011001 => Opcode::ReturnInterrupts,
            // JP cond, imm16
            0b11000010 | 0b11001010 | 0b11010010 | 0b11011010 => Opcode::Jump {
                addr: self.arg_u16(bytes)?,
                cond: Some(RegisterCond::try_from(bits43(op_byte))?),
            },
            // JP imm16
            0b11000011 => Opcode::Jump {
                addr: self.arg_u16(bytes)?,
                cond: None,
            },
            // JP hl
            0b11101001 => Opcode::JumpHl,
            // CALL cond, imm16
            0b11000100 | 0b11001100 | 0b11010100 | 0b11011100 => Opcode::Call {
                addr: self.arg_u16(bytes)?,
                cond: Some(RegisterCond::try_from(bits43(op_byte))?),
            },
            // CALL imm16
            0b11001101 => Opcode::Call {
                addr: self.arg_u16(bytes)?,
                cond: None,
            },
            // RST tgt3
            0b11000111 | 0b11001111 | 0b11010111 | 0b11011111 | 0b11100111 | 0b11101111
            | 0b11110111 | 0b11111111 => Opcode::RstCallVec {
                tgt3: bits543(op_byte),
            },
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
        };
        Ok(opcode)
    }

    fn execute(&mut self, opcode: Opcode, external: &mut CpuExternal) {
        match opcode {
            Opcode::AddRegA(value_or_reg8, carry_opt) => {
                let value = match value_or_reg8 {
                    ValueOrReg8::Reg8(reg) => self.reg8(&reg, external.memory),
                    ValueOrReg8::Value(value) => value,
                };
                let prev_acc = self.af.acc;
                let (mut new_acc, mut overflow) = self.af.acc.overflowing_add(value);
                let carry = match carry_opt {
                    CarryOption::With => u8::from(self.af.c_flag()),
                    CarryOption::Without => 0,
                };
                let (carry_acc, overflow_carry) = new_acc.overflowing_add(carry);
                new_acc = carry_acc;
                overflow |= overflow_carry;
                self.af.acc = new_acc;
                self.af.set_z_flag(self.af.acc == 0);
                self.af.set_n_flag(false);
                self.af
                    .set_h_flag((((prev_acc & 0xF) + (value & 0xF) + carry) & 0x10) > 0);
                self.af.set_c_flag(overflow);
            }
            Opcode::SubtractRegA(value_or_reg8, carry_opt) => {
                let value = match value_or_reg8 {
                    ValueOrReg8::Reg8(reg) => self.reg8(&reg, external.memory),
                    ValueOrReg8::Value(value) => value,
                };
                let prev_acc = self.af.acc;
                let (mut new_acc, mut overflow) = self.af.acc.overflowing_sub(value);
                let carry = match carry_opt {
                    CarryOption::With => u8::from(self.af.c_flag()),
                    CarryOption::Without => 0,
                };
                let (carry_acc, overflow_carry) = new_acc.overflowing_sub(carry);
                new_acc = carry_acc;
                overflow |= overflow_carry;
                self.af.acc = new_acc;
                self.af.set_z_flag(self.af.acc == 0);
                self.af.set_n_flag(true);
                self.af.set_h_flag(
                    (((prev_acc & 0xF)
                        .wrapping_sub(value & 0xF)
                        .wrapping_sub(carry))
                        & 0x10)
                        > 0,
                );
                self.af.set_c_flag(overflow);
            }
            Opcode::BitwiseAndRegA(value_or_reg8) => {
                let value = match value_or_reg8 {
                    ValueOrReg8::Reg8(reg) => self.reg8(&reg, external.memory),
                    ValueOrReg8::Value(value) => value,
                };
                self.af.acc &= value;
                self.af.set_z_flag(self.af.acc == 0);
                self.af.set_n_flag(false);
                self.af.set_h_flag(true);
                self.af.set_c_flag(false);
            }
            Opcode::BitwiseXorRegA(value_or_reg8) => {
                let value = match value_or_reg8 {
                    ValueOrReg8::Reg8(reg) => self.reg8(&reg, external.memory),
                    ValueOrReg8::Value(value) => value,
                };
                self.af.acc ^= value;
                self.af.set_z_flag(self.af.acc == 0);
                self.af.set_n_flag(false);
                self.af.set_h_flag(false);
                self.af.set_c_flag(false);
            }
            Opcode::BitwiseOrRegA(value_or_reg8) => {
                let value = match value_or_reg8 {
                    ValueOrReg8::Reg8(reg) => self.reg8(&reg, external.memory),
                    ValueOrReg8::Value(value) => value,
                };
                self.af.acc |= value;
                self.af.set_z_flag(self.af.acc == 0);
                self.af.set_n_flag(false);
                self.af.set_h_flag(false);
                self.af.set_c_flag(false);
            }
            Opcode::ComparingRegA(value_or_reg8) => {
                let value = match value_or_reg8 {
                    ValueOrReg8::Reg8(reg) => self.reg8(&reg, external.memory),
                    ValueOrReg8::Value(value) => value,
                };
                let (new_acc, overflow) = self.af.acc.overflowing_sub(value);
                self.af.set_z_flag(new_acc == 0);
                self.af.set_n_flag(true);
                self.af
                    .set_h_flag((((self.af.acc & 0xF).wrapping_sub(value & 0xF)) & 0x10) > 0);
                self.af.set_c_flag(overflow);
            }
            Opcode::AddToHl(reg16) => {
                let value = self.reg16(&reg16);
                let prev_hl = self.hl.value;
                let (new_hl, overflow) = self.hl.value.overflowing_add(value);
                self.hl.value = new_hl;
                self.af.set_n_flag(false);
                self.af.set_h_flag((((prev_hl & 0xFFF) + (value & 0xFFF)) & 0x1000) > 0);
                self.af.set_c_flag(overflow)
            },
            Opcode::Increment(Register8or16::Reg8(reg8)) => {
                let new_value = self.reg8(&reg8, external.memory) + 1;
                self.set_reg8(&reg8, new_value, external.memory);
                self.af.set_z_flag(new_value == 0);
                self.af.set_n_flag(false);
                self.af.set_h_flag(new_value == 0x10);
            },
            Opcode::Increment(Register8or16::Reg16(reg16)) => {

            },
            Opcode::Decrement(_) => todo!(),
            Opcode::BitTest { operand, bit_idx } => todo!(),
            Opcode::BitReset { operand, bit_idx } => todo!(),
            Opcode::BitSet { operand, bit_idx } => todo!(),
            Opcode::BitSwap(_) => todo!(),
            Opcode::BitRotateLeft(_, _) => todo!(),
            Opcode::BitRotateRight(_, _) => todo!(),
            Opcode::BitShiftLeft(_) => todo!(),
            Opcode::BitShiftRight(_) => todo!(),
            Opcode::BitShiftLogicRight(_) => todo!(),
            Opcode::Load8 { src, dest } => todo!(),
            Opcode::Load16 { src, dest } => todo!(),
            Opcode::LoadMemFromA { dest_addr } => todo!(),
            Opcode::LoadAFromMem { src_addr } => todo!(),
            Opcode::LoadHighAToAddrC => todo!(),
            Opcode::LoadHighAddrCToA => todo!(),
            Opcode::LoadHighFromA { src_addr } => todo!(),
            Opcode::LoadHighToA { dest_addr } => todo!(),
            Opcode::LoadFromA { dest_addr } => todo!(),
            Opcode::LoadToA { src_addr } => todo!(),
            Opcode::JumpRelative { offset, cond } => todo!(),
            Opcode::Jump { addr, cond } => todo!(),
            Opcode::JumpHl => todo!(),
            Opcode::Return(_) => todo!(),
            Opcode::ReturnInterrupts => todo!(),
            Opcode::Call { addr, cond } => todo!(),
            Opcode::RstCallVec { tgt3 } => todo!(),
            Opcode::LoadHlFromSpOffset { offset } => todo!(),
            Opcode::AddSp(_) => todo!(),
            Opcode::LoadFromSp { dest_addr } => todo!(),
            Opcode::LoadSpFromHl => todo!(),
            Opcode::Push(_) => todo!(),
            Opcode::Pop(_) => todo!(),
            Opcode::ComplementCarryFlag => todo!(),
            Opcode::ComplementAccumulator => todo!(),
            Opcode::DecimalAdjustAccumulator => todo!(),
            Opcode::DisableInterrupts => todo!(),
            Opcode::EnableInterrupts => todo!(),
            Opcode::Halt => todo!(),
            Opcode::Nop => todo!(),
            Opcode::SetCarryFlag => todo!(),
            Opcode::Stop => todo!(),
        }
    }

    fn arg_u8(&self, bytes: &[u8]) -> Result<u8, CpuError> {
        bytes
            .get(1)
            .ok_or(CpuError::DecodeNotEnoughBytes {
                bytes_len: 2 - bytes.len() as u8,
            })
            .copied()
    }

    fn arg_u16(&self, bytes: &[u8]) -> Result<u16, CpuError> {
        bytes
            .get(1..2)
            .ok_or(CpuError::DecodeNotEnoughBytes {
                bytes_len: 3 - bytes.len() as u8,
            })
            .map(|b| ((b[1] as u16) << 8) | (b[0] as u16))
    }

    fn reg8(&self, reg: &Register8, memory: &Memory) -> u8 {
        match reg {
            Register8::B => self.bc.high(),
            Register8::C => self.bc.low(),
            Register8::D => self.de.high(),
            Register8::E => self.de.low(),
            Register8::H => self.hl.high(),
            Register8::L => self.hl.low(),
            Register8::HlAddress => memory.read_u8(self.hl.value),
            Register8::A => self.af.acc,
        }
    }

    fn set_reg8(&mut self, reg: &Register8, value: u8, memory: &mut Memory) {
        match reg {
            Register8::B => self.bc = Register::from_bytes(value, self.bc.low()),
            Register8::C => self.bc = Register::from_bytes(self.bc.high(), value),
            Register8::D => self.de = Register::from_bytes(value, self.de.low()),
            Register8::E => self.de = Register::from_bytes(self.de.high(), value),
            Register8::H => self.hl = Register::from_bytes(value, self.hl.low()),
            Register8::L => self.hl = Register::from_bytes(self.hl.high(), value),
            Register8::HlAddress => memory.write_u8(self.hl.value, value),
            Register8::A => self.af.acc = value,
        }
    }

    fn reg16(&self, reg: &Register16) -> u16 {
        match reg {
            Register16::BC => self.bc.value,
            Register16::DE => self.de.value,
            Register16::HL => self.hl.value,
            Register16::SP => self.sp,
        }
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
