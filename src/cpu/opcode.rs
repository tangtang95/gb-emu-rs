use crate::cpu::error::CpuError;

/// Operation code organized based on https://rgbds.gbdev.io/docs/v0.8.0/gbz80.7
/// https://gbdev.io/pandocs/CPU_Instruction_Set.html
pub enum Opcode {
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
    BitShiftRightLogic(Register8),

    // --- Load instructions ---
    /// LD r8, imm8 and LD r8, r8
    Load8 { src: ValueOrReg8, dest: Register8 },
    /// LD r16, imm16
    Load16 { src_value: u16, dest: Register16 },
    /// LD \[r16mem\], a
    LoadMemFromA { dest_addr: RegisterMemory },
    /// LD a, \[r16mem\]
    LoadAFromMem { src_addr: RegisterMemory },
    /// LDH \[c\], a
    LoadHighAToAddrC,
    /// LDH a, \[c\]
    LoadHighAddrCToA,
    /// LDH \[imm8\], a
    LoadHighFromA { offset: u8 },
    /// LDH a, \[imm8\]
    LoadHighToA { offset: u8 },
    /// LD \[imm16\], a
    LoadFromA { dest_addr: u16 },
    /// LD a, \[imm16\]
    LoadToA { src_addr: u16 },

    // --- Jumps and subroutines ---
    /// JR imm8 and JR cond, imm8
    JumpRelative {
        offset: i8,
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
    LoadHlFromSpOffset { offset: i8 },
    /// ADD sp, imm8
    AddSp(i8),
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

pub enum ValueOrReg8 {
    Reg8(Register8),
    Value(u8),
}

pub enum Register8or16 {
    Reg8(Register8),
    Reg16(Register16),
}

pub enum Register8 {
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

pub enum Register16 {
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

pub enum RegisterStack {
    BC,
    DE,
    HL,
    AF,
}

pub enum RegisterMemory {
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

pub enum RegisterCond {
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

pub enum CarryOption {
    With,
    Without,
}

