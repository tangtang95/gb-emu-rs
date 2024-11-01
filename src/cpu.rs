enum ValueOrReg8 {
    Reg8(Register8),
    Value(u8)
}

enum Register8or16 {
    Reg8(Register8),
    Reg16(Register16)
}

enum Register8 {
    B,
    C,
    D,
    E,
    H,
    L,
    HlAddress,
    A
}

enum Register16 {
    BC,
    DE,
    HL,
    SP
}

impl TryFrom<u8> for Register16 {
    type Error = String; // TODO: change

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::BC),
            1 => Ok(Self::DE),
            2 => Ok(Self::HL),
            3 => Ok(Self::SP),
            _ => Err("invalid".to_string())
        }
    }
}

enum RegisterStack {
    BC,
    DE,
    HL,
    AF
}

enum RegisterMemory {
    BC,
    DE,
    HLPlus,
    HLMinus
}

enum RegisterCond {
    NZ,
    Z,
    NC,
    C
}

enum CarryOption {
    With,
    Without
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
    BitTest(Register8),
    /// RES b3,r8
    BitReset(Register8),
    /// SET b3, r8
    BitSet(Register8),
    /// SWAP r8
    BitSwap(Register8),

    // --- Bit shift operations ---
    /// RL r8 and RLC r8
    BitRotateLeft(Register8, CarryOption),
    /// RR r8 and RRC r8
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
    /// LD [r16mem], a
    LoadMemFromA { dest_addr: RegisterMemory },
    /// LD a, [r16mem]
    LoadAFromMem { src_addr: RegisterMemory },
    /// LDH [c], a
    LoadHighAToAddrC,
    /// LDH a, [c]
    LoadHighAddrCToA,
    /// LDH [imm8], a
    LoadHighFromA { src_addr: u8 },
    /// LDH a, [imm8]
    LoadHighToA { dest_addr: u8 },
    /// LD [imm16], a
    LoadFromA { dest_addr: u16 },
    /// LD a, [imm16]
    LoadToA { src_addr: u16 },

    // --- Jumps and subroutines ---
    /// JR imm8 and JR cond, imm8
    JumpRelative { offset: u8, cond: Option<RegisterCond> },
    /// JP imm16 and JP cond, imm16
    Jump(u16, Option<RegisterCond>),
    /// JP hl
    JumpHl,
    /// RET and RET cond
    Return(Option<RegisterCond>),
    /// RETI
    ReturnInterrupts,
    /// CALL imm16 and CALL cond, imm16
    Call(u16, Option<RegisterCond>),
    /// RST tgt3
    RstCallVec { tgt3: u8 },

    // --- Stack operations ---
    /// LD hl, sp + imm8
    LoadHlFromSpOffset { offset: u8 },
    /// ADD sp, imm8
    AddSp(u8),
    /// LD [imm16], sp
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
    value: u16
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
    hl: Register
}

/// 16 bit memory
struct Memory {
    memory: [u8; 65536],
}
struct CpuExternal<'a> {
    memory: &'a mut Memory
}

impl Memory {
    fn read_u8(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }
}

impl Default for Memory {
    fn default() -> Self {
        Self { memory: [0u8; 65536] }
    }
}

impl Cpu {
    pub fn next(&mut self, external: &mut CpuExternal) {
        let bytes = vec![self.fetch(external.memory)];
        let opcode = match self.decode(bytes.as_ref()) {
            Ok(opcode) => opcode,
            Err(_) => {
                todo!()
            },
        };
        self.execute(opcode, external);
    }

    fn fetch(&mut self, memory: &mut Memory) -> u8 {
        let byte = memory.read_u8(self.pc);
        self.pc += 1;
        byte
    }

    fn decode(&self, bytes: &[u8]) -> Result<Opcode, String> {
        let first_code = bytes.first().unwrap(); // TODO: fix
        match first_code {
            0b00000000 => Opcode::Nop,
            // LD r16, imm16
            b if b & 0b11001111 == 0b00000001 => {
                let arg = self.arg_u16(bytes)?; // TODO: fix
                let r16 = (b & 0b00110000) >> 4;
                Opcode::Load16 { src: arg, dest: Register16::try_from(r16)? }
            }
            0b00010000 => Opcode::Stop,
            0b01110110 => Opcode::Halt,
            0b11110011 => Opcode::DisableInterrupts,
            0b11111011 => Opcode::EnableInterrupts,
            0b11001011 => {
                // prefix instructions
                todo!()
            },
            c => todo!()
        };
        todo!()
    }

    fn execute(&self, opcode: Opcode, external: &mut CpuExternal) {

    }

    fn arg_u8(&self, bytes: &[u8]) -> Result<u8, String> {
        todo!()
    }

    fn arg_u16(&self, bytes: &[u8]) -> Result<u16, String> {
        todo!()
    }
}

