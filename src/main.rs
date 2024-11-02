use cpu::{Cpu, CpuExternal};
use memory::Memory;

mod cpu;
pub mod memory;

fn main() {
    let mut cpu = Cpu::default();
    let mut memory = Memory::default();
    let _ = cpu.do_cycle(&mut CpuExternal::new(&mut memory));
}
