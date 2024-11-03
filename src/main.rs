use cpu::{Cpu, CpuExternal};
use memory::Memory;

mod cpu;
pub mod memory;
pub mod hardware;

fn main() {
    let mut cpu = Cpu::default();
    let mut memory = Memory::default();
    // NOTE: use cycles to decrease it in order to precisely implement cpu clock frequency.
    // If not enough clocks, execute anyway the instruction, but next cycles decrease its amount
    let _ = cpu.do_cycle(&mut CpuExternal::new(&mut memory));
}
