use std::io::Read;
use std::{str::FromStr, sync::Arc};
use std::rc::Rc;

use cpu::{Cpu, CpuExternal};
use memory::Memory;
use pixels::{wgpu::Color, Pixels, SurfaceTexture};
use winit::{
    application::ApplicationHandler,
    event::WindowEvent,
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    window::{Window, WindowId},
};

mod cpu;
pub mod hardware;
pub mod memory;

#[derive(Default)]
struct App {
    window: Option<Arc<Window>>,
    pixels: Option<Pixels<'static>>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window = event_loop
            .create_window(
                Window::default_attributes()
                    .with_title("Gameboy Emu")
                    .with_visible(false),
            )
            .unwrap();
        let window_size = window.inner_size();
        let window = Arc::new(window);
        let mut pixels = {
            let surface_texture =
                SurfaceTexture::new(window_size.width, window_size.height, window.clone());
            Pixels::new(320, 240, surface_texture).unwrap()
        };
        pixels.clear_color(Color::RED);
        let frame = pixels.frame_mut();
        for pixel in frame.chunks_exact_mut(4) {
            pixel[0] = 0x00; // R
            pixel[1] = 0x00; // G
            pixel[2] = 0x00; // B
            pixel[3] = 0xff; // A
        }
        pixels.render().unwrap();
        window.set_visible(true);
        self.pixels = Some(pixels);
        self.window = Some(window);
    }

    fn suspended(&mut self, _: &ActiveEventLoop) {
        let pixels = self.pixels.take();
        drop(pixels);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => {
                log::info!("The close button was pressed; stopping");
                event_loop.exit();
            }
            WindowEvent::Resized(size) => {
                let _ = self
                    .pixels
                    .as_mut()
                    .unwrap()
                    .resize_surface(size.width, size.height);
            }
            WindowEvent::RedrawRequested => {
                // Redraw the application.
                //
                // It's preferable for applications that do not render continuously to render in
                // this event rather than in AboutToWait, since rendering in here allows
                // the program to gracefully handle redraws requested by the OS.

                // Draw.

                // Queue a RedrawRequested event.
                //
                // You only need to call this if you've determined that you need to redraw in
                // applications which do not always need to. Applications that redraw continuously
                // can render here instead.
                let pixels = self.pixels.as_mut().unwrap();
                pixels.clear_color(Color::RED);
                let frame = pixels.frame_mut();
                for pixel in frame.chunks_exact_mut(4) {
                    pixel[0] = 0x00; // R
                    pixel[1] = 0x00; // G
                    pixel[2] = 0x00; // B
                    pixel[3] = 0xff; // A
                }
                pixels.render().unwrap();
                log::info!("drawing");
                self.window.as_ref().unwrap().request_redraw();
            }
            _ => (),
        }
    }
}

fn main() {
    env_logger::init();
    let rom_path = if let Ok(default_rom) = std::env::var("DEFAULT_ROM_PATH") {
        default_rom
    } else {
        let mut args = std::env::args();
        let _ = args.next(); // ignore path executable
        let Some(rom_path) = args.next() else {
            println!("Missing first argument: rom path");
            std::process::exit(-1);
        };
        rom_path
    };
    let rom_path = std::path::PathBuf::from_str(&rom_path).unwrap();
    if !rom_path.exists() {
        println!("Rom file {} does not exist!", rom_path.display());
        std::process::exit(-1);
    }

    let mut cpu = Cpu::default();
    let mut memory = Memory::default();
    cpu.reset_to_after_boot_rom();
    memory.reset_to_after_boot_rom();
    let Ok(mut rom_file) = std::fs::File::open(&rom_path) else {
        println!("Could not open rom file {}!", rom_path.display());
        std::process::exit(-1);
    };
    let mut rom_bytes = Vec::new();
    rom_file.read_to_end(&mut rom_bytes).unwrap();
    memory.load_rom(&rom_bytes).unwrap();

    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(ControlFlow::Poll);
    // NOTE: use cycles to decrease it in order to precisely implement cpu clock frequency.
    // If not enough clocks, execute anyway the instruction, but next cycles decrease its amount
    let _ = cpu.do_cycle(&mut CpuExternal::new(&mut memory));

    let mut app = App::default();
    event_loop.run_app(&mut app).unwrap();
    log::info!("App closed!");
}
