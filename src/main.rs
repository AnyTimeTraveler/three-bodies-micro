#![no_std]
#![no_main]

extern crate alloc;

use alloc::collections::VecDeque;
use alloc::format;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal_bus::spi::ExclusiveDevice;
#[allow(unused_imports)]
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    spi::master::Spi,
    spi::SpiMode,
    system::SystemExt,
    systimer::SystemTimer,
};
use esp_println::println;
use log::LevelFilter;
use micromath::*;

use crate::boot::{boot, init_heap, init_logger};
use crate::color::Color;
use crate::color::colors::WHITE;
use crate::display::Display;
use crate::vec2::{Vec2, vec2};

mod boot;
mod display;
mod rand;
mod vec2;
mod color;


#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    boot(
        &clocks,
        peripherals.LPWR,
        peripherals.TIMG0,
        peripherals.TIMG1,
    );
    init_heap();
    init_logger(LevelFilter::Debug);
    println!("Test!");

    let rng = Rng::new(peripherals.RNG);
    rand::set_rand(rng);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);


    let delay = Delay::new(&clocks);

    let miso = io.pins.gpio23;
    let mosi = io.pins.gpio19;
    let sclk = io.pins.gpio18;
    let lcd_dc = io.pins.gpio20.into_push_pull_output();
    let lcd_reset = io.pins.gpio21.into_push_pull_output();
    let lcd_cs = io.pins.gpio22.into_push_pull_output();

    let spi = Spi::new(
        peripherals.SPI2,
        40u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    )
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_miso(miso);

    // let mut st7735 = ST7735::new(spi.into(), lcd_dc, lcd_reset, true, false, 320, 240);

    // st7735.init(&mut delay).unwrap();

    let device = ExclusiveDevice::new(spi, lcd_cs, delay.clone()).unwrap();

    let mut display: Display = Display::init(device, lcd_dc, lcd_reset, delay.clone());

    let mut bodies = [
        Body::new_random(0, &mut display),
        Body::new_random(1, &mut display),
        Body::new_random(2, &mut display),
    ];
    let mut trails: VecDeque<Trail> = VecDeque::new();
    let mut running = true;
    let show_ui = Ui::Full;
    let auto_restart = true;
    let elastic_collisions = false;
    let mut time = SystemTimer::now();

    loop {
        // Reset on space, or if auto restart is on.
        if !running && auto_restart {
            bodies = [
                Body::new_random(0, &mut display),
                Body::new_random(1, &mut display),
                Body::new_random(2, &mut display),
            ];
            trails.clear();
            running = true;
        }

        // Toggle UI on U.
        // if is_key_released(KeyCode::U) {
        //     show_ui.toggle();
        // }

        // Toggle auto-restart on R.
        // if is_key_released(KeyCode::R) {
        //     auto_restart = !auto_restart;
        // }

        // Toggle elastic collisions on C.
        // if is_key_released(KeyCode::C) {
        //     elastic_collisions = !elastic_collisions;
        // }

        if running {
            // Calculate forces to apply based on last frame's positions.
            let mut new_bodies = bodies;
            new_bodies.iter_mut().for_each(|body| {
                body.update_velocity(&mut display, bodies.iter().copied(), elastic_collisions);
            });

            // Update positions based on new velocities.
            bodies = new_bodies;
            trails.iter_mut().for_each(|trail| trail.colour.a *= 0.995);
            trails.extend(bodies.iter().map(Trail::from));
            while trails.front().map_or(false, |trail| trail.colour.a < 0.01) {
                trails.pop_front();
            }
            bodies.iter_mut().for_each(|body| body.update_position(&mut display));

            if !elastic_collisions {
                // If two bodies collide, stop the simulation.
                running = !has_collision(&bodies);
            }
        }

        // Draw all bodies & trails.
        display.clear_background(Rgb565::BLACK);
        // bodies.iter().for_each(Body::draw);
        for body in &bodies {
            body.draw(&mut display);
        }
        // trails.iter().for_each(Trail::draw);
        for trail in &trails {
            trail.draw(&mut display);
        }
        draw_ui(&mut display, &bodies, show_ui, auto_restart, running, elastic_collisions);

        next_frame(&mut time, &delay);
    }
}

fn next_frame(time: &mut u64, delay: &Delay) {
    let now = SystemTimer::now();
    let sleep_target = *time + (1000 / 30);
    let remaining_sleep_time = sleep_target - now;
    if remaining_sleep_time > 0 {
        delay.delay_millis(remaining_sleep_time as u32);
    }
    *time = SystemTimer::now();
}

/// Returns true if any two bodies are colliding.
fn has_collision(bodies: &[Body]) -> bool {
    for i in 0..bodies.len() {
        for j in i + 1..bodies.len() {
            if bodies[i].collides_with(&bodies[j]) {
                return true;
            }
        }
    }
    false
}

/// Draws the UI.
fn draw_ui(
    display: &mut Display,
    bodies: &[Body],
    show_ui: Ui,
    auto_restart: bool,
    running: bool,
    elastic_collisions: bool,
) {
    if !running {
        display.draw_text(
            "COLLISION",
            display.screen_width() / 2.0 - 64.0, // NB Manually centred.
            display.screen_height() / 2.0,
            32.0,
            WHITE,
        );
    }

    // Body info
    if matches!(show_ui, Ui::Full | Ui::Minimal) {
        for body in bodies {
            display.draw_text(
                &format!("m {:.2}", body.mass),
                body.position.x + 10.0,
                body.position.y + 10.0,
                16.0,
                body.colour,
            );
            display.draw_text(
                &format!("v {:.2}", body.velocity.length()),
                body.position.x + 10.0,
                body.position.y + 20.0,
                16.0,
                body.colour,
            );
        }
    }

    // Instructions
    if matches!(show_ui, Ui::Full) {
        let instructions = [
            "[SPACE/CLICK/TAP] reset",
            "[U] toggle UI",
            &format!(
                "[R] toggle auto-restart ({})",
                if auto_restart { "on" } else { "off" }
            ),
            &format!(
                "[C] toggle elastic collisions ({})",
                if elastic_collisions { "on" } else { "off" }
            ),
        ];
        instructions
            .iter()
            .enumerate()
            .for_each(|(idx, instruction)| {
                display.draw_text(
                    instruction,
                    10.0,
                    display.screen_height() - 14.0 - idx as f32 * 14.0,
                    16.0,
                    WHITE,
                )
            });
    }
}

#[derive(Clone, Copy)]
enum Ui {
    Full,
    Minimal,
    Off,
}

impl Ui {
    /// Toggles to the next UI state.
    fn toggle(&mut self) {
        *self = match self {
            Ui::Off => Ui::Minimal,
            Ui::Minimal => Ui::Full,
            Ui::Full => Ui::Off,
        }
    }
}

/// A body in the simulation.
#[derive(Clone, Copy)]
struct Body {
    id: usize,
    colour: Color,
    position: Vec2,
    velocity: Vec2,
    mass: f32,
}

impl Body {
    /// Creates a new body with random properties.
    fn new_random(id: usize, display: &mut Display) -> Self {
        let colour = Color::new(
            rand::gen_range(0.2, 1.0),
            rand::gen_range(0.2, 1.0),
            rand::gen_range(0.2, 1.0),
            1.0,
        );
        let position = vec2(
            rand::gen_range(display.screen_width() * 0.25, display.screen_width() * 0.75),
            rand::gen_range(display.screen_height() * 0.25, display.screen_height() * 0.75),
        );
        let velocity = vec2(rand::gen_range(-1.0, 1.0), rand::gen_range(-1.0, 1.0));
        let mass = rand::gen_range(1., 10.);
        Self {
            id,
            colour,
            position,
            velocity,
            mass,
        }
    }

    /// Draws the body on the screen.
    fn draw(&self, display: &mut Display) {
        display.draw_circle(self.position.x, self.position.y, self.mass, self.colour);
    }

    /// Updates the velocity of the body based on the forces applied by other bodies.
    fn update_velocity(
        &mut self,
        display: &mut Display,
        bodies: impl Iterator<Item=Self> + Clone,
        elastic_collisions: bool,
    ) {
        let mut collided = elastic_collisions;
        if elastic_collisions {
            self.velocity = bodies
                .clone()
                .filter(|&body| body.id != self.id)
                .filter(|other| self.collides_with(other))
                .map(|other| {
                    let m1 = self.mass;
                    let m2 = other.mass;
                    let v1 = self.velocity;
                    let v2 = other.velocity;
                    let v1_prime = ((m1 - m2) / (m1 + m2)) * v1 + ((2.0 * m2) / (m1 + m2)) * v2;
                    v1_prime
                })
                .reduce(|acc, velocity| acc + velocity)
                .unwrap_or_else(|| {
                    collided = false;
                    self.velocity
                });
        }
        if collided {
            return;
        }
        self.velocity += bodies
            .filter(|&body| body.id != self.id)
            .map(|other| {
                let mut delta = other.position - self.position;
                if delta.x.abs() > display.screen_width() / 2.0 {
                    delta.x = delta.x - delta.x.signum() * display.screen_width();
                }

                if delta.y.abs() > display.screen_height() / 2.0 {
                    delta.y = delta.y - delta.y.signum() * display.screen_height();
                }
                let distance = delta.length();
                let direction = delta.normalize();
                let force = (self.mass * other.mass) / (distance * distance);
                direction * force
            })
            .reduce(|acc, force| acc + force)
            .map(|force| 9.81 * force / self.mass)
            .unwrap();
    }

    /// Updates the position of the body based on its velocity.
    fn update_position(&mut self, display: &mut Display) {
        self.position += self.velocity;
        if self.position.x > display.screen_width() {
            self.position.x -= display.screen_width();
        } else if self.position.x < 0. {
            self.position.x += display.screen_width();
        }
        if self.position.y > display.screen_height() {
            self.position.y -= display.screen_height();
        } else if self.position.y < 0. {
            self.position.y += display.screen_height();
        }
    }

    /// Returns true if this body collides with another.
    fn collides_with(&self, other: &Self) -> bool {
        self.position.distance(other.position) <= self.mass + other.mass
    }
}

/// A trail left behind by a body.
#[derive(Clone, Copy)]
struct Trail {
    position: Vec2,
    colour: Color,
}

impl Trail {
    /// Draws the trail on the screen.
    fn draw(&self, display: &mut Display) {
        display.draw_circle(self.position.x, self.position.y, 1.0, self.colour);
    }
}

impl From<&Body> for Trail {
    fn from(body: &Body) -> Self {
        Self {
            position: body.position,
            colour: body.colour,
        }
    }
}
