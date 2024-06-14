use core::cell::OnceCell;

use esp_hal::rng::Rng;

static mut RAND: OnceCell<Rng> = OnceCell::new();


pub(crate) fn gen_range(lower: f32, upper: f32) -> f32 {
    if let Some(rad) = unsafe { RAND.get_mut() } {
        let frac = rad.random() as f32 / u32::MAX as f32;
        let range = (upper - lower) * frac;
        lower + range
    } else {
        panic!("RNG broken!");
    }
}

pub(crate) fn gen() -> u32 {
    if let Some(rad) = unsafe { RAND.get_mut() } {
        rad.random()
    } else {
        panic!("RNG broken!");
    }
}

pub(crate) fn set_rand(rng: Rng) {
    let _ = unsafe { RAND.set(rng) };
}
