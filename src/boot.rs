use core::mem::MaybeUninit;

use esp_alloc::EspHeap;
use esp_hal::{
    clock::Clocks,
    peripherals::{LPWR, TIMG0, TIMG1},
    timer::TimerGroup,
};
use esp_hal::rtc_cntl::Rtc;
use esp_println::println;
use log::{LevelFilter, Log, Metadata, Record, set_logger, set_max_level};

struct ESPLogger;

static MY_LOGGER: ESPLogger = ESPLogger;

impl Log for ESPLogger {
    fn enabled(&self, _: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            if let Some(file) = record.file() {
                println!("[{}] {} - {}", record.level(), file, record.args());
            } else {
                println!("[{}] {}", record.level(), record.args());
            }
        }
    }
    fn flush(&self) {}
}

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub(crate) fn init_heap() {
    const HEAP_SIZE: usize = 128 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

pub(crate) fn init_logger(level: LevelFilter) {
    set_logger(&MY_LOGGER).unwrap();
    set_max_level(level);
}

pub(crate) fn boot(clocks: &Clocks, lpwr: LPWR, timg0: TIMG0, timg1: TIMG1) {
    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(lpwr, None);
    let timer_group0 = TimerGroup::new(timg0, clocks, None);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(timg1, clocks, None);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
}
