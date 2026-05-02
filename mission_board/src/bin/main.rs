#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod panic;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rtc_cntl::SocResetReason;
use esp_hal::system::{Stack, reset_reason};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_rtos::embassy::Executor;
use esp_rtos::start_second_core_with_stack_guard_offset;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

static SECOND_STACK: StaticCell<Stack<4096>> = StaticCell::new();
static SECOND_EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    if matches!(
        reset_reason().unwrap(),
        SocResetReason::CoreMwdt0 | SocResetReason::CoreMwdt1 | SocResetReason::CoreSw
    ) {
        println!("Reset reason: {:?}", reset_reason().unwrap());
        loop {}
    }

    spawner.spawn(task1().unwrap());

    let second_stack = SECOND_STACK.init(Stack::new());

    start_second_core_with_stack_guard_offset(
        peripherals.CPU_CTRL,
        software_interrupt.software_interrupt1,
        second_stack,
        Some(64),
        || {
            let executor = SECOND_EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.spawn(second_core_task().unwrap());
            });
        },
    );

    loop {
        println!("Hello, world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn task1() {
    loop {
        println!("Task 1");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn second_core_task() -> ! {
    loop {
        println!("Hello from the second core!");
        esp_hal::rom::ets_delay_us(1_000_000);
    }
}
