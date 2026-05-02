#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod panic;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rtc_cntl::SocResetReason;
use esp_hal::spi::{
    Mode,
    master::{Config as SpiConfig, Spi},
};
use esp_hal::system::{Stack, reset_reason};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_rtos::embassy::Executor;
use esp_rtos::start_second_core_with_stack_guard_offset;
use mission_board::{board, imu};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

type IcmSpi = Spi<'static, Async>;
type IcmSpiDevice = ExclusiveDevice<IcmSpi, Output<'static>, Delay>;

static SECOND_STACK: StaticCell<Stack<4096>> = StaticCell::new();
static SECOND_EXECUTOR: StaticCell<Executor> = StaticCell::new();
static IMU_SAMPLES: Channel<CriticalSectionRawMutex, imu::TimedImuSample, 4> = Channel::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    if let Some(reason) = reset_reason()
        && matches!(
            reason,
            SocResetReason::CoreMwdt0 | SocResetReason::CoreMwdt1 | SocResetReason::CoreSw
        )
    {
        println!("Reset reason: {:?}", reason);
        fault_backoff_loop("Refusing to continue after watchdog/software reset").await;
    }

    let spi = match Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(board::ICM_SPI_FREQUENCY_HZ))
            .with_mode(Mode::_0),
    ) {
        Ok(spi) => spi,
        Err(err) => {
            println!("Failed to initialize ICM SPI: {:?}", err);
            fault_backoff_loop("ICM SPI configuration failed").await;
        }
    };

    let spi = spi
        .with_sck(peripherals.GPIO37)
        .with_mosi(peripherals.GPIO36)
        .with_miso(peripherals.GPIO35)
        .into_async();

    let cs = Output::new(peripherals.GPIO38, Level::High, OutputConfig::default());

    let spi_device = match ExclusiveDevice::new(spi, cs, Delay) {
        Ok(device) => device,
        Err(err) => {
            println!("Failed to create ICM SPI device: {:?}", err);
            fault_backoff_loop("ICM SPI device creation failed").await;
        }
    };

    match imu_task(spi_device) {
        Ok(task) => spawner.spawn(task),
        Err(err) => {
            println!("Failed to create ICM task: {:?}", err);
            fault_backoff_loop("ICM task creation failed").await;
        }
    }

    match imu_log_task() {
        Ok(task) => spawner.spawn(task),
        Err(err) => {
            println!("Failed to create ICM log task: {:?}", err);
        }
    }

    let second_stack = SECOND_STACK.init(Stack::new());

    start_second_core_with_stack_guard_offset(
        peripherals.CPU_CTRL,
        software_interrupt.software_interrupt1,
        second_stack,
        Some(64),
        || {
            let executor = SECOND_EXECUTOR.init(Executor::new());
            executor.run(|spawner| match second_core_task() {
                Ok(task) => spawner.spawn(task),
                Err(err) => {
                    println!("Failed to create second-core task: {:?}", err);
                }
            });
        },
    );

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

async fn fault_backoff_loop(message: &str) -> ! {
    println!("{}", message);

    loop {
        Timer::after(Duration::from_millis(board::ICM_ERROR_RETRY_BACKOFF_MS)).await;
    }
}

#[embassy_executor::task]
async fn imu_task(spi_device: IcmSpiDevice) -> ! {
    let mut imu = match imu::Imu::new(spi_device, Delay, icm426xx::Config::default()).await {
        Ok(imu) => imu,
        Err(err) => {
            println!("ICM init failed: {:?}", err);
            fault_backoff_loop("ICM init failed; SPI ownership is consumed, staying in fault loop")
                .await;
        }
    };

    let mut dropped_channel_samples = 0;
    let mut last_channel_drop_log = Instant::now();

    loop {
        let mut drained = 0;

        while drained < board::ICM_FIFO_DRAIN_BUDGET_PER_POLL {
            match imu.poll_one().await {
                Ok(Some(sample)) => {
                    drained += 1;
                    let more_data_pending = sample.more_data_pending;
                    publish_imu_sample(
                        sample,
                        &mut dropped_channel_samples,
                        &mut last_channel_drop_log,
                    );

                    if !more_data_pending {
                        break;
                    }
                }
                Ok(None) => {
                    Timer::after(Duration::from_millis(board::ICM_EMPTY_POLL_BACKOFF_MS)).await;
                    break;
                }
                Err(err) => {
                    println!("ICM read error: {:?}; stats: {:?}", err, imu.stats());

                    if let Err(reset_err) = imu.reset_fifo().await {
                        println!("ICM FIFO reset failed: {:?}", reset_err);
                    }

                    Timer::after(Duration::from_millis(board::ICM_ERROR_RETRY_BACKOFF_MS)).await;
                    break;
                }
            }
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embassy_executor::task]
async fn second_core_task() -> ! {
    loop {
        println!("Hello from the second core!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn imu_log_task() -> ! {
    loop {
        let mut sample = IMU_SAMPLES.receive().await;
        while let Ok(newer_sample) = IMU_SAMPLES.try_receive() {
            sample = newer_sample;
        }

        println!(
            "Latest ICM sample: captured_at={:?}, more_data_pending={}, sample={:?}",
            sample.captured_at, sample.more_data_pending, sample.sample
        );
        Timer::after(Duration::from_secs(1)).await;
    }
}

fn publish_imu_sample(
    sample: imu::TimedImuSample,
    dropped_channel_samples: &mut u64,
    last_channel_drop_log: &mut Instant,
) {
    match IMU_SAMPLES.try_send(sample) {
        Ok(()) => {}
        Err(TrySendError::Full(sample)) => {
            let _ = IMU_SAMPLES.try_receive();
            let _ = IMU_SAMPLES.try_send(sample);
            *dropped_channel_samples += 1;

            if last_channel_drop_log.elapsed() >= Duration::from_secs(5) {
                println!(
                    "ICM sample channel full; replaced/dropped {} samples",
                    *dropped_channel_samples
                );
                *dropped_channel_samples = 0;
                *last_channel_drop_log = Instant::now();
            }
        }
    }
}
