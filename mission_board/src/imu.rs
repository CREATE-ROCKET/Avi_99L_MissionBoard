use embassy_time::Instant;
use embedded_hal_async::spi::SpiDevice;
use icm426xx::Sample;

use crate::board;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuPollStatus {
    Sample,
    Empty,
}

#[derive(Debug)]
pub enum ImuError<BusError> {
    Init(icm426xx::Error<BusError>),
    Read(icm426xx::Error<BusError>),
    ResetFifo(BusError),
    WhoAmI { expected: u8, actual: u8 },
}

#[derive(Debug, Clone)]
pub struct TimedImuSample {
    pub sample: Sample,
    pub captured_at: Instant,
    pub more_data_pending: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ImuStats {
    pub samples: u64,
    pub empty_polls: u64,
    pub read_errors: u64,
    pub fifo_resets: u64,
}

pub struct Imu<SPI>
where
    SPI: SpiDevice,
{
    driver: icm426xx::ICM42688<SPI, icm426xx::Ready>,
    stats: ImuStats,
}

impl<SPI> Imu<SPI>
where
    SPI: SpiDevice,
{
    pub async fn new(
        spi: SPI,
        delay: impl embedded_hal_async::delay::DelayNs,
        config: icm426xx::Config,
    ) -> Result<Self, ImuError<SPI::Error>> {
        let mut driver = icm426xx::ICM42688::new(spi)
            .initialize(delay, config)
            .await
            .map_err(|err| match err {
                icm426xx::Error::WhoAmIMismatch(actual) => ImuError::WhoAmI {
                    expected: board::ICM_WHO_AM_I_EXPECTED,
                    actual,
                },
                err => ImuError::Init(err),
            })?;

        let actual = driver
            .ll()
            .bank::<{ icm426xx::register_bank::BANK0 }>()
            .who_am_i()
            .async_read()
            .await
            .map_err(|err| ImuError::Read(icm426xx::Error::Bus(err)))?
            .value();

        let expected = board::ICM_WHO_AM_I_EXPECTED;
        if actual != expected {
            return Err(ImuError::WhoAmI { expected, actual });
        }

        let mut imu = Self {
            driver,
            stats: ImuStats::default(),
        };
        imu.reset_fifo().await?;

        Ok(imu)
    }

    pub async fn poll_one(&mut self) -> Result<Option<TimedImuSample>, ImuError<SPI::Error>> {
        match self.driver.read_sample().await {
            Ok(Some((sample, more_data_pending))) => {
                self.stats.samples += 1;
                Ok(Some(TimedImuSample {
                    sample,
                    captured_at: Instant::now(),
                    more_data_pending,
                }))
            }
            Ok(None) => {
                self.stats.empty_polls += 1;
                Ok(None)
            }
            Err(err) => {
                self.stats.read_errors += 1;
                Err(ImuError::Read(err))
            }
        }
    }

    pub async fn reset_fifo(&mut self) -> Result<(), ImuError<SPI::Error>> {
        self.driver
            .reset_fifo()
            .await
            .map_err(ImuError::ResetFifo)?;
        self.stats.fifo_resets += 1;
        Ok(())
    }

    pub fn stats(&self) -> ImuStats {
        self.stats
    }
}
