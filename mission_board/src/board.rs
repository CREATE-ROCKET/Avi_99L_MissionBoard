pub const ICM_SPI_FREQUENCY_HZ: u32 = 1_000_000;
pub const ICM_WHO_AM_I_EXPECTED: u8 = 0x47;
pub const ICM_FIFO_DRAIN_BUDGET_PER_POLL: usize = 8;
pub const ICM_EMPTY_POLL_BACKOFF_MS: u64 = 2;
pub const ICM_ERROR_RETRY_BACKOFF_MS: u64 = 100;

// ICM-42688-P U4 SPI pin map:
// MISO: GPIO35
// MOSI: GPIO36
// SCLK: GPIO37
// CS:   GPIO38
//
// INT1 is not connected.
// INT2/FSYNC/CLKIN is tied to GND.
// Therefore the IMU driver must poll over SPI/FIFO.
