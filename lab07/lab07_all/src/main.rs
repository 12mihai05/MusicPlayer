#![no_std]
#![no_main]

use core::panic::PanicInfo;
use core::time::Duration;

use cortex_m::delay::Delay;
use embassy_executor::Spawner;


//GPIO
use embassy_rp::gpio::{Level, Output, Pin, Input, Pull};

//USB driver
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};

use embassy_rp::{
    peripherals::{SPI0, PIN_9, PIN_10, PIN_11, PIN_12, PWM_CH0, PIN_1,PIN_2,PIN_3, PIN_4, PIN_0},
    spi::{Config as SpiConfig, Spi, Phase,Polarity},
    pwm::{Config as PwmConfig, Pwm}, // PWM config
    adc::{Config as ConfigAdc, Adc, Channel, InterruptHandler as AdcInterruptHandler},
};
use embassy_time::{Timer};
use log::info;

use embassy_rp::{
    peripherals::I2C0,
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2CInterruptHandler},
};

use embedded_sdmmc::{SdCard,Controller, SdMmcSpi, Spi, VolumeIdx,VolumeManager, FileMode, TimeSource, Timestamp, DirEntry};
use hd44780_driver::{HD44780, DisplayMode, Cursor, CursorBlink};
use heapless::Vec;
use heapless::String;




const LCD_I2C_ADDR: u8 = 0x27; // Example I2C address
const SDA_PIN: u8 = 4; // Change as per your hardware connection
const SCL_PIN: u8 = 5; // Change as per your hardware connection


// Bind the `ADC_IRQ_FIFO` interrupt to the Embassy's ADC handler
bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    I2C0_IRQ => I2CInterruptHandler<I2C0>;
});


// The task used by the serial port driver 
// over USB
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}


struct DummyTimeSource;
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(2024, 5, 12, 0, 0, 0).unwrap()
    }
}

pub struct MyDelay;
impl MyDelay {
    pub async fn delay_ms(ms: u32) {
        Timer::after(Duration::from_millis(ms as u64)).await;
    }
}



fn collect_wav_files(
    controller: &mut Controller<SdCard<Spi<SPI0, (PIN_11, PIN_12, PIN_10)>, PIN_9>, DummyTimeSource>,
    volume: &VolumeIdx,
) -> Result<Vec<String>, embedded_sdmmc::Error> {
    let root_dir = controller.open_root_dir(volume)?;

    let mut wav_files = Vec::with_capacity(32); // Initialize Vec with capacity

    // Open the root directory
    let mut dir = controller.open_dir(volume, &root_dir, "")?;

    // Read directory entries
    while let Some(entry) = controller.read_dir(volume, &mut dir)? {
        if entry.name().ends_with(".wav") {
            // Convert &str to String and push into the vector
            wav_files.push(entry.name().to_string())?;
        }
    }

    Ok(wav_files)
}



#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());

    //SETUP BUTTONS AND POTENTIOMETER
    let mut adc = Adc::new(peripherals.ADC, Irqs, ConfigAdc::default());
    // Initialize ADC pin
    let mut adc_pin = Channel::new_pin(peripherals.PIN_4, Pull::None); // where X should be replaced with a pin number
    
    //let level = adc.read(&mut adc_pin).await.unwrap();

    let pause_button = Input::new(peripherals.PIN_1, Pull::Up); // Assuming you want a pull-up
    let next_button = Input::new(peripherals.PIN_2, Pull::Up);
    let prev_button = Input::new(peripherals.PIN_3, Pull::Up);

    //SETUP PWM FOR SPEAKER
    let mut config: PwmConfig = Default::default();
    config.top = 0x8000;
    config.compare_a = config.top / 2;

    // Create PWM driver for pin 0
    let mut pwm_speaker = Pwm::new_output_a( // output A
    peripherals.PWM_CH0, // channel 0
    peripherals.PIN_0, // pin 0
    config.clone()
    );

    //SETUP LCD

    let sda = peripherals.PIN_X;
    let scl = peripherals.PIN_Y;

    let mut i2c = I2c::new_async(peripherals.I2C0, scl, sda, Irqs, I2cConfig::default());

    let mut lcd = HD44780::new_i2c(i2c, LCD_I2C_ADDR, &mut MyDelay);

    // Configure display
    lcd.reset()?;
    lcd.set_display_mode(DisplayMode {
        display: hd44780_driver::Display::On,
        cursor_visibility: Cursor::Invisible,
        cursor_blink: CursorBlink::Off,
    })?;
    lcd.clear()?;

    //SETUP OF THE SD CARD

    let sck = peripherals.PIN_10;
    let mosi = peripherals.PIN_11;
    let miso = peripherals.PIN_12;

    let spi_config = SpiConfig {
        frequency: 2_000_000,
        phase: Phase::CaptureOnFirstTransition,
        polarity: Polarity::IdleHigh,
    };

    // Configure CS pin
    let mut cs_pin = Output::new(peripherals.PIN_9, Level::High); // Change PIN_9 to the appropriate CS pin
    cs_pin.set_low().unwrap(); // Set CS pin low to select the SD card

    // Configure SPI peripheral
    let mut spi = Spi::new(peripherals.SPI0, sck, mosi, miso, peripherals.DMA_CH0, peripherals.DMA_CH1, spi_config);

    // Create SdMmcSpi instance
    let sdmmc_spi = SdCard::new(spi, cs_pin, );
    let mut volume_mgr = VolumeManager::new(sdmmc_spi, DummyTimeSource::default());

    // Peripheral initialization
    let controller = Controller::new(sdmmc_spi, DummyTimeSource);
    let volume = VolumeIdx(0); // Provide appropriate volume index
    let songs = collect_wav_files(&mut controller, &volume);

    if songs.is_empty() {
        panic!("No WAV file found!");
    }

    lcd.write_str("Now Playing:");

    let mut current_song_index = 0;
    let mut is_paused = false;

    loop {
        // Poll buttons for input
        if pause_button.is_low().await.unwrap() {
            is_paused = !is_paused;
            if is_paused {
                // Update LCD to show paused status
                is_paused = true;
                lcd.write_str(if is_paused { "Paused" } else { "Playing" }).await?;
            }
        }

        // Similar handling for next_button and prev_button...
        if next_button.is_low().unwrap_or(false) {
            current_song_index = (current_song_index + 1) % songs.len();
            is_paused = false; // Optionally reset pause state on song change
            continue; // Immediately proceed with the next song
        }

        if prev_button.is_low().unwrap_or(false) {
            current_song_index = if current_song_index == 0 { songs.len() - 1 } else { current_song_index - 1 };
            is_paused = false; // Optionally reset pause state on song change
            continue; // Immediately proceed with the previous song
        }

        if !is_paused {
            // Async file reading and audio playback logic...
            // Use adc.read() to get potentiometer value and adjust PWM duty cycle
            lcd.set_cursor_pos(40); // Second line
            lcd.write_str(songs[current_song_index]);

            let file = &songs[current_song_index].await?;
            let reader = hound::WavReader::new(file).map_err(|_| "Failed to create WAV reader")?;
            
            // Assuming `lcd` and `pwm_speaker` are properly initialized and can be used here
            lcd.set_cursor_pos(40); // Set cursor position asynchronously
            lcd.write_str(&songs[current_song_index]); // Write to LCD asynchronously
            
            // Iterate through WAV samples
            for sample_result in reader.samples::<i16>() {
                let sample = sample_result.map_err(|_| "Failed to read sample")?;
                
                // Read potentiometer value asynchronously
                let pot_value = adc.read(&mut adc_pin).await.unwrap(); // Handle the unwrap better in production code
                
                // Calculate volume and duty cycle
                let volume_scale = pot_value as f32 / 4095.0; // ADC value normalization
                let scaled_sample = sample as f32 * volume_scale;
                
                // Set PWM duty cycle asynchronously
                let duty = ((scaled_sample as i32 + 32768) as u32 * pwm_speaker.get_max_duty()) / 65536;
                pwm_speaker.set_duty(Channel::A, duty).await;
                
                // Async delay to maintain sample rate
                Timer::after(Duration::from_micros(22)).await;
            }
        }

        // Async delay instead of delay.delay_ms(500);
        Timer::after(Duration::from_millis(500)).await;
    }
}



#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
