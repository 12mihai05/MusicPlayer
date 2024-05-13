#![no_std]
#![no_main]


use embassy_executor::Spawner;

use core::cell::RefCell;
use core::panic::PanicInfo;

use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};
use log::info;
use log::error;

use embassy_rp::spi::{self, Spi, Config as SpiConfig};
use embassy_rp::adc::{self, Adc,Config as ADCConfig, Channel, InterruptHandler as ADCInterruptHandler};
use embassy_rp::pwm::{Config as PWMConfig, Pwm};
use embassy_rp::gpio::{Output, Level, Input,Pull};
use embassy_rp::peripherals::*;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_time::Delay;
use embassy_time::Timer;



use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx, Volume, Directory};
use embedded_sdmmc::TimeSource;
use embedded_sdmmc::Timestamp;
use embedded_sdmmc::VolumeType::Fat;


// LCD
use embassy_rp::i2c::{I2c, Config as I2CConfig};
use ag_lcd::{Cursor, LcdDisplay, AutoScroll,Size,Scroll};
use port_expander::dev::pcf8574::Pcf8574;


#[derive(Default)]
pub struct Time();

impl TimeSource for Time {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
          year_since_1970: 0,
          zero_indexed_month: 0,
          zero_indexed_day: 0,
          hours: 0,
          minutes: 0,
          seconds: 0,
        }
    }
}

bind_interrupts!(struct Irqs {
    // Use for the serial over USB driver
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    ADC_IRQ_FIFO => ADCInterruptHandler;
});

// The task used by the serial port driver over USB
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

const DISPLAY_FREQ: u32 = 200_000;


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());

    // Start the serial port over USB driver
    let driver = Driver::new(peripherals.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    //START SD CARD

    let mut miso = peripherals.PIN_4;
    let mut mosi = peripherals.PIN_3;
    let mut clk = peripherals.PIN_2;
    let mut cs = Output::new(peripherals.PIN_5, Level::High);
    let mut cs_sd = Output::new(peripherals.PIN_1, Level::High);
    let mut channel_spi = peripherals.SPI0;
    let mut dma_spi_1 = peripherals.DMA_CH0;
    let mut dma_spi_2 = peripherals.DMA_CH1;
    let mut microsd_config = SpiConfig::default();
    microsd_config.frequency = 400000;

    // let mut spi = Spi::new(
    //     channel_spi,
    //     clk,
    //     mosi,
    //     miso,
    //     dma_spi_1,
    //     dma_spi_2,
    //     microsd_config.clone(),
    // );

    Timer::after_secs(2).await;

    info!("Initialize SPI SD/MMC data structures...");

    let mut delay = Delay;

    let mut spi = Spi::new_blocking(channel_spi, clk, mosi, miso, microsd_config);

    let spi_bus = NoopMutex::new(RefCell::new(spi));

    let spi_device = SpiDevice::new(&spi_bus, cs);

    let mut sdcard = SdCard::new(spi_device, cs_sd, delay);

    let mut time_source = Time::default();

    Timer::after_millis(100).await;

    let mut volume_mgr = VolumeManager::new(sdcard, time_source);

    Timer::after_millis(100).await;

    match volume_mgr.device().num_bytes() {
        Ok(size) => info!("card size is {} bytes", size),
        Err(e) => {
            error!("Error retrieving card size: {:?}", e);
        }
    }

    Timer::after_millis(100).await;

    let mut volume_sd = match volume_mgr.get_volume(VolumeIdx(0)) {
        Ok(volume) => volume, 
        Err(err) => {
          info!("Error opening volume: {:?}", err);
          return;
        }
    };

    Timer::after_millis(100).await;

    info!("Volume: {:?}",volume_sd);

    Timer::after_millis(100).await;
    let volume_result = volume_mgr.open_root_dir(&volume_sd).unwrap();
    
    Timer::after_millis(100).await;
    info!("Root directory opened!");

    volume_mgr.iterate_dir(&volume_sd, &volume_result, |ent| {
        info!(
            "/{}.{}",
            core::str::from_utf8(ent.name.base_name()).unwrap(),
            core::str::from_utf8(ent.name.extension()).unwrap()
        );
    }).unwrap();

    let mut successful_read = false;

    if let Ok(mut file) = volume_mgr.open_file_in_dir(&mut volume_sd, &volume_result, "a4.wav", Mode::ReadOnly) {
        
        const SIZE: usize = 57300;

        let mut buf = [0u8; SIZE];
        let read_count = volume_mgr.read(&volume_sd, &mut file, &mut buf).unwrap();
        volume_mgr.close_file(&volume_sd, file).unwrap();

        let mut fmt = &buf[..36];
        let mut sub_data = &buf[36..44];
        let mut data = &buf[44..];

        let data_size = u32::from_le_bytes([sub_data[4], sub_data[5], sub_data[6], sub_data[7]]);

        let mut i = 0;
        let mut j = 1;

        let mut nr = SIZE;

        if data.len() >= 2 {
            info!("READ {} bytes:", read_count);
            info!("FMT: {:?}", fmt);
            info!("SUB_DATA {:?}", sub_data);
            while nr > 0
            {
                let bit_16 = u16::from_le_bytes([data[i], data[j]]);

                let bit_12 = bit_16 & 0x0FFF;


                info!("16-bit structure: {:#06x}", bit_16);
                info!("12-bit structure: {:#03x}", bit_12);







                i = i + 1;
                j = j + 1;
                nr = nr - 2;
            }

            // info!("DATA {:?}", data);

            successful_read = true;
        }
    }

        volume_mgr.free();

        if successful_read {
            info!("Successfully read file");
        } else {
            info!("Could not read file, which is ok for the first run.");
            info!("Reboot the pico!");
        }

        //END SD CARD



        //START LCD

        // Initiate SDA and SCL pins
    let sda = peripherals.PIN_12;
    let scl = peripherals.PIN_13;
    
    // Initiate Delay
    let delay = Delay;

    // I2C Config
    let mut config = I2CConfig::default();
    config.frequency = DISPLAY_FREQ;

    // Initiate I2C
    let i2c = I2c::new_blocking(peripherals.I2C0, scl, sda, config.clone());
    let mut i2c_expander = Pcf8574::new(i2c, true, true, true);

    // Initiate LCD
    let mut lcd: LcdDisplay<_, _> = LcdDisplay::new_pcf8574(&mut i2c_expander, delay)
    .with_cursor(Cursor::Off)
    .with_reliable_init(10000)
    .build();

    
    let msg = "qwertyuiopasdfgh";
    
    // Write to LCD
    lcd.print(msg);

    //END LCD
    
    //START BUTTONS

    //push buttons
    let mut play_pause_button = Input::new(peripherals.PIN_19, Pull::Up);
    let mut next_button = Input::new(peripherals.PIN_20, Pull::Up);
    let mut previous_button = Input::new(peripherals.PIN_21, Pull::Up);

    //potentiometer
    let mut adc = Adc::new(peripherals.ADC, Irqs, ADCConfig::default());
    let mut adc_pin = Channel::new_pin(peripherals.PIN_28, Pull::None);
    let level = adc.read(&mut adc_pin).await.unwrap();

    //END BUTTONS

    //START LEDS

    let mut configLED: PWMConfig = Default::default();
    configLED.top = 0x8000;
    let mut ledsPwm = Pwm::new_output_a( // output A
        peripherals.PWM_CH0, // channel 0
        peripherals.PIN_0, // pin 0
        configLED.clone()
    );

    //setup pwm
    configLED.compare_a += 100; // modified value of `compare_a`
    ledsPwm.set_config(&configLED); // set the new configuration for PWM

    //END LEDS

    //START SPEAKER

    let mut configSpeaker: PWMConfig = Default::default();
    configSpeaker.top = 0x8000;
    let mut SpeakerPwm = Pwm::new_output_a( // output A
        peripherals.PWM_CH3, // channel 0
        peripherals.PIN_22, // pin 0
        configSpeaker.clone()
    );

    


    //END SPEAKER
    
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}





// #[embassy_executor::task]
// async fn sd_card(
//     mut miso: PIN_4,
//     mut mosi: PIN_3,
//     mut clk: PIN_2,
//     mut cs:Output<'static, PIN_5>,
//     mut cs_sd:Output<'static, PIN_1>,
//     mut channel_spi: SPI0,
//     mut dma_spi_1: DMA_CH0,
//     mut dma_spi_2: DMA_CH1,
// ){
    

//     let mut microsd_config = spi::Config::default();
//     microsd_config.frequency = 400000;

//     // let mut spi = Spi::new(
//     //     channel_spi,
//     //     clk,
//     //     mosi,
//     //     miso,
//     //     dma_spi_1,
//     //     dma_spi_2,
//     //     microsd_config.clone(),
//     // );

//     Timer::after_secs(2).await;

//     info!("Initialize SPI SD/MMC data structures...");

//     let mut delay = Delay;

//     let mut spi = Spi::new_blocking(channel_spi, clk, mosi, miso, microsd_config);

//     let spi_bus = NoopMutex::new(RefCell::new(spi));

//     let spi_device = SpiDevice::new(&spi_bus, cs);

//     let mut sdcard = SdCard::new(spi_device, cs_sd, delay);

//     let mut time_source = Time::default();

//     Timer::after_millis(100).await;

//     let mut volume_mgr = VolumeManager::new(sdcard, time_source);

//     Timer::after_millis(100).await;

//     match volume_mgr.device().num_bytes() {
//         Ok(size) => info!("card size is {} bytes", size),
//         Err(e) => {
//             error!("Error retrieving card size: {:?}", e);
//         }
//     }

//     Timer::after_millis(100).await;

//     let mut volume_sd = match volume_mgr.get_volume(VolumeIdx(0)) {
//         Ok(volume) => volume, 
//         Err(err) => {
//           info!("Error opening volume: {:?}", err);
//           return;
//         }
//     };

//     Timer::after_millis(100).await;

//     info!("Volume: {:?}",volume_sd);

//     Timer::after_millis(100).await;
//     let volume_result = volume_mgr.open_root_dir(&volume_sd).unwrap();
    
//     Timer::after_millis(100).await;
//     info!("Root directory opened!");

//     volume_mgr.iterate_dir(&volume_sd, &volume_result, |ent| {
//         info!(
//             "/{}.{}",
//             core::str::from_utf8(ent.name.base_name()).unwrap(),
//             core::str::from_utf8(ent.name.extension()).unwrap()
//         );
//     }).unwrap();

//     let mut successful_read = false;

//     if let Ok(mut file) = volume_mgr.open_file_in_dir(&mut volume_sd, &volume_result, "a4.wav", Mode::ReadOnly) {
        
//         const SIZE: usize = 57300;

//         let mut buf = [0u8; SIZE];
//         let read_count = volume_mgr.read(&volume_sd, &mut file, &mut buf).unwrap();
//         volume_mgr.close_file(&volume_sd, file).unwrap();

//         let mut fmt = &buf[..36];
//         let mut sub_data = &buf[36..44];
//         let mut data = &buf[44..];

//         let data_size = u32::from_le_bytes([sub_data[4], sub_data[5], sub_data[6], sub_data[7]]);

//         let mut i = 0;
//         let mut j = 1;

//         let mut nr = SIZE;

//         if data.len() >= 2 {
//             info!("READ {} bytes:", read_count);
//             info!("FMT: {:?}", fmt);
//             info!("SUB_DATA {:?}", sub_data);
//             while nr > 0
//             {
//                 let bit_16 = u16::from_le_bytes([data[i], data[j]]);

//                 let bit_12 = bit_16 & 0x0FFF;


//                 info!("16-bit structure: {:#06x}", bit_16);
//                 info!("12-bit structure: {:#03x}", bit_12);







//                 i = i + 1;
//                 j = j + 1;
//                 nr = nr - 2;
//             }

//             // info!("DATA {:?}", data);

//             successful_read = true;
//         }
//     }

//     volume_mgr.free();

//     if successful_read {
//         info!("Successfully read file");
//     } else {
//         info!("Could not read file, which is ok for the first run.");
//         info!("Reboot the pico!");
//     }
// }