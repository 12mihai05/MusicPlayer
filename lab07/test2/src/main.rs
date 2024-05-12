#![no_std]
#![no_main]

use embassy_executor::Spawner;

use core::cell::RefCell;
use core::panic::PanicInfo;

use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};
use log::info;

use embassy_rp::spi::{Spi, Config};
use embassy_rp::gpio::{Output, Level};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_time::Delay;

use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx};
use embedded_sdmmc::TimeSource;
use embedded_sdmmc::Timestamp;



bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[derive(Default)]
struct Time();

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


#[embassy_executor::main]
async fn main(spawner: Spawner) {

    let peripherals = embassy_rp::init(Default::default());

    let driver = Driver::new(peripherals.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let mut clk = peripherals.PIN_2;
    let mut mosi = peripherals.PIN_3;
    let mut miso = peripherals.PIN_4;
    let mut spi_cs = Output::new(peripherals.PIN_0, Level::High);
    let mut delay = Delay;
    let mut config = Config::default();

  
    let mut spi = Spi::new_blocking(peripherals.SPI0, clk, mosi, miso, config.clone());
    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_device = SpiDevice::new(&spi_bus, spi_cs);

    let mut sd_cs = Output::new(peripherals.PIN_1, Level::High);
    let mut sdcard = SdCard::new(spi_device, sd_cs, delay);
    let mut time_source = Time::default();

    let mut volume_mgr = VolumeManager::new(sdcard, time_source);
  
    let mut volume0 = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(volume) => volume, 
        Err(err) => {
          info!("Error opening volume: {:?}", err);
          return;
        }
    };
    
    let mut root_dir = match volume0.open_root_dir() {
        Ok(root) => root,
        Err(err) => {
          info!("Error opening root directory: {:?}", err);
          return;
        }
    };

    let mut my_file = match root_dir.open_file_in_dir("file.txt", Mode::ReadOnly) {
        Ok(file) => file,
        Err(err) => {
          info!("Error opening file: {:?}", err);
          return;
        }
    };

    while !my_file.is_eof() {
      let mut buffer = [0u8, 32];
      let num_read = match my_file.read(&mut buffer)  {
          Ok(num) => num,
          Err(err) => {
            info!("Error reading from file: {:?}", err);
          return;
          }
      };
      for b in &buffer[0..num_read] {
        info!("{}",  *b as char);
      }
    }

    loop {
      info!("Working");
      embassy_time::Timer::after_secs(10).await;
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}


// #![no_std]
// #![no_main]

// use embassy_executor::Spawner;

// use core::cell::RefCell;
// use core::panic::PanicInfo;

// use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
// use embassy_rp::{bind_interrupts, peripherals::USB};
// use log::info;

// use embassy_rp::spi::{Spi, Config};
// use embassy_rp::gpio::{Output, Level};
// use embassy_sync::blocking_mutex::NoopMutex;
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
// use embassy_time::{Delay, Timer};

// use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx};
// use embedded_sdmmc::TimeSource;
// use embedded_sdmmc::Timestamp;



// bind_interrupts!(struct Irqs {
//     USBCTRL_IRQ => UsbInterruptHandler<USB>;
// });

// #[embassy_executor::task]
// async fn logger_task(driver: Driver<'static, USB>) {
//     embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
// }

// #[derive(Default)]
// struct Time();

// impl TimeSource for Time {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp {
//           year_since_1970: 0,
//           zero_indexed_month: 0,
//           zero_indexed_day: 0,
//           hours: 0,
//           minutes: 0,
//           seconds: 0,
//         }
//     }
// }

// #[embassy_executor::main]
// async fn main(spawner: Spawner) {

//     let peripherals = embassy_rp::init(Default::default());

//     let driver = Driver::new(peripherals.USB, Irqs);
//     spawner.spawn(logger_task(driver)).unwrap();

//     let mut clk = peripherals.PIN_2;
//     let mut mosi = peripherals.PIN_3;
//     let mut miso = peripherals.PIN_4;
//     let mut spi_cs = Output::new(peripherals.PIN_0, Level::High);
//     let mut delay = Delay;
//     let mut config = Config::default();
//     config.frequency =  400000;

  
//     let mut spi = Spi::new_blocking(peripherals.SPI0, clk, mosi, miso, config.clone());
//     let spi_bus = NoopMutex::new(RefCell::new(spi));
//     let spi_device = SpiDevice::new(&spi_bus, spi_cs);

//     let mut sd_cs = Output::new(peripherals.PIN_1, Level::High);
//     let mut sdcard = SdCard::new(spi_device, sd_cs, delay);
//     let mut time_source = Time::default();

//     Timer::after_millis(100).await;

//     let mut volume_mgr = VolumeManager::new(sdcard, time_source);
  
//     match volume_mgr.device().num_bytes() {
//         Ok(size) => info!("card size is {} bytes", size),
//         Err(err) => {
//           info!("Error getting card size {:?}", err);
//         }
//     }

//     Timer::after_millis(100).await;

//     let mut volume0 = match volume_mgr.get_volume(VolumeIdx(0)) {
//         Ok(volume) => volume, 
//         Err(err) => {
//           info!("Error opening volume: {:?}", err);
//           return;
//         }
//     };

//     Timer::after_millis(100).await;
    
//     let mut root_dir = match volume_mgr.open_root_dir(&volume0) {
//         Ok(root) => root,
//         Err(err) => {
//           info!("Error opening root directory: {:?}", err);
//           return;
//         }
//     };

//     Timer::after_millis(100).await;

//     volume_mgr
//     .iterate_dir(&volume0, &root_dir, |ent| {
//         info!(
//             "/{}.{}",
//             core::str::from_utf8(ent.name.base_name()).unwrap(),
//             core::str::from_utf8(ent.name.extension()).unwrap()
//         );
//     })
//     .unwrap();

//     Timer::after_millis(100).await;

//     let mut successful_read = false;

//     if let Ok(mut file) = volume_mgr.open_file_in_dir(&mut volume0, &root_dir, "file.txt", Mode::ReadOnly) {
//       let mut buf = [0u8; 32];
//       let read_count = volume_mgr.read(&volume0, &mut file, &mut buf).unwrap();
//       volume_mgr.close_file(&volume0, file).unwrap();

//       if read_count >= 2 {
//           info!("READ {} bytes: {:?}", read_count, buf);
//           successful_read = true;
//       }
//   }

//   volume_mgr.free();
  
//   loop {
//     if successful_read {
//       info!("Working");
//     } else {
//         info!("Not Working");
//     }
//     embassy_time::Timer::after_secs(10).await;
//   }
// }

// #[panic_handler]
// fn panic(_info: &PanicInfo) -> ! {
//     loop {}
// }