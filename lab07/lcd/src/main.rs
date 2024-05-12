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
use embassy_rp::peripherals::*;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_time::{Delay, Timer};

use embassy_rp::i2c::{I2c, InterruptHandler as I2CInterruptHandler, Config as I2cConfig};
use embedded_hal_async::i2c::{Error, I2c as _};


//use panic_halt as _;
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};
//use hd44780_driver::non_blocking::HD44780;


// pub struct AsyncDelay;

// impl hd44780_driver::Delay for AsyncDelay {
//     async fn delay_us(&self, us: u64) {
//         Timer::after(Duration::from_micros(us)).await;
//     }

//     async fn delay_ms(&self, ms: u16) {
//         Timer::after(Duration::from_millis(ms.into())).await;
//     }
// }

bind_interrupts!(struct Irqs {
  USBCTRL_IRQ => UsbInterruptHandler<USB>;
  I2C0_IRQ => I2CInterruptHandler<I2C0>;
});


#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
  let msg =  "Hi";
  info!("0");
  let peripherals = embassy_rp::init(Default::default());
  info!("6");
  let driver = Driver::new(peripherals.USB, Irqs);
  spawner.spawn(logger_task(driver)).unwrap();

  let sda = peripherals.PIN_0;
  let scl = peripherals.PIN_1;

  let mut i2c = I2c::new_async(peripherals.I2C0, scl, sda, Irqs, I2cConfig::default());

  let mut delay = embassy_time::Delay;
  //pin_mut!(delay);

  let mut display = HD44780::new_i2c(i2c, 0x27, &mut delay)
  .unwrap();

  display.clear(&mut delay);
  display.write_str(msg, &mut delay);
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
// use embassy_rp::peripherals::*;
// use embassy_sync::blocking_mutex::NoopMutex;
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
// use embassy_time::{Delay, Timer};

// use embassy_rp::i2c::{I2c, InterruptHandler as I2CInterruptHandler, Config as I2cConfig};
// use embedded_hal_async::i2c::{Error, I2c as _};

// use ag_lcd::{Cursor, LcdDisplay};
// use panic_halt as _;
// use port_expander::dev::pcf8574::Pcf8574;


// bind_interrupts!(struct Irqs {
//   USBCTRL_IRQ => UsbInterruptHandler<USB>;
//   I2C0_IRQ => I2CInterruptHandler<I2C0>;
// });


// #[embassy_executor::task]
// async fn logger_task(driver: Driver<'static, USB>) {
//     embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
// }


// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//   info!("0");
//   let peripherals = embassy_rp::init(Default::default());
//   info!("6");
//   let driver = Driver::new(peripherals.USB, Irqs);
//   spawner.spawn(logger_task(driver)).unwrap();

//   // With I2C
//   info!("5");
//   let delay = Delay;
//   info!("8");

//   let sda = peripherals.PIN_0;
//   let scl = peripherals.PIN_1;
//   info!("1");
//   let mut i2c = I2c::new_async(peripherals.I2C0, scl, sda, Irqs, I2cConfig::default());
//   let mut i2c_expander = Pcf8574::new(i2c, true, true, true);
//   info!("2");
//   let mut lcd: LcdDisplay<_, _> = LcdDisplay::new_pcf8574(&mut i2c_expander, delay)
//       .with_cursor(Cursor::Off)
//       .build();

//     info!("Attempting to print to LCD.");
//     lcd.print("Test message!");
//     info!("Message should be on the LCD now.");
// }

// // #[panic_handler]
// // fn panic(_info: &PanicInfo) -> ! {
// //     loop {}
// // }