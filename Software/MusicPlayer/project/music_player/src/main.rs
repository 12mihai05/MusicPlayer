#![no_std]
#![no_main]


use embassy_executor::Spawner;

use core::cell::RefCell;
use core::panic::PanicInfo;

use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};
use log::info;
use log::error;

use embassy_rp::spi::{self, Blocking, Config, Spi};
use embassy_rp::adc::{Adc,Config as ADCConfig, Channel, InterruptHandler as ADCInterruptHandler};
use embassy_rp::pwm::{Config as PWMConfig, Pwm};
use embassy_rp::gpio::{Output, Level, Input,Pull};
use embassy_rp::peripherals::*;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_time::Duration;

use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx, Directory};
use embedded_sdmmc::TimeSource;
use embedded_sdmmc::Timestamp;

use embassy_rp::i2c::{I2c, Config as I2CConfig};
use ag_lcd::{Cursor, LcdDisplay};
use port_expander::dev::pcf8574::Pcf8574;

use microfft;
use bytemuck;
use micromath::F32Ext;

use heapless::String;
use heapless::Vec;

use fixed::FixedU16;



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
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    ADC_IRQ_FIFO => ADCInterruptHandler;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}


async fn file_name(
    count: usize,
    mut volume_result: &mut Directory<'_,SdCard<SpiDevice<'_,embassy_sync::blocking_mutex::raw::NoopRawMutex, Spi<'_,SPI0, spi::Blocking>, Output<'_, PIN_5>>, Output<'_, PIN_1>, Delay>, Time, 4, 4, 1>)-> Option<Vec<u8, 64>> {
        let mut files: Vec<Vec<u8, 64>, 5> = Vec::new();
        
        volume_result.iterate_dir(|ent| {
            if files.len() < 5 {
                let mut file_name = String::<64>::new();
                let name = core::str::from_utf8(ent.name.base_name()).unwrap_or("Invalid UTF-8");
                let extension = core::str::from_utf8(ent.name.extension()).unwrap_or("Invalid UTF-8");

                file_name.push_str(name).ok();
                if !extension.is_empty() {
                    file_name.push('.').ok();
                    file_name.push_str(extension).ok();
                }
    
                let mut file_vec: Vec<u8, 64> = Vec::new();
                file_vec.extend_from_slice(file_name.as_bytes()).ok();
                files.push(file_vec).ok();
            }
        }).unwrap();
    
        files.get(count).cloned()
    }

    pub trait PwmTrait {
        fn set_pwm_config(&mut self, compare: u16);
    }
    
    impl<'d, P: embassy_rp::pwm::Channel> PwmTrait for Pwm<'d, P> {
        fn set_pwm_config(&mut self, compare: u16) {
            let mut config = embassy_rp::pwm::Config::default();
            config.compare_a = compare;
            config.compare_b = compare;
            self.set_config(&config);
        }
    }
    
    fn set_pwm(pwm: &mut dyn PwmTrait, compare: u16) {
        pwm.set_pwm_config(compare);
    }


const DISPLAY_FREQ: u32 = 200_000;


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());

    let driver = Driver::new(peripherals.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    //START LCD

        let sda = peripherals.PIN_8;
        let scl = peripherals.PIN_9;
        Timer::after_millis(100).await;
        info!("Root directory opened!1");
        Timer::after_millis(100).await;
        
        let delay = Delay;
        Timer::after_millis(100).await;
    
        let mut config = I2CConfig::default();
        config.frequency = DISPLAY_FREQ;
        Timer::after_millis(100).await;

        let i2c = I2c::new_blocking(peripherals.I2C0, scl, sda, config.clone());
        Timer::after_millis(100).await;
        let mut i2c_expander = Pcf8574::new(i2c, true, true, true);
        Timer::after_millis(100).await;

        let mut lcd: LcdDisplay<_, _> = LcdDisplay::new_pcf8574(&mut i2c_expander, delay)
        .with_cursor(Cursor::Off)
        .with_reliable_init(10000)
        .build();
        Timer::after_millis(100).await;
    
        lcd.print("Starting...");
        Timer::after_millis(100).await;
        info!("Setup LCD");
    
        //END LCD

    //START SD CARD

        let mut miso = peripherals.PIN_4;
        let mut mosi = peripherals.PIN_3;
        let mut clk = peripherals.PIN_2;
        let mut cs = Output::new(peripherals.PIN_5, Level::High);
        let mut cs_sd = Output::new(peripherals.PIN_1, Level::High);
        let mut channel_spi = peripherals.SPI0;
        let mut microsd_config = spi::Config::default();
        microsd_config.frequency = 400000;

        Timer::after_secs(2).await;

        info!("Initialize SPI SD/MMC data structures...");

        let mut delay = Delay;
        Timer::after_millis(100).await;

        let mut spi: Spi<SPI0, Blocking> = Spi::new_blocking(channel_spi, clk, mosi, miso, microsd_config);
        Timer::after_millis(100).await;

        let spi_bus: embassy_sync::blocking_mutex::Mutex<NoopRawMutex, RefCell<Spi<SPI0, Blocking>>> = NoopMutex::new(RefCell::new(spi));
        Timer::after_millis(100).await;

        let spi_device: SpiDevice<NoopRawMutex, Spi<SPI0, Blocking>, Output<PIN_5>> = SpiDevice::new(&spi_bus, cs);
        Timer::after_millis(100).await;

        let mut sdcard: SdCard<SpiDevice<NoopRawMutex, Spi<SPI0, Blocking>, Output<PIN_5>>, Output<PIN_1>, Delay> = SdCard::new(spi_device, cs_sd, delay);
        Timer::after_millis(100).await;

        let mut time_source = Time::default();

        Timer::after_millis(100).await;

        let mut volume_mgr: VolumeManager<SdCard<SpiDevice<embassy_sync::blocking_mutex::raw::NoopRawMutex, Spi<SPI0, spi::Blocking>, Output<'_, PIN_5>>, Output<'_, PIN_1>, Delay>, Time> = VolumeManager::new(sdcard, time_source);

        Timer::after_millis(100).await;

        match volume_mgr.device().num_bytes() {
            Ok(size) => info!("card size is {} bytes", size),
            Err(e) => {
                error!("Error retrieving card size: {:?}", e);
            }
        }

        Timer::after_millis(100).await;

        let mut volume_sd = match volume_mgr.open_volume(VolumeIdx(0)) {
            Ok(volume) => volume, 
            Err(err) => {
              info!("Error opening volume: {:?}", err);
              return;
            }
        };

        Timer::after_millis(100).await;

        info!("Volume: {:?}",volume_sd);

        Timer::after_millis(100).await;
        info!("Attempting to open root directory...");
        Timer::after_millis(100).await;
        
        let mut volume_result = match volume_sd.open_root_dir() {
            Ok(root) => {
                info!("Successfully opened root directory");
                root
            },
            Err(err) => {
                error!("Error opening root directory: {:?}", err);
                return;
            }
        };
        
        info!("End setup card sd");
        Timer::after_millis(100).await;

        //END SD CARD



        
    
    //START BUTTONS
    Timer::after_millis(100).await;

    //push buttons
    let mut play_pause_button = Input::new(peripherals.PIN_13, Pull::Up);
    Timer::after_millis(100).await;
    let mut next_button = Input::new(peripherals.PIN_14, Pull::Up);
    Timer::after_millis(100).await;
    let mut previous_button = Input::new(peripherals.PIN_15, Pull::Up);
    Timer::after_millis(100).await;

    //potentiometer
    let mut adc = Adc::new(peripherals.ADC, Irqs, ADCConfig::default());
    Timer::after_millis(100).await;
    let mut adc_pin = Channel::new_pin(peripherals.PIN_26, Pull::None);
    Timer::after_millis(100).await;
    let level = adc.read(&mut adc_pin).await.unwrap();
    Timer::after_millis(100).await;

    //END BUTTONS
    info!("Setup buttons");
    Timer::after_millis(100).await;
    //START LEDS

    let mut configLED1: PWMConfig = Default::default();
    configLED1.top = 0x8000;
    let mut ledsPwm1 = Pwm::new_output_ab( // output AB
        peripherals.PWM_CH0, // channel 0
        peripherals.PIN_16, // pin 16
        peripherals.PIN_17, // pin 17
        configLED1.clone()
    );

    let mut configLED2: PWMConfig = Default::default();
    configLED2.top = 0x8000;
    let mut ledsPwm2: Pwm<PWM_CH5> = Pwm::new_output_ab( // output AB
        peripherals.PWM_CH5, // channel 5
        peripherals.PIN_10, // pin 10
        peripherals.PIN_11, // pin 11
        configLED2.clone()
    );

    info!("Setup leds");
    Timer::after_millis(1000).await;
    //END LEDS

    //START SPEAKER
    let mut configSpeaker: PWMConfig = Default::default();
    configSpeaker.top = 0x8000;
    let mut SpeakerPwm = Pwm::new_output_a( // output A
        peripherals.PWM_CH3, // channel CH3
        peripherals.PIN_22, // pin 22
        configSpeaker.clone()
    );

    configSpeaker.top = 0x8000;
    configSpeaker.phase_correct = true;
    configSpeaker.invert_a = false;
    configSpeaker.divider = FixedU16::from_bits((250.0 * 16.0) as u16);
    

    Timer::after_millis(100).await;

    info!("Setup speaker");


    //END SPEAKER

    Timer::after_millis(100).await;

    let mut play = true;

    let mut count=1;    

    let mut pause_lcd = 1;

    let mut duty_value = 0;

    info!("before loop");


    //LOOP
    'outer: loop{
        info!("in loop");
        let file_name_bytes: Vec<u8, 64> = file_name(count, &mut volume_result).await.unwrap(); //get name in bytes
        let file_name_str = core::str::from_utf8(&file_name_bytes).unwrap_or("Invalid UTF-8"); //get name in string
        info!("get name");
        info!("{}",file_name_str);
        Timer::after_millis(100).await;
    
        if let Ok(mut file) = volume_result.open_file_in_dir(file_name_str, Mode::ReadOnly){
            info!("in file");
            lcd.clear();
            Timer::after_millis(100).await;
            lcd.print(file_name_str);
            Timer::after_millis(100).await;
            const SIZE: usize = 1024;
            let mut buf = [0u8; SIZE];
            
            if !file.is_eof() {
                file.read(&mut buf[..44]).unwrap();
                let mut fmt = &buf[..36];
                let mut sub_data = &buf[36..44]; //first 44 bytes of a wav file are not used for audio output
        
                while !file.is_eof() {
                    let read_count_data = file.read(&mut buf).unwrap();
                    let mut data = &mut buf;

                    for byte in data.iter_mut() {
                        if *byte == 0 {
                            *byte = 1;
                        }
                    }

                    if play_pause_button.is_low(){
                        play = false;
                        Timer::after_millis(300).await;
                    }

                    while play == false{
                        if pause_lcd == 1{
                        lcd.clear();
                        lcd.print("pause");
                        set_pwm(&mut ledsPwm2, 0);
                        let mut configPwm = PWMConfig::default();
                            configPwm.top = 0;
                            configPwm.compare_a = 0;
                            SpeakerPwm.set_config(&configPwm);
                        }
                        pause_lcd = 0;
                        Timer::after(Duration::from_millis(10)).await;
                        if play_pause_button.is_low(){
                            play = true;
                            pause_lcd = 1;
                            Timer::after(Duration::from_millis(100)).await;
                            lcd.clear();
                            lcd.print(file_name_str);
                        }
                        continue
                    }
                    
                    if next_button.is_low(){
                        if count < 4{
                            count += 1;
                            continue 'outer;
                        }
                        if count == 4{
                            count = 0;
                        }
                    }

                    if previous_button.is_low(){
                        if count > 1{
                            count -= 1;
                            continue 'outer;
                        }
                        if count == 1{
                            count = 6;
                        }
                    }

                    let pot_value = adc.read(&mut adc_pin).await.unwrap(); 

                    let pcm_samples_i16: &[i16] = bytemuck::cast_slice(data); 
                    
                    //aconvert i16 to f32
                    let mut pcm_samples_f32: [f32; 1024] = [0.0; 1024];
                    for (i, &sample) in pcm_samples_i16.iter().enumerate().take(1024) {
                        pcm_samples_f32[i] = sample as f32;
                    }

                    //compute the real-valued fast Fourier transform (FFT)
                    let fft_results = microfft::real::rfft_1024(&mut pcm_samples_f32);
                    
                    //prelucrate each freaquency and amplitude value
                    for (frequency, amplitude) in fft_results.iter().enumerate() {
                        if frequency == 0 {
                            continue;
                        }
                        
                        let amplitude_magnitude = (amplitude.re * amplitude.re + amplitude.im * amplitude.im).sqrt();
                        let adjusted_amplitude = amplitude_magnitude * (pot_value as f32 / 4095.0);
                        
                        let mut top_value =(0x8000 as f32 * (pot_value as f32 / 4095.0)) as u16;
                        duty_value = ((top_value as f32 * adjusted_amplitude) * (pot_value as f32 / 4095.0))as u16;

                        //values below aproximatively 25 are very destortinate
                        if top_value < 25{
                            top_value = 25;
                        }

                        if duty_value < 25{
                            duty_value = 25;
                        }

                        let mut configPwm = PWMConfig::default();
                        configPwm.top = top_value;
                        
                        configPwm.compare_a = duty_value;
                        SpeakerPwm.set_config(&configPwm);
                        for _ in 0..1000 {
                            cortex_m::asm::nop(); //short delay
                        }
                    }
                    
                    set_pwm(&mut ledsPwm2, duty_value);
                    set_pwm(&mut ledsPwm1, pot_value);
                
                
            }
            if count == 4{
                count = 1
            }else{
                count+=1;
            }
            
            Timer::after_millis(100).await;
            continue 'outer;
        }
    }
}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

