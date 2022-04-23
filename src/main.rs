#![no_std]
#![no_main]

use embedded_hal::prelude::_embedded_hal_serial_Write;
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use cortex_m_rt::entry;
use stm32f4xx_hal::serial::{Serial, config};
use stm32f4xx_hal::gpio::{GpioExt};
use stm32f4xx_hal::rcc::RccExt;
use stm32f4xx_hal::time::Bps;

use embedded_hal::serial::Read;
use core::fmt::Write;   // write!()マクロを使えるようにする
use stm32lib::uart::ErrorDetect;    // 追加するトレイトを使えるようにする
use stm32f4xx_hal::prelude::*;  // MHz()
use cortex_m::delay;    // Delayを使う

use stm32f4xx_hal::pac::*;
use cortex_m::interrupt::*;
use core::cell::RefCell;
use core::ops::DerefMut;    // deref_mut()
use stm32f4xx_hal::serial::*;   // USART2
use stm32f4xx_hal::gpio::*; // Pin

const BUFFER_SIZE: usize = 256;

static UART: Mutex<RefCell<Option<
    Serial<USART2, (PA2, PA3)>
>>> = Mutex::new(RefCell::new(None));

static RDATA: Mutex<RefCell<Option<
    Receiver<>
>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {

    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();    // Delayを使うので
    let gpioa = dp.GPIOA.split();   // GPIOAのclockも有効にしてくれる （AHBENRレジスタ）
    let bps = Bps(115_200_u32); // (3)通信速度
    let seri_config = config::Config {  // (4)通信パラメーターの設定
        baudrate: bps,
        wordlength: config::WordLength::DataBits8,  // 実際には7ビット
        parity: config::Parity::ParityEven,
        stopbits: config::StopBits::STOP1,
        dma: config::DmaConfig::None,
    };

    let rcc = dp.RCC.constrain();
    let clks = rcc
        .cfgr
        .use_hse(8.MHz())   // 外部クロックを使う
        .bypass_hse_oscillator()    // 矩形波を使う（水晶振動子でなく発信器を使う）
        .sysclk(84.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    let mut delay = delay::Delay::new(cp.SYST, 84000000_u32);

    let serial = Serial::new(
        dp.USART2,
        (gpioa.pa2, gpioa.pa3),
        seri_config,
        &clks,
    ).unwrap(); // (5)Serial構造体の生成

    let receiver = Receiver::new();

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USART2);
    }
    cortex_m::interrupt::free(|cs| RDATA.borrow(cs).replace(Some(receiver)));

    cortex_m::interrupt::free(|cs| UART.borrow(cs).replace(Some(serial)));

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut serial) =
            UART.borrow(cs).borrow_mut().deref_mut()
        {
            serial.listen(Event::Rxne);
            writeln!(serial, "hellow UART receive interrupt world.\r\n").unwrap();                
        }
    });

    loop {
        delay.delay_ms(1000_u32);   // 1000msec遅延
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut receiver) =
                RDATA.borrow(cs).borrow_mut().deref_mut()
            {
                while receiver.readable() {
                    let c = receiver.get();
                    if let Some(ref mut serial) =
                        UART.borrow(cs).borrow_mut().deref_mut()
                    {
                        serial.write(c).unwrap();
                    }
                }
            }
        });
    }
}

#[interrupt]
fn USART2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut serial) =
            UART.borrow(cs).borrow_mut().deref_mut()
        {
            if serial.is_rx_not_empty() {
                if serial.is_pe() {
                    let _ = serial.read();  // 読み捨てる
                }
                else if serial.is_fe() {
                    let _ = serial.read();  // 読み捨てる
                }
                else if serial.is_ore() {
                    let _ = serial.read();  // 読み捨てる
                }
                else if let Ok(c) = serial.read() {
                    if let Some(ref mut receiver) =
                    RDATA.borrow(cs).borrow_mut().deref_mut()
                    {
                        receiver.put(c);
                    }
                }
            }
        }
    });
}

struct Receiver {
    buf: [u8; BUFFER_SIZE],
    rp: usize,
    wp: usize
}

impl Receiver {
    const fn new() -> Self {
        Receiver { buf: [0; BUFFER_SIZE], rp: 0, wp: 0 }
    }
    fn readable(&self) -> bool {
        if self.rp == self.wp {
            return false;
        }
        true
    }
    fn put(&mut self, c: u8) {
        self.buf[self.wp] = c;
        self.wp = (self.wp + 1) % BUFFER_SIZE;
    }
    fn get(&mut self) -> u8 {
        let c = self.buf[self.rp];
        self.rp = (self.rp + 1) % BUFFER_SIZE;
        c
    }
}